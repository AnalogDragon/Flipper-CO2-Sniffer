#include <furi.h>
#include <gui/gui.h>
#include <input/input.h>
#include <stdio.h>
#include <string.h>

#include "co2_sniffer_i2c.h"
#include "co2_sniffer_bmp388.h"
#include "co2_sniffer_scd30.h"
#include "co2_sniffer_usb.h"

/* Serial frame streamed over CDC (big-endian):
 * [0..3]   header 0xDE 0xAD 0xFF 0x09 = the CH552's VID 0xDEAD + PID 0xFF09
 * [4]      payload length in bytes
 * [5..16]  payload, scaled to integers to keep precision:
 *          [0..1]  CO2       uint16  ppm
 *          [2..3]  T SCD30   int16   °C  x100
 *          [4..5]  RH SCD30  uint16  %RH x100
 *          [6..7]  T BMP388  int16   °C  x100
 *          [8..11] P BMP388  uint32  Pa
 * [17..18] CRC16 (Modbus: poly 0xA001, init 0xFFFF, reflected) over [0..16],
 *          little-endian
 * A failed sensor sends 0xFFFF / 0xFFFFFFFF for its values. */
#define CO2_FRAME_HDR 4
#define CO2_FRAME_LEN 1
#define CO2_FRAME_PAYLOAD 12
#define CO2_FRAME_CRC 2
#define CO2_FRAME_SIZE (CO2_FRAME_HDR + CO2_FRAME_LEN + CO2_FRAME_PAYLOAD + CO2_FRAME_CRC)

/* Sensor state machine tick, matching the CH552 firmware cadence */
#define CO2_TICK_MS 333

typedef struct {
    /* Sensor instances, owned by the worker thread */
    Co2Bmp388 bmp;
    Co2Scd30 scd;

    /* Snapshot for the UI, guarded by mutex */
    FuriMutex* mutex;
    bool bmp_ok;
    bool scd_ok;
    float T_LIN;
    float P_LIN;
    float CO2_PPM;
    float T_SCD;
    float R_SCD;
    bool usb_active;
    bool host_open;
    uint32_t frames_sent;

    Gui* gui;
    ViewPort* view_port;
    FuriThread* worker;
    volatile bool running;
    volatile bool reinit_requested;
} Co2SnifferApp;

static void co2_sniffer_draw_cb(Canvas* canvas, void* ctx) {
    Co2SnifferApp* app = ctx;

    bool bmp_ok, scd_ok, usb_active, host_open;
    float T_LIN, P_LIN;
    float CO2_PPM, T_SCD, R_SCD;
    uint32_t frames_sent;

    furi_check(furi_mutex_acquire(app->mutex, FuriWaitForever) == FuriStatusOk);
    bmp_ok = app->bmp_ok;
    scd_ok = app->scd_ok;
    T_LIN = app->T_LIN;
    P_LIN = app->P_LIN;
    CO2_PPM = app->CO2_PPM;
    T_SCD = app->T_SCD;
    R_SCD = app->R_SCD;
    usb_active = app->usb_active;
    host_open = app->host_open;
    frames_sent = app->frames_sent;
    furi_check(furi_mutex_release(app->mutex) == FuriStatusOk);

    char line[32];

    canvas_set_font(canvas, FontPrimary);
    canvas_draw_str(canvas, 2, 8, "CO2-Sniffer");

    canvas_set_font(canvas, FontSecondary);
    snprintf(line, sizeof(line), "BMP:%s SCD30:%s", bmp_ok ? "OK" : "ERR", scd_ok ? "OK" : "ERR");
    canvas_draw_str(canvas, 2, 16, line);

    snprintf(
        line,
        sizeof(line),
        "USB:%s F:%lu",
        !usb_active ? "off" : (host_open ? "TX" : "wait"),
        (unsigned long)frames_sent);
    canvas_draw_str(canvas, 2, 24, line);

    if(scd_ok)
        snprintf(line, sizeof(line), "CO2 %.0f ppm", (double)CO2_PPM);
    else
        snprintf(line, sizeof(line), "CO2 -- ppm");
    canvas_draw_str(canvas, 2, 32, line);

    if(bmp_ok)
        snprintf(line, sizeof(line), "Tb %.2f C", (double)T_LIN);
    else
        snprintf(line, sizeof(line), "Tb -- C");
    canvas_draw_str(canvas, 2, 40, line);

    if(bmp_ok)
        snprintf(line, sizeof(line), "Pb %.0f Pa", (double)P_LIN);
    else
        snprintf(line, sizeof(line), "Pb -- Pa");
    canvas_draw_str(canvas, 2, 48, line);

    if(scd_ok)
        snprintf(line, sizeof(line), "Ts %.2f C RH %.1f%%", (double)T_SCD, (double)R_SCD);
    else
        snprintf(line, sizeof(line), "Ts -- C RH --%%");
    canvas_draw_str(canvas, 2, 56, line);

    canvas_draw_str(canvas, 2, 63, "OK:reinit BACK:exit");
}

static void co2_sniffer_input_cb(InputEvent* event, void* ctx) {
    Co2SnifferApp* app = ctx;

    if(event->type == InputTypeShort) {
        if(event->key == InputKeyBack) {
            app->running = false;
        } else if(event->key == InputKeyOk) {
            app->reinit_requested = true;
        }
    }
}

static void co2_frame_put16(uint8_t* p, uint16_t v) {
    p[0] = (uint8_t)(v >> 8);
    p[1] = (uint8_t)v;
}

static void co2_frame_put32(uint8_t* p, uint32_t v) {
    p[0] = (uint8_t)(v >> 24);
    p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8);
    p[3] = (uint8_t)v;
}

/* Modbus CRC16: poly 0xA001 (reflected 0x8005), init 0xFFFF, LSB first */
static uint16_t co2_frame_crc16(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for(size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for(uint8_t b = 0; b < 8; b++) {
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
        }
    }
    return crc;
}

static int32_t co2_sniffer_worker(void* ctx) {
    Co2SnifferApp* app = ctx;
    uint8_t frame[CO2_FRAME_SIZE];
    uint16_t scd_press_bak = 0;
    uint32_t frames_sent = 0;

    app->bmp.fail = true;
    app->scd.fail = true;

    co2_i2c_acquire();
    co2_usb_stream_start();

    while(app->running) {
        if(app->reinit_requested) {
            app->reinit_requested = false;
            app->bmp.fail = true;
            app->scd.fail = true;
            scd_press_bak = 0;
        }

        /* BMP388: init is retried each tick while failing */
        if(app->bmp.fail) {
            co2_bmp388_init(&app->bmp);
        } else {
            co2_bmp388_read(&app->bmp);
        }

        /* SCD30: init is retried each tick while failing */
        if(app->scd.fail) {
            uint16_t press = (uint16_t)((app->bmp.P_FIL + 0.5f) / 100.0f);
            scd_press_bak = press;
            co2_scd30_init(&app->scd, press);
        } else {
            co2_scd30_tick(&app->scd);

            /* Ambient pressure compensation feed (CH552 state machine step 5):
             * restart the measurement when the filtered BMP388 pressure moved
             * and is within the SCD30's accepted 700..1200 mbar range. */
            uint16_t press = (uint16_t)((app->bmp.P_FIL + 0.5f) / 100.0f);
            if(!app->scd.fail && !app->bmp.fail && press >= 700 && press <= 1200 &&
               press != scd_press_bak) {
                scd_press_bak = press;
                if(!co2_scd30_set_pressure(press)) {
                    app->scd.fail = true;
                }
            }
        }

        /* Build the serial frame: header + length + scaled integer payload +
         * CRC16. A failed sensor sends 0xFF values, like the CH552 did. */
        frame[0] = 0xDE;
        frame[1] = 0xAD;
        frame[2] = 0xFF;
        frame[3] = 0x09;
        frame[4] = CO2_FRAME_PAYLOAD;
        if(!app->scd.fail) {
            co2_frame_put16(frame + 5, (uint16_t)app->scd.CO2_PPM);
            co2_frame_put16(frame + 7, (uint16_t)(int16_t)(app->scd.T_SCD * 100.0f));
            co2_frame_put16(frame + 9, (uint16_t)(app->scd.R_SCD * 100.0f));
        } else {
            memset(frame + 5, 0xFF, 6);
        }
        if(!app->bmp.fail) {
            co2_frame_put16(frame + 11, (uint16_t)(int16_t)(app->bmp.T_LIN * 100.0f));
            co2_frame_put32(frame + 13, (uint32_t)app->bmp.P_LIN);
        } else {
            memset(frame + 11, 0xFF, 6);
        }
        uint16_t crc = co2_frame_crc16(frame, CO2_FRAME_HDR + CO2_FRAME_LEN + CO2_FRAME_PAYLOAD);
        frame[CO2_FRAME_SIZE - 2] = (uint8_t)crc;
        frame[CO2_FRAME_SIZE - 1] = (uint8_t)(crc >> 8);

        if(co2_usb_stream_send(frame, CO2_FRAME_SIZE)) {
            frames_sent++;
        }

        /* Publish the snapshot for the UI */
        furi_check(furi_mutex_acquire(app->mutex, FuriWaitForever) == FuriStatusOk);
        app->bmp_ok = !app->bmp.fail;
        app->scd_ok = !app->scd.fail;
        app->T_LIN = app->bmp.T_LIN;
        app->P_LIN = app->bmp.P_LIN;
        app->CO2_PPM = app->scd.CO2_PPM;
        app->T_SCD = app->scd.T_SCD;
        app->R_SCD = app->scd.R_SCD;
        app->usb_active = co2_usb_stream_active();
        app->host_open = co2_usb_stream_host_open();
        app->frames_sent = frames_sent;
        furi_check(furi_mutex_release(app->mutex) == FuriStatusOk);

        furi_delay_ms(CO2_TICK_MS);
    }

    co2_usb_stream_stop();
    co2_i2c_release();
    return 0;
}

int32_t co2_sniffer_app(void* p) {
    UNUSED(p);

    Co2SnifferApp* app = malloc(sizeof(Co2SnifferApp));
    memset(app, 0, sizeof(*app));
    app->gui = furi_record_open(RECORD_GUI);
    app->mutex = furi_mutex_alloc(FuriMutexTypeNormal);
    app->view_port = view_port_alloc();
    app->running = true;
    app->bmp.fail = true;
    app->scd.fail = true;

    view_port_draw_callback_set(app->view_port, co2_sniffer_draw_cb, app);
    view_port_input_callback_set(app->view_port, co2_sniffer_input_cb, app);
    gui_add_view_port(app->gui, app->view_port, GuiLayerFullscreen);

    app->worker = furi_thread_alloc_ex("Co2Sniffer", 2048, co2_sniffer_worker, app);
    furi_thread_start(app->worker);

    while(app->running) {
        view_port_update(app->view_port);
        furi_delay_ms(100);
    }

    furi_thread_join(app->worker);
    furi_thread_free(app->worker);

    view_port_enabled_set(app->view_port, false);
    gui_remove_view_port(app->gui, app->view_port);
    view_port_free(app->view_port);
    furi_record_close(RECORD_GUI);
    furi_mutex_free(app->mutex);
    free(app);

    return 0;
}
