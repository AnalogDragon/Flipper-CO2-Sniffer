#include <furi.h>
#include <furi_hal_power.h>
#include <furi_hal_rtc.h>
#include <gui/gui.h>
#include <input/input.h>
#include <math.h>
#include <notification/notification_messages.h>
#include <storage/storage.h>
#include <stdio.h>
#include <string.h>

#include "co2_sniffer_i2c.h"
#include "co2_sniffer_bmp388.h"
#include "co2_sniffer_scd30.h"
#include "co2_sniffer_usb.h"

/* Serial frame streamed over CDC (big-endian):
 * [0..3]   header 0xDE 0xAD 0xFF 0x09
 * [4]      payload length in bytes
 * [5..16]  CO2, SCD30 temperature/RH, BMP388 temperature/pressure
 * [17..18] Modbus CRC16, little-endian. */
#define CO2_FRAME_HDR 4
#define CO2_FRAME_LEN 1
#define CO2_FRAME_PAYLOAD 12
#define CO2_FRAME_CRC 2
#define CO2_FRAME_SIZE (CO2_FRAME_HDR + CO2_FRAME_LEN + CO2_FRAME_PAYLOAD + CO2_FRAME_CRC)

#define CO2_TICK_MS 333
#define CO2_DATA_PAGE_COUNT 6
#define CO2_RECORD_PAGE CO2_DATA_PAGE_COUNT
#define CO2_STYLE_COUNT 4
#define CO2_TIME_SCALE_COUNT 6
#define CO2_HISTORY_POINTS 128
#define CO2_METRIC_COUNT 5
#define CO2_BMP_WARMUP_MS 5000
#define CO2_SCD_WARMUP_MS 10000
#define CO2_ALTITUDE_FILTER_ALPHA 0.2f
#define CO2_RECORD_INTERVAL_COUNT 5
#define CO2_RECORD_DURATION_COUNT 6
#define CO2_RECORD_CONTINUOUS 5
#define CO2_RECORD_DIRECTORY EXT_PATH("co2_sniffer")

typedef enum {
    Co2MetricCo2,
    Co2MetricTemperature,
    Co2MetricHumidity,
    Co2MetricPressure,
    Co2MetricAltitude,
} Co2Metric;

static const uint32_t co2_time_scale_ms[CO2_TIME_SCALE_COUNT] = {
    2000,
    5000,
    10000,
    30000,
    60000,
    600000,
};

static const char* const co2_time_scale_name[CO2_TIME_SCALE_COUNT] = {
    "2s",
    "5s",
    "10s",
    "30s",
    "1m",
    "10m",
};

static const uint32_t co2_record_interval_ms[CO2_RECORD_INTERVAL_COUNT] = {
    2000,
    5000,
    10000,
    30000,
    60000,
};

static const char* const co2_record_interval_name[CO2_RECORD_INTERVAL_COUNT] = {
    "2s",
    "5s",
    "10s",
    "30s",
    "1min",
};

static const uint32_t co2_record_duration_ms[CO2_RECORD_DURATION_COUNT - 1] = {
    60000,
    300000,
    600000,
    1800000,
    3600000,
};

static const char* const co2_record_duration_name[CO2_RECORD_DURATION_COUNT] = {
    "1min",
    "5min",
    "10min",
    "30min",
    "1h",
    "continuous",
};

static const NotificationSequence co2_backlight_enable_sequence = {
    &message_display_backlight_off,
    &message_delay_100,
    &message_display_backlight_on,
    &message_delay_100,
    &message_display_backlight_off,
    &message_delay_100,
    &message_display_backlight_on,
    &message_display_backlight_enforce_on,
    NULL,
};

static const NotificationSequence co2_backlight_disable_sequence = {
    &message_display_backlight_enforce_auto,
    &message_display_backlight_off,
    &message_delay_500,
    &message_display_backlight_on,
    NULL,
};

typedef struct {
    Co2Bmp388 bmp;
    Co2Scd30 scd;

    FuriMutex* mutex;
    bool bmp_ok;
    bool scd_ok;
    float T_LIN;
    float P_LIN;
    float altitude;
    float CO2_PPM;
    float T_SCD;
    float R_SCD;
    bool usb_active;
    bool host_open;
    uint32_t frames_sent;

    bool home_done;
    uint8_t home_category;
    uint32_t home_deadline_ms;
    uint8_t page;
    uint8_t style;
    uint8_t time_scale;
    bool clear_confirm;
    bool exit_confirm;
    bool history_reset_requested;
    bool backlight_always_on;

    uint8_t record_interval;
    uint8_t record_duration;
    uint8_t record_cursor;
    bool record_editing;
    bool record_start_requested;
    bool record_stop_requested;
    bool recording;
    bool record_error;
    bool i2c_fatal;

    /* Each time scale has its own 128-point ring. This preserves every graph
     * while keeping the session history below 20 KB. */
    float history[CO2_TIME_SCALE_COUNT][CO2_METRIC_COUNT][CO2_HISTORY_POINTS];
    uint8_t history_valid[CO2_TIME_SCALE_COUNT][CO2_HISTORY_POINTS];
    uint8_t history_head[CO2_TIME_SCALE_COUNT];
    uint8_t history_count[CO2_TIME_SCALE_COUNT];
    int32_t graph_low[CO2_TIME_SCALE_COUNT][CO2_METRIC_COUNT];
    int32_t graph_high[CO2_TIME_SCALE_COUNT][CO2_METRIC_COUNT];
    uint8_t graph_bounds_valid[CO2_TIME_SCALE_COUNT];

    Gui* gui;
    NotificationApp* notifications;
    ViewPort* view_port;
    FuriThread* worker;
    volatile bool running;
} Co2SnifferApp;

static bool co2_time_reached(uint32_t now, uint32_t deadline) {
    return (int32_t)(now - deadline) >= 0;
}

static float co2_pressure_to_altitude(float pressure_pa) {
    if(pressure_pa <= 0.0f) return 0.0f;
    return 44330.0f * (1.0f - powf(pressure_pa / 101325.0f, 0.190295f));
}

static int32_t co2_draw_text(Canvas* canvas, int32_t x, int32_t y, const char* text) {
    canvas_draw_str(canvas, x, y, text);
    return x + canvas_string_width(canvas, text);
}

/* Draw the degree sign geometrically because it is absent from some Flipper
 * font builds. Returns the x coordinate following the trailing C. */
static int32_t co2_draw_temperature(
    Canvas* canvas,
    int32_t x,
    int32_t y,
    float value,
    bool valid,
    uint8_t decimals) {
    char text[20];
    if(!valid) return co2_draw_text(canvas, x, y, "-- C");

    snprintf(text, sizeof(text), "%.*f", decimals, (double)value);
    x = co2_draw_text(canvas, x, y, text);
    canvas_draw_circle(canvas, x + 2, y - 6, 1);
    canvas_draw_str(canvas, x + 5, y, "C");
    return x + 5 + canvas_string_width(canvas, "C");
}

static void co2_draw_home_status(Canvas* canvas, int32_t y, bool online) {
    canvas_set_font(canvas, FontPrimary);
    if(online) {
        canvas_draw_str_aligned(canvas, 120, y, AlignRight, AlignBottom, "READY");
    } else {
        const char* text = "MISSING";
        uint16_t width = canvas_string_width(canvas, text);
        int32_t x = 120 - width;
        canvas_set_color(canvas, ColorBlack);
        canvas_draw_box(canvas, x - 2, y - 9, width + 4, 11);
        canvas_set_color(canvas, ColorWhite);
        canvas_draw_str_aligned(canvas, 120, y, AlignRight, AlignBottom, text);
        canvas_set_color(canvas, ColorBlack);
    }
}

static void co2_draw_home(Canvas* canvas, Co2SnifferApp* app) {
    bool bmp_ok;
    bool scd_ok;
    uint32_t deadline;

    furi_check(furi_mutex_acquire(app->mutex, FuriWaitForever) == FuriStatusOk);
    bmp_ok = app->bmp_ok;
    scd_ok = app->scd_ok;
    deadline = app->home_deadline_ms;
    furi_check(furi_mutex_release(app->mutex) == FuriStatusOk);

    canvas_set_font(canvas, FontPrimary);
    canvas_draw_str_aligned(canvas, 64, 9, AlignCenter, AlignBottom, "CO2-Sniffer");
    canvas_draw_line(canvas, 8, 12, 119, 12);

    canvas_set_font(canvas, FontSecondary);
    canvas_draw_str(canvas, 8, 28, "BMP388");
    canvas_draw_str(canvas, 49, 28, ":");
    canvas_draw_str(canvas, 8, 43, "SCD30");
    canvas_draw_str(canvas, 49, 43, ":");

    co2_draw_home_status(canvas, 28, bmp_ok);
    co2_draw_home_status(canvas, 43, scd_ok);

    uint32_t now = furi_get_tick();
    if(!bmp_ok && !scd_ok) {
        canvas_set_font(canvas, FontSecondary);
        canvas_draw_str_aligned(canvas, 64, 60, AlignCenter, AlignBottom, "Waiting for sensors...");
    } else {
        char line[24];
        uint32_t remaining = co2_time_reached(now, deadline) ? 0 : deadline - now;
        snprintf(
            line,
            sizeof(line),
            "Starting in %lu s",
            (unsigned long)((remaining + 999) / 1000));
        canvas_set_font(canvas, FontSecondary);
        canvas_draw_str_aligned(canvas, 64, 60, AlignCenter, AlignBottom, line);
    }
}

static void co2_draw_full_page(
    Canvas* canvas,
    bool bmp_ok,
    bool scd_ok,
    float t_bmp,
    float pressure,
    float altitude,
    float co2,
    float t_scd,
    float humidity) {
    char text[32];
    const int32_t colon_x = 29;
    const int32_t value_x = 36;
    canvas_set_font(canvas, FontPrimary);
    canvas_draw_str(canvas, 1, 8, "CO2-Sniffer");

    canvas_set_font(canvas, FontSecondary);
    canvas_draw_str(canvas, 1, 19, "CO2");
    canvas_draw_str(canvas, colon_x, 19, ":");
    canvas_set_font(canvas, FontPrimary);
    if(scd_ok)
        snprintf(text, sizeof(text), "%.0f ppm", (double)co2);
    else
        snprintf(text, sizeof(text), "-- ppm");
    canvas_draw_str(canvas, value_x, 19, text);

    canvas_set_font(canvas, FontSecondary);
    canvas_draw_str(canvas, 1, 30, "TEMP");
    canvas_draw_str(canvas, colon_x, 30, ":");
    canvas_set_font(canvas, FontPrimary);
    int32_t x = co2_draw_temperature(canvas, value_x, 30, t_scd, scd_ok, 2);
    canvas_set_font(canvas, FontSecondary);
    x = co2_draw_text(canvas, x + 2, 30, " (");
    x = co2_draw_temperature(canvas, x, 30, t_bmp, bmp_ok, 2);
    co2_draw_text(canvas, x, 30, ")");

    canvas_draw_str(canvas, 1, 41, "RH");
    canvas_draw_str(canvas, colon_x, 41, ":");
    if(scd_ok)
        snprintf(text, sizeof(text), "%.2f %%", (double)humidity);
    else
        snprintf(text, sizeof(text), "-- %%");
    canvas_set_font(canvas, FontPrimary);
    canvas_draw_str(canvas, value_x, 41, text);

    canvas_set_font(canvas, FontSecondary);
    canvas_draw_str(canvas, 1, 52, "Press");
    canvas_draw_str(canvas, colon_x, 52, ":");
    if(bmp_ok)
        snprintf(text, sizeof(text), "%.2f hPa", (double)(pressure / 100.0f));
    else
        snprintf(text, sizeof(text), "-- hPa");
    canvas_set_font(canvas, FontPrimary);
    canvas_draw_str(canvas, value_x, 52, text);

    canvas_set_font(canvas, FontSecondary);
    canvas_draw_str(canvas, 1, 63, "Alt");
    canvas_draw_str(canvas, colon_x, 63, ":");
    if(bmp_ok)
        snprintf(text, sizeof(text), "%.1f m", (double)altitude);
    else
        snprintf(text, sizeof(text), "-- m");
    canvas_set_font(canvas, FontPrimary);
    canvas_draw_str(canvas, value_x, 63, text);
}

static void co2_draw_big_number(
    Canvas* canvas,
    float value,
    bool valid,
    uint8_t decimals,
    int32_t y) {
    char text[20];
    if(!valid) {
        canvas_set_font(canvas, FontPrimary);
        canvas_draw_str_aligned(canvas, 64, y, AlignCenter, AlignBottom, "--");
        return;
    }
    snprintf(text, sizeof(text), "%.*f", decimals, (double)value);
    canvas_set_font(canvas, FontBigNumbers);
    canvas_draw_str_aligned(canvas, 64, y, AlignCenter, AlignBottom, text);
}

static void co2_draw_analog_bar(Canvas* canvas, bool valid, float co2) {
    const uint8_t segment_count = 30;
    const int32_t segment_x = 4;
    const int32_t segment_y = 37;
    const int32_t segment_pitch = 4;
    const size_t segment_width = 3;
    const size_t segment_height = 9;
    uint16_t range = 3000;
    if(valid && co2 > 5000.0f)
        range = 15000;
    else if(valid && co2 > 3000.0f)
        range = 5000;

    uint8_t active_segments = 0;
    if(valid && co2 > 0.0f) {
        float clamped = co2 < (float)range ? co2 : (float)range;
        active_segments = (uint8_t)ceilf(clamped * segment_count / (float)range);
    }

    for(uint8_t i = 0; i < segment_count; i++) {
        int32_t x = segment_x + i * segment_pitch;
        if(i < active_segments)
            canvas_draw_box(canvas, x, segment_y, segment_width, segment_height);
        else
            canvas_draw_frame(canvas, x, segment_y, segment_width, segment_height);
    }

    char text[16];
    canvas_set_font(canvas, FontSecondary);
    canvas_draw_str(canvas, segment_x, 56, "0");
    snprintf(text, sizeof(text), "%uppm", (unsigned int)range);
    canvas_draw_str_aligned(canvas, 124, 56, AlignRight, AlignBottom, text);
}

static void co2_draw_co2_page(Canvas* canvas, bool valid, float co2) {
    canvas_set_font(canvas, FontPrimary);
    int32_t name_x = 2 + canvas_string_width(canvas, "0");
    canvas_draw_str(canvas, name_x, 25, "CO2");
    co2_draw_big_number(canvas, co2, valid, 0, 25);
    canvas_set_font(canvas, FontSecondary);
    int32_t ppm_right = 127 - canvas_string_width(canvas, "0");
    canvas_draw_str_aligned(canvas, ppm_right, 25, AlignRight, AlignBottom, "ppm");
    co2_draw_analog_bar(canvas, valid, co2);
}

static void co2_draw_temperature_page(
    Canvas* canvas,
    bool bmp_ok,
    bool scd_ok,
    float t_bmp,
    float t_scd,
    float humidity) {
    bool temp_ok = scd_ok || bmp_ok;
    float temperature = scd_ok ? t_scd : t_bmp;
    char text[20];

    canvas_set_font(canvas, FontSecondary);
    canvas_draw_str(canvas, 2, 9, "Temperature");
    co2_draw_big_number(canvas, temperature, temp_ok, 2, 29);
    canvas_set_font(canvas, FontPrimary);
    canvas_draw_circle(canvas, 105, 18, 1);
    canvas_draw_str(canvas, 108, 24, "C");

    canvas_set_font(canvas, FontSecondary);
    canvas_draw_str(canvas, 2, 40, "Humidity");
    if(scd_ok)
        snprintf(text, sizeof(text), "%.2f", (double)humidity);
    else
        snprintf(text, sizeof(text), "--");
    canvas_set_font(canvas, scd_ok ? FontBigNumbers : FontPrimary);
    canvas_draw_str_aligned(canvas, 64, 59, AlignCenter, AlignBottom, text);
    canvas_set_font(canvas, FontPrimary);
    canvas_draw_str(canvas, 108, 55, "%");
}

static void co2_draw_pressure_page(
    Canvas* canvas,
    bool bmp_ok,
    float pressure,
    float altitude) {
    canvas_set_font(canvas, FontPrimary);
    canvas_draw_str_aligned(canvas, 64, 9, AlignCenter, AlignBottom, "Pressure");

    char pressure_text[20];
    if(bmp_ok)
        snprintf(pressure_text, sizeof(pressure_text), "%.2f", (double)(pressure / 100.0f));
    else
        snprintf(pressure_text, sizeof(pressure_text), "--");
    canvas_set_font(canvas, FontSecondary);
    int32_t unit_right = 127 - canvas_string_width(canvas, "0");
    int32_t unit_left = unit_right - canvas_string_width(canvas, "hPa");
    int32_t number_right = unit_left - 2 * canvas_string_width(canvas, "0");
    canvas_set_font(canvas, bmp_ok ? FontBigNumbers : FontPrimary);
    canvas_draw_str_aligned(canvas, number_right, 38, AlignRight, AlignBottom, pressure_text);
    canvas_set_font(canvas, FontSecondary);
    canvas_draw_str_aligned(canvas, unit_right, 38, AlignRight, AlignBottom, "hPa");

    char text[32];
    if(bmp_ok)
        snprintf(text, sizeof(text), "Altitude %.1f m", (double)altitude);
    else
        snprintf(text, sizeof(text), "Altitude -- m");
    canvas_draw_str_aligned(canvas, 64, 62, AlignCenter, AlignBottom, text);
}

static void co2_graph_bounds(
    float minimum,
    float maximum,
    uint8_t metric,
    int32_t* low,
    int32_t* high) {
    static const int32_t precision[CO2_METRIC_COUNT] = {50, 1, 5, 1, 1};
    float step = (float)precision[metric];

    *low = (int32_t)(floorf(minimum / step) * step);
    *high = (int32_t)(ceilf(maximum / step) * step);
    if(*high <= *low) *high = *low + precision[metric];
}

static void co2_draw_graph(
    Canvas* canvas,
    Co2SnifferApp* app,
    uint8_t metric,
    bool current_valid,
    float current) {
    float points[CO2_HISTORY_POINTS];
    uint8_t valid[CO2_HISTORY_POINTS];
    uint8_t count;
    uint8_t scale;
    bool bounds_valid;
    int32_t low;
    int32_t high;

    furi_check(furi_mutex_acquire(app->mutex, FuriWaitForever) == FuriStatusOk);
    scale = app->time_scale;
    count = app->history_count[scale];
    bounds_valid = app->graph_bounds_valid[scale] & (1U << metric);
    low = app->graph_low[scale][metric];
    high = app->graph_high[scale][metric];
    uint8_t oldest = (uint8_t)((app->history_head[scale] + CO2_HISTORY_POINTS - count) %
                               CO2_HISTORY_POINTS);
    for(uint8_t i = 0; i < count; i++) {
        uint8_t source = (uint8_t)((oldest + i) % CO2_HISTORY_POINTS);
        points[i] = app->history[scale][metric][source];
        valid[i] = app->history_valid[scale][source] & (1U << metric);
    }
    furi_check(furi_mutex_release(app->mutex) == FuriStatusOk);

    static const char* const metric_name[CO2_METRIC_COUNT] = {
        "CO2",
        "TEMP",
        "RH",
        "Pressure",
        "Altitude",
    };

    char text[24];
    if(current_valid) {
        if(metric == Co2MetricCo2)
            snprintf(text, sizeof(text), "%.0f", (double)current);
        else if(metric == Co2MetricTemperature)
            snprintf(text, sizeof(text), "%.2f", (double)current);
        else if(metric == Co2MetricHumidity)
            snprintf(text, sizeof(text), "%.2f %%", (double)current);
        else if(metric == Co2MetricPressure)
            snprintf(text, sizeof(text), "%.2f hPa", (double)current);
        else
            snprintf(text, sizeof(text), "%.1f m", (double)current);
    } else {
        snprintf(text, sizeof(text), "--");
    }
    /* Keep the live value outside the graph so neither hides the other. */
    canvas_set_font(canvas, FontSecondary);
    if(current_valid && metric == Co2MetricTemperature) {
        int32_t width = canvas_string_width(canvas, text) + 5 + canvas_string_width(canvas, "C");
        co2_draw_temperature(canvas, 128 - width, 63, current, true, 2);
    } else if(current_valid && metric == Co2MetricCo2) {
        int32_t ppm_x = 128 - canvas_string_width(canvas, "ppm");
        int32_t number_right = ppm_x - canvas_string_width(canvas, " ");
        canvas_draw_str_aligned(canvas, number_right, 63, AlignRight, AlignBottom, text);
        canvas_draw_str(canvas, ppm_x, 62, "ppm");
    } else {
        canvas_draw_str_aligned(canvas, 127, 63, AlignRight, AlignBottom, text);
    }

    bool found = false;
    float minimum = 0.0f;
    float maximum = 0.0f;
    for(uint8_t i = 0; i < count; i++) {
        if(!valid[i]) continue;
        if(!found) {
            minimum = maximum = points[i];
            found = true;
        } else {
            if(points[i] < minimum) minimum = points[i];
            if(points[i] > maximum) maximum = points[i];
        }
    }

    const int32_t plot_left = 0;
    const int32_t plot_right = 127;
    const int32_t curve_top = 0;
    const int32_t curve_bottom = 55;
    const int32_t scale_top = 4;
    const int32_t scale_bottom = 52;

    if(found) {
        int32_t candidate_low;
        int32_t candidate_high;
        co2_graph_bounds(minimum, maximum, metric, &candidate_low, &candidate_high);

        if(bounds_valid) {
            float current_half_cell = (float)(high - low) / 8.0f;
            float current_display_low = (float)low - current_half_cell;
            float current_display_high = (float)high + current_half_cell;
            bool within_display =
                minimum >= current_display_low && maximum <= current_display_high;
            bool candidate_expands = candidate_low < low || candidate_high > high;

            /* Keep the current scale while new extrema still fit in its outer
             * half-cells. A narrower candidate may still shrink immediately. */
            if(!within_display || !candidate_expands) {
                low = candidate_low;
                high = candidate_high;
            }
        } else {
            low = candidate_low;
            high = candidate_high;
            bounds_valid = true;
        }

        furi_check(furi_mutex_acquire(app->mutex, FuriWaitForever) == FuriStatusOk);
        app->graph_low[scale][metric] = low;
        app->graph_high[scale][metric] = high;
        app->graph_bounds_valid[scale] |= 1U << metric;
        furi_check(furi_mutex_release(app->mutex) == FuriStatusOk);

        float half_cell = (float)(high - low) / 8.0f;
        float display_low = (float)low - half_cell;
        float display_high = (float)high + half_cell;

        bool previous_valid = false;
        int32_t previous_x = 0;
        int32_t previous_y = 0;
        for(uint8_t i = 0; i < count; i++) {
            if(!valid[i]) {
                previous_valid = false;
                continue;
            }
            /* One stored sample always occupies exactly one horizontal pixel.
             * Until the ring is full, samples stay right-aligned and the old
             * graph moves left by one pixel whenever a sample is appended. */
            int32_t x = plot_right - ((int32_t)count - 1 - (int32_t)i);
            int32_t y = curve_bottom -
                        (int32_t)((points[i] - display_low) * (curve_bottom - curve_top) /
                                  (display_high - display_low));
            if(y < curve_top) y = curve_top;
            if(y > curve_bottom) y = curve_bottom;
            if(previous_valid) canvas_draw_line(canvas, previous_x, previous_y, x, y);
            canvas_draw_dot(canvas, x, y);
            previous_valid = true;
            previous_x = x;
            previous_y = y;
        }
    }

    /* Keep the existing scale positions. The curve itself uses y=0..55 and a
     * half-cell extension beyond both displayed scale values. */
    for(uint8_t tick = 0; tick < 5; tick++) {
        int32_t y = scale_top + tick * (scale_bottom - scale_top) / 4;
        canvas_draw_line(canvas, plot_left, y, plot_left + 2, y);
        canvas_draw_line(canvas, plot_right - 1, y, plot_right, y);
        if(tick < 4) {
            int32_t minor_y = y + (scale_bottom - scale_top) / 8;
            canvas_draw_line(canvas, plot_left, minor_y, plot_left + 1, minor_y);
            canvas_draw_dot(canvas, plot_right, minor_y);
        }
    }

    if(found) {
        canvas_set_font(canvas, FontSecondary);
        snprintf(text, sizeof(text), "%ld", (long)high);
        canvas_draw_str(canvas, 5, 8, text);
        snprintf(text, sizeof(text), "%ld", (long)low);
        canvas_draw_str(canvas, 5, 55, text);
    }

    canvas_set_font(canvas, FontSecondary);
    canvas_draw_str(canvas, 1, 63, metric_name[metric]);
    int32_t time_x = 64 - (int32_t)canvas_glyph_width(canvas, '0') * 2;
    canvas_draw_str_aligned(
        canvas, time_x, 63, AlignCenter, AlignBottom, co2_time_scale_name[scale]);
}

static void co2_draw_fault(Canvas* canvas, bool bmp_ok, bool scd_ok) {
    if(bmp_ok && scd_ok) return;

    const char* text = !bmp_ok && !scd_ok ? "SENSORS OFFLINE" :
                       !bmp_ok            ? "BMP388 OFFLINE" :
                                            "SCD30 OFFLINE";
    canvas_set_font(canvas, FontSecondary);
    uint16_t width = canvas_string_width(canvas, text) + 4;
    int32_t x = (128 - width) / 2;
    if(((furi_get_tick() / 500) & 1U) == 0) {
        canvas_set_color(canvas, ColorWhite);
        canvas_draw_box(canvas, x, 0, width, 10);
        canvas_set_color(canvas, ColorBlack);
        canvas_draw_str_aligned(canvas, 64, 8, AlignCenter, AlignBottom, text);
    } else {
        canvas_set_color(canvas, ColorBlack);
        canvas_draw_box(canvas, x, 0, width, 10);
        canvas_set_color(canvas, ColorWhite);
        canvas_draw_str_aligned(canvas, 64, 8, AlignCenter, AlignBottom, text);
        canvas_set_color(canvas, ColorBlack);
    }
}

static bool co2_record_blink_on(void) {
    /* A 2 second full cycle gives a 0.5 Hz REC blink. */
    return ((furi_get_tick() / 1000U) & 1U) == 0;
}

static void co2_draw_record_badge(Canvas* canvas) {
    if(!co2_record_blink_on()) return;

    canvas_set_font(canvas, FontSecondary);
    uint16_t width = canvas_string_width(canvas, "REC");
    int32_t x = 64 - (int32_t)width / 2;
    canvas_set_color(canvas, ColorWhite);
    canvas_draw_box(canvas, x - 1, 0, width + 2, 9);
    canvas_set_color(canvas, ColorBlack);
    canvas_draw_str_aligned(canvas, 64, 7, AlignCenter, AlignBottom, "REC");
}

static void co2_draw_battery_percent(Canvas* canvas, bool recording) {
    char text[6];
    snprintf(text, sizeof(text), "%u%%", (unsigned int)furi_hal_power_get_pct());
    canvas_set_font(canvas, FontSecondary);
    uint16_t width = canvas_string_width(canvas, text);
    int32_t text_left = 127 - width;
    canvas_set_color(canvas, ColorWhite);
    canvas_draw_box(canvas, 126 - width, 0, width + 2, 9);
    canvas_set_color(canvas, ColorBlack);
    canvas_draw_str_aligned(canvas, 127, 7, AlignRight, AlignBottom, text);

    if(recording && co2_record_blink_on()) {
        int32_t rec_right = text_left - canvas_glyph_width(canvas, ' ');
        uint16_t rec_width = canvas_string_width(canvas, "REC");
        canvas_set_color(canvas, ColorWhite);
        canvas_draw_box(canvas, rec_right - rec_width - 1, 0, rec_width + 2, 9);
        canvas_set_color(canvas, ColorBlack);
        canvas_draw_str_aligned(canvas, rec_right, 7, AlignRight, AlignBottom, "REC");
    }
}

static void co2_draw_record_page(
    Canvas* canvas,
    uint8_t interval,
    uint8_t duration,
    uint8_t cursor,
    bool editing,
    bool recording,
    bool error) {
    char value[24];

    canvas_set_font(canvas, FontPrimary);
    canvas_draw_str_aligned(canvas, 64, 9, AlignCenter, AlignBottom, "Data Recording");
    if(error) {
        canvas_set_font(canvas, FontSecondary);
        canvas_draw_str_aligned(canvas, 127, 8, AlignRight, AlignBottom, "SD!");
    }
    canvas_draw_line(canvas, 5, 12, 122, 12);

    canvas_set_font(canvas, FontSecondary);
    if(!recording && cursor == 0) canvas_draw_str(canvas, 1, 27, ">");
    canvas_draw_str(canvas, 9, 27, "Interval");
    if(!recording && editing && cursor == 0)
        snprintf(value, sizeof(value), "< %s >", co2_record_interval_name[interval]);
    else
        snprintf(value, sizeof(value), "%s", co2_record_interval_name[interval]);
    canvas_draw_str_aligned(canvas, 126, 27, AlignRight, AlignBottom, value);

    if(!recording && cursor == 1) canvas_draw_str(canvas, 1, 43, ">");
    canvas_draw_str(canvas, 9, 43, "Length");
    if(!recording && editing && cursor == 1)
        snprintf(value, sizeof(value), "< %s >", co2_record_duration_name[duration]);
    else
        snprintf(value, sizeof(value), "%s", co2_record_duration_name[duration]);
    canvas_draw_str_aligned(canvas, 126, 43, AlignRight, AlignBottom, value);

    if(cursor == 2) canvas_draw_str(canvas, 1, 60, ">");
    canvas_set_font(canvas, FontPrimary);
    if(recording) {
        uint16_t width = canvas_string_width(canvas, "Stop");
        canvas_draw_box(canvas, 68 - width / 2 - 3, 48, width + 6, 15);
        canvas_set_color(canvas, ColorWhite);
        canvas_draw_str_aligned(canvas, 68, 61, AlignCenter, AlignBottom, "Stop");
        canvas_set_color(canvas, ColorBlack);
    } else {
        canvas_draw_str_aligned(canvas, 68, 61, AlignCenter, AlignBottom, "Start");
    }
}

static void co2_draw_i2c_error(Canvas* canvas) {
    canvas_set_font(canvas, FontPrimary);
    canvas_draw_str_aligned(canvas, 64, 13, AlignCenter, AlignBottom, "I2C BUS ERROR");
    canvas_set_font(canvas, FontSecondary);
    canvas_draw_str_aligned(canvas, 64, 31, AlignCenter, AlignBottom, "Bus reset failed");
    canvas_draw_str_aligned(canvas, 64, 47, AlignCenter, AlignBottom, "Restart Flipper");
    canvas_draw_str_aligned(canvas, 64, 61, AlignCenter, AlignBottom, "to use sensors");
}

static void co2_draw_clear_confirm(Canvas* canvas) {
    canvas_set_font(canvas, FontPrimary);
    canvas_draw_str_aligned(canvas, 64, 19, AlignCenter, AlignBottom, "Clear all curves?");
    canvas_set_font(canvas, FontSecondary);
    canvas_draw_str_aligned(canvas, 64, 36, AlignCenter, AlignBottom, "All history will be erased");
    canvas_draw_str_aligned(canvas, 64, 57, AlignCenter, AlignBottom, "BACK: cancel   OK: clear");
}

static void co2_draw_exit_confirm(Canvas* canvas) {
    canvas_set_font(canvas, FontPrimary);
    canvas_draw_str_aligned(canvas, 64, 22, AlignCenter, AlignBottom, "Exit CO2-Sniffer?");
    canvas_set_font(canvas, FontSecondary);
    canvas_draw_str_aligned(canvas, 64, 55, AlignCenter, AlignBottom, "BACK: cancel   OK: exit");
}

static void co2_sniffer_draw_cb(Canvas* canvas, void* ctx) {
    Co2SnifferApp* app = ctx;
    bool clear_confirm;
    bool exit_confirm;
    bool home_done;
    bool bmp_ok;
    bool scd_ok;
    float t_bmp;
    float pressure;
    float altitude;
    float co2;
    float t_scd;
    float humidity;
    uint8_t page;
    uint8_t style;
    uint8_t record_interval;
    uint8_t record_duration;
    uint8_t record_cursor;
    bool record_editing;
    bool recording;
    bool record_error;
    bool i2c_fatal;

    furi_check(furi_mutex_acquire(app->mutex, FuriWaitForever) == FuriStatusOk);
    clear_confirm = app->clear_confirm;
    exit_confirm = app->exit_confirm;
    home_done = app->home_done;
    bmp_ok = app->bmp_ok;
    scd_ok = app->scd_ok;
    t_bmp = app->T_LIN;
    pressure = app->P_LIN;
    altitude = app->altitude;
    co2 = app->CO2_PPM;
    t_scd = app->T_SCD;
    humidity = app->R_SCD;
    page = app->page;
    style = app->style;
    record_interval = app->record_interval;
    record_duration = app->record_duration;
    record_cursor = app->record_cursor;
    record_editing = app->record_editing;
    recording = app->recording;
    record_error = app->record_error;
    i2c_fatal = app->i2c_fatal;
    furi_check(furi_mutex_release(app->mutex) == FuriStatusOk);

    canvas_clear(canvas);
    canvas_set_color(canvas, ColorBlack);
    if(exit_confirm) {
        co2_draw_exit_confirm(canvas);
        return;
    }
    if(clear_confirm) {
        co2_draw_clear_confirm(canvas);
        return;
    }
    if(i2c_fatal) {
        co2_draw_i2c_error(canvas);
        return;
    }
    if(!home_done) {
        co2_draw_home(canvas, app);
        return;
    }

    if(page == CO2_RECORD_PAGE) {
        co2_draw_record_page(
            canvas,
            record_interval,
            record_duration,
            record_cursor,
            record_editing,
            recording,
            record_error);
        return;
    } else if(page == 0) {
        if(style == 0)
            co2_draw_full_page(
                canvas, bmp_ok, scd_ok, t_bmp, pressure, altitude, co2, t_scd, humidity);
        else if(style == 1)
            co2_draw_co2_page(canvas, scd_ok, co2);
        else if(style == 2)
            co2_draw_temperature_page(canvas, bmp_ok, scd_ok, t_bmp, t_scd, humidity);
        else
            co2_draw_pressure_page(canvas, bmp_ok, pressure, altitude);
    } else {
        uint8_t metric = page - 1;
        bool valid = false;
        float current = 0.0f;
        if(metric == Co2MetricCo2) {
            valid = scd_ok;
            current = co2;
        } else if(metric == Co2MetricTemperature) {
            valid = scd_ok || bmp_ok;
            current = scd_ok ? t_scd : t_bmp;
        } else if(metric == Co2MetricHumidity) {
            valid = scd_ok;
            current = humidity;
        } else if(metric == Co2MetricPressure) {
            valid = bmp_ok;
            current = pressure / 100.0f;
        } else if(metric == Co2MetricAltitude) {
            valid = bmp_ok;
            current = altitude;
        }
        co2_draw_graph(canvas, app, metric, valid, current);
    }
    co2_draw_fault(canvas, bmp_ok, scd_ok);
    if(page == 0)
        co2_draw_battery_percent(canvas, recording);
    else if(recording)
        co2_draw_record_badge(canvas);
}

static void co2_clear_history_locked(Co2SnifferApp* app) {
    memset(app->history, 0, sizeof(app->history));
    memset(app->history_valid, 0, sizeof(app->history_valid));
    memset(app->history_head, 0, sizeof(app->history_head));
    memset(app->history_count, 0, sizeof(app->history_count));
    memset(app->graph_bounds_valid, 0, sizeof(app->graph_bounds_valid));
    app->history_reset_requested = true;
}

static void co2_sniffer_input_cb(InputEvent* event, void* ctx) {
    Co2SnifferApp* app = ctx;

    if(event->key == InputKeyBack && event->type == InputTypeLong) {
        furi_check(furi_mutex_acquire(app->mutex, FuriWaitForever) == FuriStatusOk);
        app->clear_confirm = false;
        app->exit_confirm = true;
        furi_check(furi_mutex_release(app->mutex) == FuriStatusOk);
        return;
    }
    if(event->key == InputKeyOk && event->type == InputTypeLong) {
        furi_check(furi_mutex_acquire(app->mutex, FuriWaitForever) == FuriStatusOk);
        bool exit_confirm = app->exit_confirm;
        bool enabled = app->backlight_always_on;
        if(exit_confirm) {
            app->exit_confirm = false;
            app->running = false;
        } else {
            app->backlight_always_on = !app->backlight_always_on;
            enabled = app->backlight_always_on;
        }
        furi_check(furi_mutex_release(app->mutex) == FuriStatusOk);
        if(exit_confirm) return;
        if(enabled)
            notification_message(app->notifications, &co2_backlight_enable_sequence);
        else
            notification_message(app->notifications, &co2_backlight_disable_sequence);
        return;
    }
    if(event->type != InputTypeShort && event->type != InputTypeRepeat) return;

    if(event->key == InputKeyBack && event->type == InputTypeShort) {
        furi_check(furi_mutex_acquire(app->mutex, FuriWaitForever) == FuriStatusOk);
        if(app->exit_confirm)
            app->exit_confirm = false;
        else if(app->i2c_fatal) {
            /* The fatal page is latched; long BACK still opens exit confirmation. */
        }
        else
            app->clear_confirm = !app->clear_confirm;
        furi_check(furi_mutex_release(app->mutex) == FuriStatusOk);
        return;
    }
    if(event->key == InputKeyOk && event->type == InputTypeShort) {
        furi_check(furi_mutex_acquire(app->mutex, FuriWaitForever) == FuriStatusOk);
        if(app->exit_confirm) {
            app->exit_confirm = false;
            app->running = false;
        } else if(app->clear_confirm) {
            co2_clear_history_locked(app);
            app->clear_confirm = false;
        } else if(app->home_done && app->page == CO2_RECORD_PAGE) {
            if(app->recording) {
                app->record_stop_requested = true;
            } else if(app->record_cursor == 2) {
                app->record_start_requested = true;
                app->record_error = false;
                app->record_editing = false;
            } else {
                app->record_editing = !app->record_editing;
            }
        }
        furi_check(furi_mutex_release(app->mutex) == FuriStatusOk);
        return;
    }

    furi_check(furi_mutex_acquire(app->mutex, FuriWaitForever) == FuriStatusOk);
    if(app->home_done && !app->clear_confirm && !app->exit_confirm && !app->i2c_fatal) {
        if(app->page == CO2_RECORD_PAGE) {
            if(event->key == InputKeyRight) {
                if(app->record_editing && !app->recording) {
                    if(app->record_cursor == 0)
                        app->record_interval =
                            (app->record_interval + 1) % CO2_RECORD_INTERVAL_COUNT;
                    else if(app->record_cursor == 1)
                        app->record_duration =
                            (app->record_duration + 1) % CO2_RECORD_DURATION_COUNT;
                } else {
                    app->page = 0;
                }
            } else if(event->key == InputKeyLeft && app->record_editing && !app->recording) {
                if(app->record_cursor == 0)
                    app->record_interval =
                        (app->record_interval + CO2_RECORD_INTERVAL_COUNT - 1) %
                        CO2_RECORD_INTERVAL_COUNT;
                else if(app->record_cursor == 1)
                    app->record_duration =
                        (app->record_duration + CO2_RECORD_DURATION_COUNT - 1) %
                        CO2_RECORD_DURATION_COUNT;
            } else if(event->key == InputKeyUp && !app->record_editing && !app->recording) {
                app->record_cursor = (app->record_cursor + 2) % 3;
            } else if(event->key == InputKeyDown && !app->record_editing && !app->recording) {
                app->record_cursor = (app->record_cursor + 1) % 3;
            }
        } else if(event->key == InputKeyRight) {
            app->page = (app->page + 1) % CO2_DATA_PAGE_COUNT;
        } else if(event->key == InputKeyLeft) {
            if(app->page == 0)
                app->page = CO2_RECORD_PAGE;
            else
                app->page--;
        } else if(event->key == InputKeyUp) {
            if(app->page == 0)
                app->style = (app->style + CO2_STYLE_COUNT - 1) % CO2_STYLE_COUNT;
            else
                app->time_scale =
                    (app->time_scale + CO2_TIME_SCALE_COUNT - 1) % CO2_TIME_SCALE_COUNT;
        } else if(event->key == InputKeyDown) {
            if(app->page == 0)
                app->style = (app->style + 1) % CO2_STYLE_COUNT;
            else
                app->time_scale = (app->time_scale + 1) % CO2_TIME_SCALE_COUNT;
        }
    }
    furi_check(furi_mutex_release(app->mutex) == FuriStatusOk);
}

static void co2_frame_put16(uint8_t* p, uint16_t value) {
    p[0] = (uint8_t)(value >> 8);
    p[1] = (uint8_t)value;
}

static void co2_frame_put32(uint8_t* p, uint32_t value) {
    p[0] = (uint8_t)(value >> 24);
    p[1] = (uint8_t)(value >> 16);
    p[2] = (uint8_t)(value >> 8);
    p[3] = (uint8_t)value;
}

static uint16_t co2_frame_crc16(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFF;
    for(size_t i = 0; i < length; i++) {
        crc ^= data[i];
        for(uint8_t bit = 0; bit < 8; bit++) {
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
        }
    }
    return crc;
}

static void co2_history_append(
    Co2SnifferApp* app,
    uint8_t scale,
    const float* values,
    uint8_t valid_mask) {
    uint8_t target = app->history_head[scale];
    for(uint8_t metric = 0; metric < CO2_METRIC_COUNT; metric++) {
        app->history[scale][metric][target] = values[metric];
    }
    app->history_valid[scale][target] = valid_mask;
    app->history_head[scale] = (target + 1) % CO2_HISTORY_POINTS;
    if(app->history_count[scale] < CO2_HISTORY_POINTS) app->history_count[scale]++;
}

static void co2_update_home_state(Co2SnifferApp* app, uint32_t now) {
    if(app->home_done) return;

    uint8_t category = app->bmp_ok && app->scd_ok ? 2 : (app->bmp_ok || app->scd_ok ? 1 : 0);
    if(category == 0) {
        app->home_category = 0;
        app->home_deadline_ms = 0;
    } else if(category != app->home_category) {
        app->home_category = category;
        app->home_deadline_ms = now + (category == 2 ? 5000 : 10000);
    } else if(co2_time_reached(now, app->home_deadline_ms)) {
        app->home_done = true;
    }
}

static void co2_record_close(File* file) {
    if(storage_file_is_open(file)) {
        storage_file_sync(file);
        storage_file_close(file);
    }
}

static bool co2_record_open(
    Storage* storage,
    File* file,
    uint8_t interval,
    uint8_t duration) {
    FS_Error mkdir_error = storage_common_mkdir(storage, CO2_RECORD_DIRECTORY);
    if(mkdir_error != FSE_OK && mkdir_error != FSE_EXIST) return false;

    DateTime datetime;
    furi_hal_rtc_get_datetime(&datetime);
    char path[128];
    bool opened = false;
    for(uint8_t suffix = 0; suffix < 100; suffix++) {
        if(suffix == 0) {
            snprintf(
                path,
                sizeof(path),
                CO2_RECORD_DIRECTORY "/%04u%02u%02u_%02u%02u%02u_%s_%s.csv",
                datetime.year,
                datetime.month,
                datetime.day,
                datetime.hour,
                datetime.minute,
                datetime.second,
                co2_record_interval_name[interval],
                co2_record_duration_name[duration]);
        } else {
            snprintf(
                path,
                sizeof(path),
                CO2_RECORD_DIRECTORY "/%04u%02u%02u_%02u%02u%02u_%s_%s_%02u.csv",
                datetime.year,
                datetime.month,
                datetime.day,
                datetime.hour,
                datetime.minute,
                datetime.second,
                co2_record_interval_name[interval],
                co2_record_duration_name[duration],
                suffix);
        }
        if(storage_file_open(file, path, FSAM_WRITE, FSOM_CREATE_NEW)) {
            opened = true;
            break;
        }
        FS_Error open_error = storage_file_get_error(file);
        storage_file_close(file);
        if(open_error != FSE_EXIST) return false;
    }
    if(!opened) return false;

    static const char header[] =
        "timestamp,elapsed_s,co2_ppm,scd30_temperature_c,humidity_pct,"
        "bmp388_temperature_c,pressure_hpa,altitude_m\n";
    if(storage_file_write(file, header, sizeof(header) - 1) != sizeof(header) - 1 ||
       !storage_file_sync(file)) {
        co2_record_close(file);
        return false;
    }
    return true;
}

static bool co2_record_write(
    File* file,
    uint32_t elapsed_ms,
    bool bmp_valid,
    bool scd_valid,
    float bmp_temperature,
    float pressure_pa,
    float altitude,
    float co2,
    float scd_temperature,
    float humidity) {
    DateTime datetime;
    furi_hal_rtc_get_datetime(&datetime);

    char co2_text[16] = "";
    char scd_temperature_text[16] = "";
    char humidity_text[16] = "";
    char bmp_temperature_text[16] = "";
    char pressure_text[16] = "";
    char altitude_text[16] = "";
    if(scd_valid) {
        snprintf(co2_text, sizeof(co2_text), "%.0f", (double)co2);
        snprintf(scd_temperature_text, sizeof(scd_temperature_text), "%.2f", (double)scd_temperature);
        snprintf(humidity_text, sizeof(humidity_text), "%.2f", (double)humidity);
    }
    if(bmp_valid) {
        snprintf(bmp_temperature_text, sizeof(bmp_temperature_text), "%.2f", (double)bmp_temperature);
        snprintf(pressure_text, sizeof(pressure_text), "%.2f", (double)(pressure_pa / 100.0f));
        snprintf(altitude_text, sizeof(altitude_text), "%.1f", (double)altitude);
    }

    char line[192];
    int length = snprintf(
        line,
        sizeof(line),
        "%04u-%02u-%02u %02u:%02u:%02u,%lu,%s,%s,%s,%s,%s,%s\n",
        datetime.year,
        datetime.month,
        datetime.day,
        datetime.hour,
        datetime.minute,
        datetime.second,
        (unsigned long)(elapsed_ms / 1000U),
        co2_text,
        scd_temperature_text,
        humidity_text,
        bmp_temperature_text,
        pressure_text,
        altitude_text);
    if(length <= 0 || (size_t)length >= sizeof(line)) return false;
    return storage_file_write(file, line, (size_t)length) == (size_t)length &&
           storage_file_sync(file);
}

static void co2_record_set_state(Co2SnifferApp* app, bool recording, bool error) {
    furi_check(furi_mutex_acquire(app->mutex, FuriWaitForever) == FuriStatusOk);
    app->recording = recording;
    app->record_error = error;
    if(recording) {
        app->record_cursor = 2;
        app->record_editing = false;
    }
    furi_check(furi_mutex_release(app->mutex) == FuriStatusOk);
}

static int32_t co2_sniffer_worker(void* ctx) {
    Co2SnifferApp* app = ctx;
    uint8_t frame[CO2_FRAME_SIZE];
    uint16_t scd_press_bak = 0;
    uint32_t frames_sent = 0;
    uint32_t bmp_warmup_until = 0;
    uint32_t scd_warmup_until = 0;
    uint32_t next_history[CO2_TIME_SCALE_COUNT];
    float carried[CO2_METRIC_COUNT] = {0};
    uint8_t carried_valid = 0;
    bool bmp_has_measurement = false;
    bool scd_has_measurement = false;
    bool history_started = false;
    bool altitude_filter_valid = false;
    float altitude_filtered = 0.0f;
    Storage* storage = furi_record_open(RECORD_STORAGE);
    File* record_file = storage_file_alloc(storage);
    bool file_recording = false;
    uint8_t active_record_interval = 0;
    uint8_t active_record_duration = 0;
    uint32_t record_started_at = 0;
    uint32_t next_record_at = 0;
    uint32_t record_stops_at = 0;
    bool i2c_fatal = false;

    uint32_t now = furi_get_tick();
    for(uint8_t scale = 0; scale < CO2_TIME_SCALE_COUNT; scale++) {
        next_history[scale] = now + co2_time_scale_ms[scale];
    }

    app->bmp.fail = true;
    app->scd.fail = true;
    co2_i2c_acquire();
    co2_usb_stream_start();

    while(app->running) {
        now = furi_get_tick();

        if(app->bmp.fail) {
            if(co2_bmp388_init(&app->bmp)) {
                /* Do not blend a post-reconnect pressure with the old sensor
                 * session before feeding SCD30 compensation. */
                app->bmp.P_FIL = 0.0f;
                bmp_warmup_until = furi_get_tick() + CO2_BMP_WARMUP_MS;
                bmp_has_measurement = false;
                altitude_filter_valid = false;
            }
        } else if(co2_bmp388_read(&app->bmp)) {
            bmp_has_measurement = true;
        } else {
            bmp_has_measurement = false;
        }

        if(!app->bmp.fail && bmp_has_measurement) {
            float altitude_raw = co2_pressure_to_altitude(app->bmp.P_LIN);
            if(!altitude_filter_valid) {
                altitude_filtered = altitude_raw;
                altitude_filter_valid = true;
            } else {
                altitude_filtered = altitude_filtered * (1.0f - CO2_ALTITUDE_FILTER_ALPHA) +
                                    altitude_raw * CO2_ALTITUDE_FILTER_ALPHA;
            }
        }

        if(app->scd.fail) {
            /* Passing zero explicitly disables pressure compensation when the
             * BMP388 is absent. */
            uint16_t pressure_mbar = 0;
            if(!app->bmp.fail && bmp_has_measurement) {
                uint16_t candidate = (uint16_t)((app->bmp.P_FIL + 0.5f) / 100.0f);
                if(candidate >= 700 && candidate <= 1200) pressure_mbar = candidate;
            }
            if(co2_scd30_init(&app->scd, pressure_mbar)) {
                scd_press_bak = pressure_mbar;
                scd_warmup_until = furi_get_tick() + CO2_SCD_WARMUP_MS;
                scd_has_measurement = false;
            }
        } else {
            if(co2_scd30_tick(&app->scd)) scd_has_measurement = true;
            if(app->scd.fail) scd_has_measurement = false;

            uint16_t pressure_mbar = (uint16_t)((app->bmp.P_FIL + 0.5f) / 100.0f);
            if(!app->scd.fail && !app->bmp.fail && bmp_has_measurement && pressure_mbar >= 700 &&
               pressure_mbar <= 1200 && pressure_mbar != scd_press_bak) {
                scd_press_bak = pressure_mbar;
                if(!co2_scd30_set_pressure(pressure_mbar)) {
                    app->scd.fail = true;
                    scd_has_measurement = false;
                }
            }
        }

        if(!co2_i2c_recover()) {
            i2c_fatal = true;
            furi_check(furi_mutex_acquire(app->mutex, FuriWaitForever) == FuriStatusOk);
            app->i2c_fatal = true;
            app->clear_confirm = false;
            app->recording = false;
            app->record_start_requested = false;
            app->record_stop_requested = false;
            furi_check(furi_mutex_release(app->mutex) == FuriStatusOk);
            break;
        }

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
        if(co2_usb_stream_send(frame, CO2_FRAME_SIZE)) frames_sent++;

        now = furi_get_tick();
        bool bmp_usable = !app->bmp.fail && bmp_has_measurement &&
                          co2_time_reached(now, bmp_warmup_until);
        bool scd_usable = !app->scd.fail && scd_has_measurement &&
                          co2_time_reached(now, scd_warmup_until);
        if(scd_usable) {
            carried[Co2MetricCo2] = app->scd.CO2_PPM;
            carried[Co2MetricTemperature] = app->scd.T_SCD;
            carried[Co2MetricHumidity] = app->scd.R_SCD;
            carried_valid |= (1U << Co2MetricCo2) | (1U << Co2MetricTemperature) |
                             (1U << Co2MetricHumidity);
        } else if(bmp_usable) {
            carried[Co2MetricTemperature] = app->bmp.T_LIN;
            carried_valid |= 1U << Co2MetricTemperature;
        }
        if(bmp_usable) {
            carried[Co2MetricPressure] = app->bmp.P_LIN / 100.0f;
            carried[Co2MetricAltitude] = altitude_filtered;
            carried_valid |= (1U << Co2MetricPressure) | (1U << Co2MetricAltitude);
        }

        furi_check(furi_mutex_acquire(app->mutex, FuriWaitForever) == FuriStatusOk);
        app->bmp_ok = !app->bmp.fail;
        app->scd_ok = !app->scd.fail;
        app->T_LIN = app->bmp.T_LIN;
        app->P_LIN = app->bmp.P_LIN;
        app->altitude = altitude_filtered;
        app->CO2_PPM = app->scd.CO2_PPM;
        app->T_SCD = app->scd.T_SCD;
        app->R_SCD = app->scd.R_SCD;
        app->usb_active = co2_usb_stream_active();
        app->host_open = co2_usb_stream_host_open();
        app->frames_sent = frames_sent;
        co2_update_home_state(app, now);

        /* Start all metric timelines together, after every sensor shown as
         * online has completed its discard/warm-up interval. */
        bool history_ready = app->home_done && (app->bmp_ok || app->scd_ok) &&
                             (!app->bmp_ok || bmp_usable) && (!app->scd_ok || scd_usable);
        if(!history_started && history_ready) {
            history_started = true;
            for(uint8_t scale = 0; scale < CO2_TIME_SCALE_COUNT; scale++) {
                next_history[scale] = now + co2_time_scale_ms[scale];
            }
        }

        if(app->history_reset_requested) {
            app->history_reset_requested = false;
            for(uint8_t scale = 0; scale < CO2_TIME_SCALE_COUNT; scale++) {
                next_history[scale] = now + co2_time_scale_ms[scale];
            }
        }

        if(history_started) {
            for(uint8_t scale = 0; scale < CO2_TIME_SCALE_COUNT; scale++) {
                if(co2_time_reached(now, next_history[scale])) {
                    co2_history_append(app, scale, carried, carried_valid);
                    next_history[scale] = now + co2_time_scale_ms[scale];
                }
            }
        }
        bool start_record = app->record_start_requested;
        bool stop_record = app->record_stop_requested;
        uint8_t requested_interval = app->record_interval;
        uint8_t requested_duration = app->record_duration;
        app->record_start_requested = false;
        app->record_stop_requested = false;
        furi_check(furi_mutex_release(app->mutex) == FuriStatusOk);

        if(stop_record && file_recording) {
            co2_record_close(record_file);
            file_recording = false;
            co2_record_set_state(app, false, false);
        }

        if(start_record && !file_recording) {
            if(co2_record_open(storage, record_file, requested_interval, requested_duration)) {
                file_recording = true;
                active_record_interval = requested_interval;
                active_record_duration = requested_duration;
                record_started_at = now;
                next_record_at = now;
                if(active_record_duration != CO2_RECORD_CONTINUOUS) {
                    record_stops_at = now + co2_record_duration_ms[active_record_duration];
                }
                co2_record_set_state(app, true, false);
            } else {
                co2_record_set_state(app, false, true);
            }
        }

        if(file_recording && active_record_duration != CO2_RECORD_CONTINUOUS &&
           co2_time_reached(now, record_stops_at)) {
            co2_record_close(record_file);
            file_recording = false;
            co2_record_set_state(app, false, false);
        }

        if(file_recording && co2_time_reached(now, next_record_at)) {
            bool write_ok = co2_record_write(
                record_file,
                now - record_started_at,
                bmp_usable,
                scd_usable,
                app->bmp.T_LIN,
                app->bmp.P_LIN,
                altitude_filtered,
                app->scd.CO2_PPM,
                app->scd.T_SCD,
                app->scd.R_SCD);
            if(write_ok) {
                next_record_at = now + co2_record_interval_ms[active_record_interval];
            } else {
                co2_record_close(record_file);
                file_recording = false;
                co2_record_set_state(app, false, true);
            }
        }

        furi_delay_ms(CO2_TICK_MS);
    }

    if(!i2c_fatal) co2_scd30_stop();
    co2_usb_stream_stop();
    co2_i2c_release();
    co2_record_close(record_file);
    storage_file_free(record_file);
    furi_record_close(RECORD_STORAGE);
    return 0;
}

int32_t co2_sniffer_app(void* p) {
    UNUSED(p);

    Co2SnifferApp* app = malloc(sizeof(Co2SnifferApp));
    memset(app, 0, sizeof(*app));
    app->gui = furi_record_open(RECORD_GUI);
    app->notifications = furi_record_open(RECORD_NOTIFICATION);
    app->mutex = furi_mutex_alloc(FuriMutexTypeNormal);
    app->view_port = view_port_alloc();
    app->running = true;
    app->bmp.fail = true;
    app->scd.fail = true;
    app->home_category = 0xFF;

    view_port_draw_callback_set(app->view_port, co2_sniffer_draw_cb, app);
    view_port_input_callback_set(app->view_port, co2_sniffer_input_cb, app);
    gui_add_view_port(app->gui, app->view_port, GuiLayerFullscreen);

    app->worker = furi_thread_alloc_ex("Co2Sniffer", 4096, co2_sniffer_worker, app);
    furi_thread_start(app->worker);

    while(app->running) {
        view_port_update(app->view_port);
        furi_delay_ms(100);
    }

    furi_thread_join(app->worker);
    furi_thread_free(app->worker);
    if(app->backlight_always_on) {
        notification_message_block(app->notifications, &sequence_display_backlight_enforce_auto);
    }
    view_port_enabled_set(app->view_port, false);
    gui_remove_view_port(app->gui, app->view_port);
    view_port_free(app->view_port);
    furi_record_close(RECORD_GUI);
    furi_record_close(RECORD_NOTIFICATION);
    furi_mutex_free(app->mutex);
    free(app);
    return 0;
}
