#include "co2_sniffer_usb.h"

#include <furi_hal_usb.h>
#include <furi_hal_usb_cdc.h>

/* CDC interface to stream on. We use usb_cdc_dual + interface 1:
 * - on DEBUG (dev) firmware, usb_cdc_single's interface 0 is claimed by the
 *   system debug console (same trick as usb_uart_bridge); the dual config
 *   keeps interface 1 free for the app,
 * - on RELEASE firmware, dual's interface 0 hosts the CLI and interface 1 is
 *   unused, so the same choice works everywhere. */
#define CO2_USB_IFACE 1

static FuriHalUsbInterface* co2_usb_prev;
static bool co2_usb_active;

void co2_usb_stream_start(void) {
    co2_usb_prev = furi_hal_usb_get_config();
    co2_usb_active = false;

    /* USB mode switching can be locked by the system; try anyway and fall
     * back to "no stream" if it fails (sensors keep working). */
    furi_hal_usb_unlock();
    if(furi_hal_usb_set_config(&usb_cdc_dual, NULL)) {
        co2_usb_active = true;
    }
}

void co2_usb_stream_stop(void) {
    if(co2_usb_active && co2_usb_prev) {
        furi_hal_usb_set_config(co2_usb_prev, NULL);
    }
    co2_usb_active = false;
}

bool co2_usb_stream_send(const uint8_t* data, size_t len) {
    if(!co2_usb_active) return false;

    /* Verified by disassembly: furi_hal_cdc_send goes straight to
     * usbd_ep_write (if 1 -> EP 0x85), which drops the frame when the
     * endpoint is busy (host not reading). It never blocks. */
    furi_hal_cdc_send(CO2_USB_IFACE, (uint8_t*)data, len);
    return true;
}

bool co2_usb_stream_host_open(void) {
    if(!co2_usb_active) return false;
    return (furi_hal_cdc_get_ctrl_line_state(CO2_USB_IFACE) & CdcCtrlLineDTR) != 0;
}

bool co2_usb_stream_active(void) {
    return co2_usb_active;
}
