#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/**
 * Stream the 64B report over the Flipper's USB as a CDC virtual COM port.
 *
 * The CH552 sniffer streamed the same report as an HID device; FAPs cannot
 * register custom HID descriptors, so CDC is the transport here. The report
 * layout is byte-compatible with the CH552 HID report.
 */

/** Switch the Flipper to CDC single (virtual COM port) mode. */
void co2_usb_stream_start(void);

/** Restore the USB mode that was active before start(). */
void co2_usb_stream_stop(void);

/** Send one report frame over the virtual COM port. The frame is dropped
 *  silently while the endpoint is busy (host not reading) - never blocks. */
bool co2_usb_stream_send(const uint8_t* data, size_t len);

/** True when a PC has opened the COM port (DTR asserted). */
bool co2_usb_stream_host_open(void);

/** True when the CDC stream is active (mode switch succeeded). */
bool co2_usb_stream_active(void);
