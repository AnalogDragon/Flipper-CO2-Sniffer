[简体中文](PROTOCOL-zh.md)

# CO2 Sniffer — USB CDC Serial Protocol

The Flipper app (`SW/co2_sniffer.c`) reads the SCD30 (CO2, temperature, humidity)
and BMP388 (temperature, pressure) sensors and streams a framed binary report to a
PC over USB CDC (virtual COM port). This document describes that stream.

## 1. Transport

- **USB mode:** the app switches the Flipper to the dual CDC configuration
  (`usb_cdc_dual`) and transmits on **interface 1** — on the PC this is the
  **second** virtual COM port. The first COM port is the Flipper CLI console.
- **Baud rate:** irrelevant. CDC is a virtual serial port; the Flipper sends
  at full USB speed and the host-side baud setting is ignored.
- **Flow control:** none. `furi_hal_cdc_send` is non-blocking — if the endpoint
  is busy (host not reading fast enough) the frame is **silently dropped**.
  There is no retry, no buffering beyond the USB endpoint.
- **Host-open detection:** the app treats the port as "open" when DTR is
  asserted (shown as `USB:TX` on the screen; `USB:wait` means no host).
- **Rate:** one frame every **333 ms** (`CO2_TICK_MS`), continuously, for as
  long as the app runs.
- **On exit** the app restores the USB mode that was active before it started.

## 2. Frame format

Every frame is **19 bytes**, big-endian payload, CRC appended little-endian:

| Offset | Size | Field | Encoding |
| ------ | ---- | ----- | -------- |
| 0 | 4 | Header | `DE AD FF 09` (CH552 VID `0xDEAD` + PID `0xFF09`) |
| 4 | 1 | Payload length | Always `0x0C` (12) |
| 5 | 2 | CO2 (SCD30) | uint16, **ppm**, big-endian |
| 7 | 2 | Temperature (SCD30) | int16, **°C × 100**, big-endian |
| 9 | 2 | Humidity (SCD30) | uint16, **%RH × 100**, big-endian |
| 11 | 2 | Temperature (BMP388) | int16, **°C × 100**, big-endian |
| 13 | 4 | Pressure (BMP388) | uint32, **Pa**, big-endian |
| 17 | 2 | CRC16 | Modbus CRC over bytes 0..16, **little-endian** |

Total: 4 + 1 + 12 + 2 = 19 bytes.

Decoding:

```
co2_ppm     = uint16_be(payload[0:2])
t_scd_c     = int16_be(payload[2:4])  / 100.0
rh_scd_pct  = uint16_be(payload[4:6]) / 100.0
t_bmp_c     = int16_be(payload[6:8])  / 100.0
p_bmp_pa    = uint32_be(payload[8:12])
```

### Sensor failure sentinels

The app re-initializes a failing sensor on every tick and keeps streaming
regardless. A failed sensor reports its fields as all-`0xFF`:

- SCD30 failed → bytes 5..10 = `FF FF FF FF FF FF`
  (CO2 `0xFFFF`, temperature `0xFFFF`, humidity `0xFFFF`)
- BMP388 failed → bytes 11..16 = `FF FF FF FF FF FF`
  (temperature `0xFFFF`, pressure `0xFFFFFFFF`)

Treat these as **"sensor error"**, not as values: `0xFFFF` temperature would
otherwise decode as −0.01 °C, `0xFFFF` humidity as 655.35 %RH.

## 3. CRC16

Modbus CRC16 over the first 17 bytes (header + length + payload):

- Polynomial: `0xA001` (reflected `0x8005`)
- Initial value: `0xFFFF`, no final XOR
- LSB-first (reflected)
- Stored little-endian (low byte at offset 17, high byte at offset 18)

Reference implementations:

```c
/* Modbus CRC16: poly 0xA001, init 0xFFFF, LSB first */
uint16_t crc16_modbus(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for(size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for(uint8_t b = 0; b < 8; b++)
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
    }
    return crc;
}
```

```python
def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc
```

Check: `crc16_modbus(frame[:17])` must equal `frame[17] | frame[18] << 8`, and
`crc16_modbus(frame) == 0` for a complete valid frame.

## 4. Example frame

| Value | Encoding |
| ----- | -------- |
| CO2 = 512 ppm | `02 00` |
| SCD30 T = 24.55 °C | `09 97` |
| SCD30 RH = 43.20 %RH | `10 E0` |
| BMP388 T = 24.62 °C | `09 9E` |
| BMP388 P = 100123 Pa | `00 01 87 1B` |

```
DE AD FF 09 0C 02 00 09 97 10 E0 09 9E 00 01 87 1B CA E0
└ header ┘ └len┘ └────────── payload (12) ──────────┘ └ CRC16 LE
```

CRC16 over bytes 0..16 = `0xE0CA`, stored little-endian as `CA E0`.

## 5. Parsing the stream

The stream is a continuous byte pipe with no idle marker. To synchronize:

1. Scan for the header `DE AD FF 09`.
2. Take the byte after the header as length; expect `0x0C`.
3. Take `1 + len + 2` more bytes and verify the CRC over `header + length + payload`
   (`crc16_modbus(frame[:17]) == crc_le`). Reject and rescan on mismatch.
4. An alternative is to chunk by 19 bytes once aligned — a valid CRC per chunk
   keeps the alignment honest.

Note that frames can be dropped by the Flipper when the host is not reading, so
a parser must never assume uninterrupted sequence numbers (there are none).

## 6. Sampling and update rates

The frame cadence (333 ms) is **faster** than the sensor update rates:

- **SCD30:** measurement interval is set to **2 s** at init, so CO2, SCD30
  temperature and humidity values change every ~2 s (every 6th frame) and
  repeat in between. ASC (automatic self-calibration) is disabled at init.
- **BMP388:** configured for 50 Hz output (OSR ×32/×32, normal mode) and is
  read on every 333 ms tick; its temperature/pressure are fresh in every frame.
- **Pressure compensation:** the app feeds the SCD30 an ambient pressure
  (mbar) derived from the BMP388 pressure low-pass filtered with
  `P_FIL = 0.95·P_FIL + 0.05·P_LIN`. When the filtered pressure changes and is
  within 700..1200 mbar, the SCD30 measurement is restarted with the new
  pressure. Rounding is `(uint16_t)((P_FIL + 0.5) / 100)`.
- **Re-init:** pressing **OK** on the Flipper re-initializes both sensors
  (soft reset; the original CH552 board power-cycled them instead).

## 7. History note

The original CH552 sniffer board streamed the same report layout as an HID
report. FAPs cannot register custom HID descriptors, so this port transmits
the identical frame over CDC. The layout is byte-compatible with the CH552 HID
report, which is why the header encodes the CH552's VID/PID.
