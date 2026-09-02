[简体中文](README-zh.md)

# CO2 Sniffer

BMP388 + SCD30 add-on board and app for the Flipper Zero. The board plugs into the
Flipper's GPIO pins; the app reads both sensors over the external I2C bus, shows the
values on screen, and streams a framed report to a PC over USB CDC.

## Current features

- Reads the SCD30 (CO2, temperature, humidity) and the BMP388 (temperature,
  pressure) over the Flipper's external I2C bus (C0 = SCL, C1 = SDA).
- Live screen: CO2 ppm, BMP388 temperature and pressure, SCD30 temperature and
  humidity, per-sensor OK/ERR status, USB state and frame counter.
- Streams a 19-byte binary frame to a PC over USB CDC every 333 ms (see
  [PROTOCOL.md](PROTOCOL.md)).
- A failing sensor is re-initialized automatically on every tick; the stream
  keeps running and reports `0xFF…` sentinel values for the failed sensor.
- **OK** re-initializes both sensors, **BACK** exits.
- The USB mode that was active before the app started is restored on exit.

## Hardware

`HW/` is a KiCad project. The board is a 2.54 mm 1×10 pin header module that
plugs into the Flipper's GPIO port:

| Device | Role | I2C address |
| ------ | ---- | ----------- |
| Sensirion SCD30 | CO2, temperature, humidity | 0x61 |
| Bosch BMP388 | temperature, pressure | 0x76 |

I2C runs on GPIO C0 (SCL) / C1 (SDA); the board is powered from the Flipper's
3V3/5V pins. `HW/Flipper-CO2-module-V10/` contains the V1.0 gerbers, ready for
fabrication.

## Software

`SW/` is a Flipper application (FAP, appid `co2_sniffer`, category DIY, v0.2).
The worker thread runs the following:

- **SCD30** — soft reset, ASC (automatic self-calibration) disabled, 2 s
  measurement interval, continuous measurement. Ambient-pressure compensation
  is fed from the BMP388: when the filtered pressure moves within the accepted
  700..1200 mbar range, the SCD30 measurement is restarted with the new value.
- **BMP388** — normal mode, 50 Hz ODR, OSR ×32/×32, NVM calibration.
- **USB** — switches to `usb_cdc_dual` and streams on interface 1, which is the
  **second** virtual COM port on the PC (the first is the Flipper CLI). Frames
  handed to the USB layer are counted on screen; a busy endpoint (host not
  reading) may still drop them silently.

## Layout

- `HW/` — KiCad board: schematic, PCB, V1.0 gerbers in `Flipper-CO2-module-V10/`.
- `SW/` — Flipper app sources; `SW/dist/co2_sniffer.fap` is a prebuilt FAP.
- `PROTOCOL.md` — full USB CDC serial protocol documentation.
- `README-zh.md` — this document in Chinese.

## License

GPL-3.0, see [LICENSE](LICENSE).

## Build & install

```sh
ufbt            # builds SW/co2_sniffer.fap
ufbt launch     # installs and runs on a connected Flipper
```

The app installs under `/ext/apps/DIY/` (FAP category DIY).

## Serial frame

Full protocol documentation (transport, CRC reference code, parsing guidance):
**[PROTOCOL.md](PROTOCOL.md)**.

Streamed over USB CDC every 333 ms, 19 bytes, big-endian:

| Offset | Size | Field |
| ------ | ---- | ----- |
| 0 | 4 | Header `DE AD FF 09` (CH552 VID 0xDEAD + PID 0xFF09) |
| 4 | 1 | Payload length (12) |
| 5 | 2 | CO2, uint16 ppm |
| 7 | 2 | SCD30 temperature, int16 °C x100 |
| 9 | 2 | SCD30 humidity, uint16 %RH x100 |
| 11 | 2 | BMP388 temperature, int16 °C x100 |
| 13 | 4 | BMP388 pressure, uint32 Pa |
| 17 | 2 | CRC16 (Modbus: poly 0xA001, init 0xFFFF, reflected, little-endian) over bytes 0..16 |

A failed sensor reports `0xFFFF` / `0xFFFFFFFF` for its values.

On the PC the stream appears on the **second** virtual COM port; the first one is the
Flipper CLI console.
