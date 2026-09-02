[简体中文](README-zh.md)

# CO2 Sniffer

BMP388 + SCD30 add-on board and app for the Flipper Zero. The board plugs into the
Flipper's GPIO pins; the app reads both sensors over the external I2C bus, shows the
values on screen, and streams a framed report to a PC over USB CDC.

## Current features

- Reads CO2, temperature and humidity from the SCD30, and temperature and
  pressure from the BMP388.
- Shows live readings in several dashboard layouts, including an estimated
  altitude calculated from pressure.
- Provides history graphs for CO2, temperature, humidity, pressure and altitude
  with selectable time scales.
- Streams sensor readings to a PC over USB CDC for logging or further analysis.
- Detects sensor faults, retries automatically and continues working when only
  one sensor is available.

## Controls

| Input | Action |
| ----- | ------ |
| **LEFT / RIGHT** | Switch between the dashboard and five history graphs |
| **UP / DOWN** | Change the dashboard layout or graph time scale |
| Short **BACK / OK** | BACK opens or cancels history clearing; OK confirms it |
| Long **OK** | Toggle always-on backlight |
| Long **BACK** | Exit the app |

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

`SW/` is a Flipper application (FAP, appid `co2_sniffer`, category DIY, v0.3).
Its main flow is:

1. Detect and initialize the SCD30 and BMP388.
2. Read the available sensors and update the live dashboard.
3. Record the readings for the history graph pages.
4. Send the same readings to the PC through USB CDC.
5. Retry disconnected sensors automatically and restore system settings on exit.

## Layout

- `HW/` — KiCad board: schematic, PCB, V1.0 gerbers in `Flipper-CO2-module-V10/`.
- `SW/` — Flipper app sources; builds are written under `SW/dist/`.
- `PROTOCOL.md` — full USB CDC serial protocol documentation.
- `README-zh.md` — this document in Chinese.

## License

GPL-3.0, see [LICENSE](LICENSE).

## Build & install

```sh
cd SW
ufbt            # builds dist/co2_sniffer.fap
ufbt launch     # installs and runs on a connected Flipper
```

The app installs under `/ext/apps/DIY/` (FAP category DIY).

## Serial frame

The app sends a compact binary report over USB CDC. On the PC, use the second
virtual COM port; the first one is the Flipper CLI console. See
**[PROTOCOL.md](PROTOCOL.md)** for the complete frame format and parsing details.
