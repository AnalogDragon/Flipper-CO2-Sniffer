[简体中文](README-zh.md)

# CO2 Sniffer

BMP388 + SCD30 add-on board and app for the Flipper Zero. The board plugs into the
Flipper's GPIO pins; the app reads both sensors over the external I2C bus, shows the
values on screen, and streams a framed report to a PC over USB CDC.
The software project in `SW/` was developed by OpenAI Codex.

## Current features

- Reads CO2, temperature and humidity from the SCD30, and temperature and
  pressure from the BMP388.
- Shows live readings in several dashboard layouts, including an estimated
  altitude calculated from pressure.
- Provides history graphs for CO2, temperature, humidity, pressure and altitude
  with selectable time scales.
- Streams readings to a PC over USB CDC and records timestamped CSV files to the
  microSD card with configurable intervals and durations.
- Detects sensor and I2C bus faults, retries or recovers automatically, keeps
  working with one sensor, and requests a restart if bus recovery fails.

## Controls

| Input | Action |
| ----- | ------ |
| **LEFT / RIGHT** | Browse the recording page, dashboard and five history graphs; change a recording value while editing it |
| **UP / DOWN** | Change the dashboard layout or graph time scale; move between fields on the recording page |
| Short **OK** | Edit or confirm a recording field, start/stop recording, or confirm a dialog |
| Short **BACK** | Open or cancel the clear-history dialog; cancel the exit dialog |
| Long **OK / BACK** | OK toggles always-on backlight; BACK opens the exit confirmation |

## Data recording

Press **LEFT** from the dashboard to open **Data Recording**. Use **UP / DOWN**
to select a field, short **OK** to edit it, and **LEFT / RIGHT** to change its
value. Select **Start** and press **OK** to begin; press **OK** again to stop.

| Setting | Choices |
| ------- | ------- |
| Interval | 2 s, 5 s, 10 s, 30 s, 1 min |
| Length | 1 min, 5 min, 10 min, 30 min, 1 h, continuous |

Files are saved as
`/ext/co2_sniffer/YYYYMMDD_HHMMSS_<interval>_<length>.csv`. The columns are
`timestamp`, `elapsed_s`, `co2_ppm`, `scd30_temperature_c`, `humidity_pct`,
`bmp388_temperature_c`, `pressure_hpa` and `altitude_m`. Values from an
unavailable sensor are left empty. A blinking `REC` badge shows that recording
is active; `SD!` reports a file or microSD write error.

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

`SW/` is a Flipper application (FAP, appid `co2_sniffer`, category DIY, v0.4).
Its main flow is:

1. Detect and initialize the SCD30 and BMP388.
2. Read the available sensors and update the live dashboard.
3. Record readings for the history graph pages.
4. Send readings through USB CDC and optionally save them as CSV.
5. Retry disconnected sensors automatically and restore system settings on exit.

## Layout

- `HW/` — KiCad board: schematic, PCB, V1.0 gerbers in `Flipper-CO2-module-V10/`.
- `SW/` — Flipper app sources; builds are written under `SW/dist/`.
- `3d/` — Printable enclosure files: a 3MF with Bambu Lab settings and the original STEP model.
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
