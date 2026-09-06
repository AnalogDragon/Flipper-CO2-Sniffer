#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    bool fail; /* true while the sensor is not working (re-init each tick) */
    float CO2_PPM;
    float T_SCD; /* °C */
    float R_SCD; /* %RH */
} Co2Scd30;

/** Soft-reset (replaces the CH552 board's power cycle), apply the requested
 *  ASC state, set the 2 s measurement interval and start continuous
 *  measurement with the given ambient pressure (mbar, 0 = no compensation).
 *  Returns false while unreachable; retry later. */
bool co2_scd30_init(Co2Scd30* s, uint16_t press_mbar, bool asc_enabled);

/** One 333 ms tick: poll the ready flag, read the measurement when fresh
 *  data is available. Returns true when a new measurement was stored. */
bool co2_scd30_tick(Co2Scd30* s);

/** Restart continuous measurement with a new ambient pressure (mbar). */
bool co2_scd30_set_pressure(uint16_t press_mbar);

/** Enable or disable automatic self-calibration. */
bool co2_scd30_set_asc(bool enabled);

/** Force recalibration against a stable 400..2000 ppm reference. */
bool co2_scd30_force_recalibration(uint16_t reference_ppm);

/** Stop continuous measurement and enter the sensor's idle low-power state. */
bool co2_scd30_stop(void);
