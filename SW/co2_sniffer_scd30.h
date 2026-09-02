#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    bool fail; /* true while the sensor is not working (re-init each tick) */
    float CO2_PPM;
    float T_SCD; /* °C */
    float R_SCD; /* %RH */
} Co2Scd30;

/** Soft-reset (replaces the CH552 board's power cycle), disable ASC, set the
 *  2 s measurement interval and start continuous measurement with the given
 *  ambient pressure (mbar, 0 = no compensation). Returns false while
 *  unreachable; retry later. */
bool co2_scd30_init(Co2Scd30* s, uint16_t press_mbar);

/** One 333 ms tick: poll the ready flag, read the measurement when fresh
 *  data is available. Returns true when a new measurement was stored. */
bool co2_scd30_tick(Co2Scd30* s);

/** Restart continuous measurement with a new ambient pressure (mbar). */
bool co2_scd30_set_pressure(uint16_t press_mbar);
