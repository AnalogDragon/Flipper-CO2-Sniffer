#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    bool fail; /* true while the sensor is not working (re-init each tick) */
    float T_LIN; /* compensated temperature, °C */
    float P_LIN; /* compensated pressure, Pa */
    float P_FIL; /* low-pass filtered pressure, Pa (SCD30 compensation feed) */
} Co2Bmp388;

/** Soft-reset, configure (OSR x32/x32, 50 Hz, normal mode) and load the NVM
 *  calibration parameters. Returns false while unreachable; retry later. */
bool co2_bmp388_init(Co2Bmp388* s);

/** Read raw data, run the temperature/pressure compensation and pack the
 *  16B report block. Returns false and sets fail on I2C error or sensor
 *  error flag. */
bool co2_bmp388_read(Co2Bmp388* s);
