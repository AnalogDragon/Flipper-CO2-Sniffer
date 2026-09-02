#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* Devices on the flipper_co2 board (found by the scanner on 2026-09-02) */
#define CO2_ADDR_BMP388 0x76
#define CO2_ADDR_SCD30 0x61

/* Scan range: skip reserved 0x00-0x07 and 10-bit addressing 0x78-0x7F */
#define CO2_I2C_SCAN_MIN 0x08
#define CO2_I2C_SCAN_MAX 0x77
#define CO2_I2C_MAX_DEVICES (CO2_I2C_SCAN_MAX - CO2_I2C_SCAN_MIN + 1)

typedef struct {
    uint8_t addresses[CO2_I2C_MAX_DEVICES];
    uint8_t count;
} Co2I2cScanResult;

/** Acquire/release the external I2C bus (C0=SCL / C1=SDA) for the whole run. */
void co2_i2c_acquire(void);
void co2_i2c_release(void);

/** Scan the external I2C bus for responding devices (debug helper). */
void co2_i2c_scan(Co2I2cScanResult* result);

/**
 * Read a device register: write the reg bytes, repeated start, read len bytes.
 *
 * addr7 is the standard 7-bit address. The 8-bit form is applied internally
 * because this firmware's furi_hal_i2c API expects 8-bit addresses (verified
 * by disassembly: the driver ORs the address into CR2.SADD without shifting).
 */
bool co2_i2c_reg_read(
    uint8_t addr7,
    const uint8_t* reg,
    size_t reg_len,
    uint8_t* data,
    size_t len);

/** Write data bytes to a device register (single transfer, stop at the end). */
bool co2_i2c_reg_write(
    uint8_t addr7,
    const uint8_t* reg,
    size_t reg_len,
    const uint8_t* data,
    size_t data_len);
