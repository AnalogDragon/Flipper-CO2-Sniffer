#include "co2_sniffer_i2c.h"

#include <furi.h>
#include <furi_hal_i2c.h>
#include <string.h>

/* All transfers complete well under 5 ms at 100 kHz (18 B read ~2 ms).
 * This cap only guards against bus hangs. */
#define CO2_I2C_TIMEOUT_MS 50

void co2_i2c_acquire(void) {
    furi_hal_i2c_acquire(&furi_hal_i2c_handle_external);
}

void co2_i2c_release(void) {
    furi_hal_i2c_release(&furi_hal_i2c_handle_external);
}

/* IMPORTANT: this firmware's furi_hal_i2c API expects the address in 8-bit
 * form (7-bit addr << 1). Verified by disassembling the SDK: the driver ORs
 * the address into CR2.SADD without shifting. Passing a raw 7-bit address
 * halves the address on the wire. */
static uint8_t co2_i2c_addr8(uint8_t addr7) {
    return (uint8_t)(addr7 << 1);
}

void co2_i2c_scan(Co2I2cScanResult* result) {
    result->count = 0;

    /* Probe with an explicit 1-byte write. Writing 0x00 is harmless for
     * the CO2 sensors: BMP388 reg 0x00 is the read-only chip ID,
     * SCD30 command 0x0000 is a no-op. */
    static const uint8_t probe_byte = 0x00;

    for(uint16_t addr = CO2_I2C_SCAN_MIN; addr <= CO2_I2C_SCAN_MAX; addr++) {
        if(furi_hal_i2c_tx(
               &furi_hal_i2c_handle_external,
               co2_i2c_addr8((uint8_t)addr),
               &probe_byte,
               1,
               CO2_I2C_TIMEOUT_MS)) {
            result->addresses[result->count++] = (uint8_t)addr;
        }
    }
}

bool co2_i2c_reg_read(
    uint8_t addr7,
    const uint8_t* reg,
    size_t reg_len,
    uint8_t* data,
    size_t len) {
    furi_check(reg_len <= 2);
    furi_check(len > 0);

    return furi_hal_i2c_trx(
        &furi_hal_i2c_handle_external,
        co2_i2c_addr8(addr7),
        reg,
        reg_len,
        data,
        len,
        CO2_I2C_TIMEOUT_MS);
}

bool co2_i2c_reg_write(
    uint8_t addr7,
    const uint8_t* reg,
    size_t reg_len,
    const uint8_t* data,
    size_t data_len) {
    furi_check(reg_len <= 2);
    furi_check(reg_len + data_len <= 18);

    uint8_t buf[18];
    memcpy(buf, reg, reg_len);
    if(data_len > 0) {
        memcpy(buf + reg_len, data, data_len);
    }

    return furi_hal_i2c_tx(
        &furi_hal_i2c_handle_external,
        co2_i2c_addr8(addr7),
        buf,
        reg_len + data_len,
        CO2_I2C_TIMEOUT_MS);
}
