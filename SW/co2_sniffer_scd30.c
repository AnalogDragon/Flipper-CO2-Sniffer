#include "co2_sniffer_scd30.h"

#include <furi.h>

#include "co2_sniffer_i2c.h"

/* SCD30 commands (big-endian 2-byte register addresses) */
#define SCD_CMD_SOFT_RESET 0xD304
#define SCD_CMD_START_MEAS 0x0010 /* arg: pressure mbar, 0 = no compensation */
#define SCD_CMD_SET_INTERVAL 0x4600
#define SCD_CMD_SET_ASC 0x5306
#define SCD_CMD_READY 0x0202
#define SCD_CMD_READ_MEAS 0x0300

#define CRC8_POLYNOMIAL 0x31
#define CRC8_INIT 0xFF

/* SCD30 CRC-8, ported verbatim from the CH552 firmware */
static uint8_t sen_crc(const uint8_t* dat, uint8_t count) {
    uint8_t current_byte;
    uint8_t crc = CRC8_INIT;
    uint8_t crc_bit;
    /* calculates 8-Bit checksum with given polynomial */
    for(current_byte = 0; current_byte < count; ++current_byte) {
        crc ^= (dat[current_byte]);
        for(crc_bit = 8; crc_bit > 0; --crc_bit) {
            if(crc & 0x80)
                crc = (crc << 1) ^ CRC8_POLYNOMIAL;
            else
                crc = (crc << 1);
        }
    }
    return crc;
}

static bool scd30_cmd(uint16_t cmd) {
    uint8_t reg[2] = {(uint8_t)(cmd >> 8), (uint8_t)cmd};
    return co2_i2c_reg_write(CO2_ADDR_SCD30, reg, 2, NULL, 0);
}

static bool scd30_write(uint16_t cmd, const uint8_t* data, size_t len) {
    uint8_t reg[2] = {(uint8_t)(cmd >> 8), (uint8_t)cmd};
    return co2_i2c_reg_write(CO2_ADDR_SCD30, reg, 2, data, len);
}

static bool scd30_read(uint16_t cmd, uint8_t* data, size_t len) {
    uint8_t reg[2] = {(uint8_t)(cmd >> 8), (uint8_t)cmd};
    return co2_i2c_reg_read(CO2_ADDR_SCD30, reg, 2, data, len);
}

static float sensirion_bytes_to_float(uint32_t bytes) {
    union {
        uint32_t u32_value;
        float float32;
    } tmp;

    tmp.u32_value = bytes;
    return tmp.float32;
}

bool co2_scd30_init(Co2Scd30* s, uint16_t press_mbar) {
    s->fail = true;

    /* The CH552 board power-cycled the sensor here; the Flipper board has
     * no power switch, so soft-reset instead. */
    if(!scd30_cmd(SCD_CMD_SOFT_RESET)) return false;
    furi_delay_ms(200);

    /* disable automatic self-calibration (ASC) */
    uint8_t d[3] = {0x00, 0x00, 0};
    d[2] = sen_crc(d, 2);
    if(!scd30_write(SCD_CMD_SET_ASC, d, 3)) return false;
    furi_delay_ms(100);

    /* measurement interval 2 s */
    d[0] = 0x00;
    d[1] = 0x02;
    d[2] = sen_crc(d, 2);
    if(!scd30_write(SCD_CMD_SET_INTERVAL, d, 3)) return false;
    furi_delay_ms(100);

    /* start continuous measurement with ambient pressure */
    if(!co2_scd30_set_pressure(press_mbar)) return false;
    furi_delay_ms(100);

    s->fail = false;
    return true;
}

bool co2_scd30_tick(Co2Scd30* s) {
    if(s->fail) return false;

    /* data ready flag: [0x00, 0x01, crc] */
    uint8_t rd[3];
    if(!scd30_read(SCD_CMD_READY, rd, 3)) {
        s->fail = true;
        return false;
    }
    if(rd[2] != sen_crc(rd, 2)) return false; /* corrupt reply, skip this tick */
    if(!(rd[0] == 0x00 && rd[1] == 0x01)) return false; /* no fresh data yet */

    /* 18 B: 6 words (3 floats) with a CRC after each word */
    uint8_t m[18];
    if(!scd30_read(SCD_CMD_READ_MEAS, m, 18)) {
        s->fail = true;
        return false;
    }
    if(m[2] != sen_crc(m + 0, 2) || m[5] != sen_crc(m + 3, 2) || m[8] != sen_crc(m + 6, 2) ||
       m[11] != sen_crc(m + 9, 2) || m[14] != sen_crc(m + 12, 2) || m[17] != sen_crc(m + 15, 2)) {
        return false; /* corrupt frame, the sensor keeps running */
    }

    uint32_t tmp = (uint32_t)m[0] << 24 | (uint32_t)m[1] << 16 | (uint32_t)m[3] << 8 |
                   (uint32_t)m[4];
    s->CO2_PPM = sensirion_bytes_to_float(tmp);

    tmp = (uint32_t)m[6] << 24 | (uint32_t)m[7] << 16 | (uint32_t)m[9] << 8 | (uint32_t)m[10];
    s->T_SCD = sensirion_bytes_to_float(tmp);

    tmp = (uint32_t)m[12] << 24 | (uint32_t)m[13] << 16 | (uint32_t)m[15] << 8 |
          (uint32_t)m[16];
    s->R_SCD = sensirion_bytes_to_float(tmp);

    return true;
}

bool co2_scd30_set_pressure(uint16_t press_mbar) {
    uint8_t d[3];
    d[0] = press_mbar >> 8;
    d[1] = press_mbar & 0xFF;
    d[2] = sen_crc(d, 2);
    return scd30_write(SCD_CMD_START_MEAS, d, 3);
}
