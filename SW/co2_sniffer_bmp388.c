#include "co2_sniffer_bmp388.h"

#include <furi.h>

#include "co2_sniffer_i2c.h"

#define BMP_REG_CMD 0x7E /* soft reset */
#define BMP_REG_PWR_CTRL 0x1B
#define BMP_REG_OSR 0x1C
#define BMP_REG_ODR 0x1D
#define BMP_REG_ERR 0x02 /* ERR_REG; read starts here, data follows 0x03..0x09 */
#define BMP_REG_NVM_START 0x31 /* 21 bytes of calibration data */

/* NVM calibration parameters, pre-scaled. Ported verbatim from the CH552
 * firmware (GPIO.c). All math there ran in 32-bit float (C51 "double" is
 * float32), so float is used here as well - this exact scheme was verified
 * on hardware. The divisors fold the Bosch reference implementation's
 * power-of-two factors into the parameters, so the formulas below yield
 * plain °C and Pa directly. Do not "fix" it into the Bosch form. */
static float PAR_T1, PAR_T2, PAR_T3;
static float PAR_P1, PAR_P2, PAR_P3, PAR_P4, PAR_P5, PAR_P6;
static float PAR_P7, PAR_P8, PAR_P9, PAR_P10, PAR_P11;

static bool bmp388_nvm_par(void) {
    uint8_t d[21];
    uint8_t reg = BMP_REG_NVM_START;
    if(!co2_i2c_reg_read(CO2_ADDR_BMP388, &reg, 1, d, sizeof(d))) {
        return false;
    }

    uint16_t NVM_T1 = (uint16_t)d[0] | ((uint16_t)d[1] << 8);
    uint16_t NVM_T2 = (uint16_t)d[2] | ((uint16_t)d[3] << 8);
    int8_t NVM_T3 = (int8_t)d[4];

    int16_t NVM_P1 = (int16_t)((uint16_t)d[5] | ((uint16_t)d[6] << 8));
    int16_t NVM_P2 = (int16_t)((uint16_t)d[7] | ((uint16_t)d[8] << 8));
    int8_t NVM_P3 = (int8_t)d[9];
    int8_t NVM_P4 = (int8_t)d[10];
    uint16_t NVM_P5 = (uint16_t)d[11] | ((uint16_t)d[12] << 8);
    uint16_t NVM_P6 = (uint16_t)d[13] | ((uint16_t)d[14] << 8);
    int8_t NVM_P7 = (int8_t)d[15];
    int8_t NVM_P8 = (int8_t)d[16];
    int16_t NVM_P9 = (int16_t)((uint16_t)d[17] | ((uint16_t)d[18] << 8));
    int8_t NVM_P10 = (int8_t)d[19];
    int8_t NVM_P11 = (int8_t)d[20];

    float temp_var;
    temp_var = 0.00390625f;
    PAR_T1 = ((float)NVM_T1 / temp_var);
    temp_var = 1073741824.0f;
    PAR_T2 = ((float)NVM_T2 / temp_var);
    temp_var = 281474976710656.0f;
    PAR_T3 = ((float)NVM_T3 / temp_var);
    temp_var = 1048576.0f;
    PAR_P1 = ((float)(NVM_P1 - (16384)) / temp_var);
    temp_var = 536870912.0f;
    PAR_P2 = ((float)(NVM_P2 - (16384)) / temp_var);
    temp_var = 4294967296.0f;
    PAR_P3 = ((float)NVM_P3 / temp_var);
    temp_var = 137438953472.0f;
    PAR_P4 = ((float)NVM_P4 / temp_var);
    temp_var = 0.125f;
    PAR_P5 = ((float)NVM_P5 / temp_var);
    temp_var = 64.0f;
    PAR_P6 = ((float)NVM_P6 / temp_var);
    temp_var = 256.0f;
    PAR_P7 = ((float)NVM_P7 / temp_var);
    temp_var = 32768.0f;
    PAR_P8 = ((float)NVM_P8 / temp_var);
    temp_var = 281474976710656.0f;
    PAR_P9 = ((float)NVM_P9 / temp_var);
    temp_var = 281474976710656.0f;
    PAR_P10 = ((float)NVM_P10 / temp_var);
    temp_var = 36893488147419103232.0f;
    PAR_P11 = ((float)NVM_P11 / temp_var);

    return true;
}

static void temp_conv(uint32_t raw_temp, float* t_lin) {
    float partial_data1;
    float partial_data2;

    partial_data1 = (float)((float)raw_temp - PAR_T1);
    partial_data2 = (float)(partial_data1 * PAR_T2);

    *t_lin = partial_data2 + (partial_data1 * partial_data1) * PAR_T3;
}

static void press_conv(uint32_t raw_press, float t_lin, float* p_lin) {
    float in_press = raw_press;

    float partial_data1;
    float partial_data2;
    float partial_data3;
    float partial_data4;
    float partial_out1;
    float partial_out2;

    partial_data1 = PAR_P6 * t_lin;
    partial_data2 = PAR_P7 * t_lin * t_lin;
    partial_data3 = PAR_P8 * t_lin * t_lin * t_lin;
    partial_out1 = PAR_P5 + partial_data1 + partial_data2 + partial_data3;
    partial_data1 = PAR_P2 * t_lin;
    partial_data2 = PAR_P3 * t_lin * t_lin;
    partial_data3 = PAR_P4 * t_lin * t_lin * t_lin;
    partial_out2 = raw_press * (PAR_P1 + partial_data1 + partial_data2 + partial_data3);
    partial_data1 = in_press * in_press;
    partial_data2 = PAR_P9 + PAR_P10 * t_lin;
    partial_data3 = partial_data1 * partial_data2;
    partial_data4 = partial_data3 + (in_press * in_press * in_press) * PAR_P11;

    *p_lin = partial_out1 + partial_out2 + partial_data4;
}

static bool bmp388_write_reg(uint8_t reg, uint8_t val) {
    return co2_i2c_reg_write(CO2_ADDR_BMP388, &reg, 1, &val, 1);
}

bool co2_bmp388_init(Co2Bmp388* s) {
    s->fail = true;

    if(!bmp388_write_reg(BMP_REG_CMD, 0xB6)) return false; /* reset */
    furi_delay_ms(20);
    if(!bmp388_write_reg(BMP_REG_OSR, 0x2D)) return false; /* osr_t x32, osr_p x32 */
    if(!bmp388_write_reg(BMP_REG_ODR, 0x05)) return false; /* 50 Hz */
    if(!bmp388_write_reg(BMP_REG_PWR_CTRL, 0x33)) return false; /* normal mode, T+P on */
    furi_delay_ms(10);

    uint8_t err;
    uint8_t reg = BMP_REG_ERR;
    if(!co2_i2c_reg_read(CO2_ADDR_BMP388, &reg, 1, &err, 1)) return false;
    if(err != 0) return false;

    if(!bmp388_nvm_par()) return false;

    s->fail = false;
    return true;
}

bool co2_bmp388_read(Co2Bmp388* s) {
    if(s->fail) return false;

    /* regs: [0]=ERR [1]=STATUS [2..4]=pressure raw LE24 [5..7]=temp raw LE24 */
    uint8_t reg = BMP_REG_ERR;
    uint8_t regs[8];
    if(!co2_i2c_reg_read(CO2_ADDR_BMP388, &reg, 1, regs, sizeof(regs))) {
        s->fail = true;
        return false;
    }
    if(regs[0] != 0) { /* fatal error flag set */
        s->fail = true;
        return false;
    }

    uint32_t raw_p = (uint32_t)regs[2] | ((uint32_t)regs[3] << 8) | ((uint32_t)regs[4] << 16);
    uint32_t raw_t = (uint32_t)regs[5] | ((uint32_t)regs[6] << 8) | ((uint32_t)regs[7] << 16);

    float t_lin;
    float p_lin;
    temp_conv(raw_t, &t_lin);
    press_conv(raw_p, t_lin, &p_lin);

    s->T_LIN = t_lin;
    s->P_LIN = p_lin;
    if(s->P_FIL == 0)
        s->P_FIL = s->P_LIN;
    else
        s->P_FIL = s->P_FIL * 0.95f + s->P_LIN * 0.05f;

    return true;
}
