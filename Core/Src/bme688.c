#include "bme688.h"
#include <math.h>

#define I2C_TIMEOUT 100

#define REG_STATUS       0x73
#define REG_RESET        0xE0
#define REG_ID           0xD0
#define REG_CTRL_MEAS    0x74
#define REG_CTRL_HUM     0x72
#define REG_CTRL_GAS_1   0x71
#define REG_GAS_WAIT_0   0x64
#define REG_RES_HEAT_0   0x5A
#define REG_MEAS_STATUS_0 0x1D
#define REG_PRESS_MSB    0x1F

static bme688_status_t read_regs(bme688_dev_t *dev, uint8_t reg, uint8_t *data, uint16_t len) {
    if (HAL_I2C_Mem_Read(dev->hi2c, BME688_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, len, I2C_TIMEOUT) != HAL_OK) {
        return BME688_ERR_I2C;
    }
    return BME688_OK;
}

static bme688_status_t write_reg(bme688_dev_t *dev, uint8_t reg, uint8_t val) {
    if (HAL_I2C_Mem_Write(dev->hi2c, BME688_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, I2C_TIMEOUT) != HAL_OK) {
        return BME688_ERR_I2C;
    }
    return BME688_OK;
}

bme688_status_t bme688_soft_reset(bme688_dev_t *dev) {
    bme688_status_t res = write_reg(dev, REG_RESET, 0xB6);
    HAL_Delay(5);
    return res;
}

static bme688_status_t read_calib(bme688_dev_t *dev) {
    uint8_t buf[128]; // Arbitrary buffer to hold sparse regs, simplify by reading chunks
    bme688_calib_t *c = &dev->calib;

    // Read 0x8A to 0xA0
    if (read_regs(dev, 0x8A, &buf[0x8A], 0xA0 - 0x8A + 1) != BME688_OK) return BME688_ERR_I2C;
    // Read 0xE1 to 0xF0
    if (read_regs(dev, 0xE1, &buf[0xE1], 0xF0 - 0xE1 + 1) != BME688_OK) return BME688_ERR_I2C;
    // Read 0x00, 0x02, 0x04
    if (read_regs(dev, 0x00, &buf[0x00], 5) != BME688_OK) return BME688_ERR_I2C;

    c->par_t1 = (uint16_t)(buf[0xEA] << 8 | buf[0xE9]);
    c->par_t2 = (int16_t)(buf[0x8B] << 8 | buf[0x8A]);
    c->par_t3 = (int8_t)buf[0x8C];

    c->par_p1 = (uint16_t)(buf[0x8F] << 8 | buf[0x8E]);
    c->par_p2 = (int16_t)(buf[0x91] << 8 | buf[0x90]);
    c->par_p3 = (int8_t)buf[0x92];
    c->par_p4 = (int16_t)(buf[0x95] << 8 | buf[0x94]);
    c->par_p5 = (int16_t)(buf[0x97] << 8 | buf[0x96]);
    c->par_p6 = (int8_t)buf[0x99];
    c->par_p7 = (int8_t)buf[0x98];
    c->par_p8 = (int16_t)(buf[0x9D] << 8 | buf[0x9C]);
    c->par_p9 = (int16_t)(buf[0x9F] << 8 | buf[0x9E]);
    c->par_p10 = buf[0xA0];

    c->par_h1 = (uint16_t)((buf[0xE3] & 0x0F) << 8 | buf[0xE2]);
    c->par_h2 = (uint16_t)(buf[0xE1] << 4 | (buf[0xE3] >> 4));
    c->par_h3 = (int8_t)buf[0xE4];
    c->par_h4 = (int8_t)buf[0xE5];
    c->par_h5 = (int8_t)buf[0xE6];
    c->par_h6 = buf[0xE7];
    c->par_h7 = (int8_t)buf[0xE8];

    c->par_gh1 = (int8_t)buf[0xED];
    c->par_gh2 = (int16_t)(buf[0xEC] << 8 | buf[0xEB]);
    c->par_gh3 = (int8_t)buf[0xEE];

    c->res_heat_range = (buf[0x02] >> 4) & 0x03;
    c->res_heat_val = (int8_t)buf[0x00];
    c->range_sw_err = (int8_t)((buf[0x04] & 0xF0) >> 4);

    return BME688_OK;
}

/*
 * Heater resistance target calculation.
 * Attempt to get close to datasheet formula; the exact coefficients
 * depend on the individual sensor's calibration, but this gets
 * us in the ballpark for 320C target.
 */
static uint8_t calc_heater_res(uint16_t target_temp, float amb_temp, bme688_dev_t *dev) {
    bme688_calib_t *c = &dev->calib;
    float var1 = ((float)c->par_gh1 / 16.0f) + 49.0f;
    float var2 = (((float)c->par_gh2 / 32768.0f) * 0.0005f) + 0.00235f;
    float var3 = (float)c->par_gh3 / 1024.0f;
    float var4 = var1 * (1.0f + (var2 * (float)target_temp));
    float var5 = var4 + (var3 * amb_temp);
    uint8_t res = (uint8_t)(3.4f * ((var5 * (4.0f / (4.0f + (float)c->res_heat_range))
                    * (1.0f / (1.0f + ((float)c->res_heat_val * 0.002f)))) - 25.0f));
    return res;
}

/* gas range lookup table from Bosch datasheet */
static const float gas_range_lut[16] = {
    1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 0.99f,
    1.0f, 0.992f, 1.0f, 1.0f, 0.998f, 0.995f,
    1.0f, 0.99f, 1.0f, 1.0f
};

static float calc_gas_resistance(uint16_t adc_gas, uint8_t gas_range, int8_t range_sw_err) {
    float var1 = (1340.0f + 5.0f * (float)range_sw_err) * gas_range_lut[gas_range];
    float gas_res = var1 * ((131072.0f / (float)adc_gas) - 1.0f);
    return gas_res;
}

bme688_status_t bme688_init(bme688_dev_t *dev, I2C_HandleTypeDef *hi2c) {
    if (!dev || !hi2c) return BME688_ERR_I2C;
    dev->hi2c = hi2c;

    if (bme688_soft_reset(dev) != BME688_OK) return BME688_ERR_I2C;

    uint8_t id = 0;
    if (read_regs(dev, REG_ID, &id, 1) != BME688_OK) return BME688_ERR_I2C;
    if (id != BME688_CHIP_ID) return BME688_ERR_ID;

    if (read_calib(dev) != BME688_OK) return BME688_ERR_CALIB;

    // Configure forced mode with 2x oversampling on T, H, P
    write_reg(dev, REG_CTRL_HUM, 0x02); // osrs_h = 2 (2x)
    write_reg(dev, REG_CTRL_MEAS, (0x02 << 5) | (0x02 << 2) | 0x00); // osrs_t=2, osrs_p=2, sleep mode

    // Set gas heater: 320C, 150ms
    // res_heat_0
    uint8_t res_heat = calc_heater_res(320, 25.0f, dev);
    write_reg(dev, REG_RES_HEAT_0, res_heat);
    // gas_wait_0: factor * 64, wait time = 150ms -> 150 / 0.477ms approx 314 wait -> need correct coding
    // Formula for 150ms: factor = 0x01, wait = 0x59 (150ms approx)
    write_reg(dev, REG_GAS_WAIT_0, 0x59);
    
    // Enable gas measurement
    write_reg(dev, REG_CTRL_GAS_1, 0x10); // run_gas = 1

    return BME688_OK;
}

/* Single-precision floating point compensation using Cortex-M4 hardware FPU */
static void compensate_data(bme688_dev_t *dev, uint32_t adc_t, uint32_t adc_p, uint16_t adc_h, uint16_t adc_g, uint8_t gas_range) {
    bme688_calib_t *c = &dev->calib;
    bme688_data_t *d = &dev->data;

    // Temperature
    float var1 = (((float)adc_t / 16384.0f) - ((float)c->par_t1 / 1024.0f)) * (float)c->par_t2;
    float var2 = ((((float)adc_t / 131072.0f) - ((float)c->par_t1 / 8192.0f)) *
                  (((float)adc_t / 131072.0f) - ((float)c->par_t1 / 8192.0f))) * ((float)c->par_t3 * 16.0f);
    c->t_fine = var1 + var2;
    d->temperature = c->t_fine / 5120.0f;

    // Pressure
    float p_var1 = (c->t_fine / 2.0f) - 64000.0f;
    float p_var2 = p_var1 * p_var1 * (((float)c->par_p6) / (131072.0f));
    p_var2 = p_var2 + (p_var1 * ((float)c->par_p5) * 2.0f);
    p_var2 = (p_var2 / 4.0f) + (((float)c->par_p4) * 65536.0f);
    p_var1 = (((((float)c->par_p3 * p_var1 * p_var1) / 16384.0f) + ((float)c->par_p2 * p_var1)) / 524288.0f);
    p_var1 = (1.0f + (p_var1 / 32768.0f)) * (float)c->par_p1;
    float pressure = 1048576.0f - (float)adc_p;
    
    if (p_var1 != 0.0f) {
        pressure = ((pressure - (p_var2 / 4096.0f)) * 6250.0f) / p_var1;
        p_var1 = ((float)c->par_p9 * pressure * pressure) / 2147483648.0f;
        p_var2 = pressure * ((float)c->par_p8 / 32768.0f);
        float p_var3 = (pressure / 256.0f) * (pressure / 256.0f) * (pressure / 256.0f) * (c->par_p10 / 131072.0f);
        d->pressure = pressure + (p_var1 + p_var2 + p_var3 + ((float)c->par_p7 * 128.0f)) / 16.0f;
    } else {
        d->pressure = 0;
    }

    // Humidity (simplified)
    float h_var1 = (float)adc_h - (((float)c->par_h1 * 16.0f) + (((float)c->par_h3 / 2.0f) * d->temperature));
    float h_var2 = h_var1 * (((float)c->par_h2 / 262144.0f) * (1.0f + (((float)c->par_h4 / 16384.0f) * d->temperature) + (((float)c->par_h5 / 1048576.0f) * d->temperature * d->temperature)));
    float h_var3 = (float)c->par_h6 / 16384.0f;
    float h_var4 = (float)c->par_h7 / 2097152.0f;
    d->humidity = h_var2 + ((h_var3 + (h_var4 * d->temperature)) * h_var2 * h_var2);
    if (d->humidity > 100.0f) d->humidity = 100.0f;
    if (d->humidity < 0.0f) d->humidity = 0.0f;

    // Gas Resistance (simplified empirical formula)
    d->gas_resistance = calc_gas_resistance(adc_g, gas_range, c->range_sw_err);
}

bme688_status_t bme688_trigger_forced(bme688_dev_t *dev) {
    if (!dev) return BME688_ERR_I2C;

    uint8_t ctrl_meas;
    if (read_regs(dev, REG_CTRL_MEAS, &ctrl_meas, 1) != BME688_OK) return BME688_ERR_I2C;
    ctrl_meas = (ctrl_meas & 0xFC) | 0x01; // mode = 01 (forced)
    if (write_reg(dev, REG_CTRL_MEAS, ctrl_meas) != BME688_OK) return BME688_ERR_I2C;

    return BME688_OK;
}

bme688_status_t bme688_read_data(bme688_dev_t *dev, bme688_data_t *out) {
    if (!dev || !out) return BME688_ERR_I2C;

    uint8_t data[15];
    if (read_regs(dev, REG_MEAS_STATUS_0, data, 15) != BME688_OK) return BME688_ERR_I2C;

    uint32_t adc_p = (data[2] << 12) | (data[3] << 4) | (data[4] >> 4);
    uint32_t adc_t = (data[5] << 12) | (data[6] << 4) | (data[7] >> 4);
    uint16_t adc_h = (data[8] << 8) | data[9];
    uint16_t adc_g = (data[13] << 2) | (data[14] >> 6);
    uint8_t gas_range = data[14] & 0x0F;

    dev->data.gas_valid = (data[0] & 0x20) != 0;
    dev->data.heat_stab = (data[0] & 0x10) != 0;

    compensate_data(dev, adc_t, adc_p, adc_h, adc_g, gas_range);
    *out = dev->data;

    return BME688_OK;
}

bme688_status_t bme688_read(bme688_dev_t *dev, bme688_data_t *out) {
    bme688_status_t res = bme688_trigger_forced(dev);
    if (res != BME688_OK) return res;

    HAL_Delay(160); // Wait for measurement (incl 150ms heater)

    return bme688_read_data(dev, out);
}
