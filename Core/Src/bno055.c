#include "bno055.h"

/* Registers */
#define BNO055_REG_CHIP_ID      0x00
#define BNO055_REG_EUL_DATA_X_LSB 0x1A
#define BNO055_REG_CALIB_STAT   0x35
#define BNO055_REG_UNIT_SEL     0x3B
#define BNO055_REG_OPR_MODE     0x3D

/* Operation Modes */
#define BNO055_OPR_MODE_CONFIG  0x00
#define BNO055_OPR_MODE_NDOF    0x0C

static HAL_StatusTypeDef read_reg(bno055_dev_t *dev, uint8_t reg, uint8_t *data, uint16_t len) {
    return HAL_I2C_Mem_Read(dev->hi2c, BNO055_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, len, 100);
}

static HAL_StatusTypeDef write_reg(bno055_dev_t *dev, uint8_t reg, uint8_t val) {
    return HAL_I2C_Mem_Write(dev->hi2c, BNO055_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
}

bno055_status_t bno055_init(bno055_dev_t *dev, I2C_HandleTypeDef *hi2c) {
    uint8_t chip_id = 0;

    if (!dev || !hi2c) {
        return BNO055_ERR_I2C;
    }

    dev->hi2c = hi2c;

    /* Verify Chip ID */
    if (read_reg(dev, BNO055_REG_CHIP_ID, &chip_id, 1) != HAL_OK) {
        return BNO055_ERR_I2C;
    }

    if (chip_id != BNO055_CHIP_ID_VAL) {
        return BNO055_ERR_ID;
    }

    /* Set CONFIG mode */
    if (write_reg(dev, BNO055_REG_OPR_MODE, BNO055_OPR_MODE_CONFIG) != HAL_OK) {
        return BNO055_ERR_I2C;
    }
    HAL_Delay(25); /* 7ms min for any->config, give it 25ms */

    /* Configure units: degrees, Celsius */
    /* UNIT_SEL bits: 0 = Windows orientation, 0 = Celsius, 0 = degrees, 0 = m/s^2 */
    if (write_reg(dev, BNO055_REG_UNIT_SEL, 0x00) != HAL_OK) {
        return BNO055_ERR_I2C;
    }

    /* Set NDOF mode */
    if (write_reg(dev, BNO055_REG_OPR_MODE, BNO055_OPR_MODE_NDOF) != HAL_OK) {
        return BNO055_ERR_I2C;
    }
    HAL_Delay(25); /* 20ms min for config->any */

    return BNO055_OK;
}

bno055_status_t bno055_read_euler(bno055_dev_t *dev, bno055_euler_t *out) {
    uint8_t buf[6];

    if (read_reg(dev, BNO055_REG_EUL_DATA_X_LSB, buf, 6) != HAL_OK) {
        return BNO055_ERR_I2C;
    }

    int16_t heading_raw = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t roll_raw    = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t pitch_raw   = (int16_t)((buf[5] << 8) | buf[4]);

    out->heading = heading_raw / 16.0f;
    out->roll    = roll_raw / 16.0f;
    out->pitch   = pitch_raw / 16.0f;

    dev->euler = *out;

    return BNO055_OK;
}

bno055_status_t bno055_get_calibration(bno055_dev_t *dev, bno055_cal_status_t *out) {
    uint8_t cal_stat = 0;

    if (read_reg(dev, BNO055_REG_CALIB_STAT, &cal_stat, 1) != HAL_OK) {
        return BNO055_ERR_I2C;
    }

    out->sys   = (cal_stat >> 6) & 0x03;
    out->gyro  = (cal_stat >> 4) & 0x03;
    out->accel = (cal_stat >> 2) & 0x03;
    out->mag   = cal_stat & 0x03;

    dev->cal = *out;

    return BNO055_OK;
}

bool bno055_is_calibrated(const bno055_cal_status_t *cal) {
    if (!cal) {
        return false;
    }
    return (cal->sys >= 1) && ((cal->gyro + cal->mag) >= 2);
}
