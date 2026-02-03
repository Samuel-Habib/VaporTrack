/*
 * bno055.h - BNO055 9-axis IMU driver (heading-only subset)
 *
 * We only care about orientation/heading for dead-reckoning
 * coordinate transforms. The BNO055 does sensor fusion internally
 * in NDOF mode, so we just grab the Euler heading from its
 * fusion output registers.
 *
 * Reference: Bosch BNO055 datasheet BST-BNO055-DS000-14
 */

#ifndef BNO055_H
#define BNO055_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* 7-bit address: COM3 low = 0x28, high = 0x29 */
#define BNO055_ADDR             (0x28 << 1)
#define BNO055_CHIP_ID_VAL      0xA0

typedef enum {
    BNO055_OK = 0,
    BNO055_ERR_I2C,
    BNO055_ERR_ID,
    BNO055_ERR_SELFTEST,
    BNO055_ERR_TIMEOUT,
} bno055_status_t;

/* Euler angles in degrees */
typedef struct {
    float heading;    /* 0..360      */
    float roll;       /* -180..180   */
    float pitch;      /* -90..90     */
} bno055_euler_t;

/* calibration status nibbles (0=uncal, 3=fully calibrated) */
typedef struct {
    uint8_t sys;
    uint8_t gyro;
    uint8_t accel;
    uint8_t mag;
} bno055_cal_status_t;

typedef struct {
    I2C_HandleTypeDef  *hi2c;
    bno055_euler_t      euler;
    bno055_cal_status_t cal;
} bno055_dev_t;

bno055_status_t bno055_init(bno055_dev_t *dev, I2C_HandleTypeDef *hi2c);
bno055_status_t bno055_read_euler(bno055_dev_t *dev, bno055_euler_t *out);
bno055_status_t bno055_get_calibration(bno055_dev_t *dev, bno055_cal_status_t *out);
bool            bno055_is_calibrated(const bno055_cal_status_t *cal);

#endif /* BNO055_H */
