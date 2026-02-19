/*
 * bme688.h - BME688 gas/environment sensor driver
 *
 * Forced-mode I2C driver for temperature, humidity, pressure,
 * and gas resistance. No BSEC library dependency -- we talk
 * directly to the register set and do the compensation math
 * ourselves so we actually understand what the sensor returns.
 *
 * Reference: Bosch BME68x datasheet rev 1.7
 */

#ifndef BME688_H
#define BME688_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* 7-bit I2C address -- SDO pulled high = 0x77, low = 0x76 */
#define BME688_ADDR             (0x77 << 1)

/* chip ID register should read 0x61 */
#define BME688_CHIP_ID          0x61

/* status codes */
typedef enum {
    BME688_OK = 0,
    BME688_ERR_I2C,
    BME688_ERR_ID,
    BME688_ERR_CALIB,
    BME688_ERR_MEAS,
} bme688_status_t;

/* compensated output */
typedef struct {
    float temperature;      /* degrees C */
    float humidity;         /* %RH       */
    float pressure;         /* Pa        */
    float gas_resistance;   /* ohms      */
    bool  gas_valid;
    bool  heat_stab;        /* heater reached target? */
} bme688_data_t;

/* calibration coefficients -- stored internally after readout */
typedef struct {
    /* temperature */
    uint16_t par_t1;
    int16_t  par_t2;
    int8_t   par_t3;

    /* pressure */
    uint16_t par_p1;
    int16_t  par_p2;
    int8_t   par_p3;
    int16_t  par_p4;
    int16_t  par_p5;
    int8_t   par_p6;
    int8_t   par_p7;
    int16_t  par_p8;
    int16_t  par_p9;
    uint8_t  par_p10;

    /* humidity */
    uint16_t par_h1;
    uint16_t par_h2;
    int8_t   par_h3;
    int8_t   par_h4;
    int8_t   par_h5;
    uint8_t  par_h6;
    int8_t   par_h7;

    /* gas */
    int8_t   par_gh1;
    int16_t  par_gh2;
    int8_t   par_gh3;

    /* shared */
    float    t_fine;
    uint8_t  res_heat_range;
    int8_t   res_heat_val;
    int8_t   range_sw_err;
} bme688_calib_t;

/* driver handle */
typedef struct {
    I2C_HandleTypeDef *hi2c;
    bme688_calib_t     calib;
    bme688_data_t      data;
} bme688_dev_t;

bme688_status_t bme688_init(bme688_dev_t *dev, I2C_HandleTypeDef *hi2c);
bme688_status_t bme688_trigger_forced(bme688_dev_t *dev);
bme688_status_t bme688_read_data(bme688_dev_t *dev, bme688_data_t *out);
bme688_status_t bme688_read(bme688_dev_t *dev, bme688_data_t *out);
bme688_status_t bme688_soft_reset(bme688_dev_t *dev);

#endif /* BME688_H */
