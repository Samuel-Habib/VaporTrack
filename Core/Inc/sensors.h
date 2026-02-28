/*
 * sensors.h - central sensor data aggregation
 *
 * Single structure that holds a complete snapshot of every
 * sensor on the rover. Protected by a FreeRTOS mutex so
 * producer tasks can write to it and consumer tasks can
 * read a consistent copy without tearing.
 */

#ifndef SENSORS_H
#define SENSORS_H

#include "FreeRTOS.h"
#include "semphr.h"
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    /* BME688 */
    float    gas_resistance;
    float    temperature;
    float    humidity;
    float    pressure;
    bool     gas_valid;
    bool     gas_heater_stable;

    /* BNO055 */
    float    heading_deg;
    float    roll_deg;
    float    pitch_deg;
    bool     imu_calibrated;
    uint8_t  cal_sys;
    uint8_t  cal_gyro;
    uint8_t  cal_accel;
    uint8_t  cal_mag;

    /* HC-SR04 */
    float    dist_front_cm;
    float    dist_left_cm;
    float    dist_right_cm;

    /* dead-reckoning position (updated by nav) */
    float    pos_x;
    float    pos_y;

    /* timestamp of last successful read for each subsystem */
    uint32_t gas_timestamp_ms;
    uint32_t imu_timestamp_ms;
    uint32_t uss_timestamp_ms;
} sensor_data_t;

/*
 * global sensor state -- extern because multiple tasks
 * need access. protected by g_sensor_mutex.
 */
extern sensor_data_t      g_sensors;
extern SemaphoreHandle_t  g_sensor_mutex;

void sensors_init(void);
void sensors_lock(void);
void sensors_unlock(void);
void sensors_copy(sensor_data_t *dst);

#endif /* SENSORS_H */
