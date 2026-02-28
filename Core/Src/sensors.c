/*
 * sensors.c - central sensor state management
 */

#include "sensors.h"
#include <string.h>

sensor_data_t     g_sensors;
SemaphoreHandle_t g_sensor_mutex;

void sensors_init(void)
{
    memset(&g_sensors, 0, sizeof(g_sensors));
    g_sensor_mutex = xSemaphoreCreateMutex();
    configASSERT(g_sensor_mutex != NULL);
}

void sensors_lock(void)
{
    xSemaphoreTake(g_sensor_mutex, portMAX_DELAY);
}

void sensors_unlock(void)
{
    xSemaphoreGive(g_sensor_mutex);
}

void sensors_copy(sensor_data_t *dst)
{
    sensors_lock();
    memcpy(dst, &g_sensors, sizeof(sensor_data_t));
    sensors_unlock();
}
