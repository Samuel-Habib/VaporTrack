/*
 * hcsr04.h - HC-SR04 ultrasonic distance sensor driver
 *
 * Three sensors: front, left, right. We trigger each one
 * sequentially (not simultaneously -- the echoes would
 * interfere) and measure the echo pulse width with a
 * free-running microsecond timer.
 *
 * The measurement is blocking within the calling task
 * context, but since UltrasonicTask runs at its own
 * priority this doesn't starve anyone.
 */

#ifndef HCSR04_H
#define HCSR04_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* measurement timeout -- anything past ~4m is meaningless indoors */
#define HCSR04_TIMEOUT_US       25000
#define HCSR04_MAX_DIST_CM      400.0f

/* which sensor */
typedef enum {
    HCSR04_FRONT = 0,
    HCSR04_LEFT  = 1,
    HCSR04_RIGHT = 2,
    HCSR04_COUNT
} hcsr04_id_t;

typedef struct {
    GPIO_TypeDef *trig_port;
    uint16_t      trig_pin;
    GPIO_TypeDef *echo_port;
    uint16_t      echo_pin;
} hcsr04_hw_t;

typedef struct {
    hcsr04_hw_t   hw[HCSR04_COUNT];
    TIM_HandleTypeDef *htim_us;     /* microsecond timebase */
    float         distance_cm[HCSR04_COUNT];
} hcsr04_dev_t;

void  hcsr04_init(hcsr04_dev_t *dev, TIM_HandleTypeDef *htim_us);
float hcsr04_measure(hcsr04_dev_t *dev, hcsr04_id_t id);
void  hcsr04_measure_all(hcsr04_dev_t *dev);

#endif /* HCSR04_H */
