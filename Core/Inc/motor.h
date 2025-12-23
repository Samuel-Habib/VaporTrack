/*
 * motor.h - L298N differential drive motor control
 *
 * Wraps the existing GPIO direction pins and TIM1 PWM channels
 * into a clean interface. Each "side" (left/right) gets an
 * independent speed + direction command.
 *
 * Pin mapping (from CubeMX config):
 *   ENA = PA8  (TIM1_CH1)  -- left side PWM
 *   ENB = PA10 (TIM1_CH3)  -- right side PWM
 *   IN1 = PB10             -- left fwd
 *   IN2 = PB4              -- left rev
 *   IN3 = PB5              -- right fwd
 *   IN4 = PB3              -- right rev
 */

#ifndef MOTOR_H
#define MOTOR_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* max PWM compare value -- TIM1 ARR is 8999 */
#define MOTOR_PWM_MAX   8999

typedef struct {
    TIM_HandleTypeDef *htim;
} motor_dev_t;

/*
 * speed is signed: positive = forward, negative = reverse.
 * magnitude clamped to MOTOR_PWM_MAX internally.
 */
void motor_init(motor_dev_t *dev, TIM_HandleTypeDef *htim);
void motor_set(motor_dev_t *dev, int16_t left, int16_t right);
void motor_stop(motor_dev_t *dev);
void motor_brake(motor_dev_t *dev);

#endif /* MOTOR_H */
