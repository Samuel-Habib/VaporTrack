/*
 * motor.c - L298N differential drive control
 *
 * Left side:  IN1 (PB10) + IN2 (PB4), ENA on TIM1_CH1
 * Right side: IN3 (PB5)  + IN4 (PB3), ENB on TIM1_CH3
 *
 * Positive speed = forward, negative = reverse.
 */

#include "motor.h"
#include "main.h"

static inline int16_t clamp(int16_t val, int16_t lo, int16_t hi)
{
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

static inline uint16_t abs16(int16_t v)
{
    return (v < 0) ? (uint16_t)(-v) : (uint16_t)v;
}

void motor_init(motor_dev_t *dev, TIM_HandleTypeDef *htim)
{
    dev->htim = htim;
    motor_stop(dev);
}

void motor_set(motor_dev_t *dev, int16_t left, int16_t right)
{
    left  = clamp(left,  -MOTOR_PWM_MAX, MOTOR_PWM_MAX);
    right = clamp(right, -MOTOR_PWM_MAX, MOTOR_PWM_MAX);

    /* left side direction */
    if (left >= 0) {
        HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_SET);
    }

    /* right side direction */
    if (right >= 0) {
        HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_SET);
    }

    /* set PWM duty */
    __HAL_TIM_SET_COMPARE(dev->htim, TIM_CHANNEL_1, abs16(left));
    __HAL_TIM_SET_COMPARE(dev->htim, TIM_CHANNEL_3, abs16(right));
}

void motor_stop(motor_dev_t *dev)
{
    /* coast: set PWM to 0 and release direction pins */
    __HAL_TIM_SET_COMPARE(dev->htim, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(dev->htim, TIM_CHANNEL_3, 0);

    HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);
}

