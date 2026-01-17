/*
 * hcsr04.c - HC-SR04 ultrasonic distance measurement
 *
 * Each sensor gets triggered individually with a 10us pulse,
 * then we busy-wait for the echo pin to go high and measure
 * the pulse width using the microsecond timer. Sequential
 * triggering avoids cross-talk between the three sensors.
 */

#include "hcsr04.h"

/* speed of sound ~343 m/s at 20C -> 0.0343 cm/us -> distance = us/58 */
#define US_TO_CM(us) ((float)(us) / 58.0f)

/*
 * read the microsecond timer counter. TIM4 is configured
 * as a free-running 1MHz counter (prescaler = sysclk/1M - 1).
 */
static inline uint32_t us_now(hcsr04_dev_t *dev)
{
    return __HAL_TIM_GET_COUNTER(dev->htim_us);
}

static inline uint32_t us_elapsed(uint32_t start, uint32_t now, uint32_t period)
{
    if (now >= start)
        return now - start;
    return (period - start) + now + 1;
}

void hcsr04_init(hcsr04_dev_t *dev, TIM_HandleTypeDef *htim_us)
{
    dev->htim_us = htim_us;
    for (int i = 0; i < HCSR04_COUNT; i++)
        dev->distance_cm[i] = -1.0f;

    HAL_TIM_Base_Start(htim_us);
}

float hcsr04_measure(hcsr04_dev_t *dev, hcsr04_id_t id)
{
    if (id >= HCSR04_COUNT)
        return -1.0f;

    const hcsr04_hw_t *hw = &dev->hw[id];
    uint32_t period = __HAL_TIM_GET_AUTORELOAD(dev->htim_us);

    /* make sure trigger is low */
    HAL_GPIO_WritePin(hw->trig_port, hw->trig_pin, GPIO_PIN_RESET);

    /* small gap to let any previous echo decay */
    uint32_t t0 = us_now(dev);
    while (us_elapsed(t0, us_now(dev), period) < 4)
        ;

    /* 10us trigger pulse */
    HAL_GPIO_WritePin(hw->trig_port, hw->trig_pin, GPIO_PIN_SET);
    t0 = us_now(dev);
    while (us_elapsed(t0, us_now(dev), period) < 10)
        ;
    HAL_GPIO_WritePin(hw->trig_port, hw->trig_pin, GPIO_PIN_RESET);

    /* wait for echo to go high (start of return pulse) */
    t0 = us_now(dev);
    while (HAL_GPIO_ReadPin(hw->echo_port, hw->echo_pin) == GPIO_PIN_RESET) {
        if (us_elapsed(t0, us_now(dev), period) > HCSR04_TIMEOUT_US) {
            dev->distance_cm[id] = -1.0f;
            return -1.0f;
        }
    }

    /* measure echo pulse width */
    uint32_t echo_start = us_now(dev);
    while (HAL_GPIO_ReadPin(hw->echo_port, hw->echo_pin) == GPIO_PIN_SET) {
        if (us_elapsed(echo_start, us_now(dev), period) > HCSR04_TIMEOUT_US) {
            dev->distance_cm[id] = HCSR04_MAX_DIST_CM;
            return HCSR04_MAX_DIST_CM;
        }
    }
    uint32_t echo_end = us_now(dev);

    uint32_t pulse_us = us_elapsed(echo_start, echo_end, period);
    float dist = US_TO_CM(pulse_us);

    if (dist > HCSR04_MAX_DIST_CM)
        dist = HCSR04_MAX_DIST_CM;

    dev->distance_cm[id] = dist;
    return dist;
}

