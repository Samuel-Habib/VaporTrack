/*
 * stm32f4xx_hal_msp.c - MSP (MCU Support Package) initialization
 *
 * Handles low-level peripheral clock enables, GPIO alternate function
 * mapping, and NVIC configuration for each HAL peripheral.
 */

#include "main.h"

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

void HAL_MspInit(void)
{
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();

    HAL_NVIC_SetPriority(PendSV_IRQn, 15, 0);
}

/* ------------------------------------------------------------------ */
/* I2C MSP                                                             */
/* ------------------------------------------------------------------ */

void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (hi2c->Instance == I2C1) {
        __HAL_RCC_GPIOB_CLK_ENABLE();

        /* PB6 = I2C1_SCL, PB7 = I2C1_SDA */
        GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        __HAL_RCC_I2C1_CLK_ENABLE();
    }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1) {
        __HAL_RCC_I2C1_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6);
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_7);
    }
}

/* ------------------------------------------------------------------ */
/* TIM PWM MSP                                                         */
/* ------------------------------------------------------------------ */

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim_pwm)
{
    if (htim_pwm->Instance == TIM1) {
        __HAL_RCC_TIM1_CLK_ENABLE();
    } else if (htim_pwm->Instance == TIM2) {
        __HAL_RCC_TIM2_CLK_ENABLE();
    }
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (htim->Instance == TIM1) {
        __HAL_RCC_GPIOA_CLK_ENABLE();

        /* PA8 = TIM1_CH1 (ENA), PA10 = TIM1_CH3 (ENB) */
        GPIO_InitStruct.Pin = ENA_Pin | ENB_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    } else if (htim->Instance == TIM2) {
        __HAL_RCC_GPIOA_CLK_ENABLE();

        /* PA5 = TIM2_CH1 (LED) */
        GPIO_InitStruct.Pin = LED_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
        HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);
    }
}

void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef *htim_pwm)
{
    if (htim_pwm->Instance == TIM1) {
        __HAL_RCC_TIM1_CLK_DISABLE();
    } else if (htim_pwm->Instance == TIM2) {
        __HAL_RCC_TIM2_CLK_DISABLE();
    }
}

/* ------------------------------------------------------------------ */
/* TIM Base MSP (for TIM4 microsecond counter)                         */
/* ------------------------------------------------------------------ */

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM4) {
        __HAL_RCC_TIM4_CLK_ENABLE();
    }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM4) {
        __HAL_RCC_TIM4_CLK_DISABLE();
    }
}
