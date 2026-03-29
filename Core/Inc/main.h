/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* --- Pin Definitions --- */

/* onboard LED */
#define LED_Pin             GPIO_PIN_5
#define LED_GPIO_Port       GPIOA

/* L298N motor direction */
#define IN1_Pin             GPIO_PIN_10
#define IN1_GPIO_Port       GPIOB
#define IN2_Pin             GPIO_PIN_4
#define IN2_GPIO_Port       GPIOB
#define IN3_Pin             GPIO_PIN_5
#define IN3_GPIO_Port       GPIOB
#define IN4_Pin             GPIO_PIN_3
#define IN4_GPIO_Port       GPIOB

/* L298N motor PWM (active on TIM1) */
#define ENA_Pin             GPIO_PIN_8
#define ENA_GPIO_Port       GPIOA
#define ENB_Pin             GPIO_PIN_10
#define ENB_GPIO_Port       GPIOA

/* HC-SR04 ultrasonic sensors */
#define USS_FRONT_TRIG_Pin  GPIO_PIN_0
#define USS_FRONT_TRIG_Port GPIOC
#define USS_FRONT_ECHO_Pin  GPIO_PIN_1
#define USS_FRONT_ECHO_Port GPIOC

#define USS_LEFT_TRIG_Pin   GPIO_PIN_2
#define USS_LEFT_TRIG_Port  GPIOC
#define USS_LEFT_ECHO_Pin   GPIO_PIN_3
#define USS_LEFT_ECHO_Port  GPIOC

#define USS_RIGHT_TRIG_Pin  GPIO_PIN_4
#define USS_RIGHT_TRIG_Port GPIOC
#define USS_RIGHT_ECHO_Pin  GPIO_PIN_5
#define USS_RIGHT_ECHO_Port GPIOC

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
