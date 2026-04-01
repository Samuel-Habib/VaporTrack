#include "auto.h"
#include "main.h"
#include "stm32f4xx_hal_gpio.h"
// move forward

int forward(void) {

  //  HAL_GPIO_WritePin(GPIOA, ENA_Pin | ENB_Pin, GPIO_PIN_SET);
  // Set Direction FORWWARD: IN1=High, IN2=Low | IN3=High, IN4=Low
  HAL_GPIO_WritePin(GPIOB, IN1_Pin | IN3_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, IN2_Pin | IN4_Pin, GPIO_PIN_RESET);

  return 0;
}

int reverse(void) {

  // HAL_GPIO_WritePin(GPIOA, ENA_Pin | ENB_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, IN1_Pin | IN3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, IN2_Pin | IN4_Pin, GPIO_PIN_SET);

  return 0;
}

int turn_right(void) {

  HAL_GPIO_WritePin(GPIOB, IN1_Pin | IN4_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, IN2_Pin | IN3_Pin, GPIO_PIN_RESET);

  return 0;
}

int turn_left(void) {

  HAL_GPIO_WritePin(GPIOB, IN2_Pin | IN3_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, IN1_Pin | IN4_Pin, GPIO_PIN_RESET);

  return 0;
}



int stop(void) {
  HAL_GPIO_WritePin(GPIOA, ENA_Pin | ENB_Pin, GPIO_PIN_RESET);
  return 0;
}
