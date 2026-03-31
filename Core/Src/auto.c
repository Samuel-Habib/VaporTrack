#include "auto.h"
// move forward

int forward(void) {

  HAL_GPIO_WritePin(GPIOA, ENA_Pin | ENB_Pin, GPIO_PIN_SET);
  // Set Direction: IN1=High, IN2=Low | IN3=High, IN4=Low
  HAL_GPIO_WritePin(GPIOB, IN1_Pin | IN3_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, IN2_Pin | IN4_Pin, GPIO_PIN_RESET);

  return 0;
}

int stop(void) {
  HAL_GPIO_WritePin(GPIOA, ENA_Pin | ENB_Pin, GPIO_PIN_RESET);
  return 0;
}
