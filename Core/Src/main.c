/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body for VaporTrack Autonomous Rover
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cmsis_os2.h"
#include "bme688.h"
#include "bno055.h"
#include "hcsr04.h"
#include "ssd1306.h"
#include "motor.h"
#include "sensors.h"
#include "mapping.h"
#include "navigation.h"

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

/* Definitions for SensorTask */
osThreadId_t SensorTaskHandle;
const osThreadAttr_t SensorTask_attributes = {
  .name = "SensorTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for UltrasonicTask */
osThreadId_t UltrasonicTaskHandle;
const osThreadAttr_t UltrasonicTask_attributes = {
  .name = "USSTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for NavTask */
osThreadId_t NavTaskHandle;
const osThreadAttr_t NavTask_attributes = {
  .name = "NavTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for MotorTask */
osThreadId_t MotorTaskHandle;
const osThreadAttr_t MotorTask_attributes = {
  .name = "MotorTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for MappingTask */
osThreadId_t MappingTaskHandle;
const osThreadAttr_t MappingTask_attributes = {
  .name = "MapTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for DisplayTask */
osThreadId_t DisplayTaskHandle;
const osThreadAttr_t DisplayTask_attributes = {
  .name = "DispTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* USER CODE BEGIN PV */
static bme688_dev_t   g_bme;
static bno055_dev_t   g_bno;
static hcsr04_dev_t   g_uss;
static ssd1306_dev_t  g_oled;
static motor_dev_t    g_motor;

static gas_map_t      g_map;
static nav_ctx_t      g_nav;

static volatile motor_cmd_t g_motor_cmd;
static SemaphoreHandle_t    g_motor_mutex;
static SemaphoreHandle_t    g_i2c_mutex;

static volatile bool g_bme_ok   = false;
static volatile bool g_bno_ok   = false;
static volatile bool g_oled_ok  = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
void SensorTask_Entry(void *argument);
void UltrasonicTask_Entry(void *argument);
void NavigationTask_Entry(void *argument);
void MotorTask_Entry(void *argument);
void MappingTask_Entry(void *argument);
void DisplayTask_Entry(void *argument);

/* USER CODE BEGIN PFP */
static inline void i2c_lock(void)
{
  if (g_i2c_mutex != NULL)
    xSemaphoreTake(g_i2c_mutex, portMAX_DELAY);
}

static inline void i2c_unlock(void)
{
  if (g_i2c_mutex != NULL)
    xSemaphoreGive(g_i2c_mutex);
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

  sensors_init();
  map_init(&g_map);
  nav_init(&g_nav, &g_map);
  motor_init(&g_motor, &htim1);

  g_motor_mutex = xSemaphoreCreateMutex();
  g_i2c_mutex   = xSemaphoreCreateMutex();
  configASSERT(g_motor_mutex != NULL);
  configASSERT(g_i2c_mutex != NULL);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  SensorTaskHandle = osThreadNew(SensorTask_Entry, NULL, &SensorTask_attributes);
  UltrasonicTaskHandle = osThreadNew(UltrasonicTask_Entry, NULL, &UltrasonicTask_attributes);
  NavTaskHandle = osThreadNew(NavigationTask_Entry, NULL, &NavTask_attributes);
  MotorTaskHandle = osThreadNew(MotorTask_Entry, NULL, &MotorTask_attributes);
  MappingTaskHandle = osThreadNew(MappingTask_Entry, NULL, &MappingTask_attributes);
  DisplayTaskHandle = osThreadNew(DisplayTask_Entry, NULL, &DisplayTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{
  /* USER CODE BEGIN I2C1_Init 0 */
  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */
  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */
  /* USER CODE END I2C1_Init 2 */
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{
  /* USER CODE BEGIN TIM1_Init 0 */
  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */
  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 8999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */
  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{
  /* USER CODE BEGIN TIM2_Init 0 */
  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */
  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);
}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{
  /* USER CODE BEGIN TIM4_Init 0 */
  /* USER CODE END TIM4_Init 0 */

  /* USER CODE BEGIN TIM4_Init 1 */
  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 89;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */
  /* USER CODE END TIM4_Init 2 */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, USS_FRONT_TRIG_Pin|USS_LEFT_TRIG_Pin|USS_RIGHT_TRIG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, IN1_Pin|IN4_Pin|IN2_Pin|IN3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : USS_FRONT_TRIG_Pin USS_LEFT_TRIG_Pin USS_RIGHT_TRIG_Pin */
  GPIO_InitStruct.Pin = USS_FRONT_TRIG_Pin|USS_LEFT_TRIG_Pin|USS_RIGHT_TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : USS_FRONT_ECHO_Pin USS_LEFT_ECHO_Pin USS_RIGHT_ECHO_Pin */
  GPIO_InitStruct.Pin = USS_FRONT_ECHO_Pin|USS_LEFT_ECHO_Pin|USS_RIGHT_ECHO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : IN1_Pin IN4_Pin IN2_Pin IN3_Pin */
  GPIO_InitStruct.Pin = IN1_Pin|IN4_Pin|IN2_Pin|IN3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_SensorTask_Entry */
/**
  * @brief  Function implementing the SensorTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_SensorTask_Entry */
void SensorTask_Entry(void *argument)
{
  /* USER CODE BEGIN 5 */
  (void)argument;

  i2c_lock();
  if (bme688_init(&g_bme, &hi2c1) == BME688_OK)
    g_bme_ok = true;

  if (bno055_init(&g_bno, &hi2c1) == BNO055_OK)
    g_bno_ok = true;
  i2c_unlock();

  for (;;)
  {
    uint32_t now = HAL_GetTick();

    if (g_bme_ok)
    {
      bme688_data_t env;
      i2c_lock();
      bme688_status_t trig_res = bme688_trigger_forced(&g_bme);
      i2c_unlock();

      if (trig_res == BME688_OK)
      {
        /* Yield to RTOS while sensor heater stabilizes (150ms dwell).
         * I2C bus is released so DisplayTask can flush without contention. */
        osDelay(160);

        i2c_lock();
        bme688_status_t read_res = bme688_read_data(&g_bme, &env);
        i2c_unlock();

        if (read_res == BME688_OK)
        {
          sensors_lock();
          g_sensors.gas_resistance   = env.gas_resistance;
          g_sensors.temperature      = env.temperature;
          g_sensors.humidity         = env.humidity;
          g_sensors.pressure         = env.pressure;
          g_sensors.gas_valid        = env.gas_valid;
          g_sensors.gas_heater_stable = env.heat_stab;
          g_sensors.gas_timestamp_ms  = now;
          sensors_unlock();
        }
      }
    }

    if (g_bno_ok)
    {
      bno055_euler_t euler;
      bno055_cal_status_t cal;
      i2c_lock();
      bno055_status_t euler_res = bno055_read_euler(&g_bno, &euler);
      if (euler_res == BNO055_OK)
      {
        bno055_get_calibration(&g_bno, &cal);
      }
      i2c_unlock();

      if (euler_res == BNO055_OK)
      {
        sensors_lock();
        g_sensors.heading_deg    = euler.heading;
        g_sensors.roll_deg       = euler.roll;
        g_sensors.pitch_deg      = euler.pitch;
        g_sensors.cal_sys        = cal.sys;
        g_sensors.cal_gyro       = cal.gyro;
        g_sensors.cal_accel      = cal.accel;
        g_sensors.cal_mag        = cal.mag;
        g_sensors.imu_calibrated = bno055_is_calibrated(&cal);
        g_sensors.imu_timestamp_ms = now;
        sensors_unlock();
      }
    }

    osDelay(100);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_UltrasonicTask_Entry */
/**
* @brief Function implementing the UltrasonicTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UltrasonicTask_Entry */
void UltrasonicTask_Entry(void *argument)
{
  /* USER CODE BEGIN UltrasonicTask_Entry */
  (void)argument;

  hcsr04_init(&g_uss, &htim4);

  g_uss.hw[HCSR04_FRONT].trig_port = USS_FRONT_TRIG_Port;
  g_uss.hw[HCSR04_FRONT].trig_pin  = USS_FRONT_TRIG_Pin;
  g_uss.hw[HCSR04_FRONT].echo_port = USS_FRONT_ECHO_Port;
  g_uss.hw[HCSR04_FRONT].echo_pin  = USS_FRONT_ECHO_Pin;

  g_uss.hw[HCSR04_LEFT].trig_port  = USS_LEFT_TRIG_Port;
  g_uss.hw[HCSR04_LEFT].trig_pin   = USS_LEFT_TRIG_Pin;
  g_uss.hw[HCSR04_LEFT].echo_port  = USS_LEFT_ECHO_Port;
  g_uss.hw[HCSR04_LEFT].echo_pin   = USS_LEFT_ECHO_Pin;

  g_uss.hw[HCSR04_RIGHT].trig_port = USS_RIGHT_TRIG_Port;
  g_uss.hw[HCSR04_RIGHT].trig_pin  = USS_RIGHT_TRIG_Pin;
  g_uss.hw[HCSR04_RIGHT].echo_port = USS_RIGHT_ECHO_Port;
  g_uss.hw[HCSR04_RIGHT].echo_pin  = USS_RIGHT_ECHO_Pin;

  for (;;)
  {
    hcsr04_measure_all(&g_uss);

    sensors_lock();
    g_sensors.dist_front_cm = g_uss.distance_cm[HCSR04_FRONT];
    g_sensors.dist_left_cm  = g_uss.distance_cm[HCSR04_LEFT];
    g_sensors.dist_right_cm = g_uss.distance_cm[HCSR04_RIGHT];
    g_sensors.uss_timestamp_ms = HAL_GetTick();
    sensors_unlock();

    osDelay(100);
  }
  /* USER CODE END UltrasonicTask_Entry */
}

/* USER CODE BEGIN Header_NavigationTask_Entry */
/**
* @brief Function implementing the NavTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_NavigationTask_Entry */
void NavigationTask_Entry(void *argument)
{
  /* USER CODE BEGIN NavigationTask_Entry */
  (void)argument;

  osDelay(500);

  for (;;)
  {
    sensor_data_t snap;
    sensors_copy(&snap);

    nav_sensor_input_t input;
    /* Convert MOX gas resistance (ohms) to relative conductance (uS):
     * As VOC concentration increases, MOX resistance drops, so conductance
     * (1/R) increases proportionally with gas plume intensity. */
    if (snap.gas_resistance > 100.0f) {
      input.gas = 1000000.0f / snap.gas_resistance;
    } else {
      input.gas = 0.0f;
    }
    input.temperature   = snap.temperature;
    input.humidity      = snap.humidity;
    input.pressure      = snap.pressure;
    input.gas_valid     = snap.gas_valid;
    input.heading_deg   = snap.heading_deg;
    input.heading_valid = snap.imu_calibrated;
    input.dist_front_cm = snap.dist_front_cm;
    input.dist_left_cm  = snap.dist_left_cm;
    input.dist_right_cm = snap.dist_right_cm;
    input.pos_x         = g_nav.pos_x;
    input.pos_y         = g_nav.pos_y;

    uint32_t now = HAL_GetTick();
    motor_cmd_t cmd = nav_update(&g_nav, &input, now);

    xSemaphoreTake(g_motor_mutex, portMAX_DELAY);
    g_motor_cmd = cmd;
    xSemaphoreGive(g_motor_mutex);

    sensors_lock();
    g_sensors.pos_x = g_nav.pos_x;
    g_sensors.pos_y = g_nav.pos_y;
    sensors_unlock();

    osDelay(150);
  }
  /* USER CODE END NavigationTask_Entry */
}

/* USER CODE BEGIN Header_MotorTask_Entry */
/**
* @brief Function implementing the MotorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MotorTask_Entry */
void MotorTask_Entry(void *argument)
{
  /* USER CODE BEGIN MotorTask_Entry */
  (void)argument;

  for (;;)
  {
    motor_cmd_t cmd;
    xSemaphoreTake(g_motor_mutex, portMAX_DELAY);
    cmd = g_motor_cmd;
    xSemaphoreGive(g_motor_mutex);

    motor_set(&g_motor, cmd.left, cmd.right);

    uint16_t activity = (uint16_t)(abs(cmd.left) + abs(cmd.right)) / 2;
    if (activity < 500) activity = 500;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, activity);

    osDelay(50);
  }
  /* USER CODE END MotorTask_Entry */
}

/* USER CODE BEGIN Header_MappingTask_Entry */
/**
* @brief Function implementing the MappingTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MappingTask_Entry */
void MappingTask_Entry(void *argument)
{
  /* USER CODE BEGIN MappingTask_Entry */
  (void)argument;

  for (;;)
  {
    sensor_data_t snap;
    sensors_copy(&snap);

    if (g_map.count > 3)
    {
      g_map.gradient = map_estimate_gradient(&g_map, snap.pos_x, snap.pos_y, 1.5f);
    }

    osDelay(500);
  }
  /* USER CODE END MappingTask_Entry */
}

/* USER CODE BEGIN Header_DisplayTask_Entry */
/**
* @brief Function implementing the DisplayTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_DisplayTask_Entry */
void DisplayTask_Entry(void *argument)
{
  /* USER CODE BEGIN DisplayTask_Entry */
  (void)argument;

  i2c_lock();
  if (ssd1306_init(&g_oled, &hi2c1) == SSD1306_OK)
    g_oled_ok = true;
  i2c_unlock();

  char line[22];

  for (;;)
  {
    if (!g_oled_ok)
    {
      osDelay(2000);
      continue;
    }

    sensor_data_t snap;
    sensors_copy(&snap);

    ssd1306_clear(&g_oled);

    snprintf(line, sizeof(line), "%-10s %3d%%",
             nav_state_name(g_nav.state),
             (int)(snap.cal_sys * 33));
    ssd1306_puts(&g_oled, 0, 0, line);

    snprintf(line, sizeof(line), "GAS:%-6d %4.1fC",
             (int)snap.gas_resistance, (double)snap.temperature);
    ssd1306_puts(&g_oled, 0, 8, line);

    snprintf(line, sizeof(line), "H:%4.1f%% P:%5.0f",
             (double)snap.humidity, (double)(snap.pressure / 100.0f));
    ssd1306_puts(&g_oled, 0, 16, line);

    snprintf(line, sizeof(line), "F:%3d L:%3d R:%3d",
             (int)snap.dist_front_cm,
             (int)snap.dist_left_cm,
             (int)snap.dist_right_cm);
    ssd1306_puts(&g_oled, 0, 24, line);

    snprintf(line, sizeof(line), "X:%5.2f Y:%5.2f",
             (double)snap.pos_x, (double)snap.pos_y);
    ssd1306_puts(&g_oled, 0, 32, line);

    snprintf(line, sizeof(line), "HDG:%3d", (int)snap.heading_deg);
    ssd1306_puts(&g_oled, 0, 40, line);

    if (g_map.gradient.valid)
    {
      snprintf(line, sizeof(line), "dG:%+.0f @%3d",
               (double)g_map.gradient.magnitude,
               (int)g_map.gradient.direction_deg);
      ssd1306_puts(&g_oled, 0, 48, line);
    }

    snprintf(line, sizeof(line), "MAP:%d/%d",
             g_map.count, MAP_MAX_SAMPLES);
    ssd1306_puts(&g_oled, 0, 56, line);

    i2c_lock();
    ssd1306_flush(&g_oled);
    i2c_unlock();

    osDelay(500);
  }
  /* USER CODE END DisplayTask_Entry */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  (void)file;
  (void)line;
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
