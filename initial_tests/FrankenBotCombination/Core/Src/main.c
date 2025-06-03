/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "qik_2s12v10_lib.h"
#include "math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ACCEL_CS_LOW()   HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET)  // Chip Select LOW
#define ACCEL_CS_HIGH()  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET)    // Chip Select HIGH

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// Motor driver commands (UART)
uint8_t const motor0[3] = {m0Forward, m0Brake, m0Reverse};	// the left motor
uint8_t const motor1[3] = {m1Reverese, m1Brake, m1Forward}; // the right motor
uint8_t rotation = 0;
uint8_t rotationOld = 0;
uint8_t speed = 0;
uint8_t brake = 127;

// PID constants
float Kp =  40.0f;
float Ki =  0.0f;
float Kd = 0.30f;

//float integralMax = 127/Ki;

// target
float setpoint = -1.230;
float setPointDelta = 0.01;
uint8_t isReady = 1;
float leftSetup[100];
float previousAngle = 0;
float minD = 100;
float minDAngle[5] = {0, 0, 0, 0, 0};
int8_t sign = 1;
int8_t signOld = 1;
uint8_t signChanges = 0;
uint8_t moved = 0;
// State variables
float error = 0.0f;
float previous_error = 0.0f;
float integral = 0.0f;
float derivative = 0.0f;
float output = 0.0f;
float outputOld = 0.0f;
uint8_t dirChange = 0;


// Sample time
float dt = 0.005f;  // 10 ms


// Gyro
uint16_t gyroValue = 0;
float angularVelocity = 0;
float measuredVoltage = 0;
float gyroAngle = 0;

// Accelerometer
uint8_t reg = 0x0F | 0x80;  // WHO_AM_I register (0x0F) with read bit (0x80)
uint8_t id = 0x0;
uint8_t accReg [6] = {0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D};
int16_t accX = 0;
int16_t accY = 0;
int16_t accZ = 0;
float accValues[3];
float accAngle = 0;
uint8_t simpleNum = 10;
float simpleAvgAngle[10];

uint8_t isEnabled = 0;
uint8_t enableAcc[2] = {0x20, 0x67};
uint8_t enableDRY[2] = {0x23, 0xC8};
uint8_t status = 0;
uint8_t data[6];
uint8_t const range = 2;

// debug
float angle = 0;
float dMAX = 0;


// encoders
int16_t encoderL = 0;
int16_t encoderR = 0;

int16_t encoderLOld = 0;
int16_t encoderROld = 0;
int16_t encoderLSpeed = 0;
int16_t encoderRSpeed = 0;
int16_t encoderLSpeedMax = 0;
int16_t encoderRSpeedMax = 0;
int16_t encoderLSpeedMin = 0;
int16_t encoderRSpeedMin = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM8_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
static void MX_TIM11_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_TIM10_Init();
  MX_TIM8_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_TIM11_Init();
  /* USER CODE BEGIN 2 */


  // Accelerometer init
  uint8_t isEnabled = 0;
  uint8_t enableAcc[2] = {0x20, 0x67};
  uint8_t enableDRY[2] = {0x23, 0xC8};



  uint8_t msg = (enableAcc[0] | 0x80);
  ACCEL_CS_LOW();
  msg = (enableAcc[0] & 0x3F);
  HAL_SPI_Transmit(&hspi1, &msg, 1, HAL_MAX_DELAY);
  HAL_SPI_Transmit(&hspi1, &enableAcc[1], 1, HAL_MAX_DELAY);
  ACCEL_CS_HIGH();
  HAL_Delay(10);

  msg = (enableAcc[0] | 0x80);
  ACCEL_CS_LOW();
  HAL_SPI_Transmit(&hspi1, &msg, 1, HAL_MAX_DELAY);
  HAL_SPI_Receive(&hspi1, &isEnabled, 1, HAL_MAX_DELAY);
  ACCEL_CS_HIGH();
  HAL_Delay(10);


  msg = enableDRY[0] & 0x3F;
  ACCEL_CS_LOW();
  HAL_SPI_Transmit(&hspi1, &msg, 1, HAL_MAX_DELAY);
  HAL_SPI_Transmit(&hspi1, &enableDRY[1], 1, HAL_MAX_DELAY);
  ACCEL_CS_HIGH();                 // Disable
  HAL_Delay(10);

  HAL_Delay(100); // possibly not enough time for the UART to set up

  // initial angle
  readAccelerometer();
  gyroAngle = accAngle;
  for(int i = 0; i< simpleNum; i++){
	  simpleAvgAngle[i] = accAngle;
  }
  //angleInit();


  // UART Motor Driver communication
  __HAL_TIM_SET_COMPARE(&htim10,TIM_CHANNEL_1, 10);
  HAL_TIM_Base_Start_IT(&htim10);
  __HAL_TIM_SET_COMPARE(&htim11,TIM_CHANNEL_1, 500);
  HAL_TIM_Base_Start_IT(&htim11);
  //HAL_Delay(400); // offsetting the tiers

  // ADC gyro read
  //__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 100);
  HAL_TIM_Base_Start(&htim8);
  HAL_ADC_Start_IT(&hadc1);

  // encoder begin
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_2);

  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_2);



  /* USER CODE END 2 */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T8_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 8;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 1000;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 8;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 5000;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 8000;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 1000;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PE0 PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_TIM_PeriodElapsedCallback could be implemented in the user file
   */

  // calculations
  // pid

  if (htim == &htim11) {
	  //gyroAngle = angle;
  }
  if (htim == &htim10){
	  	if(dirChange == 0 && moved >=10){
	  		if(rotation == 2){
		  		sign = -1;
	  		}
	  		else if(rotation == 0){
		  		sign = +1;
	  		}
	  		moved = 0;
		  	targetUpdate();
		  	signOld = sign;
	  	}
		readAccelerometer();
		getEncoders();
		calculateSpeed();

		 if(rotation != rotationOld){
			 dirChange = 1;
			 moved =0;
		 }
		 else{
			 //dirChange = 0;
			 if(abs((int)error) < 0.2){
				 moved++;
			 }

		 }
		 if(dirChange==1){
			 if(encoderRSpeed > 600){
				HAL_UART_Transmit(&huart2, &motor0[1], 1, 20);
				HAL_UART_Transmit(&huart2, &brake, 1, 20);
				HAL_UART_Transmit(&huart2, &motor1[1], 1, 20);
				HAL_UART_Transmit(&huart2, &brake, 1, 20);
			 }
			 else{
				 dirChange = 0;
				 rotationOld = rotation;
			 }
		 }
		if(dirChange == 0){
			HAL_UART_Transmit(&huart2, &motor0[rotation], 1, 20);
			HAL_UART_Transmit(&huart2, &speed, 1, 20);
			// HAL_Delay(10);
			HAL_UART_Transmit(&huart2, &motor1[rotation], 1, 20);
			HAL_UART_Transmit(&huart2, &speed, 1, 20);
			//HAL_Delay(10);
		}

  }

}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hadc);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_ADC_ConvCpltCallback could be implemented in the user file
   */


  gyroValue = HAL_ADC_GetValue(&hadc1);
  calculateGyroAngle();
}

void readAccelerometer(){
	uint8_t msg;
	for(short i = 0; i<6; i++){
		msg = accReg[i] | 0x80;
		ACCEL_CS_LOW();                  // Enable SPI communication
		HAL_SPI_Transmit(&hspi1, &msg, 1, HAL_MAX_DELAY);  // Send register address
		HAL_SPI_Receive(&hspi1, &data[i], 1, HAL_MAX_DELAY);    // Receive the ID
		ACCEL_CS_HIGH();                 // Disable SPI

	}
	accX = (data[0] | (data[1] << 8));
	accY = (data[2] | (data[3] << 8));
	accZ = (data[4] | (data[5] << 8));

	accValues[0] = (accX / 32767.0) * 2;
	accValues[1] = (accY / 32767.0) * 2;
	accValues[2] = (accZ / 32767.0) * 2;

	accAngle = atan(accValues[0]/accValues[2])*180/M_PI;

	// moving average
	for(int i = 0; i< simpleNum; i++){
	  simpleAvgAngle[i] = simpleAvgAngle[i+1];
	}
	simpleAvgAngle[simpleNum-1] = accAngle;
	angle =0;
	for(int i = 0; i< simpleNum; i++){
	  angle += simpleAvgAngle[i];
	}
	angle /= simpleNum;
}

void calculateGyroAngle(){
	measuredVoltage = (gyroValue / 4095.0)*2.9;
	angularVelocity = (measuredVoltage - 1.47)/0.01375/2; // degrees/ time
	if (angularVelocity > 0.7 || angularVelocity < -0.7){
		gyroAngle -= angularVelocity * 0.001; // 1/1000 s
		// - because the positive and negative sides of the Acc and Gyro are different
	}
}

void calculateSpeed(){
	gyroAngle = angle;
	error = setpoint - gyroAngle;
	integral += error * dt;
	derivative = (gyroAngle - previousAngle) / dt;
	output = Kp * error + Ki * integral + Kd * derivative;
	previous_error = error;
	previousAngle = angle;

	if (error<-0.0){
		rotation = 2;
	}
	else if (error>0.0){
		rotation = 0;
	}
	else{
		rotation = 1;
	}


	if (output<-127){
		output = -127;
	}
	else if (output>127){
		output = 127;
	}
	speed = abs((int)output);

	if (dMAX < derivative){
		dMAX = derivative;
	}
}

void getEncoders(){
	  encoderL = __HAL_TIM_GET_COUNTER(&htim2);
	  encoderR = __HAL_TIM_GET_COUNTER(&htim3);

	  encoderLSpeed = (encoderL-encoderLOld)/dt;
	  encoderLOld = encoderL;

	  encoderRSpeed = (encoderR-encoderROld)/dt;
	  encoderROld = encoderR;

	  if (encoderLSpeed > encoderLSpeedMax){
	 	  encoderLSpeedMax = encoderLSpeed;
	   }else if(encoderLSpeed < encoderLSpeedMin){
	 	  encoderLSpeedMin = encoderLSpeed;
	   }

	   if (encoderRSpeed > encoderRSpeedMax){
	 	  encoderRSpeedMax = encoderRSpeed;
	   }else if(encoderRSpeed < encoderRSpeedMin){
	 	  encoderRSpeedMin = encoderRSpeed;
	   }
}


void targetUpdate(){
	setpoint += setPointDelta*sign;
	if(signOld != sign){
		signChanges++;
		if(signChanges > 5 && error < 0.2 && speed < 20){
			setPointDelta *= 0.8f;
			signChanges = 0;
		}

	}
}


void angleInit(){
	float D = 0;
	uint8_t counter = 0;
	uint8_t flag = 0;
	for(int i = 0; i < 100; i++){
		leftSetup[i] = 0;
	}
	while(isReady){
		readAccelerometer();
		//for(int i = 0; i < 99; i++){
		//	leftSetup[i] = leftSetup[i+1];
		//}
		//leftSetup[99] = angle;
		D = leftSetup[98] - angle;
		if (fabs(D) < fabs(minD) && fabs(angle) < 3){
			minD = D;
			minDAngle[counter] = angle;
			flag = 1;
		}
		if (fabs(angle) > 3 && flag){
			counter++;
			flag = 0;
			minD=100;
		}
		if(minDAngle[4]){
			isReady = 0;
		}
		HAL_Delay(30);
	}
	setpoint = 0;
	for(int i = 0; i<5; i++){
		setpoint += minDAngle[i];
	}
	setpoint /= 5;
}

/* USER CODE END 4 */

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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
