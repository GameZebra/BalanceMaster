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
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
uint8_t reg = 0x0F | 0x80;  // WHO_AM_I register (0x0F) with read bit (0x80)
uint8_t id = 0x0;
uint8_t accReg [6] = {0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D};
int16_t accX = 0;
int16_t accY = 0;
int16_t accZ = 0;
double accValues[3];

uint8_t isEnabled = 0;
uint8_t enableAcc[2] = {0x20, 0x67};
uint8_t enableDRY[2] = {0x23, 0xC8};
uint8_t status = 0;
uint8_t data[6];
uint8_t const range = 2;


double angleData, angleValue;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
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
  /* USER CODE BEGIN 2 */


  HAL_Delay(100);
  // which is my accelerometer?
  //ACCEL_CS_LOW();                  // Enable SPI communication
  //HAL_SPI_Transmit(&hspi1, &reg, 1, HAL_MAX_DELAY);  // Send register address
  //HAL_SPI_Receive(&hspi1, &id, 1, HAL_MAX_DELAY);    // Receive the ID
  //ACCEL_CS_HIGH();

  uint8_t msg = (enableAcc[0] | 0x80);
  ACCEL_CS_LOW();
  HAL_SPI_Transmit(&hspi1, &msg, 1, HAL_MAX_DELAY);
  HAL_SPI_Receive(&hspi1, &isEnabled, 1, HAL_MAX_DELAY);
  ACCEL_CS_HIGH();

  //initial setup
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
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	msg = 0x27 & 0x3F;
	ACCEL_CS_LOW();
	HAL_SPI_Transmit(&hspi1, &msg, 1, 10);
	HAL_SPI_Receive(&hspi1, &status, 1, 10);
	ACCEL_CS_HIGH();
	HAL_Delay(10);
	if ((status & 0x4) != 0x4){
		continue;
	}
	for(short i = 0; i<6; i++){
		msg = accReg[i] | 0x80;
		ACCEL_CS_LOW();                  // Enable SPI communication
		HAL_SPI_Transmit(&hspi1, &msg, 1, HAL_MAX_DELAY);  // Send register address
		HAL_SPI_Receive(&hspi1, &data[i], 1, HAL_MAX_DELAY);    // Receive the ID
		ACCEL_CS_HIGH();                 // Disable SPI
		HAL_Delay(10);
	}

	accX = (data[0] | (data[1] << 8));
	accY = (data[2] | (data[3] << 8));
	accZ = (data[4] | (data[5] << 8));

	accValues[0] = (accX / 32767.0) * 2;	//measured value / max value * max measurement value
	accValues[1] = (accY / 32767.0) * 2;
	accValues[2] = (accZ / 32767.0) * 2;

	angleData = atan(accX/accZ)*180/M_PI; // in degrees
	angleValue = atan(accValues[0]/accValues[2])*180/M_PI;

	HAL_Delay(10);

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(chipSelect_GPIO_Port, chipSelect_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : chipSelect_Pin */
  GPIO_InitStruct.Pin = chipSelect_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(chipSelect_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PE0 PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
