/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "CANSPI.h"
#include "MPU9250.h"
#include <math.h>
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
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

/* USER CODE BEGIN PV */
uint8_t irq_mcu9250 = 0;

int32_t ACCEL_X = 0;
int32_t ACCEL_Y = 0;
int32_t ACCEL_Z = 0;

int32_t ACCEL_XANGLE = 0;
int32_t ACCEL_YANGLE = 0;
int32_t ACCEL_ZANGLE = 0;

uCAN_MSG txMessage;
uCAN_MSG rxMessage;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
void PhysVectorsToAccel();
void AccelToAngel();
void SerializeData(char* buff, int16_t val1, int16_t val2, int16_t val3);
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
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  int st;
  st = MPU_begin();
  HAL_Delay(100);
  enableDataReadyInterrupt();
  HAL_Delay(100);
  setSrd(0);

  CANSPI_Initialize();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(irq_mcu9250 == 1) {
		  irq_mcu9250 = 0;


		  if(CANSPI_Receive(&rxMessage)) { //if received request frame
			  AccelToAngel(); //get angels in global vars

			if(rxMessage.frame.id == 0x12) { //id frame that request ACCEL_XANGLE
				txMessage.frame.idType = rxMessage.frame.idType;
				txMessage.frame.id = rxMessage.frame.id;
				txMessage.frame.dlc = rxMessage.frame.dlc;
				txMessage.frame.data0 = (ACCEL_XANGLE >> 24) & 0xFF;
				txMessage.frame.data1 = (ACCEL_XANGLE >> 16) & 0xFF;
				txMessage.frame.data2 = (ACCEL_XANGLE >> 8) & 0xFF;
				txMessage.frame.data3 = ACCEL_XANGLE & 0xFF;
				txMessage.frame.data4 = 0;
				txMessage.frame.data5 = 0;
				txMessage.frame.data6 = 0;
				txMessage.frame.data7 = 0;
				CANSPI_Transmit(&txMessage);

			} else if(rxMessage.frame.id == 0x13) { //id frame that request ACCEL_YANGLE
				txMessage.frame.idType = rxMessage.frame.idType;
				txMessage.frame.id = rxMessage.frame.id;
				txMessage.frame.dlc = rxMessage.frame.dlc;
				txMessage.frame.data0 = (ACCEL_YANGLE >> 24) & 0xFF;
				txMessage.frame.data1 = (ACCEL_YANGLE >> 16) & 0xFF;
				txMessage.frame.data2 = (ACCEL_YANGLE >> 8) & 0xFF;
				txMessage.frame.data3 = ACCEL_YANGLE & 0xFF;
				txMessage.frame.data4 = 0;
				txMessage.frame.data5 = 0;
				txMessage.frame.data6 = 0;
				txMessage.frame.data7 = 0;
				CANSPI_Transmit(&txMessage);

			} else if(rxMessage.frame.id == 0x14) { //id frame that request ACCEL_ZANGLE
				txMessage.frame.idType = rxMessage.frame.idType;
				txMessage.frame.id = rxMessage.frame.id;
				txMessage.frame.dlc = rxMessage.frame.dlc;
				txMessage.frame.data0 = (ACCEL_ZANGLE >> 24) & 0xFF;
				txMessage.frame.data1 = (ACCEL_ZANGLE >> 16) & 0xFF;
				txMessage.frame.data2 = (ACCEL_ZANGLE >> 8) & 0xFF;
				txMessage.frame.data3 = ACCEL_ZANGLE & 0xFF;
				txMessage.frame.data4 = 0;
				txMessage.frame.data5 = 0;
				txMessage.frame.data6 = 0;
				txMessage.frame.data7 = 0;
				CANSPI_Transmit(&txMessage);
			}
		  }


		  //Send 3-axys accel in package every 800ms
		  PhysVectorsToAccel(); //get accel in global vars

		  //Package 1. Transmit ACCEL_X
		  txMessage.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
		  txMessage.frame.id = 0x82; //id packages with accel
		  txMessage.frame.dlc = 8;
		  txMessage.frame.data0 = (ACCEL_X >> 24) & 0xFF;
		  txMessage.frame.data1 = (ACCEL_X >> 16) & 0xFF;
		  txMessage.frame.data2 = (ACCEL_X >> 8) & 0xFF;
		  txMessage.frame.data3 = ACCEL_X & 0xFF;
		  txMessage.frame.data4 = 0;
		  txMessage.frame.data5 = 0;
		  txMessage.frame.data6 = 0;
		  txMessage.frame.data7 = 0;
		  CANSPI_Transmit(&txMessage);

		  //Package 2. Transmit ACCEL_Y
		  txMessage.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
		  txMessage.frame.id = 0x83; //id packages with accel
		  txMessage.frame.dlc = 8;
		  txMessage.frame.data0 = (ACCEL_Y >> 24) & 0xFF;
		  txMessage.frame.data1 = (ACCEL_Y >> 16) & 0xFF;
		  txMessage.frame.data2 = (ACCEL_Y >> 8) & 0xFF;
		  txMessage.frame.data3 = ACCEL_Y & 0xFF;
		  txMessage.frame.data4 = 0;
		  txMessage.frame.data5 = 0;
		  txMessage.frame.data6 = 0;
		  txMessage.frame.data7 = 0;
		  CANSPI_Transmit(&txMessage);

		  //Package 3. Transmit ACCEL_Z
		  txMessage.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
		  txMessage.frame.id = 0x84; //id packages with accel
		  txMessage.frame.dlc = 8;
		  txMessage.frame.data0 = (ACCEL_Z >> 24) & 0xFF;
		  txMessage.frame.data1 = (ACCEL_Z >> 16) & 0xFF;
		  txMessage.frame.data2 = (ACCEL_Z >> 8) & 0xFF;
		  txMessage.frame.data3 = ACCEL_Z & 0xFF;
		  txMessage.frame.data4 = 0;
		  txMessage.frame.data5 = 0;
		  txMessage.frame.data6 = 0;
		  txMessage.frame.data7 = 0;
		  CANSPI_Transmit(&txMessage);

		  HAL_Delay(800);
	  }
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
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CAN_CS_GPIO_Port, CAN_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : CAN_CS_Pin */
  GPIO_InitStruct.Pin = CAN_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(CAN_CS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

}

/* USER CODE BEGIN 4 */
void PhysVectorsToAccel() {
	 readSensor(); //get data from registers
	 ACCEL_X = _axcounts * 256 + _gxcounts;
	 ACCEL_Y = _aycounts * 256 + _gycounts;
	 ACCEL_Z = _azcounts * 256 + _gzcounts;
}

void AccelToAngel() {
	 PhysVectorsToAccel();

	 ACCEL_XANGLE = 57.295* atan((float)-ACCEL_X/ sqrt(pow((float)ACCEL_Y, 2)+ pow((float)ACCEL_Z, 2)));
	 ACCEL_YANGLE = 57.295* atan((float)-ACCEL_Y/ sqrt(pow((float)ACCEL_X, 2)+ pow((float)ACCEL_Z, 2)));
	 ACCEL_ZANGLE = 57.295* atan((float)-ACCEL_Z/ sqrt(pow((float)ACCEL_X, 2)+ pow((float)ACCEL_Y, 2)));
}

void SerializeData(char* buff, int16_t val1, int16_t val2, int16_t val3) {

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin==GPIO_PIN_1) //PB1 EXTI1 (from MPU9250)
	{
		irq_mcu9250 = 1;
	}
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
