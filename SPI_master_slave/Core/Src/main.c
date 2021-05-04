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
#include "MPU9250.h"
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RECV_BUFF_SIZE 60
#define NELEMS(x) (sizeof(x) / sizeof((x)[0]))
#define MASTER_CS_RESET HAL_GPIO_WritePin(GPIOC, SPI2_SS_Pin, GPIO_PIN_RESET)
#define MASTER_CS_SET HAL_GPIO_WritePin(GPIOC, SPI2_SS_Pin, GPIO_PIN_SET)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;
DMA_HandleTypeDef hdma_spi3_rx;
DMA_HandleTypeDef hdma_spi3_tx;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
/* DMA flags */
volatile uint8_t irq_mcu9250 = 0;
volatile uint8_t has_command = 0;
volatile uint8_t received_from_master = 0;
volatile uint8_t received_from_slave = 0;
volatile uint8_t flag = 0; //Command flag
volatile uint8_t cycle_flag = 0;
volatile uint8_t Unprocessed_UART_buff_detected = RESET;

uint8_t SPI_recv_buff[RECV_BUFF_SIZE];
uint8_t UART_recv_buff[RECV_BUFF_SIZE]; //Array for storing UART receive data
char response_transmit[RECV_BUFF_SIZE];
char response_receive[RECV_BUFF_SIZE];

/* Wildcards for checking command */
uint8_t get_accelrule[] = {'G','e','t',' ','A','c','c','e','l'};
uint8_t get_angelrule[] = {'G','e','t',' ','A','n','g','e','l'};
uint8_t get_timerule[] = {'S','e','t',' ','t','i','m','e'};
uint8_t return_err[RECV_BUFF_SIZE];
uint8_t args_buff[4] = {'\n','\n','\n','\n'}; //Max length of args: 4.
uint16_t checksum_accelrule = NELEMS(get_accelrule);
uint16_t checksum_angelrule = NELEMS(get_angelrule);
uint16_t checksum_timerule = NELEMS(get_timerule);
uint32_t command_delay = 500;


/* Data from sensor */
int32_t ACCEL_X = 0;
int32_t ACCEL_Y = 0;
int32_t ACCEL_Z = 0;

int32_t ACCEL_XANGLE = 0;
int32_t ACCEL_YANGLE = 0;
int32_t ACCEL_ZANGLE = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void TransmitSensorData();
void PhysVectorsToAccel();
void AccelToAngel();
void ClearBuff(uint8_t* buff);
void GetAccel();
void GetAngel();
void USER_UART_IDLECallback(UART_HandleTypeDef *huart);
void SPISendError();
void SPISendData(uint32_t x_val, uint32_t y_val, uint32_t z_val);
uint8_t ParseCommand(uint8_t* UART_receive);
void ClearBuff(uint8_t* buff);
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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE); //Enble IDLE IRQs
    HAL_UART_Receive_DMA(&huart2, UART_recv_buff, RECV_BUFF_SIZE-1); //Start DMA

    int st;
    st = MPU_begin();
    HAL_Delay(100);
    enableDataReadyInterrupt();
    HAL_Delay(100);
    setSrd(0);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(irq_mcu9250 == 1) {
			irq_mcu9250 = 0;

			//Get data to MCU2 from USART PC (to UART2)
			__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
			HAL_UART_Receive_DMA(&huart2, UART_recv_buff, RECV_BUFF_SIZE-1);

			if(has_command) {
				has_command = 0; //after transmit completed disable flag
				//Ready receiving bytes from SPI2 Master
				MASTER_CS_RESET;
				uint32_t plug = *(uint32_t*)hspi3.Instance->DR; //check hardware buffer SPI3
				HAL_SPI_Receive_DMA(&hspi3, SPI_recv_buff, RECV_BUFF_SIZE-1);
				//Transmit bytes from SPI2 Master to SPI3 Slave
				HAL_SPI_Transmit_DMA(&hspi2, UART_recv_buff, RECV_BUFF_SIZE-1);
			}

			if(received_from_master) {
				received_from_master = 0;
				//Parse received command from SLAVE SPI3 on side MCU1
				cycle_flag = ParseCommand(SPI_recv_buff);
			}

			if(received_from_slave) {
				received_from_slave = 0;

				if(cycle_flag == 1) {
					GetAccel();
					HAL_Delay(command_delay-100);
				} else if(cycle_flag == 2) {
					GetAngel();
					HAL_Delay(command_delay-100);
				}

				//UART terminal print on side MCU2
				HAL_UART_Transmit_DMA(&huart2, (uint8_t*)response_receive, strlen(response_receive));
				HAL_Delay(100);

				/* Clear buffers */
				ClearBuff(UART_recv_buff);
				ClearBuff(SPI_recv_buff);
				ClearBuff(response_transmit);
				ClearBuff(response_receive);
			}
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
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
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_SLAVE;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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
  huart2.Init.BaudRate = 115200;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA1_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI2_SS_GPIO_Port, SPI2_SS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_SS_GPIO_Port, SPI1_SS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SPI2_SS_Pin */
  GPIO_InitStruct.Pin = SPI2_SS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(SPI2_SS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_SS_Pin */
  GPIO_InitStruct.Pin = SPI1_SS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(SPI1_SS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

}

/* USER CODE BEGIN 4 */
void PhysVectorsToAccel() {
         ACCEL_X = _axcounts * 256 + _gxcounts;
         ACCEL_Y = _aycounts * 256 + _gycounts;
         ACCEL_Z = _azcounts * 256 + _gzcounts;
}

void AccelToAngel() {
         ACCEL_XANGLE = 57.295* atan((float)-ACCEL_X/ sqrt(pow((float)ACCEL_Y, 2)+ pow((float)ACCEL_Z, 2)));
         ACCEL_YANGLE = 57.295* atan((float)-ACCEL_Y/ sqrt(pow((float)ACCEL_X, 2)+ pow((float)ACCEL_Z, 2)));
         ACCEL_ZANGLE = 57.295* atan((float)-ACCEL_Z/ sqrt(pow((float)ACCEL_X, 2)+ pow((float)ACCEL_Y, 2)));
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
        if (GPIO_Pin==GPIO_PIN_2) //PB1 EXTI1 (from MPU9250)
        {
                irq_mcu9250 = 1;
        }
}

//Print error about invalid command
void SPISendError() {
	sprintf(return_err, "Invalid command!\r\n");
	HAL_Delay(500);
	//Ready receiving bytes from SPI3 Slave
	MASTER_CS_RESET;
	//Transmit bytes from SPI3 Slave to SPI2 Master
	HAL_SPI_Transmit_DMA(&hspi3, (uint8_t*)return_err, RECV_BUFF_SIZE-1);
	HAL_SPI_Receive_DMA(&hspi2, (uint8_t*)response_receive, RECV_BUFF_SIZE-1);
}

//Determine state machine with 2^3-1 combinations and print results
//for GetAccel() and GetAngel()
void SPISendData(uint32_t x_val, uint32_t y_val, uint32_t z_val) {
	char X = ' ';
	char Y = ' ';
	char Z = ' ';

	X = args_buff[0];
	Y = args_buff[1];
	Z = args_buff[2];

	//State machine with 2^3-1 combinations
	if(X == 'X' && Y == '\n' && Z == '\n') {
		//X
		if(flag == 1) {
			//Accel

			sprintf(response_transmit, "x:%08d rad/sec\r\n", x_val);

			//Ready receiving bytes from SPI3 Slave
			MASTER_CS_RESET;
			//Transmit bytes from SPI3 Slave to SPI2 Master
			HAL_SPI_Transmit_DMA(&hspi3, (uint8_t*)response_transmit, RECV_BUFF_SIZE-1);

			uint32_t plug = *(uint32_t*)hspi2.Instance->DR;
			HAL_SPI_Receive_DMA(&hspi2, (uint8_t*)response_receive, RECV_BUFF_SIZE-1);

		} else if(flag == 2) {
			//Angel

			sprintf(response_transmit, "x:%04d gr\r\n", x_val);

			//Ready receiving bytes from SPI3 Slave
			MASTER_CS_RESET;
			//Transmit bytes from SPI3 Slave to SPI2 Master
			HAL_SPI_Transmit_DMA(&hspi3, (uint8_t*)response_transmit, RECV_BUFF_SIZE-1);

			uint32_t plug = *(uint32_t*)hspi2.Instance->DR;
			HAL_SPI_Receive_DMA(&hspi2, (uint8_t*)response_receive, RECV_BUFF_SIZE-1);

		}

	} else if (X == '\n' && Y == 'Y' && Z == '\n') {
		//Y

		if(flag == 1) {
			//Accel

			sprintf(response_transmit, "y:%08d rad/sec\r\n", y_val);

			//Ready receiving bytes from SPI3 Slave
			MASTER_CS_RESET;
			//Transmit bytes from SPI3 Slave to SPI2 Master
			HAL_SPI_Transmit_DMA(&hspi3, (uint8_t*)response_transmit, RECV_BUFF_SIZE-1);

			uint32_t plug = *(uint32_t*)hspi2.Instance->DR;
			HAL_SPI_Receive_DMA(&hspi2, (uint8_t*)response_receive, RECV_BUFF_SIZE-1);

		} else if(flag == 2) {
			//Angel

			sprintf(response_transmit, "y:%04d gr\r\n", y_val);

			//Ready receiving bytes from SPI3 Slave
			MASTER_CS_RESET;
			//Transmit bytes from SPI3 Slave to SPI2 Master
			HAL_SPI_Transmit_DMA(&hspi3, (uint8_t*)response_transmit, RECV_BUFF_SIZE-1);

			uint32_t plug = *(uint32_t*)hspi2.Instance->DR;
			HAL_SPI_Receive_DMA(&hspi2, (uint8_t*)response_receive, RECV_BUFF_SIZE-1);

		}

	} else if (X == '\n' && Y == '\n' && Z == 'Z') {
		//Z

		if(flag == 1) {
			//Accel

			sprintf(response_transmit, "z:%08d rad/sec\r\n", z_val);

			//Ready receiving bytes from SPI3 Slave
			MASTER_CS_RESET;
			//Transmit bytes from SPI3 Slave to SPI2 Master
			HAL_SPI_Transmit_DMA(&hspi3, (uint8_t*)response_transmit, RECV_BUFF_SIZE-1);

			uint32_t plug = *(uint32_t*)hspi2.Instance->DR;
			HAL_SPI_Receive_DMA(&hspi2, (uint8_t*)response_receive, RECV_BUFF_SIZE-1);

		} else if(flag == 2) {
			//Angel

			sprintf(response_transmit, "z:%04d gr\r\n", z_val);

			//Ready receiving bytes from SPI3 Slave
			MASTER_CS_RESET;
			//Transmit bytes from SPI3 Slave to SPI2 Master
			HAL_SPI_Transmit_DMA(&hspi3, (uint8_t*)response_transmit, RECV_BUFF_SIZE-1);

			uint32_t plug = *(uint32_t*)hspi2.Instance->DR;
			HAL_SPI_Receive_DMA(&hspi2, (uint8_t*)response_receive, RECV_BUFF_SIZE-1);

		}

	} else if (X == 'X' && Y == 'Y' && Z == '\n') {
		//XY

		if(flag == 1) {
			//Accel

			sprintf(response_transmit, "x:%08d rad/sec;y:%08d rad/sec\r\n", x_val, y_val);

			//Ready receiving bytes from SPI3 Slave
			MASTER_CS_RESET;
			//Transmit bytes from SPI3 Slave to SPI2 Master
			HAL_SPI_Transmit_DMA(&hspi3, (uint8_t*)response_transmit, RECV_BUFF_SIZE-1);

			uint32_t plug = *(uint32_t*)hspi2.Instance->DR;
			HAL_SPI_Receive_DMA(&hspi2, (uint8_t*)response_receive, RECV_BUFF_SIZE-1);

		} else if(flag == 2) {
			//Angel

			sprintf(response_transmit, "x:%04d gr;y:%04d gr\r\n", x_val, y_val);

			//Ready receiving bytes from SPI3 Slave
			MASTER_CS_RESET;
			//Transmit bytes from SPI3 Slave to SPI2 Master
			HAL_SPI_Transmit_DMA(&hspi3, (uint8_t*)response_transmit, RECV_BUFF_SIZE-1);

			uint32_t plug = *(uint32_t*)hspi2.Instance->DR;
			HAL_SPI_Receive_DMA(&hspi2, (uint8_t*)response_receive, RECV_BUFF_SIZE-1);
		}

	} else if (X == '\n' && Y == 'Y' && Z == 'Z') {
		//YZ

		if(flag == 1) {
			//Accel

			sprintf(response_transmit, "y:%08d rad/sec;z:%08d rad/sec\r\n", y_val, z_val);

			//Ready receiving bytes from SPI3 Slave
			MASTER_CS_RESET;
			//Transmit bytes from SPI3 Slave to SPI2 Master
			HAL_SPI_Transmit_DMA(&hspi3, (uint8_t*)response_transmit, RECV_BUFF_SIZE-1);

			uint32_t plug = *(uint32_t*)hspi2.Instance->DR;
			HAL_SPI_Receive_DMA(&hspi2, (uint8_t*)response_receive, RECV_BUFF_SIZE-1);

		} else if(flag == 2) {
			//Angel

			sprintf(response_transmit, "y:%04d gr;z:%04d gr;\r\n", y_val, z_val);

			//Ready receiving bytes from SPI3 Slave
			MASTER_CS_RESET;
			//Transmit bytes from SPI3 Slave to SPI2 Master
			HAL_SPI_Transmit_DMA(&hspi3, (uint8_t*)response_transmit, RECV_BUFF_SIZE-1);

			uint32_t plug = *(uint32_t*)hspi2.Instance->DR;
			HAL_SPI_Receive_DMA(&hspi2, (uint8_t*)response_receive, RECV_BUFF_SIZE-1);
		}

	} else if (X == 'X' && Y == '\n' && Z == 'Z') {
		//XZ

		if(flag == 1) {
			//Accel

			sprintf(response_transmit, "x:%08d rad/sec;z:%08d rad/sec\r\n", x_val, z_val);

			//Ready receiving bytes from SPI3 Slave
			MASTER_CS_RESET;
			//Transmit bytes from SPI3 Slave to SPI2 Master
			HAL_SPI_Transmit_DMA(&hspi3, (uint8_t*)response_transmit, RECV_BUFF_SIZE-1);

			uint32_t plug = *(uint32_t*)hspi2.Instance->DR;
			HAL_SPI_Receive_DMA(&hspi2, (uint8_t*)response_receive, RECV_BUFF_SIZE-1);

		} else if(flag == 2) {
			//Angel

			sprintf(response_transmit, "x:%04d gr;z:%04d gr\r\n", x_val, z_val);

			//Ready receiving bytes from SPI3 Slave
			MASTER_CS_RESET;
			//Transmit bytes from SPI3 Slave to SPI2 Master
			HAL_SPI_Transmit_DMA(&hspi3, (uint8_t*)response_transmit, RECV_BUFF_SIZE-1);

			uint32_t plug = *(uint32_t*)hspi2.Instance->DR;
			HAL_SPI_Receive_DMA(&hspi2, (uint8_t*)response_receive, RECV_BUFF_SIZE-1);

		}

	} else if (X == 'X' && Y == 'Y' && Z == 'Z') {
		//XYZ

		if(flag == 1) {
			//Accel
			sprintf(response_transmit, "x:%08d rad/sec;y:%08d rad/sec;z:%08d rad/sec\r\n", x_val, y_val, z_val);
			//Ready receiving bytes from SPI3 Slave
			MASTER_CS_RESET;
			//Transmit bytes from SPI3 Slave to SPI2 Master
			HAL_SPI_Transmit_DMA(&hspi3, (uint8_t*)response_transmit, RECV_BUFF_SIZE-1);

			uint32_t plug = *(uint32_t*)hspi2.Instance->DR;
			HAL_SPI_Receive_DMA(&hspi2, (uint8_t*)response_receive, RECV_BUFF_SIZE-1);

		} else if(flag == 2) {
			//Angel
			sprintf(response_transmit, "x:%04d gr;y:%04d gr;z:%04d gr\r\n", x_val, y_val, z_val);
			//Ready receiving bytes from SPI3 Slave
			MASTER_CS_RESET;
			//Transmit bytes from SPI3 Slave to SPI2 Master
			HAL_SPI_Transmit_DMA(&hspi3, (uint8_t*)response_transmit, RECV_BUFF_SIZE-1);

			uint32_t plug = *(uint32_t*)hspi2.Instance->DR;
			HAL_SPI_Receive_DMA(&hspi2, (uint8_t*)response_receive, RECV_BUFF_SIZE-1);
		}
	}
}

void GetAccel() {
	readSensor(); //get data from registers
	PhysVectorsToAccel(); //set values in global vars
	SPISendData(ACCEL_X, ACCEL_Y, ACCEL_Z); //pass next to determinate and print values
}

void GetAngel() {
	readSensor(); //get data from registers
	PhysVectorsToAccel(); //get accel vectors (globals)
	AccelToAngel(); //get angels from accels vectors (global)
	SPISendData(ACCEL_XANGLE, ACCEL_YANGLE, ACCEL_ZANGLE); //pass next to determinate and print values
}

void SetInterval(uint16_t interval) {
	if(interval > 500 && interval <= 9999) {
		command_delay = interval;
	} else {
		//pass wrong value of interval
		//set default
		command_delay = 500;
	}
}

//Interpreting received UART commands
uint8_t ParseCommand(uint8_t* UART_receive) {
	/* Commands templates */
	//Get Accel ; Get Accel X ; Get Accel Y ; Get Accel Z ; Get Accel XY ; Get Accel YZ;
														 //Get Accel XZ ; GetAccel XYZ ;

	//Get Angel ; Get Angel X ; Get Angel Y ; Get Angel Z ; Get Angel XY ; Get Angel YZ;
														//Get Angel XZ ; Get Angel XYZ ;

	//Set time 500 ; Set time 1000 ; Set time 1500 ; Set time 2000 ;

	//Copy array to buffer
	uint8_t buffer[RECV_BUFF_SIZE];
	for(uint16_t i = 0; i < RECV_BUFF_SIZE; i++) {
		buffer[i] = UART_receive[i];
	}


	/* Search wildcards using checksum */
	uint8_t local_flag = 0;

	uint16_t checksum_1 = 0;
	for(uint16_t i = 0; i < RECV_BUFF_SIZE; i++) {
		if(checksum_1 == checksum_accelrule) {
			local_flag = 1; //Get Accel
			flag = 1;
			break; //Stop cycle after searching command-name wildcard
		}

		if(buffer[i] == get_accelrule[i]) {
			checksum_1++;
		} else {
			break; //break cycle if have mismatch
		}
	}

	uint16_t checksum_2 = 0;
	for(uint16_t i = 0; i < RECV_BUFF_SIZE; i++) {
		if(checksum_2 == checksum_angelrule) {
			local_flag = 2; //Get Angel
			flag = 2;
			break; //Stop cycle after searching command-name wildcard
		}

		if(buffer[i] == get_angelrule[i]) {
			checksum_2++;
		} else {
			break;
		}
	}

	uint16_t checksum_3 = 0;
	for(uint16_t i = 0; i < RECV_BUFF_SIZE; i++) {
		if(checksum_3 == checksum_timerule) {
			local_flag = 3; //Get time
			break; //Stop cycle after searching command-name wildcard
		}

		if(buffer[i] == get_timerule[i]) {
			checksum_3++;
		} else {
			break;
		}
	}

	//Determine range of args and push params in arr
	uint16_t lhs_range = 0;
	uint16_t rhs_range = 0;
	switch(local_flag) {
	case 1:
		//checksum + 1; //checksum + 5;
		lhs_range = checksum_1 + 1;
		rhs_range = checksum_1 + 5;
		break;
	case 2:
		lhs_range = checksum_2 + 1;
		rhs_range = checksum_2 + 5;
		break;
	case 3:
		lhs_range = checksum_3 + 1;
		rhs_range = checksum_3 + 5;
		break;
	}

	for(uint16_t i = lhs_range, j = 0; i < rhs_range; i++, j++) {
		args_buff[j] = buffer[i];
	}

	/* Parse arguments using checksum and wildcards */
	if(local_flag == 1) {

		GetAccel();

	} else if(local_flag == 2) {

		GetAngel();

	} else if (local_flag == 3) {
		uint16_t value = 0;
		value = atoi(args_buff);

		SetInterval(value);
	} else if (local_flag == 0) {
		SPISendError();
	}

	return local_flag;
}

void USER_UART_IRQHandler(UART_HandleTypeDef *huart) {
	if(huart->Instance == USART2) { //Determine serial port
		if(RESET != __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE)) { //Checking source IDLE IRQ
			__HAL_UART_CLEAR_IDLEFLAG(huart); //Clear IDLE IRQ sign
			USER_UART_IDLECallback(huart); //Call interrupt handler
		}
	}
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
	if(hspi->Instance == SPI2) {
		MASTER_CS_SET;
	}
}
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
	if(hspi->Instance == SPI3) {
		received_from_master = 1;
	}
	if(hspi->Instance == SPI2) {
		received_from_slave = 1;
		MASTER_CS_SET;
	}
}

void USER_UART_IDLECallback(UART_HandleTypeDef *huart) {
	HAL_UART_DMAStop(huart); //Stop DMA transmission
	Unprocessed_UART_buff_detected = SET;
	//Next DMA transmission will be allowed after buffer processing
	has_command = 1;
}

//Clear USART receive buff (pushing '\0' in all positions)
void ClearBuff(uint8_t* buff) {
	for(uint16_t i = 0; i < RECV_BUFF_SIZE; i++) {
		buff[i] = '\0';
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
