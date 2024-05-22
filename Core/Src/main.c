/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <fatfs_sd.h>
//#include <stc3100.h>
//#include <stc31xx_I2cCustomReadWrite.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CO2_address 0x29<<1
#define CO2_BINARY 0x3615
//#define CO2_ARG 0x36150001B0 //CO2 0-100% in air 
#define CO2_ARG 0x36150003D2 //CO2 0-25% in air

#define EEPROM_I2C_ADDRESS 0xA0

#define STC3100_address 0x70<<1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
char message_buffer[200] = {0};
uint8_t Cmd_End[3] = {0xFF,0xFF,0xFF};  // command end sequence
char ID_1[4];
char ID_2[9];

uint8_t NC[8]={};
uint8_t Port = 1;
uint8_t Data;

double O2_perc;
char O2_msg[5];
char O2_data[41];
char O2_fixed[41];
char ch = 'O';
uint8_t index_O = 0;
	
uint8_t arg[5];
uint8_t CO2_Read[2];
uint8_t rawCO2[2];
short s16_CO2;
double CO2_perc;

uint8_t RX_Nextion;
uint8_t Start=1;
uint8_t SDStart=0;

char dir[20];

char time[9];
char date[10];

uint32_t adc_val = 0;
float NTC_Varza = 0;
float NTC_Temp = 0;
float NTC_Volt = 0;

//NTC koeficientai
float A = 5.780e-4;
float B = 2.951e-4;
float C = 7.409e-8;

uint8_t rawBM[8];
short s16_value;
float batTemp;
float batVolt;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM6_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */
void RTC_Set_Time();
void RTC_Set_Date();
void MUX_SET(uint8_t MUX_S);
void SWITCH_SET(uint8_t SWITCH_S);
void I2C_MUX_SET(uint8_t I2C_S);
void EEPROM_READ(uint8_t *Data);
void NTC_read();
void BatteryMonitor();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
FATFS fs;  // file system
FIL fil; // File
FILINFO fno;
FRESULT fresult;  // result
UINT br, bw;  // File read/write count

/**** capacity related *****/
FATFS *pfs;
DWORD fre_clust;
uint32_t total, free_space;
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
  MX_ADC2_Init();
  MX_TIM6_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_FATFS_Init();
  MX_RTC_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
	
	
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Base_Start_IT(&htim7);
	HAL_UART_Receive_IT(&huart2, &RX_Nextion, 1);
	HAL_UART_Receive_IT(&huart1, &O2_data, 41);
	if (HAL_ADCEx_Calibration_Start(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
	
	if (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1) != 0x32F2)
		RTC_Set_Time();
	
	if (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR2) != 0xBEBE)
		RTC_Set_Date();
	
	HAL_GPIO_WritePin(I2C_A0_GPIO_Port, I2C_A0_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(I2C_A1_GPIO_Port, I2C_A1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(I2C_A2_GPIO_Port, I2C_A2_Pin, GPIO_PIN_SET);
	
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  hi2c1.Init.ClockSpeed = 400000;
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
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef DateToUpdate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
	
  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x22;
  sTime.Minutes = 0x13;
  sTime.Seconds = 0x59;

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  DateToUpdate.WeekDay = RTC_WEEKDAY_SUNDAY;
  DateToUpdate.Month = RTC_MONTH_MAY;
  DateToUpdate.Date = 0x12;
  DateToUpdate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
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
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 1000;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 24000;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 1000;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 24000;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MUX_A0_Pin|MUX_A1_Pin|MUX_A2_Pin|SWITCH_S_Pin
                          |I2C_A0_Pin|I2C_A1_Pin|I2C_A2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SPI_CS_Pin */
  GPIO_InitStruct.Pin = SPI_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MUX_A0_Pin MUX_A1_Pin MUX_A2_Pin SWITCH_S_Pin
                           I2C_A0_Pin I2C_A1_Pin I2C_A2_Pin */
  GPIO_InitStruct.Pin = MUX_A0_Pin|MUX_A1_Pin|MUX_A2_Pin|SWITCH_S_Pin
                          |I2C_A0_Pin|I2C_A1_Pin|I2C_A2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void RTC_Set_Date()
{
	RTC_DateTypeDef DateToUpdate = {0};
	
	DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
  DateToUpdate.Month = RTC_MONTH_MAY;
  DateToUpdate.Date = 0x20;
  DateToUpdate.Year = 0x24;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
	
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR2, 0xBEBE);
}

void RTC_Set_Time()
{
	RTC_TimeTypeDef sTime = {0};
  
  sTime.Hours = 0x10;
  sTime.Minutes = 0x05;
  sTime.Seconds = 0x40;

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
	
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x32F2);
}


//Multiplekserio kanalo parinkimas
void MUX_SET(uint8_t MUX_S)
{
	switch(MUX_S)
	{
		case 1:
			HAL_GPIO_WritePin(MUX_A2_GPIO_Port, MUX_A2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(MUX_A1_GPIO_Port, MUX_A1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(MUX_A0_GPIO_Port, MUX_A0_Pin, GPIO_PIN_SET);
			break;
		case 2:
			HAL_GPIO_WritePin(MUX_A2_GPIO_Port, MUX_A2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(MUX_A1_GPIO_Port, MUX_A1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(MUX_A0_GPIO_Port, MUX_A0_Pin, GPIO_PIN_RESET);
			break;
		case 3:
			HAL_GPIO_WritePin(MUX_A2_GPIO_Port, MUX_A2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(MUX_A1_GPIO_Port, MUX_A1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MUX_A0_GPIO_Port, MUX_A0_Pin, GPIO_PIN_SET);
			break;
		case 4:
			HAL_GPIO_WritePin(MUX_A2_GPIO_Port, MUX_A2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(MUX_A1_GPIO_Port, MUX_A1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MUX_A0_GPIO_Port, MUX_A0_Pin, GPIO_PIN_RESET);
			break;
		case 5:
			HAL_GPIO_WritePin(MUX_A2_GPIO_Port, MUX_A2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MUX_A1_GPIO_Port, MUX_A1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(MUX_A0_GPIO_Port, MUX_A0_Pin, GPIO_PIN_SET);
			break;
		case 6:
			HAL_GPIO_WritePin(MUX_A2_GPIO_Port, MUX_A2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MUX_A1_GPIO_Port, MUX_A1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(MUX_A0_GPIO_Port, MUX_A0_Pin, GPIO_PIN_RESET);
			break;
		case 7:
			HAL_GPIO_WritePin(MUX_A2_GPIO_Port, MUX_A2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MUX_A1_GPIO_Port, MUX_A1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MUX_A0_GPIO_Port, MUX_A0_Pin, GPIO_PIN_SET);
			break;
		case 8:
			HAL_GPIO_WritePin(MUX_A2_GPIO_Port, MUX_A2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MUX_A1_GPIO_Port, MUX_A1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MUX_A0_GPIO_Port, MUX_A0_Pin, GPIO_PIN_RESET);
			break;
	}
}

void SWITCH_SET(uint8_t SWITCH_S)
{
	switch(SWITCH_S)
	{
		case 1: //UART RX
			HAL_GPIO_WritePin(SWITCH_S_GPIO_Port, SWITCH_S_Pin, GPIO_PIN_RESET);
			break;
		
		case 2: //ADC_IN
			HAL_GPIO_WritePin(SWITCH_S_GPIO_Port, SWITCH_S_Pin, GPIO_PIN_SET);
			break;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) 
{
	
	if(RX_Nextion==0xB1)
		Start=1;
	else if(RX_Nextion==0xB2)
		Start=0;
	else if(RX_Nextion==0xB3)
		SDStart=1;
	else if(RX_Nextion==0xB4)
		SDStart=0;
	
	else if(RX_Nextion==0x01){
		__HAL_TIM_SET_PRESCALER(&htim7, 2000);   //1sek.
		fresult = f_mount(&fs, "/", 1); // Ijungiamas SPI CS
		sprintf(dir, "/Temp.txt"); // Sukuriama direktorija
		
		// Atidaromas failas arba sukuriamas, jeigu neegzistuoja
		fresult = f_open(&fil, dir, FA_OPEN_ALWAYS | FA_READ | FA_WRITE );
		fresult = f_lseek(&fil, fil.fsize);

		// Irasomi duomenys
		f_puts("Irasymo periodas: 1sek.\n", &fil);
						
		// Failas uzdaromas
		fresult = f_close(&fil);
		
		sprintf(dir, "/O2.txt"); // Sukuriama direktorija
		
		// Atidaromas failas arba sukuriamas, jeigu neegzistuoja
		fresult = f_open(&fil, dir, FA_OPEN_ALWAYS | FA_READ | FA_WRITE );
		fresult = f_lseek(&fil, fil.fsize);

		// Irasomi duomenys
		f_puts("Irasymo periodas: 1sek.\n", &fil);
						
		// Failas uzdaromas
		fresult = f_close(&fil);
		
		sprintf(dir, "/CO2.txt");
					
		// Open file to write/ create a file if it doesn't exist
		fresult = f_open(&fil, dir, FA_OPEN_ALWAYS | FA_READ | FA_WRITE );
		fresult = f_lseek(&fil, fil.fsize);
		
		// Irasomi duomenys
		f_puts("Irasymo periodas: 1sek.\n", &fil);
						
		// Failas uzdaromas
		fresult = f_close(&fil);
		
	}
	else if(RX_Nextion==0x02){
		__HAL_TIM_SET_PRESCALER(&htim7, 20000);  //10sek.
		fresult = f_mount(&fs, "/", 1); // Ijungiamas SPI CS
		sprintf(dir, "/Temp.txt"); // Sukuriama direktorija
		
		// Atidaromas failas arba sukuriamas, jeigu neegzistuoja
		fresult = f_open(&fil, dir, FA_OPEN_ALWAYS | FA_READ | FA_WRITE );
		fresult = f_lseek(&fil, fil.fsize);

		// Irasomi duomenys
		f_puts("Irasymo periodas: 10sek.\n", &fil);
						
		// Failas uzdaromas
		fresult = f_close(&fil);
		
		sprintf(dir, "/O2.txt"); // Sukuriama direktorija
		
		// Atidaromas failas arba sukuriamas, jeigu neegzistuoja
		fresult = f_open(&fil, dir, FA_OPEN_ALWAYS | FA_READ | FA_WRITE );
		fresult = f_lseek(&fil, fil.fsize);

		// Irasomi duomenys
		f_puts("Irasymo periodas: 10sek.\n", &fil);
						
		// Failas uzdaromas
		fresult = f_close(&fil);
		
		sprintf(dir, "/CO2.txt");
					
		// Open file to write/ create a file if it doesn't exist
		fresult = f_open(&fil, dir, FA_OPEN_ALWAYS | FA_READ | FA_WRITE );
		fresult = f_lseek(&fil, fil.fsize);
		
		// Irasomi duomenys
		f_puts("Irasymo periodas: 1sek.\n", &fil);
						
		// Failas uzdaromas
		fresult = f_close(&fil);
	}
	else if(RX_Nextion==0x03){
		__HAL_TIM_SET_PRESCALER(&htim7, 60600);  //30sek.
		fresult = f_mount(&fs, "/", 1); // Ijungiamas SPI CS
		sprintf(dir, "/Temp.txt"); // Sukuriama direktorija
		
		// Atidaromas failas arba sukuriamas, jeigu neegzistuoja
		fresult = f_open(&fil, dir, FA_OPEN_ALWAYS | FA_READ | FA_WRITE );
		fresult = f_lseek(&fil, fil.fsize);

		// Irasomi duomenys
		f_puts("Irasymo periodas: 30sek.\n", &fil);
						
		// Failas uzdaromas
		fresult = f_close(&fil);
		
		sprintf(dir, "/O2.txt"); // Sukuriama direktorija
		
		// Atidaromas failas arba sukuriamas, jeigu neegzistuoja
		fresult = f_open(&fil, dir, FA_OPEN_ALWAYS | FA_READ | FA_WRITE );
		fresult = f_lseek(&fil, fil.fsize);

		// Irasomi duomenys
		f_puts("Irasymo periodas: 30sek.\n", &fil);
						
		// Failas uzdaromas
		fresult = f_close(&fil);
		
		sprintf(dir, "/CO2.txt");
					
		// Open file to write/ create a file if it doesn't exist
		fresult = f_open(&fil, dir, FA_OPEN_ALWAYS | FA_READ | FA_WRITE );
		fresult = f_lseek(&fil, fil.fsize);
		
		// Irasomi duomenys
		f_puts("Irasymo periodas: 1sek.\n", &fil);
						
		// Failas uzdaromas
		fresult = f_close(&fil);
	}
	else if(RX_Nextion==0x04){
		__HAL_TIM_SET_PRESCALER(&htim7, 106000); //1min. 
		fresult = f_mount(&fs, "/", 1); // Ijungiamas SPI CS
		sprintf(dir, "/Temp.txt"); // Sukuriama direktorija
		
		// Atidaromas failas arba sukuriamas, jeigu neegzistuoja
		fresult = f_open(&fil, dir, FA_OPEN_ALWAYS | FA_READ | FA_WRITE );
		fresult = f_lseek(&fil, fil.fsize);

		// Irasomi duomenys
		f_puts("Irasymo periodas: 1min.\n", &fil);
						
		// Failas uzdaromas
		fresult = f_close(&fil);
		
		sprintf(dir, "/O2.txt"); // Sukuriama direktorija
		
		// Atidaromas failas arba sukuriamas, jeigu neegzistuoja
		fresult = f_open(&fil, dir, FA_OPEN_ALWAYS | FA_READ | FA_WRITE );
		fresult = f_lseek(&fil, fil.fsize);

		// Irasomi duomenys
		f_puts("Irasymo periodas: 1min.\n", &fil);
						
		// Failas uzdaromas
		fresult = f_close(&fil);
		
		sprintf(dir, "/CO2.txt");
					
		// Open file to write/ create a file if it doesn't exist
		fresult = f_open(&fil, dir, FA_OPEN_ALWAYS | FA_READ | FA_WRITE );
		fresult = f_lseek(&fil, fil.fsize);
		
		// Irasomi duomenys
		f_puts("Irasymo periodas: 1sek.\n", &fil);
						
		// Failas uzdaromas
		fresult = f_close(&fil);
	}
	else if(RX_Nextion==0x05){
		__HAL_TIM_SET_PRESCALER(&htim7, 606000); //5min.
		fresult = f_mount(&fs, "/", 1); // Ijungiamas SPI CS
		sprintf(dir, "/Temp.txt"); // Sukuriama direktorija
		
		// Atidaromas failas arba sukuriamas, jeigu neegzistuoja
		fresult = f_open(&fil, dir, FA_OPEN_ALWAYS | FA_READ | FA_WRITE );
		fresult = f_lseek(&fil, fil.fsize);

		// Irasomi duomenys
		f_puts("Irasymo periodas: 5min.\n", &fil);
						
		// Failas uzdaromas
		fresult = f_close(&fil);
		
		sprintf(dir, "/O2.txt"); // Sukuriama direktorija
		
		// Atidaromas failas arba sukuriamas, jeigu neegzistuoja
		fresult = f_open(&fil, dir, FA_OPEN_ALWAYS | FA_READ | FA_WRITE );
		fresult = f_lseek(&fil, fil.fsize);

		// Irasomi duomenys
		f_puts("Irasymo periodas: 5min.\n", &fil);
						
		// Failas uzdaromas
		fresult = f_close(&fil);
		
		sprintf(dir, "/CO2.txt");
					
		// Open file to write/ create a file if it doesn't exist
		fresult = f_open(&fil, dir, FA_OPEN_ALWAYS | FA_READ | FA_WRITE );
		fresult = f_lseek(&fil, fil.fsize);
		
		// Irasomi duomenys
		f_puts("Irasymo periodas: 1sek.\n", &fil);
						
		// Failas uzdaromas
		fresult = f_close(&fil);
	}
	
  HAL_UART_Receive_IT(&huart2, &RX_Nextion, 1); //Ekrano pertrauktis
	
	HAL_UART_Receive_IT(&huart1, &O2_data, 41); //O2 pertrauktis
	
}

void I2C_MUX_SET(uint8_t I2C_S){
  uint8_t channel = 1 << I2C_S;
  HAL_I2C_Master_Transmit(&hi2c1, (0x77 << 1), &channel, 1, 1000);
}

void NTC_READ()
{
	HAL_ADCEx_Calibration_Start(&hadc2);
	HAL_ADC_Start(&hadc2);
	HAL_ADC_PollForConversion(&hadc2, 30);
	adc_val = HAL_ADC_GetValue(&hadc2);
		
	NTC_Volt = adc_val * (3.3/4095); // Itampos skaiciavimas
	NTC_Varza = (NTC_Volt*10e3)/(3.3-NTC_Volt);
	NTC_Temp = 1/(A+B*log(NTC_Varza)+C*pow(log(NTC_Varza),3))-273;
}


void EEPROM_READ(uint8_t *Data)
{
	if (HAL_I2C_Mem_Read(&hi2c1, EEPROM_I2C_ADDRESS, 0x00, 1, Data, 1, 2000)!=HAL_OK)
			{
				NC[Port] = 0;
			}
}

void NEXTION_SendString (char *ID, char *string)
{
		char buf[50];
		int len = sprintf (buf, "%s.txt=\"%s\"", ID, string);
		HAL_UART_Transmit(&huart2, (uint8_t *)buf, len, 100);
		HAL_UART_Transmit(&huart2, Cmd_End, 3, 100);
}

void BatteryMonitor()
{
		uint8_t data = 0x02;
		HAL_I2C_Mem_Write(&hi2c1, (STC3100_address), 0x01, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);
			
		data = 0x10;
		HAL_I2C_Mem_Write(&hi2c1, (STC3100_address), 0x00, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);
			
		HAL_I2C_Mem_Read(&hi2c1, (STC3100_address), 0x02, I2C_MEMADD_SIZE_8BIT, &rawBM[0], 1, 1000);
		HAL_I2C_Mem_Read(&hi2c1, (STC3100_address), 0x03, I2C_MEMADD_SIZE_8BIT, &rawBM[1], 1, 1000);
			
		HAL_I2C_Mem_Read(&hi2c1, (STC3100_address), 0x06, I2C_MEMADD_SIZE_8BIT, &rawBM[2], 1, 1000);
		HAL_I2C_Mem_Read(&hi2c1, (STC3100_address), 0x07, I2C_MEMADD_SIZE_8BIT, &rawBM[3], 1, 1000);
			
		HAL_I2C_Mem_Read(&hi2c1, (STC3100_address), 0x08, I2C_MEMADD_SIZE_8BIT, &rawBM[4], 1, 1000);
		HAL_I2C_Mem_Read(&hi2c1, (STC3100_address), 0x09, I2C_MEMADD_SIZE_8BIT, &rawBM[5], 1, 1000);
			
		HAL_I2C_Mem_Read(&hi2c1, (STC3100_address), 0x0A, I2C_MEMADD_SIZE_8BIT, &rawBM[6], 1, 1000);
		HAL_I2C_Mem_Read(&hi2c1, (STC3100_address), 0x0B, I2C_MEMADD_SIZE_8BIT, &rawBM[7], 1, 1000);
			
		s16_value = (rawBM[5] << 8) | rawBM[4];
		batVolt = s16_value * 0.00224;
			
		s16_value = (rawBM[7] << 8) | rawBM[6];
		batTemp = s16_value * 0.125;
		
		sprintf(message_buffer, "%0.2f V", batVolt);
		NEXTION_SendString("t23", message_buffer);
		sprintf(message_buffer, "%0.2f °C", batTemp);
		NEXTION_SendString("t25", message_buffer);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim==&htim6){
		
		if (Start == 1){
			
			//BatteryMonitor(); // Baterijos monitoringo funkcija
			
			//Pagrindinis ciklas
			Data = 0x00;
			uint8_t Roll_i = 0;
			
			if(Port>8){
				Port=1;
			}
			I2C_MUX_SET(Port-1);
			EEPROM_READ(&Data);
			sprintf(ID_1, "t%d", Port+8); //Nextion tekstu ID "t" sudarymas
			sprintf(ID_2, "t%d", Port-1);
				
			//CO2 I2C
			arg[0] = (uint8_t)(CO2_ARG >> 32);
			arg[1] = (uint8_t)(CO2_ARG >> 24);
			arg[2] = (uint8_t)(CO2_ARG >> 16);
			arg[3] = (uint8_t)(CO2_ARG >> 8);
			arg[4] = (uint8_t)(CO2_ARG);
			
			if (Data == 0x01){ //O2 jutiklio ID
				NC[Port] = 1;
				MUX_SET(Port);
				SWITCH_SET(1); //UART_RX
				NEXTION_SendString(ID_2, "O2 %");
				
				for (uint8_t i = 0; i < 41; i++){   //tikrinama visa zinute
					if (strcmp(O2_data[i], ch) == 0){ //ieskoma "O" simbolio
						index_O = i;
						break;
					}
				}
				
				for (uint8_t j = 0; j < 41; j++){
						if ((j+index_O) > 40){
							O2_fixed[j] = O2_data[Roll_i];
							Roll_i++;
						}
						else
							O2_fixed[j] = O2_data[j+index_O];
				}
				
				for (uint8_t i = 0; i < 5; i++)
					O2_msg[i] = O2_fixed[i+27]; //O2 % issaugomi atskirame kintamajame
						
				sscanf(O2_msg, "%lf", &O2_perc);
				O2_perc = O2_perc + 2;
				
				sprintf(message_buffer, "%0.2f", O2_perc);
				NEXTION_SendString(ID_1, message_buffer);
				
			}
			
			else if (Data == 0x02){ //Temperaturos jutiklio ID
				NEXTION_SendString(ID_2, "Temp C");
				NC[Port] = 2;
				MUX_SET(Port);
				SWITCH_SET(2); //ADC_IN9
				NTC_READ();
				sprintf(message_buffer, "%0.2f", NTC_Temp);
				NEXTION_SendString(ID_1, message_buffer);
			}
			
			
			else if (HAL_I2C_Master_Transmit(&hi2c1, CO2_address, arg, 5, 1000) == HAL_OK){
			
				NC[Port] = 3;
				
				/*uint8_t ASC[2];
				ASC[0] = (uint8_t)(0x3FEF >> 8);
				ASC[1] = (uint8_t)(0x3FEF);
				HAL_I2C_Master_Transmit(&hi2c1, CO2_address, ASC, 2, 3000);*/
			
				uint8_t Humidity[5];
				Humidity[0] = (uint8_t)(0x36248000A2 >> 32);
				Humidity[1] = (uint8_t)(0x36248000A2 >> 24);
				Humidity[2] = (uint8_t)(0x36248000A2 >> 16);
				Humidity[3] = (uint8_t)(0x36248000A2 >> 8);
				Humidity[4] = (uint8_t)(0x36248000A2);
				HAL_I2C_Master_Transmit(&hi2c1, CO2_address, Humidity, 5, 1000);
			
				
				CO2_Read[0] = (uint8_t)(0x3639 >> 8);
				CO2_Read[1] = (uint8_t)(0x3639);
				HAL_I2C_Master_Transmit(&hi2c1, CO2_address, CO2_Read, 2, 1000);
			
				HAL_I2C_Master_Receive(&hi2c1, CO2_address, rawCO2, 2, 1000);
			
				s16_CO2 = (rawCO2[0] << 8) | rawCO2[1];
				CO2_perc = (((s16_CO2 - 16384)/32768)*100);
			
				
				NEXTION_SendString(ID_2, "CO2 %");
				sprintf(message_buffer, "%0.2f", CO2_perc);
				NEXTION_SendString(ID_1, message_buffer);
			}
			
			else {
				NEXTION_SendString(ID_1, "Empty");
				sprintf(message_buffer, "Slot %d", Port);
				NEXTION_SendString(ID_2, message_buffer);
			}
			
			Port++;
			
			
		}
	}

	
	if(SDStart==1){
	
		if (htim==&htim7){
		
			//Nuskaitomas laikas is RTC
			RTC_DateTypeDef gDate; 
			RTC_TimeTypeDef gTime; 
			// Get the RTC current Time 
			HAL_RTC_GetTime(&hrtc, &gTime, RTC_FORMAT_BIN); 
			// Get the RTC current Date  
			HAL_RTC_GetDate(&hrtc, &gDate, RTC_FORMAT_BIN);
	
				for(uint8_t i=0; i<8; i++){
				
				if(NC[i] == 1){

						fresult = f_mount(&fs, "/", 1);
						sprintf(dir, "/O2.txt");
					
						// Open file to write/ create a file if it doesn't exist
						fresult = f_open(&fil, dir, FA_OPEN_ALWAYS | FA_READ | FA_WRITE );
						fresult = f_lseek(&fil, fil.fsize);

						// Writing text
				
						sprintf(message_buffer, "O2 konc. procentais: %s -- %2d-%02d-%02d %02d:%02d:%02d\n", O2_msg, 
						2000 + gDate.Year, gDate.Month, gDate.Date, gTime.Hours, gTime.Minutes, gTime.Seconds);
						f_puts(message_buffer, &fil);

						// Close file
						fresult = f_close(&fil);
					}
				
				if(NC[i] == 2){
					
						fresult = f_mount(&fs, "/", 1); // Ijungiamas SPI CS
						sprintf(dir, "/Temp.txt"); // Sukuriama direktorija
		
						// Atidaromas failas arba sukuriamas, jeigu neegzistuoja
						fresult = f_open(&fil, dir, FA_OPEN_ALWAYS | FA_READ | FA_WRITE );
						fresult = f_lseek(&fil, fil.fsize);

						// Irasomi duomenys
						sprintf(message_buffer, "Temperatura: %0.1f C -- %2d-%02d-%02d %02d:%02d:%02d\n", NTC_Temp,
						2000 + gDate.Year, gDate.Month, gDate.Date, gTime.Hours, gTime.Minutes, gTime.Seconds);
						f_puts(message_buffer, &fil);
						
						// Failas uzdaromas
						fresult = f_close(&fil);
					}
				
					if(NC[i] == 3){

						fresult = f_mount(&fs, "/", 1);
						sprintf(dir, "/CO2.txt");
					
						// Open file to write/ create a file if it doesn't exist
						fresult = f_open(&fil, dir, FA_OPEN_ALWAYS | FA_READ | FA_WRITE );
						fresult = f_lseek(&fil, fil.fsize);

						// Writing text
				
						sprintf(message_buffer, "CO2 konc. procentais: %0.2f -- %2d-%02d-%02d %02d:%02d:%02d\n", CO2_perc, 
						2000 + gDate.Year, gDate.Month, gDate.Date, gTime.Hours, gTime.Minutes, gTime.Seconds);
						f_puts(message_buffer, &fil);

						// Close file
						fresult = f_close(&fil);
					}
					
				}
				if(SDStart==0)
				fresult = f_mount(&fs, "/", 0);	
		}
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
