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
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define VOORUIT (GPIOB->BSRR = 0xF800B000);
#define ACHTERUIT (GPIOB->BSRR = 0xF8006800);
#define LINKS (GPIOB->BSRR = 0xF800A800);
#define RECHTS (GPIOB->BSRR = 0xF8007000);
#define UIT (GPIOB->BSRR = 0xF8000000);
#define NORMAL_SPEED 255
#define DOCK_SPEED 145
#define IR_SENSOR_COUNT 3
#define IR_SENSOR_DISTANCE 1500
#define IR_MID_SENSOR_DISTANCE 2200
#define ACHTERUIT_DELAY 500
#define DRAAI_DELAY 1500
#define LANG_DRAAI_DELAY 3000
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t sample = 0;
uint8_t data = 0;
uint8_t data_ready = 0;
uint8_t data_beacon = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
int _write(int, char *, int);
void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *);
void HAL_GPIO_EXTI_Callback(uint16_t);
void readAdc(uint32_t []);
uint8_t driveDock(uint8_t, uint32_t [], uint8_t *, uint8_t *);
void driveNormal(uint32_t []);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len) {
	for(int i = 0; i < len; i++){
		if(ptr[i]=='\n'){
			HAL_UART_Transmit(&huart1, (uint8_t*)"\r", 1, HAL_MAX_DELAY);
		}
		HAL_UART_Transmit(&huart1, (uint8_t*)&ptr[i], 1, HAL_MAX_DELAY);
	}
    return len;
}
void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim)
{
	//HAL_GPIO_TogglePin(PIN_GPIO_Port, PIN_Pin);
	sample = !HAL_GPIO_ReadPin(IR_GPIO_Port, IR_Pin); //MSB eerst
	data = (data << 1) | sample; //actief lage pin => bit toggelen
	HAL_TIM_Base_Stop_IT(&htim6);
	__HAL_TIM_SET_COUNTER(&htim6, 0);
	if ((data == 0xa4) || (data == 0xa8) || (data == 0xac))
	{
		data_ready = data;
	}
	if (data == 0xa1)
	{
		data_beacon = data;
	}
}
void HAL_GPIO_EXTI_Callback(uint16_t IR_EXTI_IRQn)
{
	HAL_TIM_Base_Start_IT(&htim6);
}
void readAdc(uint32_t result[])
{
	ADC_ChannelConfTypeDef sConfig = {0};

	sConfig.Channel = ADC_CHANNEL_0;
	HAL_ADC_ConfigChannel(&hadc, &sConfig);
	HAL_ADC_Start(&hadc);
	HAL_ADC_PollForConversion(&hadc, 1);
	result[0] = HAL_ADC_GetValue(&hadc);

	sConfig.Channel = ADC_CHANNEL_1;
	HAL_ADC_ConfigChannel(&hadc, &sConfig);
	HAL_ADC_Start(&hadc);
	HAL_ADC_PollForConversion(&hadc, 1);
	result[2] = HAL_ADC_GetValue(&hadc);

	sConfig.Channel = ADC_CHANNEL_2;
	HAL_ADC_ConfigChannel(&hadc, &sConfig);
	HAL_ADC_Start(&hadc);
	HAL_ADC_PollForConversion(&hadc, 1);
	result[1] = HAL_ADC_GetValue(&hadc);

	printf("RECHTS: %ld, MIDDEN: %ld, LINKS: %ld\n", result[0], result[2], result[1]);
}
uint8_t driveDock(uint8_t state_dock, uint32_t adc_values[], uint8_t *drive_dock, uint8_t *detect_dock)
{
	switch (state_dock)
	{
	case 0:
		htim2.Instance->CCR1 = DOCK_SPEED; //Duty Cycle op 50% => led 38kHz
		htim2.Instance->CCR3 = DOCK_SPEED; //Duty Cycle op 50% => led 38kHz
		readAdc(adc_values);
		driveNormal(adc_values);

		if (data_ready == 0xac)
		{
			(*detect_dock)++;
		}

		if (*detect_dock == 1) //150 ms delay
		{
			state_dock = 1;
		}
		break;
	case 1:
		if (data_ready == 0xa4)
		{
			VOORUIT;
			htim2.Instance->CCR1 = 0; //Duty Cycle op 50% => led 38kHz
			htim2.Instance->CCR3 = DOCK_SPEED; //Duty Cycle op 50% => led 38kHz
		}
		else if (data_ready == 0xa8)
		{
			VOORUIT;
			htim2.Instance->CCR1 = DOCK_SPEED; //Duty Cycle op 50% => led 38kHz
			htim2.Instance->CCR3 = 0; //Duty Cycle op 50% => led 38kHz
		}
		else if (data_ready == 0xac)
		{
			VOORUIT;
			htim2.Instance->CCR1 = DOCK_SPEED; //Duty Cycle op 50% => led 38kHz
			htim2.Instance->CCR3 = DOCK_SPEED; //Duty Cycle op 50% => led 38kHz
		}

		readAdc(adc_values);
		if (adc_values[2] > IR_SENSOR_DISTANCE)
		{
			state_dock = 2;
		}
		break;
	case 2:
		UIT;
		data_ready = 0;
		data_beacon = 0;
		if (!HAL_GPIO_ReadPin(DRUK0_GPIO_Port, DRUK0_Pin))
		{
			while (!HAL_GPIO_ReadPin(DRUK0_GPIO_Port, DRUK0_Pin));
			ACHTERUIT;
			htim2.Instance->CCR1 = DOCK_SPEED; //Duty Cycle op 50% => led 38kHz
			htim2.Instance->CCR3 = DOCK_SPEED; //Duty Cycle op 50% => led 38kHz
			HAL_Delay(ACHTERUIT_DELAY);
			RECHTS;
			HAL_Delay(DRAAI_DELAY);
			*drive_dock = 0;
			*detect_dock = 0;
			state_dock = 0;
		}
		break;
	}
	return state_dock;
}
void driveNormal(uint32_t adc_values[])
{
	if (HAL_GPIO_ReadPin(DRUK4_GPIO_Port, DRUK4_Pin) == 0) //bumper links
	{
		ACHTERUIT;
		HAL_Delay(ACHTERUIT_DELAY);
		LINKS;
		HAL_Delay(DRAAI_DELAY);
	}
	else if (HAL_GPIO_ReadPin(DRUK2_GPIO_Port, DRUK2_Pin) == 0) //bumper rechts
	{
		ACHTERUIT;
		HAL_Delay(ACHTERUIT_DELAY);
		RECHTS;
		HAL_Delay(DRAAI_DELAY);
	}
	else if (HAL_GPIO_ReadPin(DRUK1_GPIO_Port, DRUK1_Pin) == 0) //bumper rechts
	{
		ACHTERUIT;
		HAL_Delay(ACHTERUIT_DELAY);
		LINKS;
		HAL_Delay(LANG_DRAAI_DELAY);
	}

	if (adc_values[0] > IR_SENSOR_DISTANCE && adc_values[1] < IR_SENSOR_DISTANCE)
	{
		LINKS;
		//VOORUIT;
		//htim2.Instance->CCR1 = NORMAL_SPEED; //Duty Cycle op 50% => led 38kHz
		//htim2.Instance->CCR3 = 0; //Duty Cycle op 50% => led 38kHz
	}
	else if (adc_values[1] > IR_SENSOR_DISTANCE && adc_values[0] < IR_SENSOR_DISTANCE)
	{
		RECHTS;
		//VOORUIT;
		//htim2.Instance->CCR1 = 0; //Duty Cycle op 50% => led 38kHz
		//htim2.Instance->CCR3 = NORMAL_SPEED; //Duty Cycle op 50% => led 38kHz
	}
	else if (adc_values[0] > IR_MID_SENSOR_DISTANCE && adc_values[1] > IR_MID_SENSOR_DISTANCE)
	{
		ACHTERUIT;
		HAL_Delay(ACHTERUIT_DELAY);
		if (adc_values[0] > adc_values[1])
		{
			LINKS;
			HAL_Delay(DRAAI_DELAY);
		}
		else
		{	RECHTS;
			HAL_Delay(DRAAI_DELAY);
		}
	}
	else if (adc_values[2] > IR_MID_SENSOR_DISTANCE)
	{
		ACHTERUIT;
		HAL_Delay(ACHTERUIT_DELAY);
		if (adc_values[0] > adc_values[1])
		{
			LINKS;
			HAL_Delay(DRAAI_DELAY);
		}
		else
		{
			RECHTS;
			HAL_Delay(DRAAI_DELAY);
		}
	}
	else
	{
		VOORUIT;
		//htim2.Instance->CCR1 = NORMAL_SPEED; //Duty Cycle op 50% => led 38kHz
		//htim2.Instance->CCR3 = NORMAL_SPEED; //Duty Cycle op 50% => led 38kHz
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	uint32_t adc_values[IR_SENSOR_COUNT];
	uint8_t drive_dock = 0;
	uint8_t state_dock = 0;
	uint8_t detect_dock = 0;
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
  MX_ADC_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(IR0_GPIO_Port, IR0_Pin, 1);
  HAL_GPIO_WritePin(IR1_GPIO_Port, IR1_Pin, 1);
  HAL_GPIO_WritePin(IR2_GPIO_Port, IR2_Pin, 1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_Delay(50); //sensors moeten opstarten
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  switch (drive_dock)
	  {
	  case 0:
		  if (!HAL_GPIO_ReadPin(DRUK0_GPIO_Port, DRUK0_Pin))
		  {
			  while (!HAL_GPIO_ReadPin(DRUK0_GPIO_Port, DRUK0_Pin));
			  drive_dock ^= 1;
		  }
		  HAL_GPIO_WritePin(STOF_GPIO_Port, STOF_Pin, 1);
		  htim2.Instance->CCR1 = NORMAL_SPEED; //Duty Cycle op 50% => led 38kHz
		  htim2.Instance->CCR3 = NORMAL_SPEED; //Duty Cycle op 50% => led 38kHz
		  readAdc(adc_values);
		  printf("l: %d, m: %d, r: %d, ", HAL_GPIO_ReadPin(DRUK4_GPIO_Port, DRUK4_Pin), HAL_GPIO_ReadPin(DRUK1_GPIO_Port, DRUK1_Pin), HAL_GPIO_ReadPin(DRUK2_GPIO_Port, DRUK2_Pin));
		  driveNormal(adc_values);
		  break;
	  case 1:
		  HAL_GPIO_WritePin(STOF_GPIO_Port, STOF_Pin, 0);
		  printf("data: %x, %x, %d\n", data_ready, data_beacon, state_dock);
		  state_dock = driveDock(state_dock, adc_values, &drive_dock, &detect_dock);
		  break;
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = ENABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerFrequencyMode = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 93;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 255;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  htim6.Init.Prescaler = 192;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 255;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(STATUS_GPIO_Port, STATUS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, IR1_Pin|IR2_Pin|IR3_Pin|IR4_Pin
                          |IR5_Pin|IR0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BIN2_Pin|BIN1_Pin|STBY_Pin|AIN2_Pin
                          |AIN1_Pin|STOF_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : STATUS_Pin */
  GPIO_InitStruct.Pin = STATUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(STATUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : IR1_Pin IR2_Pin IR3_Pin IR4_Pin
                           IR5_Pin IR0_Pin */
  GPIO_InitStruct.Pin = IR1_Pin|IR2_Pin|IR3_Pin|IR4_Pin
                          |IR5_Pin|IR0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DRUK5_Pin DRUK4_Pin DRUK3_Pin DRUK2_Pin
                           DRUK1_Pin */
  GPIO_InitStruct.Pin = DRUK5_Pin|DRUK4_Pin|DRUK3_Pin|DRUK2_Pin
                          |DRUK1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : BIN2_Pin BIN1_Pin STBY_Pin AIN2_Pin
                           AIN1_Pin STOF_Pin */
  GPIO_InitStruct.Pin = BIN2_Pin|BIN1_Pin|STBY_Pin|AIN2_Pin
                          |AIN1_Pin|STOF_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : DRUK0_Pin */
  GPIO_InitStruct.Pin = DRUK0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(DRUK0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IR_Pin */
  GPIO_InitStruct.Pin = IR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IR_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

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
