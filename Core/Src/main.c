/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
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
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM16_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
uint8_t changeDisplayState(uint8_t code);
bool uartRxComplete(uint8_t last_byte);
void interpret_rx_message(uint8_t *rx_array, uint8_t length);
void request_measurement(uint8_t parameter);
void request_status(uint8_t output);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t std_num[13] = "@,21593698,!\n";
uint8_t message_received = 0;
uint8_t btn_mid_flag = 0;
uint8_t btn_right_flag = 0;
uint8_t btn_up_flag = 0;
uint8_t btn_left_flag = 0;
uint8_t btn_down_flag = 0;
uint8_t adc_timer_flag = 0;
uint32_t last_ticks = 0;

uint8_t rx_byte[1];

// VARIABLES OF INTEREST
uint16_t amplitude = 0;
uint16_t frequency = 0;
uint16_t period = 0;
uint16_t offset = 0;
uint8_t measurement_mode = 0;
uint8_t display_state = 1; // Default of 1 is Measurement Display State
uint8_t output_active = 0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	message_received = 1;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	last_ticks = HAL_GetTick();
	if(GPIO_Pin == btn_mid_Pin)
	{
		btn_mid_flag = 1;
	}
	else if(GPIO_Pin == btn_right_Pin)
	{
		btn_right_flag = 1;
	}
	else if(GPIO_Pin == btn_up_Pin)
	{
		btn_up_flag = 1;
	}
	else if(GPIO_Pin == btn_left_Pin)
	{
		btn_left_flag = 1;
	}
	else if(GPIO_Pin == btn_down_Pin)
	{
		btn_down_flag = 1;
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
	uint16_t raw;
	uint16_t millivolts;
	char msg[100];
	uint16_t adc_array[1000];
	uint16_t adc_count = 0;


	uint8_t rx_bytes[10] = {0};
	uint8_t rx_bytes_counter = 0;
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
  MX_USART2_UART_Init();
  MX_TIM16_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Transmit(&huart2, std_num, 13, 10);
  HAL_UART_Receive_IT(&huart2, rx_byte, 1);
  HAL_TIM_Base_Start_IT(&htim16);


  // Set up the default state of the device
  display_state = changeDisplayState(1); // 1 == Measurement mode

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(message_received && rx_byte[0] != '\n')
	  {
		  rx_bytes[rx_bytes_counter] = rx_byte[0];
		  if(rx_bytes_counter == 0 && rx_byte[0] == '@'){
			  rx_bytes_counter++;
		  } else if(rx_bytes_counter > 0){
			  rx_bytes_counter++;
			  if(uartRxComplete(rx_byte[0]))
			  {
				  interpret_rx_message(rx_bytes, rx_bytes_counter);
				  rx_bytes_counter = 0;
			  }
		  }
		  HAL_UART_Receive_IT(&huart2, rx_byte, 1);
		  message_received = 0;
	  }
	  if(btn_up_flag)
	  {
		  if(HAL_GetTick() - last_ticks >= 4)
		  {
			  if(HAL_GPIO_ReadPin(btn_up_GPIO_Port, btn_up_Pin))
			  {
//				  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
			  }
			  btn_up_flag = 0;
		  }
	  }
	  else if(btn_left_flag)
	  {
		  if(HAL_GetTick() - last_ticks >= 4)
		  {
			  if(HAL_GPIO_ReadPin(btn_left_GPIO_Port, btn_left_Pin))
			  {
//				  HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
			  }
			  btn_left_flag = 0;
		  }
	  }
	  else if(btn_down_flag)
	  {
		  if(HAL_GetTick() - last_ticks >= 4)
		  {
			  if(HAL_GPIO_ReadPin(btn_down_GPIO_Port, btn_down_Pin))
			  {
//				  HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
			  }
			  btn_down_flag = 0;
		  }
	  }
	  else if(btn_right_flag)
	  {
		  if(HAL_GetTick() - last_ticks >= 4)
		  {
			  if(HAL_GPIO_ReadPin(btn_right_GPIO_Port, btn_right_Pin))
			  {
//				  HAL_GPIO_TogglePin(LD5_GPIO_Port, LD5_Pin);
			  }
			  btn_right_flag = 0;
		  }
	  }
	  else if(btn_mid_flag)
	  {
		  if(HAL_GetTick() - last_ticks >= 4)
		  {
			  if(HAL_GPIO_ReadPin(btn_mid_GPIO_Port, btn_mid_Pin))
			  {
				  // Toggle Menu Display state
				  if(display_state == 0){
					  changeDisplayState(1);
				  } else if(display_state == 1){
					  changeDisplayState(0);
				  }
//				  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
			  }
			  btn_mid_flag = 0;
		  }
	  }

	  // ADC TIM16 interrupt
	  if(adc_timer_flag)
	  {
		  if(adc_count > 999)
		  {
			  // Do calculations every 1000 readings
			  adc_count = 0;
			  uint32_t total = 0;
			  uint16_t max = 0;
			  uint16_t min = adc_array[99]; // arbitrary value
			  int16_t diff = 0;
			  int16_t prev_diff = 0;
			  uint16_t mid_passes = 0;
			  // 1000 measurements at 5kHz take 200ms
			  for(int x = 0; x < 1000; x++)
			  {
				  total += adc_array[x];
				  if(adc_array[x] > max)
				  {
					  max = adc_array[x];
				  }
				  else if(adc_array[x] < min)
				  {
					  min = adc_array[x];
				  }
			  }
			  offset = total/1000;
			  for(int x = 0; x < 1000; x++)
			  {
				  // Calculate frequency
				  diff = adc_array[x] - offset;
				  if((diff * prev_diff) < 0)
				  {
					  mid_passes++;
				  }
				  prev_diff = diff;
			  }
			  period = 0.2/(0.5*mid_passes);
//			  frequency = 1/period;
			  frequency = 5000;
			  amplitude = max - min;
			  sprintf(msg, "Offset: %u\nMax: %u\nMin: %u\n \n", offset, max, min);
//			  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 10);
		  }
		  else
		  {
			  HAL_ADC_Start(&hadc1);
			  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
			  raw = HAL_ADC_GetValue(&hadc1);
			  HAL_ADC_Stop(&hadc1);
			  millivolts = raw*3300/4095;
			  adc_array[adc_count] = millivolts;
			  adc_count++;
		  }

		  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_4);
		  adc_timer_flag = 0;
	  }

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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_TIM16
                              |RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.Tim16ClockSelection = RCC_TIM16CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode 
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 72-1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 200 - 1;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|LD5_Pin|LD4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin LD5_Pin LD4_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|LD5_Pin|LD4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : btn_mid_Pin */
  GPIO_InitStruct.Pin = btn_mid_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(btn_mid_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : btn_right_Pin */
  GPIO_InitStruct.Pin = btn_right_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(btn_right_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC4 LD3_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_4|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : btn_down_Pin btn_left_Pin btn_up_Pin */
  GPIO_InitStruct.Pin = btn_down_Pin|btn_left_Pin|btn_up_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	// Check which version of the timer triggered this interrupt
	if(htim == &htim16)
	{
		adc_timer_flag = 1;
	}
}

uint8_t changeDisplayState(uint8_t code)
{
	if (code == 0)
	{
		// Change to Menu Display State
		display_state = 0;
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_RESET);
		return 0;
	}
	else if (code == 1)
	{
		// Change to Measurement Display State
		display_state = 1;
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_RESET);
		return 1;
	}
	else if (code == 2)
	{
		// Change to Output Display State
		display_state = 2;
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_RESET);
		return 2;
	}
	else
	{
		// An error has occurred
		return 3;
	}
}

bool uartRxComplete(uint8_t last_byte)
{
	if(last_byte == '!')
	{
		return true;
	}
	else
	{
		return false;
	}
}

void interpret_rx_message(uint8_t *rx_array, uint8_t length)
{
//	for(int x = 0; x < length; x++)
//	{
//		// pass
//	}

//	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_RESET);

	if(length > 7)
	{
		if(rx_array[2] == '*')
		{
			// Requests
//			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
			switch(rx_array[4])
			{
				case 'm':
					// Request Measurement
					request_measurement(rx_array[6]);
					break;

				case 's':
					// Request Status
					request_status(rx_array[6]);
					break;

				default:
					// Problems
					break;
			}
		}
		else if(rx_array[2] == '$')
		{
			// Set
//			HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
			uint8_t key1 = rx_array[4];
			uint8_t key2 = rx_array[5];
			if(key1 == 'D' && key2 == 'V'){
				// DC Voltage
				measurement_mode = 0;
			} else if (key1 == 'A' && key2 == 'V'){
				// AC Voltage
				measurement_mode = 1;
			} else if (key1 == 'D' && key2 == 'I'){
				// DC Current
				measurement_mode = 2;
			} else if (key1 == 'A' && key2 == 'I'){
				// AC Current
				measurement_mode = 3;
			} else if (key1 == 'T' && key2 == 'C'){
				// Temperature
				measurement_mode = 4;
			}
		}
		else
		{
//			HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET);
		}
	}
}

void request_measurement(uint8_t parameter)
{
	char msg[20];
	switch(parameter){
		case 't':
			// Type
			break;
		case 'a':
			// Amplitude (peak-to-peak)
			sprintf(msg, "@,m,a,%u,!\n", amplitude);
			HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 10);
			HAL_UART_Receive_IT(&huart2, rx_byte, 1);
			break;
		case 'o':
			// Offset
			sprintf(msg, "@,m,o,%u,!\n", offset);
			HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 10);
			HAL_UART_Receive_IT(&huart2, rx_byte, 1);
			break;
		case 'f':
			// Frequency
			sprintf(msg, "@,m,f,%u,!\n", frequency);
			HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 10);
			HAL_UART_Receive_IT(&huart2, rx_byte, 1);
			break;
		case 'd':
			// Duty Cycle
			break;
		case 'c':
			// Temperature
			break;
		default:
			// Problems
			break;
	}
}

void request_status(uint8_t output)
{
	char msg[20];
	if(output == '1')
	{
		// Output On
		output_active = 1;
		switch(measurement_mode){
			case 0:
				// DV
				sprintf(msg, "@,DV,p,%u,!\n", output_active);
				break;
			case 1:
				// AV
				sprintf(msg, "@,AV,p,%u,!\n", output_active);
				break;
			case 2:
				// DI
				sprintf(msg, "@,DI,p,%u,!\n", output_active);
				break;
			case 3:
				// AI
				sprintf(msg, "@,AI,p,%u,!\n", output_active);
				break;
			case 4:
				// TC
				sprintf(msg, "@,TC,p,%u,!\n", output_active);
				break;
		}
		HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 10);
		HAL_UART_Receive_IT(&huart2, rx_byte, 1);
	}
	else if (output == '0')
	{
		// Output Off
		output_active = 0;
		switch(measurement_mode){
			case 0:
				// DV
				sprintf(msg, "@,DV,p,%u,!\n", output_active);
				break;
			case 1:
				// AV
				sprintf(msg, "@,AV,p,%u,!\n", output_active);
				break;
			case 2:
				// DI
				sprintf(msg, "@,DI,p,%u,!\n", output_active);
				break;
			case 3:
				// AI
				sprintf(msg, "@,AI,p,%u,!\n", output_active);
				break;
			case 4:
				// TC
				sprintf(msg, "@,TC,p,%u,!\n", output_active);
				break;
		}
		HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 10);
		HAL_UART_Receive_IT(&huart2, rx_byte, 1);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
