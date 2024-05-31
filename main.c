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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "stdbool.h"
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
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t volatile start_listen = 0;
uint8_t volatile transmit_uart = 0;
uint16_t adc_value[2];
float TempC = 0; 
float RH = 0; 
char data_TX[50]; 
uint8_t UV_index = 0;
uint32_t PPM = 0;
uint8_t clap_count = 0;


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == Enable_Button_Pin){
		if(start_listen == 1){
			start_listen = 0;
		}else{
			start_listen = 1;
		}
		while(HAL_GPIO_ReadPin(Enable_Button_GPIO_Port, Enable_Button_Pin) == 0);
		HAL_Delay(50);
		__HAL_GPIO_EXTI_CLEAR_IT(Enable_Button_Pin);
		HAL_NVIC_ClearPendingIRQ(Enable_Button_EXTI_IRQn);
	}else if(GPIO_Pin == Transmit_data_Button_Pin){
		transmit_uart = !transmit_uart;
		while(HAL_GPIO_ReadPin(Transmit_data_Button_GPIO_Port, Transmit_data_Button_Pin) == 0);
		HAL_Delay(50);
		__HAL_GPIO_EXTI_CLEAR_IT(Transmit_data_Button_Pin);
		HAL_NVIC_ClearPendingIRQ(Transmit_data_Button_EXTI_IRQn);
			
		if(transmit_uart == 1){
			HAL_TIM_Base_Start_IT(&htim2);
		__HAL_TIM_SET_COUNTER(&htim2, 0);
			HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_value, 2);
		}else{
			HAL_TIM_Base_Stop(&htim2);
			HAL_ADC_Stop_DMA(&hadc1);
			}
	}
}


void delay_us (uint16_t delay)
{
  __HAL_TIM_SET_COUNTER(&htim1, 0);
  while (__HAL_TIM_GET_COUNTER(&htim1) < delay);
}

uint8_t DHT22_Start (void)
{
  uint8_t Response = 0; 
  GPIO_InitTypeDef GPIO_InitStruct = {0}; 
	
  GPIO_InitStruct.Pin = DHT22_Sensor_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DHT22_Sensor_GPIO_Port, &GPIO_InitStruct); 
  HAL_GPIO_WritePin (DHT22_Sensor_GPIO_Port, DHT22_Sensor_Pin, 0); 
  delay_us (1300);
  HAL_GPIO_WritePin (DHT22_Sensor_GPIO_Port, DHT22_Sensor_Pin, 1);
  delay_us (30);
	
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(DHT22_Sensor_GPIO_Port, &GPIO_InitStruct); 
  delay_us (40);
	
  if (!(HAL_GPIO_ReadPin (DHT22_Sensor_GPIO_Port, DHT22_Sensor_Pin))){
    delay_us (80);
    if ((HAL_GPIO_ReadPin (DHT22_Sensor_GPIO_Port, DHT22_Sensor_Pin))){
			Response = 1;
		}
  }
	delay_us(80);
  return Response; 
}

uint8_t DHT22_Read (void)
{
  uint8_t result; 
  for (int i=0; i < 8; i++){
    uint32_t previous_time = HAL_GetTick();
    uint32_t current_time = HAL_GetTick();
    while (!(HAL_GPIO_ReadPin (DHT22_Sensor_GPIO_Port, DHT22_Sensor_Pin)) && current_time - previous_time < 1){ 
      current_time = HAL_GetTick();
    }
    delay_us(40);   
    if (!(HAL_GPIO_ReadPin (DHT22_Sensor_GPIO_Port, DHT22_Sensor_Pin))){
			result &= ~(1<<(7-i));
		}else{
			result |= (1<<(7-i));
		}
    previous_time = HAL_GetTick();
    current_time = HAL_GetTick();
    while ((HAL_GPIO_ReadPin (DHT22_Sensor_GPIO_Port, DHT22_Sensor_Pin)) && current_time - previous_time < 1){ 
      current_time = HAL_GetTick();
    }
  }
  return result; 
}

void DHT22_Get_Data(float *TempC, float *RH){
	Z: if(DHT22_Start()){
      uint8_t RH1 = DHT22_Read(); // Doc 8 bit dau tien cua do am
      uint8_t RH2 = DHT22_Read(); // Doc 8 bit tiep theo cua do am
      uint8_t TC1 = DHT22_Read(); // Doc 8 bit dau tien cua nhiet do
      uint8_t TC2 = DHT22_Read(); // Doc 8 bit tiep theo cua nhiet do
      uint8_t SUM = DHT22_Read(); // Doc gia tri checksum
      uint8_t CHECK = RH1 + RH2 + TC1 + TC2; //Tinh toan checksum
      if (CHECK == SUM){
				*TempC = (float)(((TC1 & 0x7F) << 8) | TC2) / 10;
				if(TC1 & 0x80){
					*TempC = -(*TempC);
				}
        *RH = (float) ((RH1 << 8) | RH2) / 10;
      }
    }
}

uint8_t voltage_to_UV_index(uint32_t uv_adc_value){
	uint8_t UV_index;
	if(uv_adc_value <= 225){
		UV_index = 1;
	}else if(uv_adc_value <= 320){
		UV_index = 2;
	}else if(uv_adc_value <= 405){
		UV_index = 3;
	}else if(uv_adc_value <= 500){
		UV_index = 4;
	}else if(uv_adc_value <= 600){
		UV_index = 5;
	}else if(uv_adc_value <= 700){
		UV_index = 6;
	}else if(uv_adc_value <= 800){
		UV_index = 7;
	}else if(uv_adc_value <= 880){
		UV_index = 8;
	}else if(uv_adc_value <= 975){
		UV_index = 9;
	}else if(uv_adc_value <= 1075){
		UV_index = 10;
	}else{
		UV_index = 11;
	}
	return UV_index;
}

uint32_t voltage_to_PPM(uint32_t air_adc_value){
	return ((adc_value[1]*3.3/4096)*332 + 5); 
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM2){
		DHT22_Get_Data(&TempC, &RH);
		UV_index = voltage_to_UV_index(adc_value[0]);
		PPM = voltage_to_PPM(adc_value[1]);
		sprintf(data_TX, "%.2f;%.2f;%d;%d\n", TempC, RH, UV_index, PPM);
		HAL_UART_Transmit(&huart1, (uint8_t *)data_TX, strlen(data_TX), 10);
	}
}

void enableDevice(uint8_t number){
	
	switch(number){
		case 1:
			HAL_GPIO_TogglePin(LED_1_GPIO_Port, LED_1_Pin);
			break;
		case 2:
			HAL_GPIO_TogglePin(LED_2_GPIO_Port, LED_2_Pin);
			break;
		case 3:
			HAL_GPIO_TogglePin(LED_3_GPIO_Port, LED_3_Pin);
			break;
		default:
			break;
	}
}

uint32_t x = 0;

void listen_and_act(){
	if(HAL_GPIO_ReadPin(Sound_Sensor_GPIO_Port, Sound_Sensor_Pin) == 1){
		x++;
		X: clap_count++;
		uint32_t current_time = HAL_GetTick();
		while(HAL_GetTick() - current_time < 30){
			if(HAL_GPIO_ReadPin(Sound_Sensor_GPIO_Port, Sound_Sensor_Pin) == 1){
				current_time = HAL_GetTick();
			}
		}
		if(clap_count < 3){
			current_time = HAL_GetTick();
			while(HAL_GetTick() - current_time < 1700){
	
				if(HAL_GPIO_ReadPin(Sound_Sensor_GPIO_Port, Sound_Sensor_Pin) == 1){
					goto X;
				}
			}
		}
		enableDevice(clap_count);
		clap_count = 0;
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
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start(&htim1);
	
	HAL_Delay(200);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if(start_listen == 1){
			listen_and_act();
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 35;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 35999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 199;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
	
  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_3_Pin|LED_2_Pin|LED_1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_3B11_Pin|DHT22_Sensor_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : Enable_Button_Pin Transmit_data_Button_Pin */
  GPIO_InitStruct.Pin = Enable_Button_Pin|Transmit_data_Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_3_Pin LED_2_Pin LED_1_Pin */
  GPIO_InitStruct.Pin = LED_3_Pin|LED_2_Pin|LED_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Sound_Sensor_Pin */
  GPIO_InitStruct.Pin = Sound_Sensor_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Sound_Sensor_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_3B11_Pin DHT22_Sensor_Pin */
  GPIO_InitStruct.Pin = LED_3B11_Pin|DHT22_Sensor_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

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
