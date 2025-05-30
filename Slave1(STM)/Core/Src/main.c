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
#include "stdio.h"
#include "stdint.h"
#include "Neopixel.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// Communication
#define RxSIZE  1
#define TxSIZE  1
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
DMA_HandleTypeDef hdma_tim1_ch1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

// Communication
uint8_t RxData[RxSIZE];					// Rx Buffer
uint8_t TxData[TxSIZE] = {0};					// Tx Buffer

// Neopixel colors
float brilloled;

// Sensors variables
uint8_t start = 0;
uint8_t keys_memory[8]={0,0,0,0,0,0,0,0};
uint8_t key[8]={0,0,0,0,0,0,0,0};
uint8_t previousStateI2C[4] = {0,0,0,0};
uint8_t currentStateI2C[4] = {0,0,0,0};

// Display 7 segmentos
uint8_t keys = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
void showDisplay(uint8_t number);


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
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
	// Neopixels Settings (unique color all the time)

	//I2C Communication
	if(HAL_I2C_EnableListen_IT(&hi2c1) != HAL_OK){
	  Error_Handler();
	}

	// Display 7 segmentos
	showDisplay(keys);


	GPIO_PinState currentState = HAL_GPIO_ReadPin(S1_GPIO_Port, S1_Pin);
	switch (currentState) {
	  case GPIO_PIN_SET: key[0] = 0; break;
	  case GPIO_PIN_RESET: key[0] = 1; break;
	  default: break;
	}

	currentState = HAL_GPIO_ReadPin(S2_GPIO_Port, S2_Pin);
	switch (currentState) {
		case GPIO_PIN_SET: key[1] = 0; break;
		case GPIO_PIN_RESET: key[1] = 1; break;
		default: break;
	}

	currentState = HAL_GPIO_ReadPin(S3_GPIO_Port, S3_Pin);
	switch (currentState) {
	  	  case GPIO_PIN_SET: key[2] = 0; break;
		case GPIO_PIN_RESET: key[2] = 1; break;
		default: break;
	}

	currentState = HAL_GPIO_ReadPin(S4_GPIO_Port, S4_Pin);
	switch (currentState){
			case GPIO_PIN_SET: key[3] = 0; break;
			case GPIO_PIN_RESET: key[3] = 1; break;
			default: break;
	}

	start = 1;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(start == 1){
		  pixelClear();
		  for(int i = 0; i< 8; i++){
			  if(keys_memory[i] == 1){
				  setPixelColor(i, 255,  0,  0);
			  }
			  else{
				  setPixelColor(i, 0,  255,  0);
			  }
			  HAL_Delay(2);
		  }
		  setBrightness(20);
		  pixelShow();
		  // Display 7 segmentos
		  showDisplay(8-keys);
		  start = 0;
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  hi2c1.Init.OwnAddress1 = 64;
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
  htim1.Init.Period = 150-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

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
  HAL_GPIO_WritePin(GPIOC, A_Pin|B_Pin|G_Pin|F_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, E_Pin|D_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(C_GPIO_Port, C_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : A_Pin B_Pin G_Pin F_Pin */
  GPIO_InitStruct.Pin = A_Pin|B_Pin|G_Pin|F_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : E_Pin D_Pin */
  GPIO_InitStruct.Pin = E_Pin|D_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : S1_Pin S2_Pin */
  GPIO_InitStruct.Pin = S1_Pin|S2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : C_Pin */
  GPIO_InitStruct.Pin = C_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(C_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : S3_Pin S4_Pin */
  GPIO_InitStruct.Pin = S3_Pin|S4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* Display 7 segmentos*/
void showDisplay(uint8_t number) {	// It updates every display's pin, it compares every pattern bit to set or reset the display pin
	// Numbers for display, hex, 0-F
	uint8_t displayN[] = {0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F,0x77,0x7C,0x39,0x5E,0x79,0x71};
	// pin port, add dot if necessary
	GPIO_TypeDef *display_ports[] = {A_GPIO_Port, B_GPIO_Port, C_GPIO_Port, D_GPIO_Port, E_GPIO_Port, F_GPIO_Port, G_GPIO_Port};
	// pin number, add dot if necessary
	uint16_t display_pins[] = {A_Pin, B_Pin, C_Pin, D_Pin, E_Pin, F_Pin, G_Pin};
	// Not using dot, if using dot, k<8
	for (int k = 0; k < 7; k++) HAL_GPIO_WritePin(display_ports[k], display_pins[k], (displayN[number]>> k) & 1);
}

/* External Interruptions: Sensors */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == S1_Pin && start == 0){
			GPIO_PinState currentState1 = HAL_GPIO_ReadPin(S1_GPIO_Port, S1_Pin);
			switch (currentState1) {
				case GPIO_PIN_SET: key[0] = 0; break;
				case GPIO_PIN_RESET: key[0] = 1; break;
				default: break;
			}
		}

		if(GPIO_Pin == S2_Pin && start == 0){
			GPIO_PinState currentState2 = HAL_GPIO_ReadPin(S2_GPIO_Port, S2_Pin);
			switch (currentState2) {
				case GPIO_PIN_SET: key[1] = 0; break;
				case GPIO_PIN_RESET: key[1] = 1; break;
				default: break;
			}
		}

		if(GPIO_Pin == S3_Pin && start == 0){
			GPIO_PinState currentState3 = HAL_GPIO_ReadPin(S3_GPIO_Port, S3_Pin);
			switch (currentState3) {
				case GPIO_PIN_SET: key[2] = 0; break;
				case GPIO_PIN_RESET: key[2] = 1; break;
				default: break;
			}
		}

		if(GPIO_Pin == S4_Pin && start == 0){
			GPIO_PinState currentState4 = HAL_GPIO_ReadPin(S4_GPIO_Port, S4_Pin);
			switch (currentState4) {
				case GPIO_PIN_SET: key[3] = 0; break;
				case GPIO_PIN_RESET: key[3] = 1; break;
				default: break;
			}
		}
}

/* I2C Callbacks */
void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c){													//At the end of the each data analysis
	HAL_I2C_EnableListen_IT(hi2c);
	keys = 0;
	uint8_t acceptData = 0;
	for (int i = 0; i < 8; i++) if(keys_memory[i] != (RxData[0] & (1 << i))) acceptData = 1;
	if(acceptData){
		for (int i = 0; i < 8; i++) keys_memory[i] = 0;
		for (int i = 0; i < 8; i++) {
			if((RxData[0] & (1 << i))) {
				keys++;
				keys_memory[i] = 1;
			}
		}
		start = 1;
		acceptData = 0;
	}
	TxData[0] = 0;
	for (int j = 0; j < 4; j++){
		if(key[j]==1){
			TxData[0] |= (1<<j);
		}
	}
}
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *I2cHandle){												//Tx Transfer completed callback -> End Transmission
}
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *I2cHandle){												//Rx Transfer completed callback -> End Receiving
	//HAL_UART_Transmit(&huart2, RxData, 4, 1000);
}
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)		// Slave Address Match Callback
{
	if (TransferDirection == I2C_DIRECTION_TRANSMIT)  // if the master wants to transmit the data
	{
		if (HAL_I2C_Slave_Seq_Receive_IT(&hi2c1, (uint8_t *) RxData, RxSIZE, I2C_FIRST_AND_LAST_FRAME) != HAL_OK){
			Error_Handler();
		}
	}
	else if (TransferDirection == I2C_DIRECTION_RECEIVE){
		if (HAL_I2C_Slave_Seq_Transmit_IT(&hi2c1, (uint8_t *) TxData, TxSIZE, I2C_FIRST_AND_LAST_FRAME) != HAL_OK){
			Error_Handler();
		}
	}
}
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *I2cHandle)													// I2C Communication Error callback
{
	if(HAL_I2C_GetError(I2cHandle) != HAL_I2C_ERROR_AF){
		Error_Handler();
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
