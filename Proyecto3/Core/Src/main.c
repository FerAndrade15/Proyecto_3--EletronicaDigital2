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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fatfs_sd.h"
#include "string.h"
#include "stdio.h"
#include "ili9341.h"
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
FATFS fs;
FATFS *pfs;
FIL fil;
FRESULT fres;
DWORD fre_clust;
uint32_t totalSpace, freeSpace;

uint8_t buffer[1];
char cars[9][20] = {
		"carro1.bin", "carro2.bin", "carro3.bin", "carro4.bin",
		"carro5.bin", "carro6.bin", "carro7.bin", "carro8.bin", "empty.bin"};
char numbers[9][20] = {
		"cero.bin", "uno.bin", "dos.bin", "tres.bin",
		"cuatro.bin", "cinco.bin", "seis.bin", "siete.bin", "ocho.bin"};
uint8_t start = 0;
uint8_t key[8]={0,0,0,0,0,0,0,0};
uint8_t previousStateI2C[4] = {0,0,0,0};
uint8_t currentStateI2C[4] = {0,0,0,0};
uint8_t spaces = 0;
uint8_t cont = 0;



#define RxSIZE  1
#define TxSIZE  1
#define COLOR_RED_H     0xF8
#define COLOR_RED_L     0x00
#define COLOR_GREEN_H   0x07
#define COLOR_GREEN_L   0xE0
#define COLOR_BG_H      0x5A
#define COLOR_BG_L      0xAB

// Buffer I2C
uint8_t RxData[RxSIZE];					// Rx Buffer
uint8_t TxData[TxSIZE] = {0};					// Tx Buffer

const uint8_t square_red_circle_10x10_max_bytes[200] = {
    // Fila 0
    COLOR_BG_H, COLOR_BG_L, 0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00, COLOR_BG_H, COLOR_BG_L,
    // Fila 1
    0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00,
    // Fila 2
    0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00,
    // Fila 3
    0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00,
    // Fila 4
    0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00,
    // Fila 5
    0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00,
    // Fila 6
    0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00,
    // Fila 7
    0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00,
    // Fila 8
    0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00,
    // Fila 9
    COLOR_BG_H, COLOR_BG_L, 0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00, 0xF8, 0x00, COLOR_BG_H, COLOR_BG_L
};

// Arreglo para el cuadrado de 10x10 con el círculo VERDE más grande
const uint8_t square_green_circle_10x10_max_bytes[200] = {
    // Fila 0
    COLOR_BG_H, COLOR_BG_L, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, COLOR_BG_H, COLOR_BG_L,
    // Fila 1
    0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0,
    // Fila 2
    0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0,
    // Fila 3
    0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0,
    // Fila 4
    0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0,
    // Fila 5
    0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0,
    // Fila 6
    0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0,
    // Fila 7
    0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0,
    // Fila 8
    0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0,
    // Fila 9
    COLOR_BG_H, COLOR_BG_L, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, COLOR_BG_H, COLOR_BG_L
};
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
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
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart2, buffer, 1);

  //I2C Communication
  if(HAL_I2C_EnableListen_IT(&hi2c1) != HAL_OK){
	  Error_Handler();
  }

  LCD_Init();
  LCD_Clear(0x0000);
  //LCD_Print("Hola Mundo", 20, 100, 1, 0x001F, 0xCAB9);
  fres = f_mount(&fs, "/", 0);	// Mounting system

  //LCD_Bitmap(0, 0, 320, 240, fondo);
  //LCD_ImageFromSD(0, 0, 240, 240, "prueba.bin");

  //HAL_Delay(1000);
  //LCD_Clear(0x0000);
  LCD_ImageFromSD(0, 0, 320, 240, "parking.bin");
  LCD_Print(" Fer-22676 ", 230, 15, 1, 0, 0xffff);
  LCD_Print(" Alex-22648", 230, 25, 1, 0, 0xffff);
  LCD_Print("  Parking  ", 230, 50, 1, 0, 0xffff);
  LCD_Print("  spaces   ", 230, 62, 1, 0, 0xffff);
  LCD_Print(" available ", 230, 74, 1, 0, 0xffff);


  /*
  LCD_ImageFromSD(0, 0, 230, 240, "emptyParking.bin");
  LCD_ImageFromSD(26, 20, 40, 80, cars[0]);
  LCD_ImageFromSD(76, 20, 40, 80, cars[1]);
  LCD_ImageFromSD(124, 20, 40, 80, cars[2]);
  LCD_ImageFromSD(174, 20, 40, 80, cars[3]);
  LCD_ImageFromSD(26, 140, 40, 80, cars[4]);
  LCD_ImageFromSD(76, 140, 40, 80, cars[5]);
  LCD_ImageFromSD(124, 140, 40, 80, cars[6]);
  LCD_ImageFromSD(174, 140, 40, 80, cars[7]);
  */

  LCD_ImageFromSD(26, 20, 40, 80, cars[8]);
  LCD_ImageFromSD(76, 20, 40, 80, cars[8]);
  LCD_ImageFromSD(124, 20, 40, 80, cars[8]);
  LCD_ImageFromSD(174, 20, 40, 80, cars[8]);
  LCD_ImageFromSD(26, 140, 40, 80, cars[8]);
  LCD_ImageFromSD(76, 140, 40, 80, cars[8]);
  LCD_ImageFromSD(124, 140, 40, 80, cars[8]);
  LCD_ImageFromSD(174, 140, 40, 80, cars[8]);

  LCD_Bitmap(41, 5, 10, 10, square_red_circle_10x10_max_bytes);
  LCD_Bitmap(91, 5, 10, 10, square_red_circle_10x10_max_bytes);
  LCD_Bitmap(139, 5, 10, 10, square_red_circle_10x10_max_bytes);
  LCD_Bitmap(189, 5, 10, 10, square_red_circle_10x10_max_bytes);
  LCD_Bitmap(41, 225, 10, 10, square_red_circle_10x10_max_bytes);
  LCD_Bitmap(91, 225, 10, 10, square_red_circle_10x10_max_bytes);
  LCD_Bitmap(139, 225, 10, 10, square_red_circle_10x10_max_bytes);
  LCD_Bitmap(189, 225, 10, 10, square_red_circle_10x10_max_bytes);

  LCD_Bitmap(41, 5, 10, 10, square_green_circle_10x10_max_bytes);
  LCD_Bitmap(91, 5, 10, 10, square_green_circle_10x10_max_bytes);
  LCD_Bitmap(139, 5, 10, 10, square_green_circle_10x10_max_bytes);
  LCD_Bitmap(189, 5, 10, 10, square_green_circle_10x10_max_bytes);
  LCD_Bitmap(41, 225, 10, 10, square_green_circle_10x10_max_bytes);
  LCD_Bitmap(91, 225, 10, 10, square_green_circle_10x10_max_bytes);
  LCD_Bitmap(139, 225, 10, 10, square_green_circle_10x10_max_bytes);
  LCD_Bitmap(189, 225, 10, 10, square_green_circle_10x10_max_bytes);

  LCD_ImageFromSD(255, 100, 40, 80, numbers[0]);
  LCD_ImageFromSD(255, 100, 40, 80, numbers[1]);
  LCD_ImageFromSD(255, 100, 40, 80, numbers[2]);
  LCD_ImageFromSD(255, 100, 40, 80, numbers[3]);
  LCD_ImageFromSD(255, 100, 40, 80, numbers[4]);
  LCD_ImageFromSD(255, 100, 40, 80, numbers[5]);
  LCD_ImageFromSD(255, 100, 40, 80, numbers[6]);
  LCD_ImageFromSD(255, 100, 40, 80, numbers[7]);
  LCD_ImageFromSD(255, 100, 40, 80, numbers[8]);

  GPIO_PinState currentState = HAL_GPIO_ReadPin(sensor1_GPIO_Port, sensor1_Pin);
  switch (currentState) {
  	  case GPIO_PIN_SET: key[0] = 2; break;
  	  case GPIO_PIN_RESET: key[0] = 1; break;
  	  default: break;
  }

  currentState = HAL_GPIO_ReadPin(sensor2_GPIO_Port, sensor2_Pin);
  switch (currentState) {
  	  case GPIO_PIN_SET: key[1] = 2; break;
	  case GPIO_PIN_RESET: key[1] = 1; break;
	  default: break;
  }

  currentState = HAL_GPIO_ReadPin(sensor3_GPIO_Port, sensor3_Pin);
  switch (currentState) {
  	  case GPIO_PIN_SET: key[2] = 2; break;
      case GPIO_PIN_RESET: key[2] = 1; break;
      default: break;
  }

  currentState = HAL_GPIO_ReadPin(sensor4_GPIO_Port, sensor4_Pin);
  switch (currentState) {
	  case GPIO_PIN_SET: key[3] = 2; break;
	  case GPIO_PIN_RESET: key[3] = 1; break;
	  default: break;
  }

  start = 1;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(key[0] == 1){
		  LCD_ImageFromSD(26, 20, 40, 80, cars[0]);
		  key[0] = 3; HAL_Delay(1);
		  LCD_Bitmap(41, 5, 10, 10, square_red_circle_10x10_max_bytes);
		  spaces |= (1 << 0);
	  }else if(key[0] == 2){
		  LCD_ImageFromSD(26, 20, 40, 80, cars[8]);
		  key[0] = 3; HAL_Delay(1);
		  LCD_Bitmap(41, 5, 10, 10, square_green_circle_10x10_max_bytes);
		  spaces &= ~(1 << 0);
	  }

	  if(key[1] == 1){
		  LCD_ImageFromSD(76, 20, 40, 80, cars[1]);
		  key[1] = 3; HAL_Delay(1);
		  LCD_Bitmap(91, 5, 10, 10, square_red_circle_10x10_max_bytes);
		  spaces |= (1 << 1);
	  }else if(key[1] == 2){
		  LCD_ImageFromSD(76, 20, 40, 80, cars[8]);
		  key[1] = 3; HAL_Delay(1);
		  LCD_Bitmap(91, 5, 10, 10, square_green_circle_10x10_max_bytes);
		  spaces &= ~(1 << 1);
	  }

	  if(key[2] == 1){
		  LCD_ImageFromSD(124, 20, 40, 80, cars[2]);
		  key[2] = 3; HAL_Delay(1);
		  LCD_Bitmap(139, 5, 10, 10, square_red_circle_10x10_max_bytes);
		  spaces |= (1 << 2);
	  }else if(key[2] == 2){
		  LCD_ImageFromSD(124, 20, 40, 80, cars[8]);
		  key[2] = 3; HAL_Delay(1);
		  LCD_Bitmap(139, 5, 10, 10, square_green_circle_10x10_max_bytes);
		  spaces &= ~(1 << 2);
	  }

	  if(key[3] == 1){
		  LCD_ImageFromSD(174, 20, 40, 80, cars[3]);
		  key[3] = 3; HAL_Delay(1);
		  LCD_Bitmap(189, 5, 10, 10, square_red_circle_10x10_max_bytes);
		  spaces |= (1 << 3);
	  }else if(key[3] == 2){
		  LCD_ImageFromSD(174, 20, 40, 80, cars[8]);
		  key[3] = 3; HAL_Delay(1);
		  LCD_Bitmap(189, 5, 10, 10, square_green_circle_10x10_max_bytes);
		  spaces &= ~(1 << 3);
	  }

	  if(key[4] == 1){
		  LCD_ImageFromSD(26, 140, 40, 80, cars[4]);
		  key[4] = 3; HAL_Delay(1);
		  LCD_Bitmap(41, 225, 10, 10, square_red_circle_10x10_max_bytes);
		  spaces |= (1 << 4);
	  }else if(key[4] == 2){
		  LCD_ImageFromSD(26, 140, 40, 80, cars[8]);
		  key[4] = 3; HAL_Delay(1);
		  LCD_Bitmap(41, 225, 10, 10, square_green_circle_10x10_max_bytes);
		  spaces &= ~(1 << 4);
	  }

	  if(key[5] == 1){
		  LCD_ImageFromSD(76, 140, 40, 80, cars[5]);
		  key[5] = 3; HAL_Delay(1);
		  LCD_Bitmap(91, 225, 10, 10, square_red_circle_10x10_max_bytes);
		  spaces |= (1 << 5);
	  }else if(key[5] == 2){
		  LCD_ImageFromSD(76, 140, 40, 80, cars[8]);
		  key[5] = 3; HAL_Delay(1);
		  LCD_Bitmap(91, 225, 10, 10, square_green_circle_10x10_max_bytes);
		  spaces &= ~(1 << 5);
	  }

	  if(key[6] == 1){
		  LCD_ImageFromSD(124, 140, 40, 80, cars[6]);
		  key[6] = 3; HAL_Delay(1);
		  LCD_Bitmap(139, 225, 10, 10, square_red_circle_10x10_max_bytes);
		  spaces |= (1 << 6);
	  }else if(key[6] == 2){
		  LCD_ImageFromSD(124, 140, 40, 80, cars[8]);
		  key[6] = 3; HAL_Delay(1);
		  LCD_Bitmap(139, 225, 10, 10, square_green_circle_10x10_max_bytes);
		  spaces &= ~(1 << 6);
	  }

	  if(key[7] == 1){
		  LCD_ImageFromSD(174, 140, 40, 80, cars[7]);
		  key[7] = 3; HAL_Delay(1);
		  LCD_Bitmap(189, 225, 10, 10, square_red_circle_10x10_max_bytes);
		  spaces |= (1 << 7);
	  }else if(key[7] == 2){
		  LCD_ImageFromSD(174, 140, 40, 80, cars[8]);
		  key[7] = 3; HAL_Delay(1);
		  LCD_Bitmap(189, 225, 10, 10, square_green_circle_10x10_max_bytes);
		  spaces &= ~(1 << 7);
	  }


	  for(int i =  0; i < 8; i++)
		  if(key[i] == 3){
			  key[i] = 0;			// restart key
			  cont = 0;				// restart spaces counter
			  for(int j = 0; j < 8; j++) spaces & (1 << j) ? (cont++) : (cont = cont);		// counting from bits
			  LCD_ImageFromSD(255, 100, 40, 80, numbers[8-cont]);
		  }
	  HAL_Delay(1);
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
  RCC_OscInitStruct.PLL.PLLN = 160;
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
  hi2c1.Init.OwnAddress1 = 96;
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LCD_RST_Pin|LCD_D1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LCD_RD_Pin|LCD_WR_Pin|LCD_RS_Pin|LCD_D7_Pin
                          |LCD_D0_Pin|LCD_D2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_CS_Pin|LCD_D6_Pin|LCD_D3_Pin|LCD_D5_Pin
                          |LCD_D4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_SS_GPIO_Port, SD_SS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_RST_Pin LCD_D1_Pin */
  GPIO_InitStruct.Pin = LCD_RST_Pin|LCD_D1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : sensor3_Pin sensor4_Pin sensor1_Pin */
  GPIO_InitStruct.Pin = sensor3_Pin|sensor4_Pin|sensor1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_RD_Pin LCD_WR_Pin LCD_RS_Pin LCD_D7_Pin
                           LCD_D0_Pin LCD_D2_Pin */
  GPIO_InitStruct.Pin = LCD_RD_Pin|LCD_WR_Pin|LCD_RS_Pin|LCD_D7_Pin
                          |LCD_D0_Pin|LCD_D2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_CS_Pin LCD_D6_Pin LCD_D3_Pin LCD_D5_Pin
                           LCD_D4_Pin */
  GPIO_InitStruct.Pin = LCD_CS_Pin|LCD_D6_Pin|LCD_D3_Pin|LCD_D5_Pin
                          |LCD_D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : sensor2_Pin */
  GPIO_InitStruct.Pin = sensor2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(sensor2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_SS_Pin */
  GPIO_InitStruct.Pin = SD_SS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(SD_SS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == sensor1_Pin && start == 1){
		GPIO_PinState currentState1 = HAL_GPIO_ReadPin(sensor1_GPIO_Port, sensor1_Pin);
		switch (currentState1) {
			case GPIO_PIN_SET: key[0] = 2; break;
			case GPIO_PIN_RESET: key[0] = 1; break;
			default: break;
		}
	}

	if(GPIO_Pin == sensor2_Pin && start == 1){
		GPIO_PinState currentState2 = HAL_GPIO_ReadPin(sensor2_GPIO_Port, sensor2_Pin);
		switch (currentState2) {
			case GPIO_PIN_SET: key[1] = 2; break;
			case GPIO_PIN_RESET: key[1] = 1; break;
			default: break;
		}
	}

	if(GPIO_Pin == sensor3_Pin && start == 1){
		GPIO_PinState currentState3 = HAL_GPIO_ReadPin(sensor3_GPIO_Port, sensor3_Pin);
		switch (currentState3) {
			case GPIO_PIN_SET: key[2] = 2; break;
			case GPIO_PIN_RESET: key[2] = 1; break;
			default: break;
		}
	}

	if(GPIO_Pin == sensor4_Pin && start == 1){
		GPIO_PinState currentState4 = HAL_GPIO_ReadPin(sensor4_GPIO_Port, sensor4_Pin);
		switch (currentState4) {
			case GPIO_PIN_SET: key[3] = 2; break;
			case GPIO_PIN_RESET: key[3] = 1; break;
			default: break;
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

	for(int i = 0; i < 4; i++){
		currentStateI2C[i] = buffer[0] & (1 << i) ? 1:0;
		if(previousStateI2C[i] == currentStateI2C[i]) key[i + 4] = 0;		// verifying bits
		else{																// bit's changed
			previousStateI2C[i] = currentStateI2C[i];						// load new value
			key[i + 4] = currentStateI2C[i] == 1 ? 1:2;						// key assignment
		}
	}

	HAL_UART_Receive_IT(&huart2, buffer, 1);

}



/* I2C Callbacks */
void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c){													//At the end of the firts data analysis
	TxData[0] = (spaces &= 0x0F);
	buffer[0] = RxData[0];
	for(int i = 0; i < 4; i++){
		currentStateI2C[i] = buffer[0] & (1 << (i+4)) ? 1:0;
		if(previousStateI2C[i] == currentStateI2C[i]) key[i + 4] = 0;		// verifying bits
		else{																// bit's changed
			previousStateI2C[i] = currentStateI2C[i];						// load new value
			key[i + 4] = currentStateI2C[i] == 1 ? 1:2;						// key assignment
		}
	}
	HAL_I2C_EnableListen_IT(hi2c);
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
