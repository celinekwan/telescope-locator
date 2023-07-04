/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define adxl_address 0x53<<1 // accelerometer

// from screen_cmds.h
#define SCREEN_ADDR	0x78									// i2c address from datasheet

#define COMMAND_CLEAR_DISPLAY						0x01
#define COMMAND_RETURN_HOME							0x02
#define COMMAND_ENTRY_MODE_SET						0x04
#define ENTRY_MODE_LEFT_TO_RIGHT					0x02
#define ENTRY_MODE_SHIFT_INCREMENT					0x01
#define COMMAND_CURSORSHIFT							0x10
#define CURSORSHIFT_MOVERIGHT						0x04
#define CURSORSHIFT_MOVELEFT						0x00

#define COMMAND_ADDRESS_DDRAM						0x80
#define COMMAND_ADDRESS_CGRAM                       0x40

#define COMMAND_8BIT_4LINES_NORMAL_RE1_IS0			0x3A  //Extended command access RE = 1
#define COMMAND_8BIT_4LINES_REVERSE_RE1_IS0			0x3B  //Extended command access RE = 1
#define COMMAND_8BIT_4LINES_RE0_IS1					0x39  //Extended command access IS = 1
#define COMMAND_8BIT_4LINES_RE0_IS0					0x38  //Normal mode...


//Command from extended set (RE = 1, IS = 0)
#define COMMAND_BS1_1								0x1E
#define COMMAND_POWER_DOWN_DISABLE					0x02
#define COMMAND_SEGMENT_TOP_VIEW					0x05
#define COMMAND_SEGMENT_BOTTOM_VIEW					0x06
#define COMMAND_NW									0x09

//Command from extended set (RE = 0, IS = 1)
#define COMMAND_DISPLAY_ON_CURSOR_ON_BLINK_ON		0x0F
#define COMMAND_DISPLAY_ON_CURSOR_ON_BLINK_OFF		0x0E
#define COMMAND_DISPLAY_ON_CURSOR_OFF_BLINK_OFF		0x0C
#define COMMAND_DISPLAY_OFF_CURSOR_OFF_BLINK_OFF	0x08
#define COMMAND_BS0_1								0x1B
#define COMMAND_INTERNAL_DIVIDER					0x13
#define COMMAND_CONTRAST							0x77
#define COMMAND_POWER_ICON_CONTRAST					0x5C
#define COMMAND_FOLLOWER_CONTROL					0x6E
#define COMMAND_POWER_BOOSTER_CONTRAST				0x57		//Booster on and set contrast (DB1=C5, DB0=C4)
#define COMMAND_SET_CONTRAST_1010					0x7A		//Booster on and set contrast (DB1=C5, DB0=C4)

// Cursor Commands
#define COMMAND_SET_CURSOR_LINE_1					0x80		//Move the cursor to line 1
#define COMMAND_SET_CURSOR_LINE_2					0xA0		//Move the cursor to line 1
#define COMMAND_SET_CURSOR_LINE_3					0xC0		//Move the cursor to line 1
#define COMMAND_SET_CURSOR_LINE_4					0xE0		//Move the cursor to line 1

#define LINE_LENGTH 10 // number of chars in a line (4x10 screen)

float xg, yg, zg;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

// accelerometer stuff

uint8_t data_rec[6];

int16_t x, y, z; // original: uint16-t

float xg, yg, zg;

void display_data (float);

char x_char[3];

// write function
void adxl_write (uint8_t reg, uint8_t value) {
	uint8_t data[2];
	data[0] = reg;
	data[1] = value;
	HAL_I2C_Master_Transmit (&hi2c1, adxl_address, data, 2, 10);
}

// read function
void adxl_read (uint8_t reg, uint8_t numberofbytes) {
	HAL_I2C_Mem_Read (&hi2c1, adxl_address, reg, 1, data_rec, numberofbytes, 100);
}

// initialization
void adxl_init (void) {
	adxl_read(0x00,1); // 0xE5

	adxl_write (0x2d, 0); // reset all bits
	adxl_write (0x2d, 0x08); // measure bit-1, wakeup 0, 0 at 8hz

	adxl_write (0x31, 0x01); // +- 4g range
}

//void display_data(float data) { // TODO check if UART output formatting is correct
//	sprintf (x_char, "%.4f", data); // needed to add -u _printf_float linker flag -- might have to change to %.4f
//	HAL_UART_Transmit(&huart2,x_char,strlen((char*)x_char),HAL_MAX_DELAY);
//}

// LCD stuff

void I2Csendcmd(uint8_t addr, uint8_t cmd) {
	uint8_t data[2];
	data[0] = 0x00;
	data[1] = cmd; // data byte (not data, but command)
	HAL_I2C_Master_Transmit (&hi2c1, addr, data, 2, HAL_MAX_DELAY);
}

void I2Csenddatum (uint8_t addr, uint8_t data) { // I2Csenddatum(SCREEN_ADDR, string_to_write[letter]);
	uint8_t datum[2];
	datum[0] = 0x40;
	datum[1] = data; // data byte (not data, but command)
	HAL_I2C_Master_Transmit (&hi2c1, addr, datum, 2, HAL_MAX_DELAY);
}

void screen_init(void){

	I2Csendcmd(SCREEN_ADDR, COMMAND_CLEAR_DISPLAY); // 0x78, 0x01
	I2Csendcmd(SCREEN_ADDR, COMMAND_8BIT_4LINES_NORMAL_RE1_IS0); // 0x3A
	I2Csendcmd(SCREEN_ADDR, COMMAND_NW);
	I2Csendcmd(SCREEN_ADDR, COMMAND_SEGMENT_BOTTOM_VIEW);
	I2Csendcmd(SCREEN_ADDR, COMMAND_BS1_1);
	I2Csendcmd(SCREEN_ADDR, COMMAND_8BIT_4LINES_RE0_IS1);
	I2Csendcmd(SCREEN_ADDR, COMMAND_BS0_1);
	I2Csendcmd(SCREEN_ADDR, COMMAND_FOLLOWER_CONTROL);
	I2Csendcmd(SCREEN_ADDR, COMMAND_POWER_BOOSTER_CONTRAST);
	I2Csendcmd(SCREEN_ADDR, COMMAND_SET_CONTRAST_1010);
	I2Csendcmd(SCREEN_ADDR, COMMAND_8BIT_4LINES_RE0_IS0);
	I2Csendcmd(SCREEN_ADDR, COMMAND_DISPLAY_ON_CURSOR_ON_BLINK_ON);

	I2Csendcmd(SCREEN_ADDR, COMMAND_CLEAR_DISPLAY);

}

void screen_write_string(char string_to_write[]) {

	int letter=0; // index for string

	I2Csendcmd(SCREEN_ADDR, COMMAND_CLEAR_DISPLAY);
	I2Csendcmd(SCREEN_ADDR, COMMAND_SET_CURSOR_LINE_1);
	int current_line = COMMAND_SET_CURSOR_LINE_1;

	while(string_to_write[letter]!='\0') // check if end of string
	{
		if ((letter != 0) && (letter % LINE_LENGTH == 0)) // move to a next line if index is a multiple of 10
		{
			if (current_line == COMMAND_SET_CURSOR_LINE_4){ // clear screen and put cursor to line 1 when screen is filled
				current_line = COMMAND_SET_CURSOR_LINE_1;// We've gone past the end of the line, go to the next one
				I2Csendcmd(SCREEN_ADDR, COMMAND_CLEAR_DISPLAY);
			}
			else {
				current_line = current_line+0x20;
				I2Csendcmd(SCREEN_ADDR, current_line); // We've gone past the end of the line, go to the next one
			}
		}

		I2Csenddatum(SCREEN_ADDR, string_to_write[letter]);
		letter++;
	}

}

void screen_write_xyz (float x, float y, float z) {

	char str_x[11]; // not 10 bc dont want cursor to do funky stuff if reach multiple of 10
	char str_y[11];
	char str_z[11];
	char str_xyz[30];

	for (int i=0; i<30; i++) {
		str_xyz[i] = '\0';
	}
	// check if all chars initialized to NULL

	sprintf (str_x, "X:%.4f  ", x); // check str_x
	sprintf (str_y, "Y:%.4f  ", y); // check str_y
	sprintf (str_z, "Z:%.4f  ", z); // check str_z

	strcpy (str_xyz, str_x);
	strcat (str_xyz, str_y);
	strcat (str_xyz, str_z);
	// check str_xyz

	screen_write_string(str_xyz);

}

void screen_write_accel_data (float x, float y, float z) {
	I2Csendcmd(SCREEN_ADDR, COMMAND_CLEAR_DISPLAY);
	screen_write_xyz(x,y,z);
	HAL_Delay(100);
}

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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  // initialize accelerometer
  adxl_init();

  // turn off display, turn on display (translated from PORTD section of Stewart's code)
    HAL_Delay(5);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0,GPIO_PIN_RESET);
    HAL_Delay(200);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0,GPIO_PIN_SET);
    HAL_Delay(5);

    //1. clear screen
    screen_init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  /*// TODO: uncomment section
	  adxl_read (0x32, 6);
	  x = (data_rec[1]<<8) | data_rec[0];
	  y = (data_rec[3]<<8) | data_rec[2];
	  z = (data_rec[5]<<8) | data_rec[4];

	  xg = x * .0078; // convert the x value to the g
	  yg = y * .0078;
	  zg = z * .0078;

	  // display data to screen
	  screen_write_accel_data(xg,yg,zg);
	  */
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
  hi2c1.Init.Timing = 0x10909CEC;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD4_Pin */
  GPIO_InitStruct.Pin = LD4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD4_GPIO_Port, &GPIO_InitStruct);

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
