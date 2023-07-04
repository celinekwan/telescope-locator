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
  * Date completed: Jun 13, 2023
  * Celine
  * Initializes RTC with date and time -> RTC will continue incrementing the time from there
  * SUCCESS
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include "RTClib.h"
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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
#define rtc_address 0x68<<1
#define DS3231_STATUSREG 0x0F ///< Status register
#define DS3231_TIME 0x00 ///< Time register
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

// function definitions
void rtc_read_reg_bytes (uint8_t, uint8_t); // 1+ bytes
uint8_t rtc_read (uint8_t); // 1 byte
void rtc_write_reg (uint8_t, uint8_t); // 1 byte
bool lostPower(void);
uint8_t bin2bcd(uint8_t);

// variables
uint8_t data_rec[8];

// read function - for several bytes
void rtc_read_reg_bytes (uint8_t reg, uint8_t numberofbytes) {
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read (&hi2c1, rtc_address, reg, 1, data_rec, numberofbytes, 100);

	// printing status
	HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n rtc_read_reg_bytes", sizeof("\r\n rtc_read_reg_bytes"), 1000);
	if (HAL_ERROR == status) {
		HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n HAL_ERROR returned", sizeof("\r\n HAL_ERROR returned"), 1000);
	} else if (HAL_BUSY == status){
		HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n HAL_BUSY returned", sizeof("\r\n HAL_BUSY returned"), 100);
	} else if (HAL_OK == status) {
		HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n HAL_OK returned", sizeof("\r\n HAL_OK returned"), 100);
	}
}

// read into a buffer
void rtc_read_buf (uint8_t* buf, uint8_t size) {
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, rtc_address, buf[0], 1, buf, size, 100);// reg = buf[0]

	// printing status
	HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n rtc_read_buf", sizeof("\r\n rtc_read_buf"), 1000);
	if (HAL_ERROR == status) {
		HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n HAL_ERROR returned", sizeof("\r\n HAL_ERROR returned"), 1000);
	} else if (HAL_BUSY == status){
		HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n HAL_BUSY returned", sizeof("\r\n HAL_BUSY returned"), 100);
	} else if (HAL_OK == status) {
		HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n HAL_OK returned", sizeof("\r\n HAL_OK returned"), 100);
	}
}

// read function - for one byte
uint8_t rtc_read_reg (uint8_t reg) {
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read (&hi2c1, rtc_address, reg, 1, data_rec, 1, 100);

	// printing status
	HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n rtc_read_reg", sizeof("\r\n rtc_read_reg"), 1000);
	if (HAL_ERROR == status) {
		HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n HAL_ERROR returned", sizeof("\r\n HAL_ERROR returned"), 1000);
	} else if (HAL_BUSY == status){
		HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n HAL_BUSY returned", sizeof("\r\n HAL_BUSY returned"), 100);
	} else if (HAL_OK == status) {
		HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n HAL_OK returned", sizeof("\r\n HAL_OK returned"), 100);
	}

	return data_rec[0];
}

// write function - for one byte - TODO: test this function
void rtc_write_reg (uint8_t reg, uint8_t value) {
	uint8_t data[2];
	data[0] = reg;
	data[1] = value;
	HAL_StatusTypeDef status = HAL_I2C_Master_Transmit (&hi2c1, rtc_address, data, 2, 10);
	// printing status
	HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n rtc_write_reg", sizeof("\r\n rtc_write_reg"), 1000);
	if (HAL_ERROR == status) {
		HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n HAL_ERROR returned", sizeof("\r\n HAL_ERROR returned"), 1000);
	} else if (HAL_BUSY == status){
		HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n HAL_BUSY returned", sizeof("\r\n HAL_BUSY returned"), 100);
	} else if (HAL_OK == status) {
		HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n HAL_OK returned", sizeof("\r\n HAL_OK returned"), 100);
	}
}

// write function - for 8 bytes - TODO: test this function
void rtc_write_reg_bytes (uint8_t* buf, uint8_t size) {
	uint8_t data[size] = {0};

	// copy buffer data into i2c data buffer
	for (int i=0; i<size; i++) {
		data[i] = buf[i];
	}

	HAL_StatusTypeDef status = HAL_I2C_Master_Transmit (&hi2c1, rtc_address, data, size, 100);

	// printing status
	HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n rtc_write_reg_bytes", sizeof("\r\n rtc_write_reg_bytes"), 1000);
	if (HAL_ERROR == status) {
		HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n HAL_ERROR returned", sizeof("\r\n HAL_ERROR returned"), 1000);
	} else if (HAL_BUSY == status){
		HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n HAL_BUSY returned", sizeof("\r\n HAL_BUSY returned"), 100);
	} else if (HAL_OK == status) {
		HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n HAL_OK returned", sizeof("\r\n HAL_OK returned"), 100);
	}
}

bool lostPower(void) {
  return rtc_read_reg(DS3231_STATUSREG)>>7; // check bit 7 of status register
}

/*
	@brief  Convert a binary value to BCD format for the RTC registers
	@param val Binary value
	@return BCD value
*/
uint8_t bin2bcd(uint8_t val) { // TODO: check bin2bcd - whether it works or not
	return val + 6 * (val / 10);
}

/*!
  @brief  Convert the day of the week to a representation suitable for
		  storing in the DS3231: from 1 (Monday) to 7 (Sunday).
  @param  d Day of the week as represented by the library:
		  from 0 (Sunday) to 6 (Saturday).
  @return the converted value
*/
static uint8_t dowToDS3231(uint8_t d) { return d == 0 ? 7 : d; };

/*
    @brief  Set the date and flip the Oscillator Stop Flag
    @param dt DateTime object containing the date/time to set
*/
void adjust(const DateTime &dt) { // TODO: create DateTime class
										// reference: https://github.com/adafruit/RTClib/blob/33fb0400c8241a5e604d0d9bd1c3a79b8a19cc4a/src/RTClib.cpp#LL202C1-L202C1
	uint8_t buffer[] = {DS3231_TIME, // register to write to (time register)
	                       bin2bcd(dt.second()), // write bin2bcd function
						   	   	   	   	   	   	   // reference: https://github.com/adafruit/RTClib/blob/33fb0400c8241a5e604d0d9bd1c3a79b8a19cc4a/src/RTClib.h#L340
	                       bin2bcd(dt.minute()),
	                       bin2bcd(dt.hour()),
	                       bin2bcd(dowToDS3231(dt.dayOfTheWeek())), // TODO: write dayOfTheWeek function inside DateTime class - https://github.com/adafruit/RTClib/blob/33fb0400c8241a5e604d0d9bd1c3a79b8a19cc4a/src/RTClib.cpp#L553
						   	   	   	   	   	   	   	   	   	   	   	   // TODO: dowToDS3231 function - https://github.com/adafruit/RTClib/blob/33fb0400c8241a5e604d0d9bd1c3a79b8a19cc4a/src/RTClib.h#L398
	                       bin2bcd(dt.day()),
	                       bin2bcd(dt.month()),
	                       bin2bcd(dt.year() - 2000U)};

	rtc_write_reg_bytes(buffer,sizeof(buffer)); //TODO: write rtc_write_reg_bytes function for several bytes
							// reference: https://github.com/adafruit/Adafruit_BusIO/blob/42e31a252f3cfd81aaa5668258d3dcb837524660/Adafruit_I2CDevice.cpp#L95
	uint8_t statreg = rtc_read_reg(DS3231_STATUSREG);
//	statreg &= ~0x80; // flip OSF bit
	statreg &= ~0x8; // flip OSF bit
	rtc_write_reg(DS3231_STATUSREG, statreg);
}

/*!
  @brief  Convert a binary coded decimal value to binary. RTC stores
time/date values as BCD.
  @param val BCD value
  @return Binary value
*/
static uint8_t bcd2bin(uint8_t val) { //TODO: check functionality
	return val - 6 * (val >> 4);
}

/**************************************************************************/
/*!
    @brief  Get the current date/time
    @return DateTime object with the current date/time
*/
/**************************************************************************/
DateTime now(void) {
  uint8_t buf1[1] = {0};
  uint8_t buf2[7] = {0};
  buf1[0] = 0; // first element it 0
  buf2[0] = 0; // first element it 0

  rtc_write_reg_bytes (buf1,sizeof(buf1));
  rtc_read_buf (buf2, sizeof(buf2));

  return DateTime(bcd2bin(buf2[6]) + 2000U, // uint16_t year
		  	  	  bcd2bin(buf2[5] & 0x7F), //uint8_t month
                  bcd2bin(buf2[4]), //uint8_t day
				  bcd2bin(buf2[2]), //uint8_t hour
				  bcd2bin(buf2[1]), //uint8_t min
                  bcd2bin(buf2[0] & 0x7F)); //uint8_t sec
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

  HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n ========Starting======== \r\n ========================",
		  sizeof("\r\n ========Starting======== \r\n ========================"), 100);

  // sample initialization of Datetime
  uint16_t year = 2023;
  uint8_t month = 6;
  uint8_t day = 13;
  uint8_t hour = 10;
  uint8_t min = 57;
  uint8_t sec = 2;

  DateTime dt (year,month,day,hour,min,sec);

  // check if there's a power loss
  if (lostPower()) {
	  char msg1[] = "\r\n RTC lost power, let's set the time!";
	  HAL_UART_Transmit(&huart2, (uint8_t*)msg1, sizeof(msg1), 100);

	  // When time needs to be set on a new device, or after a power loss, the
	  // following line sets the RTC to the date & time this sketch was compiled
	  adjust(dt);
	  // This line sets the RTC with an explicit date & time

  } else {
	  char msg2[] = "\r\n RTC has power";
	  HAL_UART_Transmit(&huart2, (uint8_t*)msg2, sizeof(msg2), 100);
  }

  adjust(dt);
  HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n adjust done", sizeof("\r\n adjust done"), 100);

  HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n xxxxxxxxxxxxxxxxxxxxxxxxxxxxx",
  		  sizeof("\r\n xxxxxxxxxxxxxxxxxxxxxxxxxxxxx"), 100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	DateTime current = now();

	uint16_t year = current.year();
	uint8_t month = current.month();
	uint8_t day = current.day();
	uint8_t hour = current.hour();
	uint8_t min = current.minute();
	uint8_t sec = current.second();

	char buffer[100] = {};

	// Convert the integer to a string
	sprintf(buffer, "\r\n year: %u \r\n month: %d \r\n day: %d \r\n hour: %d \r\n min: %d \r\n sec: %d\r\n", year,month,day,hour,min,sec);

	// Transmit the string via UART
	HAL_UART_Transmit(&huart2, (uint8_t *)buffer, sizeof(buffer), 100);

	HAL_Delay(1000);


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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

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
