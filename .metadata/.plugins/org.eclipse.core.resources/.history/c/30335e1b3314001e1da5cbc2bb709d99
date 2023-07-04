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
  * Date: Jun 21, 2023
  * Celine Kwan
  * Desc: figuring out the calibration for the BNO055 in NDOF Fusion mode
  * If this code runs, it outputs whether calibration is complete based on whether
  * the sys_stat value is 3 (i.e. AMG are all calibrated).
  * Some movements have to be made to calibrate the three sensors. See the datasheet
  * for how to move the sensor to calibrate it.
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define BNO055_ADDRESS 0x28 << 1 // 0x29 (primary), 0x28 (alternative)

// Configuration Registers
// Note: switching time from op mode to any other op mode is 7ms
#define OPR_MODE 0x3D // Operating mode register
#define AMG_MODE 0x07 // Non-fusion mode
#define NDOF_MODE 0x0C // Fusion mode

// Calibration Status Register
#define CALIB_STAT 0x35
#define SYS_MASK 0xC0
#define GYR_MASK 0x30
#define ACC_MASK 0x0C
#define MAG_MASK 0x03

// Unit Selection Register
#define UNIT_SEL 0x3B
#define DATA_FORMAT_SETTING 0x01 // windows orientation, celsius,deg,dps,mg

// Accelerometer Data Registers
#define ACC_DATA_X_LSB 0x08
#define ACC_DATA_X_MSB 0x09
#define ACC_DATA_Y_LSB 0x0A
#define ACC_DATA_Y_MSB 0x0B
#define ACC_DATA_Z_LSB 0x0C
#define ACC_DATA_Z_MSB 0x0D

// Magnetometer Data Registers
#define MAG_DATA_X_LSB 0x0E
#define MAG_DATA_X_MSB 0x0F
#define MAG_DATA_Y_LSB 0x10
#define MAG_DATA_Y_MSB 0x11
#define MAG_DATA_Z_LSB 0x12
#define MAG_DATA_Z_MSB 0x13

// Gyroscope Data Registers
#define GYR_DATA_X_LSB 0x14
#define GYR_DATA_X_MSB 0x15
#define GYR_DATA_Y_LSB 0x16
#define GYR_DATA_Y_MSB 0x17
#define GYR_DATA_Z_LSB 0x18
#define GYR_DATA_Z_MSB 0x19

// Euler Data Registers
#define EUL_DATA_X_LSB 0x1A // Heading
#define EUL_DATA_X_MSB 0x1B
#define EUL_DATA_Y_LSB 0x1C // Roll
#define EUL_DATA_Y_MSB 0x1D
#define EUL_DATA_Z_LSB 0x1E // Pitch
#define EUL_DATA_Z_MSB 0x1F

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

uint8_t data_rec_bno[6]; // store data from i2c read
int16_t x, y, z; // store data from accel registers
char disp_buf[50] = {0}; // store data to display to UART serial port
// calibration status: 3 [fully calibrated], 0 [not calibrated]
uint8_t sys_stat = 0; // system (all sensors)
uint8_t gyr_stat = 0;
uint8_t acc_stat = 0;
uint8_t mag_stat = 0;

// Function Declaration ********************************
void bno055_write_datum (uint8_t reg, uint8_t value);
void bno055_read_data (uint8_t reg, uint8_t numberofbytes);

// I2C Functions ********************************
// writes 1 byte
void bno055_write_datum (uint8_t reg, uint8_t value) {
	uint8_t data[2];
	data[0] = reg;
	data[1] = value;
	HAL_I2C_Master_Transmit (&hi2c1, BNO055_ADDRESS, data, 2, 10);
}

// reads multiple bytes
void bno055_read_data (uint8_t reg, uint8_t numberofbytes) {

	HAL_StatusTypeDef status = HAL_I2C_Mem_Read (&hi2c1, BNO055_ADDRESS, reg, 1, data_rec_bno, numberofbytes, 100);

	// printing status
	HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n bno055_read_data", sizeof("\r\n bno055_read_data"), 10);
	if (HAL_ERROR == status) {
		HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n HAL_ERROR returned", sizeof("\r\n HAL_ERROR returned"), 10);
	} else if (HAL_BUSY == status){
		HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n HAL_BUSY returned", sizeof("\r\n HAL_BUSY returned"), 10);
	} else if (HAL_OK == status) {
		HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n HAL_OK returned", sizeof("\r\n HAL_OK returned"), 10);
	}
}

// Config Functions ********************************
void bno055_config (void) {
	bno055_write_datum(OPR_MODE,NDOF_MODE); // Set operating mode to NDOF
//	bno055_write_datum(UNIT_SEL, DATA_FORMAT_SETTING); // Set unit selection to windows orientation, celsius, deg, dps, mg
}

// Calibrate Function ********************************
void bno055_read_calibrate_status (void) {

	while ( (sys_stat!=3) || (gyr_stat!=3)|| (acc_stat!=3) || (mag_stat!=3) ) {
		bno055_read_data(CALIB_STAT,1);

		sys_stat = ( data_rec_bno[0] & SYS_MASK ) >> 6;
		gyr_stat = ( data_rec_bno[0] & GYR_MASK ) >> 4;
		acc_stat = ( data_rec_bno[0] & ACC_MASK ) >> 2;
		mag_stat = ( data_rec_bno[0] & MAG_MASK ) ;

		// Display data to UART port
		sprintf (disp_buf, "\r\n sys= %d \t gyr= %d \t acc= %d \t mag= %d", sys_stat, gyr_stat, acc_stat, mag_stat);
		HAL_UART_Transmit(&huart2,(uint8_t*)disp_buf,sizeof(disp_buf),10);

		HAL_Delay(100);
	}

	HAL_UART_Transmit(&huart2,(uint8_t*)"\r\n Calibration Complete!",sizeof("\r\n Calibration Complete!"),10);

}

// Print Calibration Register Values
void bno055_print_calibrate_status (void) {
	bno055_read_data(CALIB_STAT,1);

	sys_stat = ( data_rec_bno[0] & SYS_MASK ) >> 6;
	gyr_stat = ( data_rec_bno[0] & GYR_MASK ) >> 4;
	acc_stat = ( data_rec_bno[0] & ACC_MASK ) >> 2;
	mag_stat = ( data_rec_bno[0] & MAG_MASK ) ;

	// Display data to UART port
	sprintf (disp_buf, "\r\n sys= %d \t gyr= %d \t acc= %d \t mag= %d", sys_stat, gyr_stat, acc_stat, mag_stat);
	HAL_UART_Transmit(&huart2,(uint8_t*)disp_buf,sizeof(disp_buf),10);

}


// Utility functions ********************************
// prints x y z axis data of specified register
void print_EUL_data (uint8_t reg) { // changed from print_RAW_data
	  // Note: Data output rate for fusion data in NDOF mode is 100Hz (i.e. every 10ms)

	  // Clear display buffer
	  for (int i=0; i<50; i++) {
		  disp_buf[i] = NULL;
	  }

	  // Get 6 bytes of data
	  bno055_read_data(reg, 6);
	  x = (data_rec_bno[1]<<8) | data_rec_bno[0];
	  y = (data_rec_bno[3]<<8) | data_rec_bno[2];
	  z = (data_rec_bno[5]<<8) | data_rec_bno[4];

	  // Convert with 16 LSB/ 1 deg
	  x = x >> 4; // Divide by 16
	  y = y >> 4;
	  z = z >> 4;

	  // Display data to UART port
	  sprintf (disp_buf, "\r\n x= %d \t y= %d \t z= %d", x, y, z);
	  HAL_UART_Transmit(&huart2,(uint8_t*)disp_buf,sizeof(disp_buf),10);
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
  bno055_config();
  bno055_read_calibrate_status();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

//	HAL_UART_Transmit(&huart2,(uint8_t*)"\r\n Accel:********",sizeof("\r\n Accel:********"),10);
//	print_RAW_data(ACC_DATA_X_LSB);
//	HAL_Delay(10);
//	HAL_UART_Transmit(&huart2,(uint8_t*)"\r\n Gyro:********",sizeof("\r\n Gyro:********"),10);
//	print_RAW_data(GYR_DATA_X_LSB);
//	HAL_Delay(10);
//	HAL_UART_Transmit(&huart2,(uint8_t*)"\r\n Magn:********",sizeof("\r\n Magn:********"),10);
//	print_RAW_data(MAG_DATA_X_LSB);
//	HAL_Delay(10);

	HAL_UART_Transmit(&huart2,(uint8_t*)"\r\n Eul:********",sizeof("\r\n Eul:********"),10);
	print_EUL_data(EUL_DATA_X_LSB);

	bno055_print_calibrate_status(); // print & check calibration registers

	HAL_Delay(100);

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
