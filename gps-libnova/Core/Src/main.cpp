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
  * date completed: jun 19, 2023
  * Celine Kwan
  * sometimes works and sometimes doesnt - microsReady and microSeconds weird, unstable behaviour
  * ^^ FIXED -- now works fine
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>

#include <gps/uartRingBuffer.h>
#include <gps/NMEA.h>

#include <GeographicLib/DMS.hpp>

#include <libnova/ln_types.h>
#include <libnova/julian_day.h>
#include <libnova/transform.h>
#include <libnova/utility.h>

using namespace GeographicLib;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define MICROS_READY 30000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char GGA[100];
char RMC[100];
GPSSTRUCT gpsData;

//uint32_t microsReady, microsCurrent; // unsigned long
//int microsReady, microsCurrent; // unsigned long
int microsStart = 0;
int microsCurrent = 0;
int microsReady = 30000;
int flag;

// input into hrz-to-equ
struct lnh_equ_posn hobject, hequ;
struct lnh_lnlat_posn hobserver;
struct ln_equ_posn object, equ;
struct ln_hrz_posn hrz;
struct lnh_hrz_posn hhrz;
struct ln_lnlat_posn observer;
double JD;
struct ln_date date;

// pointers for printing
ln_date *date_ptr = &date;
ln_hrz_posn *hrz_ptr = &hrz;
ln_lnlat_posn *observer_ptr = &observer;
ln_equ_posn *equ_ptr = &equ;

// buffers for printing
char buf1[50] = {0};
char buf2[50] = {0};
char buf3[50] = {0};
char buf4[50] = {0};

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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
//  	microsReady = 30000; // 30s
  	flag = 0; // non-zero after first time accessed
	Ringbuf_init();

	HAL_Delay(1000);
	microsStart = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	//	  ========== GPS Ring Buffer ==================================
	if (Wait_for("GG") == 1) { // "GPGGA" - need "A" for decodeGGA to work properly
	  int value_1 = Copy_upto("*", GGA); // 1=successful copy
	  int value_2 = decodeGGA(GGA,&gpsData.ggastruct); // 0=fixed; 1=error
	}

	if (Wait_for("RM") == 1) { // "GPRMC" - need "C" for decodeRMC to work properly
	  int value_3 = Copy_upto("*", RMC); // 1=successful copy
	  int value_4 = decodeRMC(RMC,&gpsData.rmcstruct); // 0=fixed; 1=error
	}

	microsCurrent = HAL_GetTick();
	char ready_msg[] = "\r\n GPS data is stable and ready for reading.";
	if ( ((microsCurrent-microsStart) >= microsReady) && (flag == 0) ) { // 30s
	  flag = 1;
	  HAL_UART_Transmit(&huart2, (uint8_t*)ready_msg, sizeof(ready_msg), 10);
	  continue;
	} else if ((microsCurrent-microsStart) < microsReady) {
		continue;
	} else if ((microsCurrent-microsStart) >= microsReady) {
		//	  ========== Conversions ==================================

		// get lat -> Encode(lat...)
		GGASTRUCT *gga = &gpsData.ggastruct;

		// float format
		float lat = gga->lcation.latitude;
		float lng = gga->lcation.longitude;

		char lat_dir = gga->lcation.NS;
		char lng_dir = gga->lcation.EW;

//		if (lat_dir == 'S') {
//			lat *= -1;
//		}
//		if (lng_dir == 'W') {
//			lng *= -1;
//		}

//		char buf[30] = {0};
//		sprintf(buf,"\r\n % 4f % 4f",lat,lng);

		// integer format - works fine
//		int lat = (int)gga->lcation.latitude;
//		int lng = (int)gga->lcation.longitude;
//		char buf[100] = {0};
//		sprintf(buf,"\r\n %d %d",lat,lng);
//		HAL_UART_Transmit(&huart2, (uint8_t*)buf, sizeof(buf), 100);

		/**
		 * Split angle into degrees and minutes and seconds.
		 *
		 * @param[in] ang angle (degrees)
		 * @param[out] d degrees (an integer returned as a real)
		 * @param[out] m arc minutes (an integer returned as a real)
		 * @param[out] s arc seconds.
		 * static void Encode(real ang, real& d, real& m, real& s) {...}
		 **********************************************************************/
		double d_1=0, d_2 =0;
		double m_1=0, m_2 =0;
		double s_1=0, s_2 =0;

		DMS::Encode(lat,d_1,m_1,s_1); // latitude
		DMS::Encode(lng,d_2,m_2,s_2); // longitude

		// deal with negative direction
		int neg_lat = 0;
		int neg_lng = 0;

		if (lat_dir == 'S') neg_lat = 1;
		if (lng_dir == 'W') neg_lng = 1;

		// works until here

		// conversion ***********************
		/*
		 * Bahen
		 * lat: 43.659822
		 * lng: -79.397056
		 * DMS lat: 43° 39' 35.3592'' N
		 * DMS lng: 79° 23' 49.4016'' W
		 */
		hobserver.lng.neg = neg_lng; //1
		hobserver.lng.degrees = d_2; //79
		hobserver.lng.minutes = m_2; //14
		hobserver.lng.seconds = s_2; //16
		hobserver.lat.neg = neg_lat; //0
		hobserver.lat.degrees = d_1; //43
		hobserver.lat.minutes = m_1; //23
		hobserver.lat.seconds = s_1; //44

		/* Alnilam
		 * RA: 05h 36m 12s
		 * DEC: -01° 12' 06"
		 */
		hobject.ra.hours = 5;
		hobject.ra.minutes = 36;
		hobject.ra.seconds = 12;
		hobject.dec.neg = 1;
		hobject.dec.degrees = 1;
		hobject.dec.minutes = 12;
		hobject.dec.seconds = 6;

		// works

		/* UT date and time */
		RMCSTRUCT *rmc = &gpsData.rmcstruct;

		int gps_year = rmc->date.Yr; /*!< Years. All values are valid */
		int gps_month = rmc->date.Mon; /*!< Months. Valid values : 1 (January) - 12 (December) */
		int gps_day = rmc->date.Day; /*!< Days. Valid values 1 - 28,29,30,31 Depends on month.*/
		int gps_hour = gga->tim.hour; /*!< Hours. Valid values 0 - 23. */
		int gps_minutes = gga->tim.min; /*!< Minutes. Valid values 0 - 59. */
		int gps_seconds = gga->tim.sec; /*!< Seconds. Valid values 0 - 59.99999.... */

		date.years = 2000 + gps_year;
		date.months = gps_month;
		date.days = gps_day;
		date.hours = gps_hour;
		date.minutes = gps_minutes;
		date.seconds = gps_seconds;

		// Conversion *********************
		JD = ln_get_julian_day (&date);
		ln_hequ_to_equ (&hobject, &object);
		ln_hlnlat_to_lnlat (&hobserver, &observer);

		ln_get_hrz_from_equ (&object, &observer, JD, &hrz);
		ln_hrz_to_hhrz(&hrz, &hhrz);

		ln_get_equ_from_hrz (&hrz, &observer, JD, &equ);

		d_1 = 0; // breakpoint

		// Print out information *********************
		// day/mon/yr (int), hr/min/sec (int), JD (double)
		// az/alt (double), lng/lat (double), ra/dec (double)

//		// problems here
//		HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n\n\n NEW TEST **************", sizeof("\r\n\n\n NEW TEST **************"), 10);
//		sprintf(buf1,"\r\n dd/mm/yy:%d,%d,%d", date_ptr->days,date_ptr->months,date_ptr->years);
//		HAL_UART_Transmit(&huart2, (uint8_t*)buf1, sizeof(buf1), 10);
//		sprintf(buf2,"\r\n hr/min/sec:%d,%d,%d", date_ptr->hours,date_ptr->minutes,date_ptr->seconds);
//		HAL_UART_Transmit(&huart2, (uint8_t*)buf2, sizeof(buf2), 10);
//		sprintf(buf3,"\r\n JD:%d \r\n alt/az: %d, %d", JD, hrz_ptr->alt, hrz_ptr->az);
//		HAL_UART_Transmit(&huart2, (uint8_t*)buf3, sizeof(buf3), 10);
//		sprintf(buf4,"\r\n lng/lat: %d, %d \r\n ra/dec: %d, %d", observer_ptr->lng, observer_ptr->lat, equ_ptr->ra, equ_ptr->dec);
//		HAL_UART_Transmit(&huart2, (uint8_t*)buf4, sizeof(buf4), 10);

	}
	// check if time is still incrementing

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
