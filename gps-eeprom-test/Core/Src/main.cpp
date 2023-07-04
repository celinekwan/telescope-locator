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
  * Date: Jun 22, 2023
  * Celine Kwan
  * store GPS's lat/lng to EEPROM 0x00 and 0x04 locations
  * SUCCESSFUL
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>

#include <gps/NMEA.h>
#include <gps/uartRingBuffer.h>

#include <RTClib/RTClib.h>

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

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

// EEPROM ******************
#define EEPROM_ADDR 0x50 << 1

// RTC ******************
#define RTC_ADDR 0x68<<1
#define DS3231_STATUSREG 0x0F ///< Status register
#define DS3231_TIME 0x00 ///< Time register

// Data Buffers *********************************
uint8_t data_rec4[4] = {0};
uint8_t data_rec8[8] = {0};
uint8_t buf_lat[4] = {0};  // floats are 4 bytes!
uint8_t buf_lng[4] = {0};

char string_to_LCD[40] = {0}; // printing to LCD

// BNO055 *********************************
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

// For LCD Screen******************************
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

// GPS *********************************
char GGA[100];
char RMC[100];
GPSSTRUCT gpsData;

// BNO *********************************
uint8_t data_rec_bno[6] = {0}; // store data from i2c read
int16_t x, y, z; // store data from accel registers
char disp_buf[60] = {0}; // store data to display to UART serial port
// calibration status: 3 [fully calibrated], 0 [not calibrated]
uint8_t sys_stat = 0; // system (all sensors)
uint8_t gyr_stat = 0;
uint8_t acc_stat = 0;
uint8_t mag_stat = 0;

// GEOGRAPHICLIB + LIBNOVA ****************************
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

// Stellarium *****************************
char * buf1_stel;
char * buf2_stel;
char * buf3_stel;
int random_counter = 0;
int sizeof_ra = 0;
int temp;
HAL_StatusTypeDef status_stel;
HAL_StatusTypeDef status_transmit_ra;
HAL_StatusTypeDef status_transmit_dec;
int result1;
int result2;

char ra_str[9]={0};
char dec_str[10]={0};
char stel_cmds[5] = {0};

// LCD Screen
float xg, yg, zg;

// Function Declarations ==========================================

// EEPROM **********************************
void eeprom_write_byte (uint16_t addr, uint8_t value);
void eeprom_write_bytes (uint16_t addr, uint8_t *buf, uint8_t numberofbytes);
void eeprom_read_bytes (uint16_t addr, uint8_t numberofbytes);
void print_status (HAL_StatusTypeDef);

// RTC **********************************
void rtc_read_reg_bytes (uint8_t, uint8_t); // 1+ bytes
uint8_t rtc_read (uint8_t); // 1 byte
void rtc_write_reg (uint8_t, uint8_t); // 1 byte
bool lostPower(void);
uint8_t bin2bcd(uint8_t);

// BNO **********************************
void bno055_write_datum (uint8_t reg, uint8_t value);
void bno055_read_data (uint8_t reg, uint8_t numberofbytes);

// LCD **********************************
void screen_write_string(char string_to_write[]);

// Function Definitions ==========================================

// EEPROM **********************************
// writes one byte
void eeprom_write_byte (uint16_t addr, uint8_t value) {
	uint8_t data[1] = {0};
	data[0] = value;
	HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&hi2c1, EEPROM_ADDR, addr, I2C_MEMADD_SIZE_16BIT, data, 1, 10);
}

// writes several bytes
void eeprom_write_bytes (uint16_t addr, uint8_t *buf, uint8_t numberofbytes) {
	for (int i=0; i<numberofbytes; i++) {
		eeprom_write_byte(addr,buf[i]);
		addr++;
	}
}

// reads a number of bytes
void eeprom_read_bytes (uint16_t addr, uint8_t numberofbytes) {
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read (&hi2c1, EEPROM_ADDR, addr, I2C_MEMADD_SIZE_16BIT, data_rec4, numberofbytes, 100);
}

// RTC **********************************
// read function - for several bytes
void rtc_read_reg_bytes (uint8_t reg, uint8_t numberofbytes) {
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read (&hi2c1, RTC_ADDR, reg, 1, data_rec8, numberofbytes, 100);
}

// read into a buffer
void rtc_read_buf (uint8_t* buf, uint8_t size) {
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, RTC_ADDR, buf[0], 1, buf, size, 100);// reg = buf[0]
}

// read function - for one byte
uint8_t rtc_read_reg (uint8_t reg) {
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read (&hi2c1, RTC_ADDR, reg, 1, data_rec8, 1, 100);
	return data_rec8[0];
}

// write function - for one byte
void rtc_write_reg (uint8_t reg, uint8_t value) {
	uint8_t data[2];
	data[0] = reg;
	data[1] = value;
	HAL_StatusTypeDef status = HAL_I2C_Master_Transmit (&hi2c1, RTC_ADDR, data, 2, 10);
}

// write function - for 8 bytes
void rtc_write_reg_bytes (uint8_t* buf, uint8_t size) {
	uint8_t data[size] = {0};

	// copy buffer data into i2c data buffer
	for (int i=0; i<size; i++) {
		data[i] = buf[i];
	}

	HAL_StatusTypeDef status = HAL_I2C_Master_Transmit (&hi2c1, RTC_ADDR, data, size, 100);
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
static uint8_t dowToDS3231(uint8_t d) {
	return d == 0 ? 7 : d ;
}

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

/*!
    @brief  Get the current date/time
    @return DateTime object with the current date/time
*/
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

// BNO ************************************
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

//	// printing status
//	HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n bno055_read_data", sizeof("\r\n bno055_read_data"), 10);
//	if (HAL_ERROR == status) {
//		HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n HAL_ERROR returned", sizeof("\r\n HAL_ERROR returned"), 10);
//	} else if (HAL_BUSY == status){
//		HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n HAL_BUSY returned", sizeof("\r\n HAL_BUSY returned"), 10);
//	} else if (HAL_OK == status) {
//		HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n HAL_OK returned", sizeof("\r\n HAL_OK returned"), 10);
//	}
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
		sprintf (disp_buf, "sys= %d    gyr= %d    acc= %d    mag= %d", sys_stat, gyr_stat, acc_stat, mag_stat);
//		HAL_UART_Transmit(&huart2,(uint8_t*)disp_buf,sizeof(disp_buf),10);
		screen_write_string(disp_buf);

		HAL_Delay(500);
	}

//	HAL_UART_Transmit(&huart2,(uint8_t*)"\r\n Calibration Complete!",sizeof("\r\n Calibration Complete!"),10);

}

// Print Calibration Register Values
void bno055_print_calibrate_status (void) {
	bno055_read_data(CALIB_STAT,1);

	sys_stat = ( data_rec_bno[0] & SYS_MASK ) >> 6;
	gyr_stat = ( data_rec_bno[0] & GYR_MASK ) >> 4;
	acc_stat = ( data_rec_bno[0] & ACC_MASK ) >> 2;
	mag_stat = ( data_rec_bno[0] & MAG_MASK ) ;

	// Display data to UART port
//	sprintf (disp_buf, "\r\n sys= %d \t gyr= %d \t acc= %d \t mag= %d", sys_stat, gyr_stat, acc_stat, mag_stat);
//	HAL_UART_Transmit(&huart2,(uint8_t*)disp_buf,sizeof(disp_buf),10);

}


// Utility functions ********************************
// prints x y z axis data of specified register
void print_EUL_data (uint8_t reg) { // changed from print_RAW_data
	  // Note: Data output rate for fusion data in NDOF mode is 100Hz (i.e. every 10ms)

	  // Clear display buffer
	  for (int i=0; i<50; i++) {
		  disp_buf[i] = 0;
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
	  sprintf (disp_buf, "hd = %d  rol= %d  pit= %d", x, y, z);
//	  screen_write_string(disp_buf);
//	  HAL_UART_Transmit(&huart2,(uint8_t*)disp_buf,sizeof(disp_buf),10);
}


// STELLARIUM functions ********************************
// init buffer elements to all 0
void init_buf123() {
  	temp = sizeof(buf1_stel);
  	for (int i=0; i<sizeof(buf1_stel); i++) {
  		buf1_stel[i] = 0;
  		buf2_stel[i] = 0;
  		buf3_stel[i] = 0;
  	}
}

// LCD Screen functions **************************************
/*!
 * @brief Write an I2C cmd to DOGS164N LCD display,
 * including start, stop and address operations
 *
 * @param addr - Slave address of display
 * @param cmd  - "command" byte to be send
 *
 * @return none
 */
void I2Csendcmd(uint8_t addr, uint8_t cmd) {
	// start transmission -- should be done by HAL_I2C_Master_Transmit function already
	uint8_t data[2];
	data[0] = 0x00;
	data[1] = cmd; // data byte (not data, but command)

	HAL_I2C_Master_Transmit (&hi2c1, addr, data, 2, HAL_MAX_DELAY);
}

/*!
	@brief Write a byte of data to DOGS164N LCD display,
	including start, stop and address operations

	@param addr - Slave address of display
	@param data  - "data" byte to be send

	@return none
 */
void I2Csenddatum (uint8_t addr, uint8_t data) { // I2Csenddatum(SCREEN_ADDR, string_to_write[letter]);
	uint8_t datum[2];
	datum[0] = 0x40;
	datum[1] = data; // data byte (not data, but command)

	HAL_I2C_Master_Transmit (&hi2c1, addr, datum, 2, HAL_MAX_DELAY);
}

void I2Cread (uint8_t addr) {
	uint8_t data[2];
	HAL_I2C_Master_Receive(&hi2c1, addr, data, 1, 10); // TODO: might be wrong
	HAL_UART_Transmit(&huart2,data,strlen((char*)data),HAL_MAX_DELAY); // TODO: delete this after - just for debugging
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

// EEPROM *********************************
GGASTRUCT *gga = &gpsData.ggastruct;
float lat = 0; // write to EEPROM
float lng = 0;
char lat_dir;
char lng_dir;
float lat_ret = 0; // read from EEPROM
float lng_ret = 0;

// RTC *********************************
RMCSTRUCT *rmc = &gpsData.rmcstruct;

int gps_year = 0;
int gps_month = 0;
int gps_day = 0;
int gps_hour = 0;
int gps_minutes = 0;
int gps_seconds = 0;

// Printing *********************************
char print_buf_1[20] = {0};
char print_buf_2[20] = {0};
char print_buf_3[40] = {0};
char print_buf_4[60] = {0};
char print_buf_5[40] = {0};
char print_buf_6[40] = {0};

// Printing RA/DEC in Stel format *********
char print_buf_7[30] = {0};
char print_buf_8[30] = {0};


// Lat/Lng->DMS Conversion ******************
double d_1=0, d_2 =0;
double m_1=0, m_2 =0;
double s_1=0, s_2 =0;

// GPS NMEA **********************************
const char* GG = "GG";
const char* RM = "RM";
const char* asterisk = "*";

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

  /* Reset of all bno055_read_calibrate_statusperipherals, Initializes the Flash interface and the Systick. */
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

	Ringbuf_init();

	uint16_t year = 0;
	uint8_t month = 0;
	uint8_t day = 0;
	uint8_t hour = 0;
	uint8_t min = 0;
	uint8_t sec = 0;

	//	  ========== LCD Screen Initialization ==================================
	// turn off display, turn on display (translated from PORTD section of Stewart's code)
	HAL_Delay(5);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0,GPIO_PIN_RESET);
	HAL_Delay(200);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0,GPIO_PIN_SET);
	HAL_Delay(5);

	//1. clear screen
	screen_init();

	// BNO config *********************************
	bno055_config();
	bno055_read_calibrate_status();

	// Get lat/lng and date/time value from GPS
	while ( (lat == 0 && lng == 0 ) || (gps_year == 0) ) { // Condition: lat/lng are not updated OR year not extracted

		//	 ========== GPS Ring Buffer ==================================
		if (Wait_for(GG) == 1) { // "GPGGA" - need "A" for decodeGGA to work properly
			Copy_upto(asterisk, GGA); // 1=successful copy
			decodeGGA(GGA,&gpsData.ggastruct); // 0=fixed; 1=error
		}

		if (Wait_for(RM) == 1) { // "GPRMC" - need "C" for decodeRMC to work properly
		  Copy_upto(asterisk, RMC); // 1=successful copy
		  decodeRMC(RMC,&gpsData.rmcstruct); // 0=fixed; 1=error
		}

		//	  ========== Extract lng/lat Data ==================================
		lat = gga->lcation.latitude;
		lng = gga->lcation.longitude;

		// Modify +/- sign of lat/lng
		lat_dir = gga->lcation.NS;
		lng_dir = gga->lcation.EW;
		if (lat_dir == 'S') lat *= -1;
		if (lng_dir == 'W') lng *= -1;

		//	  ========== Extract date & time ==================================
		gps_year = rmc->date.Yr; /*!< Years. All values are valid */
		gps_month = rmc->date.Mon; /*!< Months. Valid values : 1 (January) - 12 (December) */
		gps_day = rmc->date.Day; /*!< Days. Valid values 1 - 28,29,30,31 Depends on month.*/
		gps_hour = gga->tim.hour; /*!< Hours. Valid values 0 - 23. */
		gps_minutes = gga->tim.min; /*!< Minutes. Valid values 0 - 59. */
		gps_seconds = gga->tim.sec; /*!< Seconds. Valid values 0 - 59.99999.... */

	}

//  	HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n Lat/lng & date/time extraction done", sizeof("\r\n Lat/lng & date/time extraction done"), 10);


  	//	  ========== RTC Setup: Store time into RTC ==================================
	// initialization of Datetime
	year = 2000 + gps_year;
	month = gps_month;
	day = gps_day;
	hour = gps_hour;
	min = gps_minutes;
	sec = gps_seconds;

	DateTime dt (year,month,day,hour,min,sec);

//	HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n Date/time initialization done", sizeof("\r\n Date/time initialization done"), 10);

	// check if there's a power loss
	if (lostPower()) {
//	  HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n RTC lost power, let's set the time!", sizeof("\r\n RTC lost power, let's set the time!"), 10);

	  // When time needs to be set on a new device, or after a power loss, the
	  // following line sets the RTC to the date & time this sketch was compiled
	  adjust(dt); // Sets the RTC with an explicit date & time

	} else {
//	  HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n RTC has power", sizeof("\r\n RTC has power"), 10);
	}

	adjust(dt);

//	HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n RTC Setup Done", sizeof("\r\n RTC Setup Done"), 10);

	//	  ========== EEPROM Write ==================================
	memcpy(buf_lat, (void *)&lat, 4);
	eeprom_write_bytes(0x00,buf_lat,4);
	memcpy(buf_lng, (void *)&lng, 4);
	eeprom_write_bytes(0x04,buf_lng,4);

//	HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n EEPROM Write Done", sizeof("\r\n EEPROM Write Done"), 10);

	//	  ========== STELLARIUM stuff ==================================
	// initialize buffers
	buf1_stel = (char*)malloc(128);
	buf2_stel = (char*)malloc(128);
	buf3_stel = (char*)malloc(128);

	char *pound = "#";
	char *pos = "+";
	char *neg = "-";

	init_buf123();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	//	  ========== Get current time from RTC ==================================
	DateTime current = now();

	year = current.year();
	month = current.month();
	day = current.day();
	hour = current.hour();
	min = current.minute();
	sec = current.second();

	//	  ========== Print current time ==================================
	char buffer[100] = {};
//	sprintf(buffer, "\r\n year: %u \r\n month: %d \r\n day: %d \r\n hour: %d \r\n min: %d \r\n sec: %d\r\n", year,month,day,hour,min,sec); // Convert the integer to a string
//	HAL_UART_Transmit(&huart2, (uint8_t *)buffer, sizeof(buffer), 10); // Transmit the string via UART
	sprintf(buffer, "%u/%d/%d  %d/%d/%d  ", year,month,day,hour,min,sec);
	strcpy(string_to_LCD,buffer);

	//	  ========== Read Lat/Lng from EEPROM ==================================
	// Read lat/lng
	eeprom_read_bytes (0x00, 4); // read
	memcpy((void *)&lat_ret, data_rec4, 4);

	eeprom_read_bytes (0x04, 4); // read
	memcpy((void *)&lng_ret, data_rec4, 4);

//	HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n EEPROM Read Done", sizeof("\r\n EEPROM Read Done"), 10);

	//	  ========== Print Lat/Lng (float) ==================================
//	sprintf(print_buf_1,"\r\n lat: % 4f", lat_ret);
//	HAL_UART_Transmit(&huart2, (uint8_t*)print_buf_1, sizeof(print_buf_1), 10);
//	sprintf(print_buf_2,"\r\n lng: % 4f", lng_ret);
//	HAL_UART_Transmit(&huart2, (uint8_t*)print_buf_2, sizeof(print_buf_2), 10);

	//	  ========== Print Euler Roll/Pitch/Yaw angles ==================================
//	HAL_UART_Transmit(&huart2,(uint8_t*)"\r\n Eul:********",sizeof("\r\n Eul:********"),10);
	print_EUL_data(EUL_DATA_X_LSB);

	// GEOGRAPHIC LIB + LIBNOVA STUFF ********************************************************************
	//	  ========== Conversion ==================================
	// use lat_ret and lng_ret
	DMS::Encode(lat_ret,d_1,m_1,s_1); // latitude
	DMS::Encode(lng_ret,d_2,m_2,s_2); // longitude

	// deal with negative direction
	int neg_lat = 0;
	int neg_lng = 0;

	if (lat_dir == 'S') neg_lat = 1;
	if (lng_dir == 'W') neg_lng = 1;

	// conversion ***********************
	/*
	 * Bahen
	 * lat: 43.659822
	 * lng: -79.397056
	 * DMS lat: 43° 39' 35.3592'' N
	 * DMS lng: 79° 23' 49.4016'' W
	 */
//	hobserver.lng.neg = neg_lng; //1
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

	/* UT date and time */
	date.years = year;
	date.months = month;
	date.days = day;
	date.hours = hour;
	date.minutes = min;
	date.seconds = sec;

	// Conversion *********************
	JD = ln_get_julian_day (&date);
	ln_hequ_to_equ (&hobject, &object);
	ln_hlnlat_to_lnlat (&hobserver, &observer);
	ln_get_hrz_from_equ (&object, &observer, JD, &hrz);
	ln_hrz_to_hhrz(&hrz, &hhrz);

	// Include roll/pitch/yaw Euler angles in alt/az *********
	hrz.alt -= z; // alt + (-pitch)
	hrz.az += x; // az + heading

	ln_get_equ_from_hrz (&hrz, &observer, JD, &equ); // the one we need

	//	  ========== Print Hrz and Equ Values ==================================
	double az = hrz.az;
	double alt = hrz.alt;
	double lat = observer.lat;
	double lng = observer.lng;
	// double JD
	double ra = equ.ra;
	double dec = equ.dec;

//	sprintf(print_buf_3,"\r\n hrz==az: % 4f , alt: % 4f", az, alt);
//	HAL_UART_Transmit(&huart2, (uint8_t*)print_buf_3, sizeof(print_buf_3), 10);
//	sprintf(print_buf_4,"\r\n observer==lat: % 4f , lng: % 4f ", lat, lng);
//	HAL_UART_Transmit(&huart2, (uint8_t*)print_buf_4, sizeof(print_buf_4), 50);
//	sprintf(print_buf_5,"\r\n JD: % 4f", JD);
//	HAL_UART_Transmit(&huart2, (uint8_t*)print_buf_5, sizeof(print_buf_5), 10);
//	sprintf(print_buf_6,"\r\n equ==ra: % 4f , dec: % 4f", ra, dec);
//	HAL_UART_Transmit(&huart2, (uint8_t*)print_buf_6, sizeof(print_buf_6), 10);

	sprintf(print_buf_4,"%.1f %.1f  ", lat, lng);
	strcat(string_to_LCD,print_buf_4);
	sprintf(print_buf_6,"%.1f %.1f", ra, dec);
	strcat(string_to_LCD,print_buf_6);

	//	  ========== Clear all print buffers ==================================
	// clear data buffer
	for (unsigned int i=0; i<sizeof(print_buf_3); i++) {
		print_buf_3[i] = 0;
	}
	for (unsigned int i=0; i<sizeof(print_buf_4); i++) {
		print_buf_4[i] = 0;
	}
	for (unsigned int i=0; i<sizeof(print_buf_5); i++) {
		print_buf_5[i] = 0;
	}
	for (unsigned int i=0; i<sizeof(print_buf_6); i++) {
		print_buf_6[i] = 0;
	}

	//	  ========== Check IMU calibration status ==================================
//	bno055_print_calibrate_status();


	// ************** Stellarium Interface Code *******************************************
	// Convert Deg to HMS/DMS RA/DEC
	ln_equ_to_hequ(&equ, &hequ);

	// HH MM SS for RA
	unsigned short ra_hh = hequ.ra.hours;
	unsigned short ra_mm = hequ.ra.minutes;
	double ra_ss = hequ.ra.seconds;

	// DD MM SS for DEC
	unsigned short dec_neg = hequ.dec.neg;
	unsigned short dec_dd = hequ.dec.degrees;
	unsigned short dec_mm = hequ.dec.minutes;
	double dec_ss = hequ.dec.seconds;

	// concatenate to the full RA string
	snprintf(buf1_stel, 128, "%u", ra_hh);
	snprintf(buf2_stel, 128, "%u", ra_mm);
	//	snprintf(buf3_stel, 128, "% .4f", ra_ss);
	snprintf(buf3_stel, 128, "%u", (short)ra_ss);

	if (ra_hh < 10) { // pad 0
		strcpy(ra_str,"0");
		strcat(ra_str,buf1_stel);
	} else { // no pad 0
		strcpy(ra_str,buf1_stel);
	}
	strcat(ra_str,":");
	if (ra_mm < 10) { strcat(ra_str,"0");}
	strcat(ra_str,buf2_stel);
	strcat(ra_str,":");
	if (ra_ss < 10) { strcat(ra_str,"0");}
	strcat(ra_str,buf3_stel);
	strcat(ra_str,pound);

	// clear buffers
	init_buf123();

	// concatenate to the full DEC string
	snprintf(buf1_stel, 128, "%u", dec_dd);
	snprintf(buf2_stel, 128, "%u", dec_mm);
	snprintf(buf3_stel, 128, "%u", (short)dec_ss);

	if (dec_neg == 0) {
		strcpy(dec_str,pos);
	} else {
		strcpy(dec_str,neg);
	}
	if (dec_dd < 10) { strcat(dec_str,"0");}
	strcat(dec_str,buf1_stel);
	strcat(dec_str,"\'");
	if (dec_mm < 10) { strcat(dec_str,"0");}
	strcat(dec_str,buf2_stel);
	strcat(dec_str,":");
	if (dec_ss < 10) { strcat(dec_str,"0");}
	strcat(dec_str,buf3_stel);
	strcat(dec_str,pound);

	// check ra_str and dec_str -- without stellarium
//	HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n\n Stellarium strings:\n", sizeof("\r\n\n Stellarium strings:\n"), 10);
//	HAL_UART_Transmit(&huart2, (uint8_t*)ra_str, sizeof(ra_str), 10);
//	HAL_UART_Transmit(&huart2, (uint8_t*)"\t\t", sizeof("\t\t"), 10);
//	HAL_UART_Transmit(&huart2, (uint8_t*)dec_str, sizeof(dec_str), 10);

	// Detect commands from Stellarium *****************************************************************
	status_stel = HAL_UART_Receive(&huart2, (uint8_t*)stel_cmds, sizeof(stel_cmds), 1000);
	random_counter++;

	if (status_stel == HAL_TIMEOUT) {
	  random_counter++; // for breakpoint/debugging
	} else if (status_stel == HAL_ERROR) {
	  random_counter++;
	} else if (status_stel == HAL_BUSY) {
	  random_counter++;
	} else if (status_stel == HAL_OK) {

	  result1 = strcmp(stel_cmds, "#:GR#");
	  result2 = strcmp(stel_cmds, "#:GD#");

	  sizeof_ra = sizeof(ra_str);

	  if (result1==0) {
		  status_transmit_ra = HAL_UART_Transmit(&huart2, (uint8_t*)ra_str, sizeof(ra_str), 10);
		  ra_mm += 5;
		  random_counter ++ ;
	  }

	  if (result2==0) {
		  status_transmit_dec = HAL_UART_Transmit(&huart2, (uint8_t*)dec_str, sizeof(dec_str), 10);
		  dec_mm += 5;
		  random_counter ++ ;
	  }

	}

	// clear buffer
	for (int i=0; i<5; i++) {
	  stel_cmds[i] = 0;
	}

	//	  ========== Wait 1s for the time to increment per second ==================================
//	HAL_Delay(1000);

	//	  ========== Print info to LCD screen ==================================
	screen_write_string(string_to_LCD);

	int counter = 0; // breakpoint
	counter++;
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
//  HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|LD4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

//  /*Configure GPIO pin : LD4_Pin */
//  GPIO_InitStruct.Pin = LD4_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(LD4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 LD4_Pin */
    GPIO_InitStruct.Pin = GPIO_PIN_0|LD4_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
