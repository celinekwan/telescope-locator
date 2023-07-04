/*
 * RTClib.h
 *
 *  Created on: Jun 13, 2023
 *      Author: celin
 */

#ifndef INC_RTCLIB_H_
#define INC_RTCLIB_H_

#include <iostream>

class DateTime {
public:
	DateTime(uint16_t year, uint8_t month, uint8_t day, uint8_t hour = 0,
	           uint8_t min = 0, uint8_t sec = 0);

	/*!
	  @brief  Return the second.
	  @return Second (0--59).
	*/
	uint8_t second() const { return ss; }

	/*!
		@brief  Return the minute.
		@return Minute (0--59).
	*/
	uint8_t minute() const { return mm; }

	/*!
		@brief  Return the hour
		@return Hour (0--23).
	*/
	uint8_t hour() const { return hh; }

	uint8_t dayOfTheWeek() const;

	/*!
	  @brief  Return the day of the month.
	  @return Day of the month (1--31).
	*/
	uint8_t day() const { return d; }

	/*!
	  @brief  Return the month.
	  @return Month number (1--12).
	*/
	uint8_t month() const { return m; }

	/*!
	  @brief  Return the year.
	  @return Year (range: 2000--2099).
	*/
	uint16_t year() const { return 2000U + yOff; }

protected:
  uint8_t yOff; ///< Year offset from 2000
  uint8_t m;    ///< Month 1-12
  uint8_t d;    ///< Day 1-31
  uint8_t hh;   ///< Hours 0-23
  uint8_t mm;   ///< Minutes 0-59
  uint8_t ss;   ///< Seconds 0-59
};

#endif /* INC_RTCLIB_H_ */
