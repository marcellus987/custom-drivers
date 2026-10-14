/*
 * ds3231_driver.h
 *
 *  Created on: Sep 30, 2026
 *      Author: mvps1
 */

#ifndef INC_DS3231_DRIVER_H_
#define INC_DS3231_DRIVER_H_

#include "gpio_driver.h"

typedef enum {
	SUNDAY = 1,
	MONDAY,
	TUESDAY,
	WEDNESDAY,
	THURSDAY,
	FRIDAY,
	SATURDAY, /* SATURDAY = 7*/
} DayOfTheWeek;

/* Member variables are in the order listed in data sheet: Register Map.
 * The value for each field will be converted to BCD format before writing
 * to register. Or, will be converted to decimal for displaying purposes. */
typedef struct {
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hours;
} raw_time_t;

/* Width of attributes of time_t is consistent with raw time_t.
 * I.e. each max value of attribute is less than 255, which is
 * the max value that fits in uint8_t. So, this can be 'typedef'.
 *
 * Processed raw date. */
typedef raw_time_t current_time_t;

/* Member variables are in the order listed in data sheet: Register Map.
 * The value for each field will be converted to BCD format before writing
 * to register. Or, will be converted to decimal for displaying purposes.*/
typedef struct {
	uint8_t dayOfWeek;	/* 1-7: Monday to Sunday. */
	uint8_t date;		/* 1 to 31. */
	uint8_t month;		/* 1 to 12. */
	uint8_t year; 		/* 0 to 99: Base century must be added to this.  */
} raw_date_t;

/* Processed raw date. */
typedef struct {
	uint8_t dayOfWeek;
	uint8_t date;
	uint8_t month;
	uint32_t year;
} current_date_t;


typedef void (*i2c_transmit_func)(void *i2c_handle, uint16_t devAddress, uint16_t regAddress, uint16_t regAddrSize, uint8_t *src, uint16_t size);
typedef void (*i2c_receive_func)(void *i2c_handle, uint16_t devAddress, uint16_t regAddress, uint16_t regAddrSize, uint8_t *dest, uint16_t size);

typedef struct {
	void *i2cHandle;

	i2c_transmit_func i2cTransmit;
	i2c_receive_func i2cReceive;

	uint8_t initialized;
	uint8_t isTwentyFourHourFormat;
	uint8_t isPM;
	uint16_t baseCentury;
} ds3231_instance_t;



void ds3231_rtc_module_init(ds3231_instance_t *dev);
void ds3231_getCurrentTime(const ds3231_instance_t *dev, current_time_t* curTime);
void ds3231_getCurrentDate(ds3231_instance_t *dev, current_date_t* curDate);
void ds3231_setCurrentTime(ds3231_instance_t *dev, raw_time_t curTime, uint8_t twenty_four_hr_format, uint8_t isPM);
void ds3231_setCurrentDate(ds3231_instance_t *dev, raw_date_t curDate);
void ds3231_setAMPM(ds3231_instance_t *dev, uint8_t toPM);
void ds3231_setTimeFormat(ds3231_instance_t *dev, uint8_t twenty_four_hr_format);
void ds3231_getTimeStamp(ds3231_instance_t *dev, char* dest);
void ds3231_printFormattedDateTime(const ds3231_instance_t *dev, current_time_t time, const current_date_t date, uint8_t selection);

/* Made for updating MCU RTC peripheral that accepts BCD format. */
void ds3231_getRawCurrentTime(const ds3231_instance_t *dev, raw_time_t* curRawTime);
void ds3231_getRawCurrentDate(ds3231_instance_t *dev, raw_date_t* curRawDate);

#endif /* INC_DS3231_DRIVER_H_ */
