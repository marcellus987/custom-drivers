/*
 * ds3231_driver.c
 *
 *  Created on: Sep 30, 2026
 *      Author: mvps1
 */

#include "ds3231_driver.h"
#include <stdio.h>

//#define DEBUG_MODE__

//#define DS3231_MODULE__
//#define DS1307_MODULE__

/* Addresses from datasheet. */
#define RTC_MODULE_SLAVE_ADDRESS 			(0x68U)
#define RTC_MODULE_SLAVE_REG_ADDRESS_SIZE	(1U)	/* 1 byte register size. */
#define RTC_MODULE_SECONDS_REGISTER_ADDRESS (0x00U)
#define RTC_MODULE_MINUTES_REGISTER_ADDRESS (0x01U)
#define RTC_MODULE_HOURS_REGISTER_ADDRESS 	(0x02U)
#define RTC_MODULE_DAY_REGISTER_ADDRESS 	(0x03U)
#define RTC_MODULE_DATE_REGISTER_ADDRESS 	(0x04U)
#define RTC_MODULE_MONTH_REGISTER_ADDRESS 	(0x05U)
#define RTC_MODULE_YEAR_REGISTER_ADDRESS 	(0x06U)

/* AM/PM mask. */
#define AM_PM_BIT_Pos (5U)
#define AM_PM_BIT_Msk (1U << AM_PM_BIT_Pos)

/* 12/24-hour format mask. */
#define HOUR_FORMAT_BIT_Pos (6U)
#define HOUR_FORTMAT_BIT_Msk (1U << HOUR_FORMAT_BIT_Pos)

#define CENTURY_BIT_Pos (7U)
#define CENTURY_BIT_Msk (1U << CENTURY_BIT_Pos)

#define READ_ACCESS_BIT  (1U)
#define WRITE_ACCESS_BIT (0U)


/***** Global variables START. *****/
//const uint8_t slaveAddress = RTC_MODULE_SLAVE_ADDRESS;
/***** Global variables END. *****/


/***** Helper functions definition START. *****/

/* Data length must be 1-256 since DS3231's address space is 2^8 = 256 Bytes. */
static void ds3231_write_register(const ds3231_instance_t *dev, uint8_t registerAddress, uint8_t* dataBuff, uint8_t dataLen) {
	if((dataLen > 0 && dataLen <= 256)) {
		dev->i2cTransmit(dev->i2cHandle, RTC_MODULE_SLAVE_ADDRESS, registerAddress, RTC_MODULE_SLAVE_REG_ADDRESS_SIZE, dataBuff, dataLen);
	}

} /* End of ds3231_write_register(). */

/* Data length must be 1-256 since DS3231's address space is 2^8 = 256 Bytes. */
static void ds3231_read_register(const ds3231_instance_t *dev, uint8_t registerAddress, uint8_t* dataBuff, uint8_t dataLen) {
	if((dataLen > 0 && dataLen <= 256)) {
		dev->i2cReceive(dev->i2cHandle, RTC_MODULE_SLAVE_ADDRESS, registerAddress, RTC_MODULE_SLAVE_REG_ADDRESS_SIZE, dataBuff, dataLen);
	}
} /* End of ds3231_read_register(). */



static void ds3231_convertToBCD(uint8_t* fields, uint8_t fieldCount) {
	uint8_t low;  /* Lower 4-bits. */
	uint8_t high; /* Higher 4-bits. */

	for(int i = 0; i < fieldCount; ++i) {
		low = fields[i] % 10;  /* Extract Ones-place. */
		high = fields[i] / 10; /* Extract Tens-place. */
		fields[i] = (high << 4) | low;
	}

#ifdef DEBUG_MODE__
	for(int i = 0; i < fieldCount; ++i) {
		printf("Fields[%d]: %02X\r\n", i, fields[i]);
	}
#endif
} /* End of convertToBCD(). */

static void ds3231_convertToDecimal(uint8_t* fields, uint8_t fieldCount) {
	uint8_t low;  /* Lower 4-bits. */
	uint8_t high; /* Higher 4-bits. */

	for(int i = 0; i < fieldCount; ++i) {
		low = fields[i] & 0x0FU;
		high = fields[i] & 0xF0U;
		high >>= 4;
		fields[i] = (high * 10) + low;
	}

#ifdef DEBUG_MODE__
	for(int i = 0; i < fieldCount; ++i) {
		printf("Fields[%d]: %02X\r\n", i, fields[i]);
	}
#endif
} /* End of convertToDecimal(). */

static raw_time_t ds3231_getRawTime(const ds3231_instance_t *dev) {
	raw_time_t r_time;

	/* Start address of time read. */
	uint8_t regAddress = RTC_MODULE_SECONDS_REGISTER_ADDRESS;

	ds3231_read_register(dev, regAddress, (uint8_t*)&r_time, sizeof(raw_time_t));

#ifdef DEBUG_MODE__
	printf("r_time: 0x%02X raw_hours; 0x%02X raw_minutes; 0x%02X raw_seconds\r\n", r_time.hours, r_time.minutes, r_time.seconds);
#endif

	return r_time;
} /* End of getRawTime(). */


static raw_date_t ds3231_getRawDate(const ds3231_instance_t *dev) {
	raw_date_t r_date;
	uint8_t regAddress = RTC_MODULE_DAY_REGISTER_ADDRESS;

	ds3231_read_register(dev, regAddress, (uint8_t*)&r_date, sizeof(raw_date_t));

#ifdef DEBUG_MODE__
	printf("r_date: 0x%02X raw_Month; 0x%02Xd raw_Date; 0x%02X raw_Year\r\n", r_date.month, r_date.date, r_date.year);
#endif

	return r_date;
} /* End of getRawDate(). */
/***** Helper functions definition END. *****/


/***** API definition START. *****/

void ds3231_rtc_module_init(ds3231_instance_t *dev) {
//	i2c_init();

	raw_time_t rTime = ds3231_getRawTime(dev);

	/* If bit 6 is set then time is in 12-hour format. */
	if(rTime.hours & HOUR_FORTMAT_BIT_Msk) {
		dev->isTwentyFourHourFormat = 0;

		/* If bit 5 is set then time is PM. */
		dev->isPM = (rTime.hours & AM_PM_BIT_Msk) ? 1 : 0;
	}
	else {
		dev->isTwentyFourHourFormat = 1;
	}

	dev->initialized = 1;
} /* End of rtc_module_init(). */


void ds3231_setAMPM(ds3231_instance_t *dev, uint8_t toPM) {
	/* If not in 24-hour format. */
	if(!dev->isTwentyFourHourFormat) {
		uint8_t regAddress = RTC_MODULE_HOURS_REGISTER_ADDRESS;
		raw_time_t rTime = ds3231_getRawTime(dev);

		if(toPM) {
			if(!dev->isPM) {
				/* Set PM bit to set to PM. */
				rTime.hours |= AM_PM_BIT_Msk;

				/* Transmit data. */
				ds3231_write_register(dev, regAddress, (uint8_t *) &rTime, sizeof(raw_time_t));
			}
		}
		else {
			if(dev->isPM) {
				/* Set PM bit to set to PM. */
				rTime.hours &= ~AM_PM_BIT_Msk;

				/* Transmit data. */
				ds3231_write_register(dev, regAddress, (uint8_t *) &rTime, sizeof(raw_time_t));
			}
		}
	}
} /* End of setAMPM(). */

void ds3231_setTimeFormat(ds3231_instance_t *dev, uint8_t twenty_four_hr_format) {
	uint8_t regAddress = RTC_MODULE_HOURS_REGISTER_ADDRESS;

	raw_time_t rTime = ds3231_getRawTime(dev);

	if(twenty_four_hr_format) {
		/* If not currently in 24-hour format. */
		if(!dev->isTwentyFourHourFormat) {
			rTime.hours &= ~HOUR_FORTMAT_BIT_Msk;

			/* Transmit data. */
			ds3231_write_register(dev, regAddress, (uint8_t *) &rTime, sizeof(raw_time_t));
		}
	}
	else {
		/* If not currently in 12-hour format. */
		if(dev->isTwentyFourHourFormat) {
			rTime.hours |= HOUR_FORTMAT_BIT_Msk;

			/* Transmit data. */
			ds3231_write_register(dev, regAddress, (uint8_t *) &rTime, sizeof(raw_time_t));
		}
	}
} /* End of setTimeFormat(). */



void ds3231_setCurrentTime(ds3231_instance_t *dev, raw_time_t curTime, uint8_t twenty_four_hr_format, uint8_t isPM) {
	uint8_t regAddress = RTC_MODULE_SECONDS_REGISTER_ADDRESS;

#ifdef DS1307_MODULE__
	/* To prevent halting the oscillator on accident. */
	if(time.seconds & (0x80U)) {
		time.seconds &= ~(0x80U);
	}
#endif

	ds3231_convertToBCD((uint8_t*)&curTime, sizeof(raw_time_t));

	if(twenty_four_hr_format) {
		/* Set 24-hour format bit. */
		curTime.hours &= ~HOUR_FORTMAT_BIT_Msk;
	}
	else {
		/* Set 12-hour format bit. */
		curTime.hours |= HOUR_FORTMAT_BIT_Msk;
		if(isPM) {
			/* Set to PM. */
			curTime.hours |= AM_PM_BIT_Msk;
		}
	}

	/* Transmit data. */
	ds3231_write_register(dev, regAddress, (uint8_t *) &curTime, sizeof(raw_time_t));
} /* End of setCurrentTime(). */

void ds3231_setCurrentDate(ds3231_instance_t *dev, raw_date_t curDate) {
	uint8_t regAddress = RTC_MODULE_DAY_REGISTER_ADDRESS;

	ds3231_convertToBCD((uint8_t*)&curDate, sizeof(raw_date_t));

	/* Transmit data. */
	ds3231_write_register(dev, regAddress, (uint8_t *) &curDate, sizeof(raw_date_t));
} /* End of setCurrentDate(). */


/* Fetch the time in decimal. Always returns to time in 24-hour format. */
void ds3231_getCurrentTime(const ds3231_instance_t *dev, current_time_t* curTime) {
	raw_time_t r_time = ds3231_getRawTime(dev);

	/* Check hour format. */
	if(r_time.hours & HOUR_FORTMAT_BIT_Msk) { /* 12-Hour format. */
		uint8_t isPM = (r_time.hours & AM_PM_BIT_Msk) ? 1 : 0;

		/* Extract the bits for 12-hour format. */
		r_time.hours &= (0x1FU);

#ifdef DEBUG_MODE__
		printf("cr_time: 0x%02X raw_hours; 0x%02X raw_minutes; 0x%02X raw_seconds\r\n", c_time.hours, c_time.minutes, c_time.seconds);
#endif

		ds3231_convertToDecimal((uint8_t*)&r_time, sizeof(raw_time_t));

		/* Force 'hours' portion to 24-hour format. */
		if(isPM) {
			if(r_time.hours < 12) {
				r_time.hours += 12;
			}
		}
		else {
			if(r_time.hours == 12) {
				r_time.hours = 0;
			}
		}
	}
	else { /* 24-hour format. */
		ds3231_convertToDecimal((uint8_t*)&r_time, sizeof(raw_time_t));
	}

	curTime->hours = r_time.hours;
	curTime->minutes = r_time.minutes;
	curTime->seconds = r_time.seconds;
} /* End of getCurrentTime(). */

void ds3231_getCurrentDate(ds3231_instance_t *dev, current_date_t* curDate) {

	raw_date_t r_date = ds3231_getRawDate(dev);

	if(r_date.month & CENTURY_BIT_Msk) {
		dev->baseCentury += 100;
	}

	r_date.month &= ~CENTURY_BIT_Msk;

	ds3231_convertToDecimal((uint8_t*)&r_date, sizeof(raw_date_t));

	curDate->date = r_date.date;
	curDate->dayOfWeek = r_date.dayOfWeek;
	curDate->month = r_date.month;
	curDate->year = dev->baseCentury + r_date.year;
} /* End of getCurrentDate(). */

void ds3231_printFormattedDateTime(const ds3231_instance_t *dev, const current_time_t time, const current_date_t date, uint8_t selection) {

} /* End of printFormattedDateTime(). */

/* Get the formatted timerstamp: [mm/dd/yyyy] (h-m-s)*/
void ds3231_getTimeStamp(ds3231_instance_t *dev, char* dest) {
	current_time_t curTime;
	current_date_t curDate;

	ds3231_getCurrentTime(dev, &curTime);
	ds3231_getCurrentDate(dev, &curDate);

	sprintf(dest, "%02d/%02d/%04lu,%02d:%02d:%02d",
			curDate.month, curDate.date, curDate.year,
			curTime.hours, curTime.minutes, curTime.seconds);
}


/* Made for updating MCU RTC peripheral that accepts BCD format. */
void ds3231_getRawCurrentTime(const ds3231_instance_t *dev, raw_time_t* curRawTime) {
	*curRawTime = ds3231_getRawTime(dev);
}

void ds3231_getRawCurrentDate(ds3231_instance_t *dev, raw_date_t* curRawDate) {
	*curRawDate = ds3231_getRawDate(dev);
}

/***** API definition END. *****/
