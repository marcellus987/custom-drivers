/*
 * dht22_driver.c
 *
 *  Created on: Oct 3, 2026
 *      Author: mvps1
 */

#include "dht22_driver.h"

#define BIT_COUNT 40 /* Per datasheet, sensor transmit 40-bit data back to master MCU. */

/* State of each communication phase.*/
typedef enum {
	WAITING_RESPONSE_LOW,
	RESPONSE_LOW,
	RESPONSE_HIGH,
	DATA_LOW,
	DATA_HIGH,
} State;


/* API definition START. */

uint8_t dht22_read(const dht22_instance_t *dev, raw_dht22_data_t* data) {
	uint32_t startTime;
	State state;
	*data = (raw_dht22_data_t){0};
	uint8_t *curByte = (uint8_t*)data;
	int bitCount = 0;

	uint32_t timeout;

	/* Pull-down line to initiate communication with sensor. */
	dev->pinWrite(dev->ioPin.port, dev->ioPin.pin, 0);
	dev->delay_ms(10); /* 10 milli-seconds delay. */


	/* Configure as Input mode to capture response.  Commented out for testing Open-drain OUTPUT MODE. */
//	dev->changePinMode(dev->ioPin.port, dev->ioPin.pin, 1);

	/* Pull-high line to hand-over bus control to sensor. */
	dev->pinWrite(dev->ioPin.port, dev->ioPin.pin, 1);

	dev->delay_us(dev->timerHandle, 35); /* 35 micro-seconds delay. */

	state = WAITING_RESPONSE_LOW;

	while(bitCount < BIT_COUNT) {
			timeout = dev->getTime(dev->timerHandle);
			if(dev->pinRead(dev->ioPin.port, dev->ioPin.pin)) {
				switch(state) {
				case RESPONSE_LOW:
					state = RESPONSE_HIGH;
					break;
				case DATA_LOW:
					startTime = dev->getTime(dev->timerHandle);
					state = DATA_HIGH;
					break;
				default:
					while(dev->pinRead(dev->ioPin.port, dev->ioPin.pin)) {
						if((dev->getTime(dev->timerHandle) - timeout) > 100) {
							return 0;
						}
					}
				}
			}
			else {
				switch(state) {
				case WAITING_RESPONSE_LOW:
					state = RESPONSE_LOW;
					break;
				case RESPONSE_HIGH:
					state = DATA_LOW;
					break;
				case DATA_HIGH:
					uint32_t time = dev->getTime(dev->timerHandle) - startTime;
					uint8_t bit = (time > 40);
					curByte[bitCount >> 3] <<= 1;
					curByte[bitCount >> 3] |= bit;
					++bitCount;
					state = DATA_LOW;
					break;
				default:
					while(!(dev->pinRead(dev->ioPin.port, dev->ioPin.pin))) {
						if((dev->getTime(dev->timerHandle) - timeout) > 5000) {
							return 0;
						}
					}
				}
			}
		}

		/* Revert to output mode for next polling. Commented out for testing Open-drain OUTPUT MODE.*/
//		dev->changePinMode(dev->ioPin.port, dev->ioPin.pin, 0);
//		dev->pinWrite(dev->ioPin.port, dev->ioPin.pin, 1);
		return 1;
} /* End of sensor_read(). */

void processTempHumidSensorData(const raw_dht22_data_t *rawSensorData, dht22_data_t *sensorData) {
	uint16_t humRawBits = rawSensorData->raw_humidityInt << 8 | rawSensorData->raw_humidityDec;
	uint16_t tempRawBits = rawSensorData->raw_tempInt << 8 | rawSensorData->raw_tempDec;

	/***** Humidity data. *****/
	int wholeHumReading = (int)humRawBits;
	sensorData->humidityInt = wholeHumReading / 10;
	sensorData->humidityDec = wholeHumReading % 10;

	/***** Temperature data. *****/
	int wholeTempReading = ((int)(tempRawBits & 0x7FFF));

	/* Negate if MSB is set. */
	if(tempRawBits & 0x8000U) {
		wholeTempReading *= -1;
	}

	/* Extract Bits excluding MSB which is sign-bit. */
	sensorData->tempInt = wholeTempReading / 10;
	sensorData->tempDec = wholeTempReading % 10;
}


/* The function below are specific to STM32F411xe. */
//
///* Converts the raw temperature reading to Fahrenheit.
// * Param: raw bits temperature reading.
// * Return value: int value where decimal is moved to right one place.
// *               hence, it must be divided by 10 afterwards to get
// *               correct Integer and Decimal value. */
//int celsiusToFahrenheit(uint16_t rawTempReading) {
//	/* Enable full access to FPU Coprocessor. */
//	SCB->CPACR |= ((3UL << 10*2) | (3UL << 11*2));
//
//	/* Convert to int first. */
//	int wholeTempReading = ((int)(rawTempReading & 0x7FFF));
//
//	/* Negate if MSB is set. */
//	if(rawTempReading & 0x8000U) {
//		wholeTempReading *= -1;
//	}
//
//	/* Convert to float and adjust decimal point position. */
//	float tempInCelsius = (float)wholeTempReading / 10.0;
//
//	/* Formula: (Celsius * 9/5) + 32 = Fahrenheit. */
//	float tempInFahrenheit = (tempInCelsius * 9.0/5.0) + 32.0f;
//
//	/* Convert back to int. */
//	int rawFahr = tempInFahrenheit * 10;
//
//	/* Disable full access to FPU Coprocessor to save power. */
//	SCB->CPACR &= ~((3UL << 10*2) | (3UL << 11*2));
//
//	return rawFahr;
//} /* End of celsiusToFahrenheit(). */
//


/* API definition END. */



