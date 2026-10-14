/*
 * Filename: dht22_driver.h
 *
 * Author: Marcellus Von Sacramento
 *
 * Purpose: API for STM32F411 MCU with DHT22(AM2302) temperature-humidity sensor module.
 *
 * Copyright(c).
 *
 */

#ifndef INC_DHT22_DRIVER_H_
#define INC_DHT22_DRIVER_H_



#include <stdint.h>
#include <stdio.h>
#include "gpio_driver.h"

/* Structure of data from sensor. */
typedef struct {
	uint8_t raw_humidityInt;
	uint8_t raw_humidityDec;
	uint8_t raw_tempInt;
	uint8_t raw_tempDec;
	uint8_t raw_checksum;
} raw_dht22_data_t;

/* Structure of data from sensor. */
typedef struct {
	int humidityInt;
	int humidityDec;
	int tempInt;
	int tempDec;
} dht22_data_t;

/* Timer has to have micro-second precision. */
typedef void (*timer_delay_func)(void *timer_handle, uint32_t delay);
typedef uint32_t (*timer_get_time_func)(void *timer_handle);
typedef void (*gpio_pin_mode_change_func)(void* port, uint16_t pin, uint8_t isInputMode);

/* HAL_Delay. Usually, a milli-second precision.*/
typedef void (*hal_delay_func)(const uint32_t delay);

typedef struct {
	void *timerHandle;
	gpio_t ioPin;
	write_func pinWrite;
	read_func pinRead;

	timer_delay_func delay_us;
	hal_delay_func delay_ms;

	timer_get_time_func getTime;
	gpio_pin_mode_change_func changePinMode;
} dht22_instance_t;

uint8_t dht22_read(const dht22_instance_t *dev, raw_dht22_data_t* data);
void processTempHumidSensorData(const raw_dht22_data_t *rawSensorData, dht22_data_t *sensorData);

#endif /* INC_DHT22_DRIVER_H_ */
