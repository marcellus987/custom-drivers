/*
 * sdCard_driver.h
 *
 *  Created on: Oct 1, 2026
 *      Author: mvps1
 */

#ifndef INC_SDCARD_DRIVER_H_
#define INC_SDCARD_DRIVER_H_

#include "gpio_driver.h"

/*
 * typedef void (*spi_receive_func)(void *spi_handle, uint8_t *dest, uint32_t size);
 *
 * */
typedef void (*spi_transmit_receive_func)(void *spi_handle, const uint8_t *src, uint8_t *dest, uint16_t Size);
typedef void (*spi_transmit_func)(void *spi_handle, const uint8_t *src, uint32_t size);
typedef void (*spi_setBaudrate_func)(void *spi_handle, uint8_t forInit);
typedef void (*delay_func)(const uint32_t delay);
typedef uint8_t (*spi_state_func)(void *spi_handle);

typedef struct {

	gpio_t csPin;
	write_func pinWrite;
	read_func pinRead;

	delay_func delay;

	void *spiHandle;
	spi_transmit_func spiTransmit;
	spi_transmit_receive_func spiTransmitReceive;
	spi_setBaudrate_func setBaudrate;
	spi_state_func getState;

	uint8_t initialized;         /* Init. status. */
} sdCard_instance_t;


uint8_t xSd_Init(sdCard_instance_t *dev);
uint8_t xSd_Read(const sdCard_instance_t *dev, uint32_t sector, uint8_t* buffer);
uint8_t xSd_Write(const sdCard_instance_t *dev, uint32_t sector, const uint8_t* buffer);
uint8_t xSd_Busy(const sdCard_instance_t *dev);

/* MISC commands. */
uint8_t xSend_CMD9(const sdCard_instance_t *dev, uint8_t* buffer, uint32_t bufferSize);
uint8_t xSend_CMD13(const sdCard_instance_t *dev);



#endif /* INC_SDCARD_DRIVER_H_ */
