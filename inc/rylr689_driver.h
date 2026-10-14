/*
 * rylr689_driver.h
 *
 *  Created on: Sep 29, 2026
 *      Author: mvps1
 */

#ifndef INC_RYLR689_DRIVER_H_
#define INC_RYLR689_DRIVER_H_

#include <stdint.h>
#include <stdio.h>
#include "gpio_driver.h"
#include "llcc68_defines.h"



typedef void (*spi_transmit_func)(void *spi_handle, const uint8_t *src, uint32_t size);
typedef void (*spi_receive_func)(void *spi_handle, uint8_t *dest, uint32_t size);


typedef struct {

	gpio_t csPin;
	gpio_t nresetPin;
	gpio_t busyPin;
	gpio_t rfswV1Pin;
	gpio_t rfswV2Pin;
	gpio_t itPin;

	write_func pinWrite;
	read_func pinRead;

	void *spiHandle;
	spi_transmit_func spiTransmit;
	spi_receive_func spiReceive;
	uint8_t initialized;

	uint8_t *rxBuff;
	uint32_t rxBuffSize;
	uint8_t *txBuff;
	uint32_t txBuffSize;
	/*
	 * uint8_t state;
	 *
	 * */
}rylr689_instance_t;


void rylr689_radio_init(rylr689_instance_t *dev);

/* Radio modes with polling. */
void rylr689_radio_receive(const rylr689_instance_t *dev, uint32_t* packetSize);
void rylr689_radio_send(const rylr689_instance_t *dev, const uint32_t packetSize);

/* Radio modes with interrupt. */
void rylr689_radio_receive_it(const rylr689_instance_t *dev);
void rylr689_radio_send_it(const rylr689_instance_t *dev, const uint32_t packetSize);
void rylr689_radio_get_packet(const rylr689_instance_t *dev, uint32_t* packetSize);



#endif /* INC_RYLR689_DRIVER_H_ */
