/*
 * oled_SSH1106_driver.h
 *
 *  Created on: Sep 28, 2026
 *      Author: Marcellus Von Sacramento
 */

#ifndef INC_OLED_SH1106_DRIVER_H_
#define INC_OLED_SH1106_DRIVER_H_

#include <stdint.h>
#include "stm32f4xx.h"
#include "gpio_driver.h"

typedef void (*spi_transmit_func)(void *spi_handle, const uint8_t *src, uint32_t size);
typedef void (*gpio_pin_write_func)(void* port, uint16_t pin, uint8_t state);
typedef void (*delay_func)(const uint32_t delay); /* For now, systick-based delay. */

typedef struct {
	uint8_t page; /* Current page of the cursor. */
	uint8_t column; /* Current column of the cursor. */
} cursor_instance_t;

typedef struct {
	gpio_t dc;
	gpio_t cs;
	gpio_t nreset;
	gpio_pin_write_func gpio_write;

	delay_func delay_ms;

	void *spiHandle; /* SPI peripheral used for communication. */
	spi_transmit_func spiTransmit;

	cursor_instance_t cursor;

	uint8_t initialized;         /* Init. status. */
} oled_sh1106_instance_t;


uint8_t sh1106_init(oled_sh1106_instance_t *dev);
void sh1106_clearScreen(oled_sh1106_instance_t *dev);
void sh1106_nextLine(oled_sh1106_instance_t *dev);
void sh1106_previousLine(oled_sh1106_instance_t *dev);
void sh1106_clearPortion(const oled_sh1106_instance_t *dev, uint8_t startChar, uint8_t charCount);
void sh1106_clearPages(oled_sh1106_instance_t *dev, uint8_t start, uint8_t end);
void sh1106_setAtCharacter(oled_sh1106_instance_t *dev, uint8_t linePos, uint8_t charPos);
void sh1106_write_line(oled_sh1106_instance_t *dev, char *msg, uint8_t len);



#endif /* INC_OLED_SH1106_DRIVER_H_ */
