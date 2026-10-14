/*
 * common_wrapper.h
 *
 *  Created on: Sep 30, 2026
 *      Author: Marcellus Von Sacramento
 */

#ifndef INC_STM32_COMMONS_H_
#define INC_STM32_COMMONS_H_

#include <stdint.h>
#include <string.h>

#define COM_TIMEOUT 100
#define SD_TIMEOUT 1000
#define RX_BUFF_SIZE 256
#define TX_BUFF_SIZE 256


void wrapper_delay(const uint32_t delay);

/***** SPI related commons. *****/

/* Use only for SD card in SPI mode. Sets baudrate of spi for SD card
 * depending on whether it is in initialization phase or normal use. */
void wrapper_sdCard_spi_setBaudrate(void *spi_handle, uint8_t forInit);
void wrapper_spi_transmit(void *spi_handle, const uint8_t *src, uint32_t size);
void wrapper_spi_receive(void *spi_handle, uint8_t *dest, uint32_t size);
uint8_t wrapper_get_spi_state(void *spi_handle);
void wrapper_spi_transmit_receive(void *spi_handle, const uint8_t *src, uint8_t *dest, uint16_t size);

/***** GPIO related commons. *****/
void wrapper_pin_write(void* port, uint16_t pin, uint8_t state);
uint8_t wrapper_pin_read(void* port, uint16_t pin);
void wrapper_gpio_pin_mode_change(void* port, uint16_t pin, uint8_t isInputMode);

/***** I2C related commons. *****/
void wrapper_i2c_transmit_repeat_start(void *i2c_handle, uint16_t devAddress, uint16_t regAddress, uint16_t regAddrSize, uint8_t *src, uint16_t size);
void wrapper_i2c_receive_repeat_start(void *i2c_handle, uint16_t devAddress, uint16_t regAddress, uint16_t regAddrSize, uint8_t *dest, uint16_t size);


/***** Power related commons. *****/
void enterStandby(void);
uint8_t wakeFromStandby(void);
void clearWakeUpFlags(void);
void enterStopMode(void);


/***** Timer related commons. *****/
uint32_t wrapper_timer_get_time(void *timer_handle);
void wrapper_timer_delay(void *timer_handle, uint32_t delay);

/***** RTC related commons. *****/
void RTC_convertToDecimal(uint8_t* fields, uint8_t fieldCount);
void RTC_convertToBCD(uint8_t* fields, uint8_t fieldCount);



#endif /* INC_STM32_COMMONS_H_ */
