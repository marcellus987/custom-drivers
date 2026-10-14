/*
 * common_wrappers.c
 *
 *  Created on: Sep 30, 2026
 *      Author: Marcellus Von Sacramento
 */


#include "stm32f4xx_hal.h"
#include "stm32_commons.h"


void wrapper_delay(const uint32_t delay) {
	HAL_Delay(delay);
}

/***** SPI related commons START. *****/

/* Use only for SD card in SPI mode. Sets baudrate of spi for SD card
 * depending on whether it is in initialization phase or normal use. */
void wrapper_sdCard_spi_setBaudrate(void *spi_handle, uint8_t forInit) {
	if(spi_handle == NULL) {
		return;
	}

	SPI_HandleTypeDef *hspi = spi_handle;
	SPI_TypeDef *spix = hspi->Instance;

	/* Change only when ready and enabled. */
	if(hspi->State == HAL_SPI_STATE_READY) {
		/* Poll these bits to ensure proper disabling of SPI.*/
		while(spix->SR & SPI_SR_RXNE) {}
		while(!(spix->SR & SPI_SR_TXE)){}
		while(spix->SR & SPI_SR_BSY) {}
		/* Disable first before changing baudrate. */
		spix->CR1 &= ~(SPI_CR1_SPE);
		/* Clear Baud rate control bits configuration. */
		spix->CR1 &= ~(SPI_CR1_BR_Msk);

		/* Set new Baud rate. */
		if(forInit) { /* For initialization phase. */
			spix->CR1 |= SPI_BAUDRATEPRESCALER_64;
		}
		else { /* For normal use. */
			spix->CR1 |= SPI_BAUDRATEPRESCALER_2;
		}
//		/* Turn SPI on. */
//		spix->CR1 |= SPI_CR1_SPE; /* Automatically enabled by HAL Transactions. */
	}
} /* End of wrapper_sdCard_spi_setBaudrate(). */


uint8_t wrapper_get_spi_state(void *spi_handle) {
	SPI_HandleTypeDef *hspi = spi_handle;
	return(hspi->State == HAL_SPI_STATE_READY);
}


/* HAL_Transmit size is only 16-bit wide. */
void wrapper_spi_transmit(void *spi_handle, const uint8_t *src, uint32_t size) {
	SPI_HandleTypeDef *hspi = spi_handle;

	/* Transmit and reduce size until size <= UINT16_MAX. */
	while(size > UINT16_MAX) {
		HAL_SPI_Transmit(hspi, src, UINT16_MAX, COM_TIMEOUT);
		size -= UINT16_MAX;
		src += UINT16_MAX; /* Point to next starting byte to transfer. */
	}

	if(size > 0) {
		HAL_SPI_Transmit(hspi, src, (uint16_t)size, COM_TIMEOUT);
	}
}

/* For device that require specific dummy bytes when receiving from that device.
 * For instance SD card requires 0xFF dummy to be sent when receiving data from it. */
void wrapper_spi_transmit_receive(void *spi_handle, const uint8_t *src, uint8_t *dest, uint16_t size) {
	SPI_HandleTypeDef *hspi = spi_handle;
	/* Receive and reduce size until size <= UINT16_MAX. */
	while(size > UINT16_MAX) {
		HAL_SPI_TransmitReceive(hspi, src, dest, UINT16_MAX, COM_TIMEOUT);
		size -= UINT16_MAX;
		src += UINT16_MAX; /* Point to next starting byte to transfer. */
		dest += UINT16_MAX; /* Point to next starting byte to fill. */
	}

	if(size > 0) {
		HAL_SPI_TransmitReceive(hspi, src, dest, (uint16_t)size, COM_TIMEOUT);
	}
}

void wrapper_spi_receive(void *spi_handle, uint8_t *dest, uint32_t size) {
	SPI_HandleTypeDef *hspi = spi_handle;

	/* Receive and reduce size until size <= UINT16_MAX. */
	while(size > UINT16_MAX) {
		HAL_SPI_Receive(hspi, dest, UINT16_MAX, COM_TIMEOUT);
		size -= UINT16_MAX;
		dest += UINT16_MAX; /* Point to next starting byte to fill. */
	}

	if(size > 0) {
		HAL_SPI_Receive(hspi, dest, (uint16_t)size, COM_TIMEOUT);
	}
}

/***** SPI related commons END. *****/

/***** GPIO related commons START. *****/
void wrapper_pin_write(void* port, uint16_t pin, uint8_t state) {
	GPIO_TypeDef* pinPort = (GPIO_TypeDef*)port;
	HAL_GPIO_WritePin(pinPort, pin, state);
}
uint8_t wrapper_pin_read(void* port, uint16_t pin) {
	GPIO_TypeDef* pinPort = (GPIO_TypeDef*)port;
	return(HAL_GPIO_ReadPin(pinPort, pin));
}

/* This function only changes mode between input or output. */
void wrapper_gpio_pin_mode_change(void* port, uint16_t pin, uint8_t isInputMode) {
	GPIO_TypeDef *gpioPort = port;
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Pin = pin;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

	if(isInputMode) {
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
	}
	else {
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
	}
	HAL_GPIO_Init(gpioPort, &GPIO_InitStruct);
}

/***** GPIO related commons END. *****/

/***** I2C related commons START. *****/
void wrapper_i2c_transmit_repeat_start(void *i2c_handle, uint16_t devAddress, uint16_t regAddress, uint16_t regAddrSize, uint8_t *src, uint16_t size) {
	I2C_HandleTypeDef *hi2c = i2c_handle;

	/* Must shift by one before i2c transaction. */
	devAddress <<= 1;
	HAL_I2C_Mem_Write(hi2c, devAddress, regAddress, regAddrSize, src, size, COM_TIMEOUT);
}

/* Size is limited by the address space. 16-bit address = ~64 KB. */
void wrapper_i2c_receive_repeat_start(void *i2c_handle, uint16_t devAddress, uint16_t regAddress, uint16_t regAddrSize, uint8_t *dest, uint16_t size) {
	I2C_HandleTypeDef *hi2c = i2c_handle;

	/* Must shift by one before i2c transaction. */
	devAddress <<= 1;
	HAL_I2C_Mem_Read(hi2c, devAddress, regAddress, regAddrSize, dest, size, COM_TIMEOUT);
}
/***** I2C related commons END. *****/


/***** Power related commons START. *****/
void enterStandby(void) {
	/* Enable clock for PWR. */
	__HAL_RCC_PWR_CLK_ENABLE();

	HAL_SuspendTick();
	HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);

	/* Clear flags and pending interrupts to ensure it will enter Standby mode. */
	clearWakeUpFlags();
	EXTI->PR |= (0x67FFFFU);
	HAL_PWR_EnterSTANDBYMode();
}

/***** Power related commons. *****/

/* Clear wake up flags. */
void clearWakeUpFlags(void) {
	/* Enable clock for PWR. */
	__HAL_RCC_PWR_CLK_ENABLE();

	/* Clear standby flag, if applicable. */
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);

	/* Clear wake up flag. */
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
}

uint8_t wakeFromStandby(void) {
	/* Enable clock for PWR. */
	__HAL_RCC_PWR_CLK_ENABLE();

	return (__HAL_PWR_GET_FLAG(PWR_FLAG_SB) == SET);
}

void enterStopMode(void) {
	HAL_SuspendTick();

	/* Clear flags and pending interrupts to ensure it will enter Stop mode. */
	clearWakeUpFlags();
	EXTI->PR |= (0x67FFFFU);
	HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
	/* IT flags are handled automatically by HAL. */
}


/***** Power related commons END. *****/

/***** Timer related commons. *****/
//uint32_t wrapper_timer_get_time(void *timer_handle) {
//	TIM_HandleTypeDef * htimer = timer_handle;
//
//	return (htimer->Instance->CNT);
//}
//
///* Polling version. */
//void wrapper_timer_delay(void *timer_handle, uint32_t delay) {
//	TIM_HandleTypeDef * htimer = timer_handle;
//	TIM_TypeDef *timer = htimer->Instance;
//
//	uint32_t startTime = timer->CNT;
//
//	while((timer->CNT - startTime) < delay) {
//
//	}
//}

/***** RTC related commons START. *****/
void RTC_convertToBCD(uint8_t* fields, uint8_t fieldCount) {
	uint8_t low;  /* Lower 4-bits. */
	uint8_t high; /* Higher 4-bits. */

	for(int i = 0; i < fieldCount; ++i) {
		low = fields[i] % 10;  /* Extract Ones-place. */
		high = fields[i] / 10; /* Extract Tens-place. */
		fields[i] = (high << 4) | low;
	}
} /* End of convertToBCD(). */

void RTC_convertToDecimal(uint8_t* fields, uint8_t fieldCount) {
	uint8_t low;  /* Lower 4-bits. */
	uint8_t high; /* Higher 4-bits. */

	for(int i = 0; i < fieldCount; ++i) {
		low = fields[i] & 0x0FU;
		high = fields[i] & 0xF0U;
		high >>= 4;
		fields[i] = (high * 10) + low;
	}
} /* End of convertToDecimal(). */
/***** RTC related commons END. *****/


