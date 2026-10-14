/*
 * gpio_driver.h
 *
 *  Created on: Sep 29, 2026
 *      Author: mvps1
 */

#ifndef INC_GPIO_DRIVER_H_
#define INC_GPIO_DRIVER_H_

#include <stdint.h>

typedef void (*write_func)(void* port, uint16_t pin, uint8_t state);
typedef uint8_t (*read_func)(void* port, uint16_t pin);

typedef struct {
	void *port;
	uint16_t pin;
}gpio_t;



#endif /* INC_GPIO_DRIVER_H_ */
