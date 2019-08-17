/*
 * digitalwrite.h
 *
 *  Created on: Jul 11, 2019
 *      Author: muman
 */

#ifndef DIGITALWRITE_H_
#define DIGITALWRITE_H_

#include "stm32wbxx_hal.h"

// Pin state
#define HIGH		GPIO_PIN_SET
#define LOW     GPIO_PIN_RESET

// Shift order
#define LSBFIRST 0
#define MSBFIRST 1


struct digitalPinEntry {
	GPIO_TypeDef * port;
	uint16_t pin;
};

void    configure_shift_register (int latch, int clock, int data);
void 		digitalWrite (int pin, int value);
void 		shiftOut (uint8_t bitOrder, uint8_t val);
void 		write_value (uint8_t value);

#endif /* DIGITALWRITE_H_ */
