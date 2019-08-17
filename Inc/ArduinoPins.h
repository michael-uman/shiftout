/*
 * ArduinoPins.h
 *
 *  Created on: Aug 6, 2019
 *      Author: muman
 */

#ifndef ARDUINOPINS_H_
#define ARDUINOPINS_H_

#include "main.h"

struct digitalPinEntry {
	GPIO_TypeDef * port;
	uint16_t pin;
};

extern struct digitalPinEntry arduinoPinTable[];

#ifdef __cplusplus
extern "C" {
#endif

void digitalWrite(int pin, GPIO_PinState value);

#ifdef __cplusplus
}
#endif

#endif /* ARDUINOPINS_H_ */
