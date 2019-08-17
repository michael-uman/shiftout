/*
 * ArduinoPins.c
 *
 *  Created on: Aug 6, 2019
 *      Author: muman
 */


#include "ArduinoPins.h"

/**
 * This structure is used to translate pin # to GPIO Port and Pin on STM32
 */
#ifdef STM32F413xx
struct digitalPinEntry arduinoPinTable[] = {
		{ GPIOA, GPIO_PIN_3 },							// D0
		{ GPIOA, GPIO_PIN_2 },							// D1
		{ GPIOC, GPIO_PIN_6 },							// D2
		{ GPIOA, GPIO_PIN_10 },							// D3
		{ GPIOC, GPIO_PIN_10 }, 						// D4
		{ GPIOA, GPIO_PIN_15 },							// D5
		{ GPIOA, GPIO_PIN_8 },							// D6
		{ GPIOC, GPIO_PIN_13 },							// D7

		{ GPIOC, GPIO_PIN_12 },							// D8
		{ GPIOA, GPIO_PIN_9 },							// D9
		{ GPIOA, GPIO_PIN_4 },							// D10
		{ GPIOA, GPIO_PIN_7 },							// D11
		{ GPIOA, GPIO_PIN_6 },							// D12
		{ GPIOA, GPIO_PIN_5 },							// D13
		{ GPIOB, GPIO_PIN_9 },							// D14
		{ GPIOB, GPIO_PIN_8 },							// D15
};
#endif
#ifdef STM32WB55xx
struct digitalPinEntry arduinoPinTable[] = {
		{ GPIOA, GPIO_PIN_3 },							// D0
		{ GPIOA, GPIO_PIN_2 },							// D1
		{ GPIOC, GPIO_PIN_6 },							// D2
		{ GPIOA, GPIO_PIN_10 },							// D3
		{ GPIOC, GPIO_PIN_10 }, 						// D4
		{ GPIOA, GPIO_PIN_15 },							// D5
		{ GPIOA, GPIO_PIN_8 },							// D6
		{ GPIOC, GPIO_PIN_13 },							// D7

		{ GPIOC, GPIO_PIN_12 },							// D8
		{ GPIOA, GPIO_PIN_9 },							// D9
		{ GPIOA, GPIO_PIN_4 },							// D10
		{ GPIOA, GPIO_PIN_7 },							// D11
		{ GPIOA, GPIO_PIN_6 },							// D12
		{ GPIOA, GPIO_PIN_5 },							// D13
		{ GPIOB, GPIO_PIN_9 },							// D14
		{ GPIOB, GPIO_PIN_8 },							// D15
};
#endif

/**
 * Write a value to a pin, using the arduino pin #.
 */
void digitalWrite(int pin, GPIO_PinState value) {
	HAL_GPIO_WritePin(arduinoPinTable[pin].port, arduinoPinTable[pin].pin, value);
}

