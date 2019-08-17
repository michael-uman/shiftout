/*
 * digitalwrite.c
 *
 *  Created on: Jul 11, 2019
 *      Author: muman
 */


#include "digitalwrite.h"

static int latch_pin  = 8; 	// (ST_CP)
static int clock_pin  = 12;	// (SH_CP)
static int data_pin   = 11;	// (DS)

/**
 * This structure is used to translate pin # to GPIO Port and Pin on STM32
 */
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

/**
 * Write a value to a pin, using the arduino pin #.
 */
void digitalWrite(int pin, int value) {
	HAL_GPIO_WritePin(arduinoPinTable[pin].port, arduinoPinTable[pin].pin, value);
}

/**
 * Shift data out to the shift-register
 *
 * Uses globals 'data_pin' and 'clock_pin'
 */
void shiftOut(uint8_t bitOrder, uint8_t val) {
	uint8_t i;

	for (i = 0; i < 8; i++) {
		if (bitOrder == LSBFIRST)
			digitalWrite(data_pin, !!(val & (1 << i)));
		else
			digitalWrite(data_pin, !!(val & (1 << (7 - i))));

		digitalWrite(clock_pin, HIGH);
		digitalWrite(clock_pin, LOW);
	}
}

/**
 *	Writes the value to the shift-register, ultimately displaying on the LED's
 */
void write_value(uint8_t value) {
	digitalWrite(latch_pin, LOW);
	shiftOut(MSBFIRST, value);
	digitalWrite(latch_pin, HIGH);
}

/**
 * Configure the shift register. Default values are:
 *
 * latch = pin 8
 * clock = pin 12
 * data  = pin 11
 */
void configure_shift_register(int latch, int clock, int data) {
	latch_pin = latch;
	clock_pin = clock;
	data_pin  = data;
}
