/*
 * ShiftRegister.cpp
 *
 *  Created on: Aug 6, 2019
 *      Author: muman
 */

#include "main.h"
#include "ArduinoPins.h"
#include "ShiftRegister.h"

/**
 * Constructor
 */
ShiftRegister::ShiftRegister() {

}

/**
 * Constructor
 */
ShiftRegister::ShiftRegister(int dataPin, int clockPin, int latchPin,
		SHIFT_ORDER order) :
		_dp(dataPin), _cp(clockPin), _lp(latchPin), _order(order) {

}

/**
 * Destructor
 */
ShiftRegister::~ShiftRegister() {

}

bool ShiftRegister::begin(int dataPin, int clockPin, int latchPin, int order) {
	if (dataPin != -1) {
		_dp = dataPin;
	}
	if (clockPin != -1) {
		_cp = clockPin;
	}
	if (latchPin != -1) {
		_lp = latchPin;
	}
	if (_order != -1) {
		_order = (SHIFT_ORDER) order;
	}

	return true;
}

/**
 * Shift data out to the shift-register
 *
 * Uses globals 'data_pin' and 'clock_pin'
 */
void ShiftRegister::shiftOut(uint8_t val) {
	uint8_t i;

	for (i = 0; i < 8; i++) {
		if (_order == LSBFIRST)
			digitalWrite(_dp, (val & (1 << i)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
		else
			digitalWrite(_dp, (val & (1 << (7 - i))) ? GPIO_PIN_SET : GPIO_PIN_RESET);

		digitalWrite(_cp, GPIO_PIN_SET);
		digitalWrite(_cp, GPIO_PIN_RESET);
	}
}

/**
 *	Writes the value to the shift-register, ultimately displaying on the LED's
 */
void ShiftRegister::write(uint8_t value) {
	digitalWrite(_lp, GPIO_PIN_RESET);
	shiftOut(value);
	digitalWrite(_lp, GPIO_PIN_SET);
}
