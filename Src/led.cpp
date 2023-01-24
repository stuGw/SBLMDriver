/*
 * led.cpp
 *
 *  Created on: May 26, 2022
 *      Author: vit
 */

#include "led.h"

Led::Led(volatile uint32_t* p, uint32_t m, bool pol) {
	// TODO Auto-generated constructor stub
	port = p;
	pinMask = m;
	polarity = pol;
}

Led::~Led() {
	// TODO Auto-generated destructor stub
}

