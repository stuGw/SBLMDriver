/*
 * led.h
 *
 *  Created on: May 26, 2022
 *      Author: vit
 */

#ifndef LED_H_
#define LED_H_
#include <stdint.h>
//polarity is used to set led connection type,
//false - led connected to 3.3v & cathode to pin
//true - led connected to GND & anode to pin
class Led {
	volatile uint32_t* port { nullptr };
	uint32_t pinMask { 0x00000000 };
	bool polarity { false };
	bool isOn { false };
public:
	Led(volatile uint32_t* port, uint32_t mask, bool pol);
	void on(){ if(polarity)*port |= pinMask; else  *port &= ~pinMask; isOn = true; }
	void off() { if(polarity)*port &= ~pinMask; else *port |= pinMask; isOn = false; }
	void tuggle() { if(isOn){off();}else{on();}};
	bool onState(){ return isOn; }
	virtual ~Led();
};

#endif /* LED_H_ */
