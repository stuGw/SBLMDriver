/*
 * button.h
 *
 *  Created on: Jun 14, 2022
 *      Author: vit
 */

#ifndef BUTTON_H_
#define BUTTON_H_
#include <stdint.h>

class Button {
	volatile uint32_t* ioPort { nullptr };
	uint32_t ioMask { 0x00000001 };
	uint8_t state { FREE };
public:
	enum State : uint8_t { FREE = 0x00, FALLING = 0x01, RISING = 0x02, SINGLE = 0x04, LONG = 0x10, DOUBLE = 0x20 };

	uint8_t countRise { 0 };
	uint8_t countFall { 0 };
	uint16_t delay { 250 };
	uint16_t buttonTime { 0 };
	uint16_t longTime { 100 };
	void timeIncrease();
	void fall();
	void rise();
	void init(volatile uint32_t* port, uint32_t mask, uint16_t longPressTime, uint16_t lifeTime);
	void reset();
	bool isDown(){ if((*ioPort & ioMask) == 0) return true; else return false;}
	uint8_t getState();
	Button();
	virtual ~Button();
};

#endif /* BUTTON_H_ */
