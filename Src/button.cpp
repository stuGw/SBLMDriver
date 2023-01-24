/*
 * button.cpp
 *
 *  Created on: Jun 14, 2022
 *      Author: vit
 */

#include <button.h>

Button::Button() {
	// TODO Auto-generated constructor stub

}

Button::~Button() {
	// TODO Auto-generated destructor stub
}

void Button::reset()
{
	buttonTime = 0;
	countFall = 0;
	countRise = 0;
	state = FREE;
}

void Button::timeIncrease()
{
	if((state == DOUBLE) || (state == LONG) || (state == FREE)) return;

	buttonTime++;
	if(buttonTime>delay)
	{
		if((countFall == 1) && (countRise == 1))state = SINGLE;
		else if(countFall == 1) state = LONG;
		else if(countFall > 1) state = DOUBLE;
	}

}

void Button::fall()
{
	if(state>RISING)return;
	if(!(*ioPort&ioMask))
	{
		state = FALLING;
		countFall++;
		if((countFall == 2)&&(countRise)) state = DOUBLE;
	}
}
void Button::rise()
{
	if(state == FREE)return;
	if(state>RISING)return;
	if(*ioPort&ioMask)
	{
		state = RISING;
		countRise++;
		if((countRise == 1)&&(buttonTime > longTime)) state = LONG;
	}
}

void Button::init(volatile uint32_t* port, uint32_t mask, uint16_t longPressTime, uint16_t lifeTime)
{
	ioPort = port;
	ioMask = mask;
	delay = lifeTime;
	longTime = longPressTime;
}

uint8_t Button::getState()
{
	uint8_t lst { 0 };
	if(state >= SINGLE)
	{
		lst = state;
		reset();
	}
	return lst;
}
