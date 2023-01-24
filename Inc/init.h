#ifndef INIT_H
#define INIT_H

#define GPIOPIN_0  0x00000001
#define GPIOPIN_1  0x00000002
#define GPIOPIN_2  0x00000004
#define GPIOPIN_3  0x00000008
#define GPIOPIN_4  0x00000010
#define GPIOPIN_5  0x00000020
#define GPIOPIN_6  0x00000040
#define GPIOPIN_7  0x00000080
#define GPIOPIN_8  0x00000100
#define GPIOPIN_9  0x00000200
#define GPIOPIN_10 0x00000400
#define GPIOPIN_11 0x00000800
#define GPIOPIN_12 0x00001000
#define GPIOPIN_13 0x00002000
#define GPIOPIN_14 0x00004000
#define GPIOPIN_15 0x00008000


#define CPU_CLOCK 48000000

#define TIMER_TICK  	CPU_CLOCK/1000-1	// Нам нужен килогерц
void init();
void initTimer1(void);
void startTimer1();
void initEXTIPins();
void initSysTick();
void hardwareInit();
#endif//INIT_H
