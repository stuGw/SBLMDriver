#include <stdint.h>
#include "stm32f030x6.h"
#include "init.h"

void init(void);
void Default_Handler(void);
void HardFault_Handler(void);
void WWDG_IRQHandler(void);

void EXTI0_1_IRQHandler(void);
void SysTick_Handler(void);
int main(void);
// The following are 'declared' in the linker script
extern unsigned char  INIT_DATA_VALUES;
extern unsigned char  INIT_DATA_START;
extern unsigned char  INIT_DATA_END;
extern unsigned char  BSS_START;
extern unsigned char  BSS_END;

extern unsigned char  _sidata;//INIT_DATA_VALUES;
extern unsigned char  _sdata;//INIT_DATA_START;
extern unsigned char  _edata;//INIT_DATA_END;
extern unsigned char  _sbss;//BSS_START;
extern unsigned char  _ebss;//BSS_END;
extern unsigned int	  _estack;
//#define __bss_start__ BSS_START
//#define __bss_end__ BSS_END

extern void (*__preinit_array_start []) (void) __attribute__((weak));
extern void (*__preinit_array_end []) (void) __attribute__((weak));
extern void (*__init_array_start []) (void) __attribute__((weak));
extern void (*__init_array_end []) (void) __attribute__((weak));

typedef void (*fptr)(void);

#pragma GCC push_options
#pragma GCC optimize ("O0")
const fptr Vectors[] __attribute__((section(".vectors"))) ={
		(fptr)0x20001000,
		init,
		Default_Handler,//NMI_Handler,
		HardFault_Handler,
		Default_Handler,
		Default_Handler,
		Default_Handler,
		Default_Handler,
		Default_Handler,
		Default_Handler,
		Default_Handler,
		Default_Handler,
		Default_Handler,
		Default_Handler,
		Default_Handler,
		SysTick_Handler,
		WWDG_IRQHandler,               			/* Window Watchdog interrupt                             */
		Default_Handler,//PVD_IRQHandler,                			/* PVD and VDDIO2 supply comparator interrupt            */
		Default_Handler,//RTC_IRQHandler,                			/* RTC interrupts                                        */
		Default_Handler,//FLASH_IRQHandler,              			/* Flash global interrupt                                */
		Default_Handler,//RCC_IRQHandler,                			/* RCC global interruptr                                 */
		EXTI0_1_IRQHandler,            			/* EXTI Line[1:0] interrupts                             */
		Default_Handler,//EXTI2_3_IRQHandler,            			/* EXTI Line[3:2] interrupts                             */
		Default_Handler,//EXTI4_15_IRQHandler,           			/* EXTI Line15 and EXTI4 interrupts                      */
		Default_Handler,                             			/* Reserved                                              */
		Default_Handler,//DMA1_CH1_IRQHandler,           			/* DMA1 channel 1 interrupt                              */
		Default_Handler,//DMA1_CH2_3_IRQHandler,         			/* DMA1 channel 2 and 3 interrupt                        */
		Default_Handler,//DMA1_CH4_5_IRQHandler,         			/* DMA1 channel 4 and 5 interrupt                        */
		Default_Handler,//ADC_IRQHandler,                			/* ADC interrupt                                         */
		Default_Handler,//TIM1_BRK_UP_TRG_COM_IRQHandler,			/* TIM1 break, update, trigger and commutation interrupt */
		Default_Handler,//TIM1_CC_IRQHandler,            			/* TIM1 Capture Compare interrupt                        */
		Default_Handler,                             			/* Reserved                                              */
		Default_Handler,//TIM3_IRQHandler,               			/* TIM3 global interrupt                                 */
		Default_Handler,//TIM6_IRQHandler,               			/* TIM6 global interrupt                                 */
		Default_Handler,                            			/* Reserved                                              */
		Default_Handler,//TIM14_IRQHandler,              			/* TIM14 global interrupt                                */
		Default_Handler,//TIM15_IRQHandler,              			/* TIM15 global interrupt                                */
		Default_Handler,//TIM16_IRQHandler,              			/* TIM16 global interrupt                                */
		Default_Handler,//TIM17_IRQHandler,              			/* TIM17 global interrupt                                */
		Default_Handler,//I2C1_IRQHandler,               			/* I2C1 global interrupt                                 */
		Default_Handler,//I2C2_IRQHandler,               			/* I2C2 global interrupt                                 */
		Default_Handler,//SPI1_IRQHandler,               			/* SPI1_global_interrupt                                 */
		Default_Handler,//SPI2_IRQHandler,               			/* SPI2 global interrupt                                 */
		Default_Handler,//USART1_IRQHandler,             			/* USART1 global interrupt                               */
		Default_Handler,//USART2_IRQHandler,             			/* USART2 global interrupt                               */
		Default_Handler,//USART3_4_5_6_IRQHandler,       			/* USART3, USART4, USART5, USART6 global interrupt       */
		Default_Handler,                             			/* Reserved                                              */
		Default_Handler//USB_IRQHandler                			/* USB global interrupt                                  */

};

void initClock()
{

	FLASH->ACR = FLASH_ACR_LATENCY;

	// настраиваем систему тактирования
	RCC->CR |= RCC_CR_HSEON;  //включаем генератор HSE
	while(!(RCC->CR & RCC_CR_HSERDY));  //ожидание готовности HSE
	RCC->CFGR2 = RCC_CFGR2_PREDIV_DIV1;  // предделитель для PLL
	RCC->CFGR = RCC_CFGR_PLLSRC_HSE_PREDIV | RCC_CFGR_PLLMUL6;  // настройка PLL
	RCC->CR |= RCC_CR_PLLON;  //включаем PLL
	while(!(RCC->CR & RCC_CR_PLLRDY));  //ожидание готовности PLL
	RCC->CFGR |= RCC_CFGR_SW_PLL;  // настраиваем делители шин и переводим тактирование от PLL

}

void initTimer1(void)
{
  /* (1) Enable the peripheral clock of Timer x */
  /* (2) Enable the peripheral clock of GPIOA */
  /* (3) Select alternate function mode on GPIOA pin 8 */
  /* (4) Select AF2 on PA8 in AFRH for TIM1_CH1 */

  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; /* (1) */
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN; /* (2) */
  GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODER9)) | (GPIO_MODER_MODER9_1); /* (3) */
	//GPIOA->OTYPER|=GPIO_OTYPER_OT_8;
  GPIOA->AFR[1] |= 0x20; /* (4) */

  /* (1) Set prescaler to 47, so APBCLK/48 i.e 1MHz */
  /* (2) Set ARR = 8, as timer clock is 1MHz and center-aligned counting,
         the period is 16 us */
  /* (3) Set CCRx = 7, the signal will be high during 14 us */
  /* (4) Select PWM mode 1 on OC1  (OC1M = 110),
         enable preload register on OC1 (OC1PE = 1, reset value) */
  /* (5) Select active high polarity on OC1 (CC1P = 0, reset value),
         enable the output on OC1 (CC1E = 1)*/
  /* (6) Enable output (MOE = 1)*/
  /* (7) Enable counter (CEN = 1)
         select center-aligned mode 1 (CMS = 01) */
  /* (8) Force update generation (UG = 1) */

  TIM1->PSC = 480;//3; /* (1) */
  TIM1->ARR = 1000;//100; /* (2) */
  TIM1->CCR2 = 62;//50; /* (3) */
  TIM1->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 ;//| TIM_CCMR1_OC1PE; /* (4) *///!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  //TIM1->CCER |= TIM_CCER_CC1E; /* (5) */
  //TIM1->BDTR |= TIM_BDTR_MOE; /* (6) */
  TIM1->CR1 |= TIM_CR1_CMS_0;// | TIM_CR1_CEN; /* (7) */
  TIM1->EGR |= TIM_EGR_UG; /* (8) */
}

void initSysTick()
{
	SysTick->LOAD=TIMER_TICK;
	SysTick->VAL=TIMER_TICK;
	SysTick->CTRL=	SysTick_CTRL_CLKSOURCE_Msk |
	                SysTick_CTRL_TICKINT_Msk   |
	                SysTick_CTRL_ENABLE_Msk;
}

void initEXTIPins()
{
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN; /* (2) */

	GPIOA->MODER &=~0x0000000c;
	GPIOA->PUPDR |= 0x00000004;
	//SYSCFG->EXTICR[0] |= 0x0010;

	EXTI->RTSR |= 0x00000002;
	EXTI->FTSR |= 0x00000002;
	EXTI->IMR |= 0x00000002;
	EXTI->SWIER |= 0x00000002;

	GPIOA->MODER &=~0x00000003;
		GPIOA->PUPDR |= 0x00000001;
		//SYSCFG->EXTICR[0] |= 0x0010;

		EXTI->RTSR |= 0x00000001;
		EXTI->FTSR |= 0x00000001;
		EXTI->IMR |= 0x00000001;
		EXTI->SWIER |= 0x00000001;

}

void startTimer1()
{
	TIM1->CCER |= TIM_CCER_CC2E; /* (5) */
	TIM1->BDTR |= TIM_BDTR_MOE; /* (6) */
	TIM1->CR1|=TIM_CR1_CEN;
}

void hardwareInit()
{
	initSysTick();
	initTimer1();
	initEXTIPins();

}


void init_array()
{
    // This function calls constructors for global and static objects
    uint32_t count;
    uint32_t i;

    count = __preinit_array_end - __preinit_array_start;
    for (i = 0; i < count; i++)
        __preinit_array_start[i] ();
    count = __init_array_end - __init_array_start;
    for (i = 0; i < count; i++)
        __init_array_start[i] ();
}

void init()
{
// do global/static data initialization
	unsigned char *src;
	unsigned char *dest;
	unsigned len;
    initClock();
	src= &_sidata;//INIT_DATA_VALUES;
	dest= &_sdata;//INIT_DATA_START;
	len= &_edata-&_sdata;
	while (len--)
		*dest++ = *src++;
// zero out the uninitialized global/static variables
	dest = &_sbss;
	len = &_ebss - &_sbss;
	while (len--)
		*dest++=0;
    init_array();

    hardwareInit();

	main();
}

void Default_Handler()
{
	while(1);
}
void HardFault_Handler()
{
	while(1);
}

void WWDG_IRQHandler()
{
	while(1);
}
