/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include <stdint.h>
#include "stm32f030x6.h"

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

void systemClockInit(void)
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
  GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODER8)) | (GPIO_MODER_MODER8_1); /* (3) */
	//GPIOA->OTYPER|=GPIO_OTYPER_OT_8;
  GPIOA->AFR[1] |= 0x02; /* (4) */

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

  TIM1->PSC = 3; /* (1) */
  TIM1->ARR = 100; /* (2) */
  TIM1->CCR1 = 50; /* (3) */
  TIM1->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 ;//| TIM_CCMR1_OC1PE; /* (4) *///!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  //TIM1->CCER |= TIM_CCER_CC1E; /* (5) */
  //TIM1->BDTR |= TIM_BDTR_MOE; /* (6) */
  TIM1->CR1 |= TIM_CR1_CMS_0;// | TIM_CR1_CEN; /* (7) */
  TIM1->EGR |= TIM_EGR_UG; /* (8) */
}
//pwm cooller start
void startTimer1()
{
	TIM1->CCER |= TIM_CCER_CC1E; /* (5) */
	TIM1->BDTR |= TIM_BDTR_MOE; /* (6) */
	TIM1->CR1|=TIM_CR1_CEN;
}
int main(void)
{
	systemClockInit();
	initTimer1();
	startTimer1();
    /* Loop forever */
	for(;;);
}
