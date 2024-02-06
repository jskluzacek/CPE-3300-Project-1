/******************************************************************************
 * @assignment	   : CPE3300 Project 1
 * @file           : led.c
 * @author         : Team Edward - Kenny Gifford and Jerico Skluzacek
 * @date		   : 01/23/24
 * @brief          : LED API
 *****************************************************************************/

#include "led.h"
#include "stm32regs.h"
#include "delay.h"
#include "channel_monitor.h"

static volatile RCC* const rcc = (RCC*)RCC_ADR;
static volatile GPIOX* const gpiob = (GPIOX*)GPIOB_ADR;
static volatile TIMX* const tim4 = (TIMX*) TIM4_ADR;
static volatile NVIC* const nvic = (NVIC*) NVIC_ADR;

/**
 * led_init:
 * Enables clock to GPIOB peripheral and sets PB5-15 (-11) to output mode
 * parameters: none
 * returns: none
 */
void led_init(void) {
	/* RCC */
	rcc->AHB1ENR |= (1<<1);	// Bit 1 == GPIOB
	rcc->APB1ENR |= (1<<2); // Bit 2 == TIM4

	/* LED Pins */
	gpiob->MODER |= OUTPUT_MASK;
	gpiob->MODER &= ~(OUTPUT_MASK<<1);

	/* Enable Interrupts */
	// enable TIM4 interrupt in NVIC
	nvic->ISER0 |= (1<<TIM4n);
	// enable TIM4 interrupt
	tim4->DIER |= (1<<0);

	/* Pre-load Timer Duration */
	tim4->ARR = LED_PERIOD;

	/* Start Timer */
	tim4->CR1 |= (1<<0);
}

/*
 * led_on: (private)
 * Write's input number to PB5-15 (-11)
 * parameters: binary representation of LED's
 * returns: none
 */
void led_on(int number) {
	int temp = number & PB5TO10;			// isolate PB5-10
	temp |= ((number & PB12TO15)<<1);		// isolate and prepend PB12-15 and skip PB11
	gpiob->ODR = (temp<<5);					// shift final value into place and write to ODR
}

void TIM4_IRQHandler(void) {
	// clear UIF pending bit
	tim4->SR &= ~(1<<0);

	// pause interrupts
	tim4->DIER &= ~(1<<0);

	// get state
	STATE state;
	__asm("PUSH {r0-r12, lr}");
	state = get_state();
	__asm("POP {r0-r12, lr}");
	__asm("BX LR");

	// update LEDs
	int leds = 0;
	switch (state) {
	case IDLE: {
		leds = IDLE_LED;
		break;
		}
	case BUSY_LOW: {
		leds = BUSY_LOW_LED;
		break;
		}
	case BUSY_HIGH: {
		leds = BUSY_HIGH_LED;
		break;
		}
	case COLLISION: {
		leds = COLL_LED;
		break;
		}
	};

	// write LED pins
	int temp = leds & PB5TO10;			// isolate PB5-10
	temp |= ((leds & PB12TO15)<<1);		// isolate and prepend PB12-15 and skip PB11
	gpiob->ODR = (temp<<5);

	// resume interrupts
	tim4->DIER |= (1<<0);
}
