/******************************************************************************
 * @assignment	   : CPE3300 Project 1
 * @file           : interrupts.h
 * @author         : Team Edward - Kenny Gifford and Jerico Skluzacek
 * @date		   : 01/23/24
 * @brief          : Interrupt Configuration
 *****************************************************************************/
#include "stm32regs.h"
#include "interrupts.h"

#define CPU_FREQ 	16000000UL 			// system clock speed is 16MHz
//#define DELAY_TIME 	0.00111 * CPU_FREQ	// max time between edges is 1.11ms
//#define COLL_TIME 	0.00110 * CPU_FREQ	// collision timeout is 1.10ms
//#define IDLE_TIME	0.00113 * CPU_FREQ 	// idle timeout is 1.13ms
#define DELAY_TIME 	0.00111 * CPU_FREQ	// max time between edges is 1.11ms
#define COLL_TIME 	17600	// collision timeout is 1.10ms
#define IDLE_TIME	18080 	// idle timeout is 1.13ms

static volatile TIMX* const tim2 = (TIMX*) TIM2_ADR;
static volatile SYSCFG* const syscfg = (SYSCFG*) SYSCFG_ADR;
static volatile EXTI* const exti = (EXTI*) EXTI_ADR;
static volatile RCC* const rcc = (RCC*) RCC_ADR;
static volatile NVIC* const nvic = (NVIC*) NVIC_ADR;

static STATE state = IDLE;

/**
 * get_state:
 * Returns current channel state.
 * parameters: none
 * returns: none
 */
STATE get_state(void) {
	return state;
}

/**
 * edge_detection_init:
 * Enables rising and falling edge detection for pin PC12
 * and resulting interrupt call on EXTI12.
 * parameters: none
 * returns: none
 */
void edge_detection_init(void) {
	// enable RCC for SYSCFG
	rcc->APB2ENR |= (1<<14);

	// external interrupt configuration for PC12
	syscfg->EXTICR4 |= 0b0010;

	// enable EXTI line 12
	exti->IMR |= (1<<12);

	// enable rising and falling edge detection for EXTI12
	exti->RTSR |= (1<<12);
	exti->FTSR |= (1<<12);

	// enable interrupt in NVIC
	nvic->ISER1 |= (1<<(EXTI15_10n - 32));


	// enable RCC for TIM2
	rcc->APB1ENR |= (1<<0);

	// configure TIM2
	// TODO
	tim2->ARR = 0;
//	tim2->EGR |= (1<<0);

//	// enable update interrupt
//	tim2->DIER |= (1<<0);

	// enable interrupt in NVIC
	nvic->ISER0 |= (1<<TIM2n);

	// enable TIM2
	tim2->CR1 |= (1<<0);
}

/**
 * TIM2_IRQHandler:
 * Interrupt handler for idle/collision timer timeout whilst in busy states.
 */
void TIM2_IRQHandler(void) {
	// clear TIF pending bit
	tim2->SR &= ~(1<<6);
	// update state
	switch (state) {
	case BUSY_LOW: {
		state = COLLISION;
		tim2->DIER &= ~(1<<0);
		break;
		}
	case BUSY_HIGH: {
		state = IDLE;
		tim2->DIER &= ~(1<<0);
		break;
		}
	default: break;
	};
}

/**
 * EXTI15_10_IRQHandler:
 * Interrupt handler for rising/falling edge detection on PC12 by EXTI12.
 * parameters: none
 * returns: none
 */
void EXTI15_10_IRQHandler(void) {
	// clear EXTI12 pending bit
	exti->PR = (1<<12);
	//TODO temporarily disable interrupts to prevent race condition?
	// update state
	switch (state) {
	case IDLE: {
		state = BUSY_LOW;
		// reset collision timer
//		tim2->ARR = 0;
		tim2->CR1 &= ~(1<<0);
//		tim2->EGR = (1<<0);
//		tim2->ARR = COLL_TIME;
		tim2->CNT = 0;
		tim2->ARR = COLL_TIME;
		tim2->CR1 |= (1<<0);
		tim2->DIER |= (1<<0);
		break;
		}
	case BUSY_LOW: {
		state = BUSY_HIGH;
		// reset idle timer
//		tim2->ARR = 0;
		tim2->CR1 &= ~(1<<0);
//		tim2->EGR = (1<<0);
//		tim2->ARR = IDLE_TIME;
		tim2->CNT = 0;
		tim2->ARR = IDLE_TIME;
		tim2->CR1 |= (1<<0);
		break;
		}
	case BUSY_HIGH: {
		state = BUSY_LOW;
		// reset collision timer
//		tim2->ARR = 0;
		tim2->CR1 &= ~(1<<0);
//		tim2->EGR = (1<<0);
//		tim2->ARR = COLL_TIME;
		tim2->CNT = 0;
		tim2->ARR = COLL_TIME;
		tim2->CR1 |= (1<<0);
		break;
		}
	case COLLISION: {
		//TODO wait random amount of time before switching
		state = BUSY_HIGH;
		// reset idle timer
//		tim2->ARR = 0;
		tim2->CR1 &= ~(1<<0);
//		tim2->EGR = (1<<0);
//		tim2->ARR = IDLE_TIME;
		tim2->CNT = 0;
		tim2->ARR = IDLE_TIME;
		tim2->CR1 |= (1<<0);
		tim2->DIER |= (1<<0);
		break;
		}
	default: break;
	};
}
