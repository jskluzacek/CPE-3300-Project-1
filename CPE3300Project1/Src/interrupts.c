/******************************************************************************
 * @assignment	   : CPE3300 Project 1
 * @file           : interrupts.h
 * @author         : Team Edward - Kenny Gifford and Jerico Skluzacek
 * @date		   : 01/23/24
 * @brief          : Interrupt Configuration
 *****************************************************************************/
#include "stm32regs.h"
#include "interrupts.h"

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
}

/**
 * EXTI15_10_IRQHandler:
 * Interrupt handler for EXTI12 resulting from edge detection on pin PC12.
 * parameters: none
 * returns: none
 */
void EXTI15_10_IRQHandler(void) {
	// clear EXTI12 pending bit
	exti->PR = (1<<12);
	switch (state) {
	case IDLE: {
		state = BUSY_LOW;
		//TODO reset collision timer
		break;
		}
	case BUSY_LOW: {
		state = BUSY_HIGH;
		//TODO reset idle timer
		break;
		}
	case BUSY_HIGH: {
		state = BUSY_LOW;
		//TODO reset collision timer
		break;
		}
	case COLLISION: {
		//TODO wait random amount of time before switching
		state = BUSY_HIGH;
		//TODO reset idle timer
		break;
		}
	};

//	//verify interrupt was called by EXTI12
//	if (exti->PR & (1<<12)) {
//
//	}
}
