/******************************************************************************
 * @assignment	   : CPE3300 Project 1
 * @file           : interrupts.h
 * @author         : Team Edward - Kenny Gifford and Jerico Skluzacek
 * @date		   : 01/23/24
 * @brief          : Interrupt Configuration
 *****************************************************************************/
#include "stm32regs.h"

static volatile SYSCFG* const syscfg = (SYSCFG*) SYSCFG_ADR;
static volatile EXTI* const exti = (EXTI*) EXTI_ADR;
static volatile RCC* const rcc = (RCC*) RCC_ADR;

typedef enum {
	 IDLE,
	 BUSY,
	 COLLISION
} STATE;

static STATE state = IDLE;

/**
 * edge_detection_init:
 * Enables rising and falling edge detection for pin PC12
 * and resulting interrupt call on EXTI12.
 * parameters: none
 * returns: none
 */
void edge_detection_init(void) {
	// enable RCC for SYSCFG
	rcc->APB2RENR |= (1<<14);

	// external interrupt configuration for PC12
	syscfg->EXTICR4 |= 0x0010;

	// enable EXTI line 12
	exti->IMR |= (1<<12);

	// enable rising and falling edge detection for EXTI12
	exti->RTSR |= (1<<12);
	exti->FTSR |= (1<<12);

	NVIC_EnableIRQ(EXTI15_10n);
}

/**
 * EXTI15_10_IRQHandler:
 * Interrupt handler for EXTI12 resulting from edge detection from pin PC12
 * parameters: none
 * returns: none
 */
void EXTI15_10_IRQHandler(void) {
	//verify interrupt was called by EXTI12
	if (exti->PR & (1<<12)) {
		//TODO begin counter to detect idle or collision
		switch (state) {
		case IDLE: {

			break;
			}
		case BUSY: {

			break;
			}
		case COLLISION: {

			break;
			}
		};
	}
	NVIC_ClearPendingIRQ(EXTI15_10n);
}
