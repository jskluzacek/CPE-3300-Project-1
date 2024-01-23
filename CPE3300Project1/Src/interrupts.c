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

void edge_detection_init(void) {
	// enable RCC for SYSCFG
	rcc->APB2RENR |= (1<<14);

	// external interrupt configuration for PC12
	syscfg->EXTICR4 |= 0x0010;

	// unmask EXTI line 12
	exti->IMR |= (1<<12);

	// enable rising and falling edge detection for EXTI line 12
	exti->RTSR |= (1<<12);
	exti->FTSR |= (1<<12);

	NVIC_EnableIRQ(EXTI15_10n);
}

void EXTI15_10_IRQHandler(void) {
	NVIC_ClearPendingIRQ(EXTI15_10n);
	//TODO figure out how to differentiate that this IRQ is from EXTI12

}
