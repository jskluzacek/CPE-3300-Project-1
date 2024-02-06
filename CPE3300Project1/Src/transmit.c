/******************************************************************************
 * @assignment	   : CPE3300 Project 1
 * @file           : transmit.c
 * @author         : Team Edward - Kenny Gifford and Jerico Skluzacek
 * @date		   : 02/04/24
 * @brief          : Data Transmission API
 *****************************************************************************/
#include "channel_monitor.h"
#include "stm32regs.h"
#include "transmit.h"
#include <string.h>

static volatile GPIOX* const gpioc = (GPIOX*) GPIOC_ADR;
static volatile TIMX* const tim3 = (TIMX*) TIM3_ADR;
static volatile RCC* const rcc = (RCC*) RCC_ADR;
static volatile NVIC* const nvic = (NVIC*) NVIC_ADR;

static char buffer[BUFFER_SIZE];
static char index;

/**
 * tx_init:
 */
void tx_init(void) {
	index = 0;

	/* RCC */
	// enable RCC for GPIOC
	rcc->AHB1ENR |= (1<<2); 	// GPIOC = BIT 2
	// enable RCC for TIM3
	rcc->APB1ENR |= (1<<1);		// TIM3 = Bit 1

	/* Tx Pin */
	// set PC11 to output for Tx (rmw)
	gpioc->MODER |= (1<<PC11);
	gpioc->MODER &= ~(1<<(PC11+1));

	/* Enable Interrupts */
	// enable TIM3 interrupt in NVIC
	nvic->ISER0 |= (1<<TIM2n);
	// enable TIM3 interrupt
	tim3->DIER |= (1<<0);

	/* Pre-load Timer Duration */
	tim3->ARR = HALF_BIT_PERIOD;
}

/**
 * tx_string:
 *
 */
void tx_string(const unsigned char str[]) {
	strncpy(buffer, str, BUFFER_SIZE);
	tim3->CR1 |= (1<<0);
}

/**
 * TIM3_IRQHandler:
 * Interrupt handler for half-bit transmission periods.
 * parameters: none
 * returns: none
 */
void TIM3_IRQHandler(void) {
	// clear UIF pending bit
	tim3->SR &= ~(1<<0);

	// fetch next bit from buffer
	char bit = (buffer[index / (8 * 2)] &= (1 << ((index++ % 8) / 2)));

	// write low or high to PC11
	if ((bit && index % 2 == 1) || (!bit && index % 2 == 0)) {
		gpioc->ODR |= (1<<PC11);
	} else {
		gpioc->ODR &= !(1<<PC11);
	}
}
