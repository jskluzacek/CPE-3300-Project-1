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

static char buffer[BUFFER_SIZE];
static char index = 0;
static char prev_tx = 0;

void tx_init(void) {
	// enable RCC for TIM3
	rcc->APB1ENR |= (1<<1);		// TIM3 = Bit 1

	// set PC11 to output for Tx (rmw)
	gpioc->MODER |= (1<<PC11);
	gpioc->MODER &= ~(1<<(PC11+1));
}

void tx_string(const unsigned char str[]) {
	strncpy(buffer, str, BUFFER_SIZE);
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
