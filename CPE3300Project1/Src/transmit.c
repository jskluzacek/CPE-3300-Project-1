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

static unsigned char buffer[BUFFER_SIZE];
static int half_bit_index = 0;

/**
 * tx_init:
 */
void tx_init(void) {

	/* RCC */
	rcc->AHB1ENR |= (1<<2); 	// GPIOC = BIT 2
	rcc->APB1ENR |= (1<<1);		// TIM3 = Bit 1

	/* Tx Pin */
	// set PC11 to output for Tx (rmw)
	gpioc->MODER |= (1<<PC11);
	gpioc->MODER &= ~(1<<(PC11+1));

	/* Enable Interrupts */
	// enable TIM3 interrupt in NVIC
	nvic->ISER0 |= (1<<TIM3n);
	// enable TIM3 interrupt
	tim3->DIER |= (1<<0);

	/* Pre-load Timer Duration */
	tim3->ARR = HALF_BIT_PERIOD;
}

/**
 * tx_string:
 * conditions: channel_monitor.c -> channel_monitor_init() must be called
 * 		before running.
 */
void tx_string(const unsigned char str[]) {
	strncpy(buffer, str, BUFFER_SIZE);

	// busy wait for IDLE state
	while (get_state() != IDLE) {};

	// clear TIM3
	tim3->CNT = 0;
	// enable TIM3
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

	// check for collision
	STATE state;
	__asm("PUSH {r0-r12, lr}");
	state = get_state();
	__asm("POP {r0-r12, lr}");
	__asm("BX LR");

	if (state == COLLISION) {
		tim3->CR1 &= ~(1<<0);
		return;
	}

	// fetch next bit from buffer
	char bit = (buffer[half_bit_index / (8 * 2)] &= (1 << (half_bit_index / 2) % 8));

	// write low or high to PC11
	if ((bit && half_bit_index % 2 == 1) || (!bit && half_bit_index % 2 == 0)) {
		// write 0 to Tx pin
		gpioc->ODR |= (1<<PC11);
	} else {
		// write 1 to Tx pin
		gpioc->ODR &= !(1<<PC11);
	}

	half_bit_index++;
}
