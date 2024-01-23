/******************************************************************************
 * @assignment	   : CPE3300 Project 1
 * @file           : delay.c
 * @author         : Team Edward - Kenny Gifford and Jerico Skluzacek
 * @date		   : 01/23/24
 * @brief          : Delay API
 *****************************************************************************/

#include <stdint.h>
#include "delay.h"

static volatile uint32_t* const stk_base = (uint32_t*) STK_BASE;

static int count = 0;

/**
 * delay_ms:
 * 		Uses the Systick Timer to delay n number of MILLIseconds
 */
void delay_ms(int n) {
	// load LOAD register
	*(stk_base + LOAD_OFFSET) = MS_CYCLES;
	// clear VALUE register
	*(stk_base + VAL_OFFSET) = 0;
	// clear CTRL and set source and interrupt
	*(stk_base + CTRL_OFFSET) = (CLK_SRC | TICK_INT);

	count = n;

	*(stk_base + CTRL_OFFSET) |= STK_EN;		// start timer

	while (count != 0) {
	}

	*(stk_base + CTRL_OFFSET) &= ~STK_EN;		// stop timer
}

/**
 * delay_us:
 * 		Uses the Systick Timer to delay n number of MICROseconds
 */
void delay_us(int n) {
	// load LOAD register
		*(stk_base + LOAD_OFFSET) = US_CYCLES;
		// clear VALUE register
		*(stk_base + VAL_OFFSET) = 0;
		// clear CTRL and set source and interrupt
		*(stk_base + CTRL_OFFSET) = (CLK_SRC | TICK_INT);

		count = n;

		*(stk_base + CTRL_OFFSET) |= STK_EN;		// start timer

		while (count != 0) {
		}

		*(stk_base + CTRL_OFFSET) &= ~STK_EN;		// stop timer
}

/**
 * SysTick_Handler:
 * 		Counts down from user defined count value, ending interrupts at 0
 * 		and effectively ending delay
 */
void SysTick_Handler(void) {
	count--;
	if (count == 0) {
		*(stk_base + CTRL_OFFSET) &= ~TICK_INT;	// disable interrupt
	}
}
