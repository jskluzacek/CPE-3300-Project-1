/******************************************************************************
 * @assignment	   : CPE3300 Project 1
 * @file           : pins.c
 * @author         : Team Edward - Kenny Gifford and Jerico Skluzacek
 * @date		   : 01/23/24
 * @brief          : Pin Configuration
 *****************************************************************************/

#include "pins.h"
#include "stm32regs.h"

static volatile RCC* const rcc = (RCC*) RCC_ADR;
static volatile GPIOX* const gpioc = (GPIOX*) GPIOC_ADR;


/**
 * pin_init:
 * Enables clock to GPIOC peripheral and sets PC11 to TX_DATA (output)
 * and PC12 to RX_DATA (input)
 * parameters: none
 * returns: none
 */
void pin_init(void) {

	// Enable clocks for PC11 and PC12
	rcc->AHB1ENR |= (1<<2); // GPIOC = BIT 2

	// Set PC11 to output and PC12 to input (rmw)
	gpioc->MODER |= PINS_MASK;
	gpioc->MODER &= ~(PINS_MASK<<1);
}
