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

static volatile RCC* const rcc = (RCC*)RCC_ADR;
static volatile GPIOX* const gpiob = (GPIOX*)GPIOB_ADR;

/**
 * led_init:
 * Enables clock to GPIOB peripheral and sets PB5-15 (-11) to output mode
 * parameters: none
 * returns: none
 */
void led_init(void) {
	rcc->AHB1ENR |= (1<<1);	// Bit 1 == GPIOB
	gpiob->MODER |= OUTPUT_MASK;
	gpiob->MODER &= ~(OUTPUT_MASK<<1);
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
