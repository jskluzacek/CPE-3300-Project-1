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
static volatile GPIOX* const gpiob = (GPIOX*) GPIOC_ADR;


/**
 * pin_init:
 * Enables clock to GPIOC peripheral and sets PC11 to TX_DATA (output)
 * and PC12 to RX_DATA (input)
 * parameters: none
 * returns: none
 */
void pin_init(void) {

}
