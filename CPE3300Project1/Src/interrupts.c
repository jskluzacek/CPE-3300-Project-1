/******************************************************************************
 * @assignment	   : CPE3300 Project 1
 * @file           : interrupts.h
 * @author         : Team Edward - Kenny Gifford and Jerico Skluzacek
 * @date		   : 01/23/24
 * @brief          : Interrupt Configuration API
 *****************************************************************************/
#include "stm32regs.h"

static volatile EXTI* const exti = (EXTI*) 0x40013C00;

void edge_detection_init(void) {
	exti->IMR |= 0b11;
}
