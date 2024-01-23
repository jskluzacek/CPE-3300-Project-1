/******************************************************************************
 * @assignment	   : CPE3300 Project 1
 * @file           : main.c
 * @author         : Team Edward - Kenny Gifford and Jerico Skluzacek
 * @date		   : 01/23/24
 * @brief          : TODO
 *****************************************************************************/
#include "led.h"

int main(void)
{
	/*** INITIALIZE API's ***/
	led_init();

	led_cycle();

	for(;;); // forever loop

	return 0;
}
