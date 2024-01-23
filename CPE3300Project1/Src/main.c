/******************************************************************************
 * @assignment	   : CPE3300 Project 1
 * @file           : main.c
 * @author         : Team Edward - Kenny Gifford and Jerico Skluzacek
 * @date		   : 01/23/24
 * @brief          : TODO
 *****************************************************************************/
#include "led.h"
#include "delay.h"

int main(void)
{
	/*** INITIALIZE API's ***/
	led_init();

	for(;;) {
		led_on(0b0000000001);
		delay_ms(250);
		led_on(0b0000000010);
		delay_ms(250);
		led_on(0b0000000100);
		delay_ms(250);
	}

	return 0;
}
