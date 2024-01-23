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
		led_on(IDLE_LED);
		delay_ms(250);
		led_on(BUSY_LED);
		delay_ms(250);
		led_on(COLL_LED);
		delay_ms(250);
	}

	return 0;
}
