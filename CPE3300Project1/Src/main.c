/******************************************************************************
 * @assignment	   : CPE3300 Project 1
 * @file           : main.c
 * @author         : Team Edward - Kenny Gifford and Jerico Skluzacek
 * @date		   : 01/23/24
 * @brief          : Monitors the state of the Rx input signal and displays
 * 			it on LEDs.
 *****************************************************************************/

#include "channel_monitor.h"
#include "led.h"
#include "delay.h"
#include "stm32regs.h"
#include "console.h"
#include "uart_driver.h"


int main(void)
{
	/*** INITIALIZE API's ***/
	led_init();
	channel_monitor_init();
	tx_init();
	init_usart2(57600, CPU_FREQ);

	// led test
	for(int i = 20; i <= 50; i += 5) {
		delay_ms(i);
		led_on(IDLE_LED);
		delay_ms(i);
		led_on(BUSY_LOW_LED);
		delay_ms(i);
		led_on(BUSY_HIGH_LED);
		delay_ms(i);
		led_on(COLL_LED);
	}

	// program start
	for (int i = 0; i < 3; i++) {
		delay_ms(100);
		led_on(0);
		delay_ms(100);
		led_on(IDLE_LED | BUSY_LOW_LED | BUSY_HIGH_LED | COLL_LED);
	}

	for(;;) {
		console_scan();
	}

	return 0;
}
