/******************************************************************************
 * @assignment : CPE3300 Project 1
 * @file       : main.c
 * @author     : Team Edward - Kenny Gifford and Jerico Skluzacek
 * @date	   : 02/06/24
 * @brief      : Monitors Rx_data pin and reflects state on LEDs, and transmits
 * 					user input from USART terminal onto Tx_data pin.
 *****************************************************************************/
#include "uart_driver.h"
#include "stm32regs.h"
#include "network.h"
#include "console.h"

int main(void)
{
	/*** INITIALIZE API's ***/
	rx_init();
	tx_init();
	led_init();
	init_usart2(57600, CPU_FREQ);

	rx_string();

	for(;;) {
		console_scan();
	}

	return 0;
}
