/******************************************************************************
 * @assignment	   : CPE3300 Project 1
 * @file           : network.c
 * @author         : Team Edward - Kenny Gifford and Jerico Skluzacek
 * @date		   : 02/06/24
 * @brief          : Channel Monitor and Signal Transmitter API
 *****************************************************************************/
#include "network.h"
#include "stm32regs.h"
#include <string.h>

/* Register Accesses */
static volatile TIMX* const tim2 = (TIMX*) TIM2_ADR;
static volatile TIMX* const tim3 = (TIMX*) TIM3_ADR;
static volatile TIMX* const tim4 = (TIMX*) TIM4_ADR;
static volatile TIMX* const tim5 = (TIMX*) TIM5_ADR;
static volatile GPIOX* const gpiob = (GPIOX*)GPIOB_ADR;
static volatile GPIOX* const gpioc = (GPIOX*) GPIOC_ADR;
static volatile SYSCFG* const syscfg = (SYSCFG*) SYSCFG_ADR;
static volatile EXTI* const exti = (EXTI*) EXTI_ADR;
static volatile RCC* const rcc = (RCC*) RCC_ADR;
static volatile NVIC* const nvic = (NVIC*) NVIC_ADR;

// state of the input signal Rx_data (pin PC12)
static STATE state;

// buffer of decoded half-bits to be transmitted on Tx_data (pin PC11)
static unsigned char tx_buffer[TX_BUFFER_SIZE+1];
// buffer of encoded bits received from Rx_data (pin PC12)
static unsigned char rx_buffer[RX_BUFFER_SIZE+1];
static char hb0;

// half-bits transmitted (increments once per TIM3_IRQHandler)
static size_t tx_index;
// half-bits received (increments once per TIM5_IRQHandler)
static size_t rx_index;

/******************** Private Helper Methods ********************/

/**
 * calc_crc:
 * Performs the Cyclic Redundancy Check with given message and divisor
 * parameters:
 * 		msg - message to check
 * 	 length - length of message in bytes
 * 	divisor - the predetermined CRC divisor
 * returns: the remainder of the CRC
 */
char calc_crc(const char* msg, char length, char divisor) {
    char crc = 0;

    // iterate over bits of message
    for (int i = 0; i < (length * 8); i++) {
        if (crc & MSB) {
        	// crc = shifted crc xor'd with predetermined divisor
        	crc = (crc << 1) ^ divisor;
        } else {
        	// shift crc
        	crc <<= 1;
        	// append next bit as LSB of crc
        	crc |= msg[i / 8] | (MSB >> (i % 8));
        }
    }
    return crc;
}

/**
 * verify_message:
 * Verifies the header and tail of a message to ensure conformity
 * to project 1 standard. Prints errors to console.
 * parameters:
 * 		msg - message (including header and tail) to verify
 * returns: none
 */
void verify_message(const char* msg) {
	char preamble = msg[0];
	char source_adr = msg[1];
	char dest_adr = msg[2];
	char length = msg[3];
	char crc_flag = msg[4];
	char crc_trailer = msg[5 + length];

	if (preamble != 0x55) {
		// preamble should be 0x55
		console_print_str("Invalid preamble\n");
	}
	if (length == 0) {
		// length should be > 0
		console_print_str("Invalid length\n");
	}
	if (crc_flag > 1) {
		// crc_flag should be 0 or 1
		console_print_str("Invalid CRC Flag\n");
	}
	if (crc_flag) {
		if (calc_crc(msg[5], length, 0x87) != crc_trailer) {
			// calculated crc should match given crc
			console_print_str("CRC failed\n");
		}
	} else {
		if (crc_trailer != 0xAA) {
			console_print_str("Invalid CRC Trailer since CRC Flag is 0\n");
		}
	}
}

/******************** Public (non-init) Methods ********************/

/**
 * tx_string:
 * Transmits up to 100 bytes over Tx_data (PC11) using Manchester
 * 		encoding.
 * parameters:
 *    input - string of bytes to encode and transmit over Tx_data
 * returns: none
 */
void tx_string(const unsigned char input[]) {
	// TODO verify buffer/message sizes and ending indexes are correct, could cause issues!
	strncpy(tx_buffer, input, TX_BUFFER_SIZE);

	/* Configure */
	tx_index = 0;	// read from beginning of buffer
	tim3->CNT = 0;					// restart timer
	tim3->ARR = HALF_BIT_PERIOD;	// reset timer duration

	// busy wait for IDLE state to begin transmission
	while (state != IDLE) {};

	/* Transmit */
	tim3->CR1 |= (1<<0);			// start timer
}

/**
 * rx_string:
 * Receives, decodes, and displays up to 256 bytes over Rx_data (PC12)
 * parameters: none
 * returns: none
 */
void rx_string() {
	// TODO verify buffer/message sizes and ending indexes are correct, could cause issues!
	memset(rx_buffer, 0, RX_BUFFER_SIZE+1);

	/* Prepend half-bit 1 */
	// (messages should start with Manchester 0)
	hb0 = 1;
	rx_index = 1;

	/* Start Sampling */
	tim5->CR1 |= (1<<0);

	// busy wait until message starts (leaving idle)
	while (state == IDLE) {};

	// busy wait until message is complete (returning to idle)
	while (state != IDLE) {};

	/* Print Message */
	console_print_str(rx_buffer);

	/* Verify Message */
	verify_message(rx_buffer);

}

/******************** Initializers ********************/

/**
 * tx_init:
 * Enables output mode for pin PC11,
 * 		and sets up interrupts for half-bit transmission intervals
 * parameters: none
 * returns: none
 */
void tx_init(void) {
	/* RCC */
	rcc->AHB1ENR |= (1<<2); 	// GPIOC = BIT 2
	rcc->APB1ENR |= (1<<1);		// TIM3 = Bit 1

	/* Tx Pin */
	gpioc->MODER |= (1<<(PC11 * 2));	// set PC11 to output
	gpioc->MODER &= ~(1<<((PC11*2) + 1));

	/* Transmission Timer Duration */
	tim3->ARR = HALF_BIT_PERIOD;

	/* Enable Transmission Interrupts  */
	nvic->ISER0 |= (1<<TIM3n);	// enable TIM3 interrupt in NVIC
	tim3->DIER |= (1<<0);		// enable TIM3 interrupt

	/* Start at logic-1 */
	gpioc->ODR |= (1<<PC11);
}

/**
 * rx_init:
 * Enables rising and falling edge detection for pin PC12,
 * 		and resulting interrupt call on external interrupt line 12
 * parameters: none
 * returns: none
 */
void rx_init(void) {
	/* RCC */
	rcc->AHB1ENR |= (1<<2); 	// GPIOC = BIT 2
	rcc->APB1ENR |= (1<<0);		// TIM2 = Bit 0
	rcc->APB1ENR |= (1<<3);		// TIM5 = Bit 3
	rcc->APB2ENR |= (1<<14);	// SYSCFG = Bit 14

	/* Rx Pin */
	gpioc->MODER &= ~(0b11<<(PC12 * 2)); // set PC12 to input

	/* External Interrupt Config */
	syscfg->EXTICR4 |= 0b0010;	// enable EXTI for PC12
	exti->IMR |= (1<<12);		// enable EXTI line 12
	exti->RTSR |= (1<<12);		// enable rising edge detection for EXTI12
	exti->FTSR |= (1<<12);		// enable falling edge detection for EXTI12

	/* Enable Edge Detection Interrupts */
	nvic->ISER1 |= (1<<(EXTI15_10n - 32));	// enable EXTI interrupt in NVIC

	/* Sampling Timer Config */
	tim5->ARR = HALF_BIT_PERIOD;

	/* Enable Sampling Interrupts */
	nvic->ISER1 |= (1<<(TIM5n - 32));	// enable TIM5 interrupt in NVIC
	tim5->DIER |= (1<<0);		// enable TIM5 interrupt

	/* Enable Timeout Interrupts */
	nvic->ISER0 |= (1<<TIM2n);	// enable TIM2 interrupt in NVIC
	tim2->DIER |= (1<<0);		// enable TIM2 interrupt

	/* Poll State */
	TIM2_IRQHandler();
}

/**
 * led_init:
 * Enables Rx_data state monitoring on LEDs
 * parameters: none
 * returns: none
 */
void led_init(void) {
	/* RCC */
	rcc->AHB1ENR |= (1<<1);	// Bit 1 == GPIOB
	rcc->APB1ENR |= (1<<2); // Bit 2 == TIM4

	/* LED Pins */
	gpiob->MODER |= OUTPUT_MASK;
	gpiob->MODER &= ~(OUTPUT_MASK<<1);

	/* Enable Interrupts */
	nvic->ISER0 |= (1<<TIM4n);	// enable TIM4 interrupt in NVIC
	tim4->DIER |= (1<<0);		// enable TIM4 interrupt

	/* Pre-load Timer Duration */
	tim4->ARR = LED_PERIOD;

	/* Start Timer */
	tim4->CR1 |= (1<<0);
}

/******************** IRQHandlers ********************/

/**
 * EXTI15_10_IRQHandler: EDGE
 * Handles Rx_data (PC12) state change on rising or falling edge,
 * 		and presets timeout timer for IDLE/COLLISION period
 * parameters: none
 * returns: none
 */
void EXTI15_10_IRQHandler(void) {
	// clear EXTI12 pending bit
	exti->PR |= (1<<12);

	// stop and reset timeout timer
//	tim2->CR1 &= ~(1<<0); TODO unnecessary because timeout timer should run constantly and this method won't take too long
	tim2->CNT = 0;	// TODO high priority so timeout does not occur during this method

	/* Update State */
	if (state == COLLISION || state == BUSY_LOW) {
		// previous state was COLLISION or BUSY_LOW
		state = BUSY_HIGH;
		tim2->ARR = IDLE_TIME;
	} else {
		// previous state was IDLE or BUSY_HIGH
		state = BUSY_LOW;
		tim2->ARR = COLL_TIME;
	}

	// TODO low priority so can happen at end of method
	// realign sampling timer to halfway through half-bit period
	tim5->CNT = HALF_BIT_PERIOD / 2;

//	// start timeout timer
//	tim2->CR1 |= (1<<0); TODO unnecessary
}

/**
 * TIM2_IRQHandler: TIMEOUT
 * Handles Rx_data (PC12) timeouts and state change to IDLE/COLLISION
 * parameters: none
 * returns: none
 */
void TIM2_IRQHandler(void) {
	// clear UIF pending bit
	tim2->SR &= ~(1<<0);

//	// stop timeout timer
//	tim2->CR1 &= ~(1<<0); TODO can run constantly

//	// stop sampling timer
//	tim5->CR1 &= ~(1<<0); TODO sampling can run constantly since it has no effects in idle/collision

	// update state
	if (gpioc->IDR & (1<<PC12)) {
		state = IDLE;
	} else {
		state = COLLISION;
	}
}

/**
 * TIM3_IRQHandler: TRANSMIT
 * Encodes tx_buffer into outgoing bits every 500 us over Tx_data (PC11)
 * parameters: none
 * returns: none
 */
void TIM3_IRQHandler(void) {
	// clear UIF pending bit
	tim3->SR &= ~(1<<0);

	/* Back off and retransmit? */
	if (state == COLLISION) {
		gpioc->ODR |= (1<<PC11);	// reset Tx_data to 1
//		tim3->ARR = RANDOM TODO wait random time
		tx_index = 0;				// read from beginning of buffer
		return;
	} else {
		tim3->ARR = HALF_BIT_PERIOD;
	}

	/* End transmission? */
	if (tx_buffer[tx_index / (2*8)] == '\0') {
		tim3->CR1 &= ~(1<<0);		// stop timer
		gpioc->ODR |= (1<<PC11);	// reset Tx_data to 1
		return;
	}

	/* Encode and transmit half-bit */
	char byte = tx_buffer[tx_index / (2*8)];
	char bit = byte & (MSB >> ((tx_index / 2) % 8));

	// write 1 if:
	// 		input bit is 0 and this is first (even) half-bit
	// 		input bit is 1 and this is second (odd) half-bit
	// write 0 if:
	//		input bit is 1 and this is first half-bit
	//		input bit is 0 and this is second half-bit
	if ((bit && (tx_index % 2)) || (!bit && !(tx_index % 2))) {
		gpioc->ODR |= (1<<PC11);	// write 1 to Tx_data
	} else {
		gpioc->ODR &= ~(1<<PC11);	// write 0 to Tx_data
	}
	tx_index++;
}

/**
 * TIM5_IRQHandler: RECEIVE
 * Decodes incoming half-bits from Rx_data (PC12) every 500 us into rx_buffer
 * parameters: none
 * returns: none
 */
void TIM5_IRQHandler(void) {
	// clear UIF pending bit
	tim5->SR &= ~(1<<0);

	// when receiving every other half-bit...
	if (rx_index % 2) {
		// store second half-bit
		char hb1 = gpioc->IDR & (1<<PC12);

		/* Decode half-bits */
		if (hb0 && !hb1) {
			// half-bits '10' become '0'
			rx_buffer[rx_index / (2*8)] &= ~(MSB >> ((rx_index / 2) % 8));
		} else if (!hb0 && hb1) {
			// half bits '01' become '1'
			rx_buffer[rx_index / (2*8)] |= (MSB >> ((rx_index / 2) % 8));
		} else if (hb0 && hb1) {
			// TODO in idle state. do nothing?
		} else if (!hb0 && !hb1) {
			// TODO in collision state. do nothing?
		}
	} else {
		// store first half-bit
		hb0 = gpioc->IDR & (1<<PC12);
	}
	rx_index++;
}

/**
 * TIM4_IRQHandler: LED
 * Polls channel state and displays LED representation
 * parameters: none
 * returns: none
 */
void TIM4_IRQHandler(void) {
	// clear UIF pending bit
	tim4->SR &= ~(1<<0);

	// update LEDs
	int leds = 0;
	switch (state) {
	case IDLE: {
		leds = IDLE_LED;
		break;
		}
	case BUSY_LOW: {
		leds = BUSY_LOW_LED;
		break;
		}
	case BUSY_HIGH: {
		leds = BUSY_HIGH_LED;
		break;
		}
	case COLLISION: {
		leds = COLL_LED;
		break;
		}
	};

	// write LED pins
	int temp = leds & PB5TO10;		// isolate PB5-10
	temp |= ((leds & PB12TO15)<<1);	// prepend PB12-15 and skip PB11
	gpiob->ODR = (temp<<5);
}


















