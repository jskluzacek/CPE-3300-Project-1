/******************************************************************************
 * @assignment	   : CPE3300 Project 1
 * @file           : network.c
 * @author         : Team Edward - Kenny Gifford and Jerico Skluzacek
 * @date		   : 02/06/24
 * @brief          : Channel Monitor and Signal Transmitter API
 *****************************************************************************/
#include "network.h"
#include "stm32regs.h"
#include "console.h"
#include <stdio.h>
#include <stdlib.h>
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

/* Transmit ISR Data */
// un-encoded data to be transmitted on Tx_data (pin PC11)
static char tx_buffer[TX_BUFFER_SIZE];
// half-bits transmitted (increments once per Transmit ISR call)
static size_t tx_index;
// number of attempts at transmitting tx_buffer (0 if no msg pending)
static char tx_pending;
// randomly generated ARR values to use for retransmission delays
static uint32_t tx_rand_arr_vals[MAX_ATTEMPTS];


/* Sampling ISR Data */
// queue of decoded messages received from Rx_data (pin PC12)
static char rx_buffer[RX_BUFFER_SIZE];
// half-bits received (increments once per Sampling ISR call)
static size_t rx_index;
// first half-bit of pair received by Sampling ISR
static char rx_hb0;
// multiple of MSG_SIZE
static uint16_t rx_buffer_offset;

static char error_state;

/******************** Private Helper Methods ********************/

/**
 * calc_crc:
 * Performs the Cyclic Redundancy Check with given message and divisor
 * parameters:
 * 	data - message to check (not including header)
 * 	length - length of data in bytes
 * returns: the remainder of the CRC operation
 */
char calc_crc(const char* data, char length) {
	// make room for CRC at end of msg
    char temp[length + 1];
    strncpy(temp, data, length);
    temp[(size_t) length] = 0x00;

    char crc = 0;
    for (int i = 0; i < length; i++) {
    	// subtract next byte
        crc ^= temp[i];
        for (int j = 0; j < 8; j++) {
        	// check if MSB is 1
            if (crc & MSB) {
            	// shift then subtract
                crc = (crc << 1) ^ CRC_DIVISOR;
            } else {
            	// shift
                crc = (crc << 1);
            }
        }
    }

    return crc;
}

/**
 * verify_message:
 * 	Verifies a message conforms to the Project 1 standard.
 * 	Prints errors if message is invalid, otherwise does nothing.
 * parameters:
 * 	msg - header, data, and tail to verify
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
		printf("Invalid preamble\n");
	}
	if (length == 0) {
		// length should be > 0
		printf("Invalid length\n");
	}
	if (crc_flag > 1) {
		// crc_flag should be 0 or 1
		printf("Invalid CRC Flag\n");
	}
	if (crc_flag) {
		if (calc_crc(msg + 5, length) != 0) {
			// calculated crc should match given crc
			printf("CRC failed\n");
		}
	} else {
		if (crc_trailer != 0xAA) {
			printf("Invalid CRC Trailer since CRC Flag is 0\n");
		}
	}
}

/******************** Public Initializers ********************/

/**
 * tx_init:
 * 	Configures Transmit ISR for message transmission over Tx_data (PC11).
 * parameters: none
 * returns: none
 */
void tx_init(void) {
	/* GPIOC Config. (PC11) */
	rcc->AHB1ENR |= (1<<2); 			// GPIOC = BIT 2
	gpioc->MODER |= (1<<(PC11 * 2));	// set PC11 to output mode
	gpioc->MODER &= ~(1<<((PC11*2) + 1));

	/* Transmit ISR Config. (TIM3) */
	rcc->APB1ENR |= (1<<1);				// TIM3 = Bit 1
	tim3->ARR = HALF_BIT_PERIOD;		// set TIM3 period
	tim3->DIER |= (1<<0);				// enable TIM3 interrupt
	nvic->ISER0 |= (1<<TIM3n);			// enable TIM3 interrupt in NVIC

	/* Transmit ISR - reset (no msg) */
	memset(tx_buffer, 0, TX_BUFFER_SIZE);
	tx_index = 0;
	tx_pending = 0;
	gpioc->ODR |= (1<<PC12);

	/* Start ISRs */
	tim3->CR1 |= (1<<0);
}

/**
 * rx_init:
 * 	Enables edge and timeout detection of Rx_data (PC12), and begins
 * 		bit sampling.
 * parameters: none
 * returns: none
 */
void rx_init(void) {
	/* GPIOC Config. (PC12) */
	rcc->AHB1ENR |= (1<<2); 				// GPIOC = BIT 2
	gpioc->MODER &= ~(0b11<<(PC12 * 2));	// set PC12 to input mode

	/* Edge ISR Config. (EXTI) */
	rcc->APB2ENR |= (1<<14);	// SYSCFG = Bit 14
	syscfg->EXTICR4 |= 0b0010;	// enable EXTI for PC12
	exti->IMR |= (1<<12);		// enable EXTI line 12
	exti->RTSR |= (1<<12);		// enable rising edge detection for EXTI12
	exti->FTSR |= (1<<12);		// enable falling edge detection for EXTI12

	/* Timeout ISR Config. (TIM2) */
	rcc->APB1ENR |= (1<<0);				// TIM2 = Bit 0
	tim2->ARR = COLL_TIME;				// set TIM2 period
	tim2->DIER |= (1<<0);				// enable TIM2 interrupt
	nvic->ISER0 |= (1<<TIM2n);			// enable TIM2 interrupt in NVIC

	/* Sampling ISR Config. (TIM5) */
	rcc->APB1ENR |= (1<<3);				// TIM5 = Bit 3
	tim5->ARR = HALF_BIT_PERIOD;		// set TIM5 period
	tim5->DIER |= (1<<0);				// enable TIM5 interrupt
	nvic->ISER1 |= (1<<(TIM5n - 32));	// enable TIM5 interrupt in NVIC

	/* Sampling ISR - reset */
	memset(rx_buffer, '\0', RX_BUFFER_SIZE);
	rx_buffer_offset = 0;
	rx_hb0 = 1;
	rx_index = 1;

	/* Update State */
	if (gpioc->IDR & (1<<PC12)) {
		state = IDLE;
	} else {
		state = COLLISION;
	}

	/* Start ISRs */
	nvic->ISER1 |= (1<<(EXTI15_10n - 32));	// start edge detection
//	tim2->CR1 |= (1<<0);					// start timeout timer
	tim5->CR1 |= (1<<0);					// start sampling timer

	error_state = 0;
}

/**
 * led_init:
 * 	Enables Rx_data state monitoring on LEDs
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

/******************** Public Methods ********************/

/**
 * tx_message:
 * 	Transmits a message over Tx_data with appropriate header and error checking
 * 		information.
 * 	Also generates random retransmit delays,
 * 		 in case of COLLISION state during transmission.
 * 	Blocks during ongoing transmission before beginning new transmission.
 * parameters:
 * 	data - string of up to 255 characters to encode and transmit
 * returns: none
 */
void tx_message(const char data[]) {
	// wait for transmission to finish
	while (!(tx_pending == 0 || tx_pending > MAX_ATTEMPTS)) {};

	// verify data
	if (strlen(data) > 0xFF) {
		console_print_str("User string is too long.\n");
		return;
	}

	/* Append Header and Data */
	size_t length = strlen(data);			// length of msg's data section
	tx_buffer[0] = 0x55;					// 0: preamble
	tx_buffer[1] = 0x01;					// 1: source_adr
	tx_buffer[2] = 0x02;					// 2: dest_adr
	tx_buffer[3] = (char)length;			// 3: length
	tx_buffer[4] = 0x01;					// 4: crc_flag
	strncpy((tx_buffer + 5), data, length);	// 5+: message

	/* Append CRC Tail */
	tx_buffer[5 + length] = tx_buffer[4] ? calc_crc(data, (char)length) : 0xAA;
	tx_buffer[TX_BUFFER_SIZE-1] = '\0';		// LSB: null terminator

	/* Generate Random Delays */
	srand(tx_buffer[5 + length]);	// TODO ideally based on time, but stm32 has no time
	for (int i = 0; i < MAX_ATTEMPTS; i++) {
		int n = rand() % N_MAX + 1; // n is between 1 and N_MAX
		tx_rand_arr_vals[i] = ((double) n / N_MAX) * CPU_FREQ;
	}

	/* Transmit ISR - indicate pending message */
	tx_index = 0;
	tx_pending = 1;
}

/**
 * rx_messages:
 *	Prints any queued messages read from Rx_data, and flushes rx_buffer
 *		to prepare to receive more.
 * parameters: none
 * returns: none
 */
void rx_messages(void) {
	/* Print Message Queue */
	printf("error?:%d\n", (int)error_state);
	error_state = 0;
	for (int i = 0; i < MAX_MESSAGES; i++) {
		// get next msg from rx_buffer using offset
		char* msg_ptr = (char*)(rx_buffer + (i * MSG_SIZE));
		if (msg_ptr[0] != '\0') {
			printf("Message %d: %s\n", i, msg_ptr);
			verify_message(msg_ptr);
		}
	}

	/* Sampling ISR - reset */
	memset(rx_buffer, '\0', RX_BUFFER_SIZE);
	rx_buffer_offset = 0;
	rx_hb0 = 1;
	rx_index = 1;
}

/******************** IRQHandlers ********************/

/**
 * EXTI15_10_IRQHandler: EDGE
 * 	Handles Rx_data (PC12) state change on rising or falling edge,
 * 		and presets timeout timer for IDLE/COLLISION period.
 * race conditions: (by priority)
 * 	TIM2 - IRQ (due to timeout) may occur, but is invalid,
 * 		thus ignore by pausing TIM2 IRQs.
 * 	self - IRQ (due to edge) may occur, but is invalid,
 * 		thus ignore by pausing self IRQs.
 * 	others - other ISRs do not affect state, and can be ignored.
 */
void EXTI15_10_IRQHandler(void) {
	// clear EXTI12 pending bit
	exti->PR |= (1<<12);

	/* Timeout ISR - pause */
	// (prevents race condition)
	tim2->DIER &= ~(1<<0);

	/* Edge ISR - pause */
	// (debounce)
	nvic->ISER1 &= ~(1<<(EXTI15_10n - 32));	// stop edge detection

	/* Update State */
    switch (state) {
        case COLLISION: {
        	/* Transmit ISR - retransmit? */
			if (tx_pending && tx_pending < MAX_ATTEMPTS) {
				/* Transmit ISR - prep. for retransmission */
				tim3->ARR = tx_rand_arr_vals[(size_t)tx_pending];
				tx_pending++;
				tx_index = 0;
			}
			state = BUSY_HIGH;
			tim2->ARR = IDLE_TIME;
			break;
        }
        case BUSY_LOW: {
        	state = BUSY_HIGH;
        	tim2->ARR = IDLE_TIME;
        	break;
        }
        case BUSY_HIGH: {
			state = BUSY_LOW;
			tim2->ARR = COLL_TIME;
			break;
        }
        case IDLE: {
        	state = BUSY_LOW;
        	tim2->ARR = COLL_TIME;
        	break;
        }
        default: {
        	break;
        }
    }

    /* Sampling ISR - realign count */
	tim5->CNT = HALF_BIT_PERIOD / 2;

	/* Timeout ISR - reset and resume/start */
	tim2->CNT = 0;
	tim2->DIER |= (1<<0);
	tim2->CR1 |= (1<<0);

	/* Edge ISR - resume */
	nvic->ISER1 |= (1<<(EXTI15_10n - 32));	// start edge detection
}

/**
 * TIM2_IRQHandler: TIMEOUT
 * 	Handles Rx_data (PC12) timeouts and state change to IDLE/COLLISION.
 * 	Disables itself to prevent redundant read of IDLE/COLLISION state.
 * race conditions: (by priority)
 *  EXTI15_10 - IRQ (due to edge) may occur, but is invalid,
 * 		thus ignore by pausing EXTI15_10 IRQs.
 * 	self - IRQ will not occur if runtime is less than COLL_TIME/IDLE_TIME.
 * 	others - other ISRs do not affect state, and can be ignored.
 */
void TIM2_IRQHandler(void) {
	// clear UIF pending bit
	tim2->SR &= ~(1<<0);

	/* Edge ISR - pause */
	// (prevents race condition)
	nvic->ISER1 &= ~(1<<(EXTI15_10n - 32));	// stop edge detection

	/* Update State */
	if (gpioc->IDR & (1<<PC12)) {
		state = IDLE;
		/* Sampling ISR - message complete */
		rx_buffer_offset += MSG_SIZE;
	} else {
		state = COLLISION;
	}

	/* Sampling ISR - prep. for new message */
	rx_hb0 = 1;
	rx_index = 1;

	/* Timeout ISR - stop */
	// (prevents redundant calls)
	tim2->CR1 &= ~(1<<0);

	/* Edge ISR - resume */
	nvic->ISER1 |= (1<<(EXTI15_10n - 32));	// start edge detection
}

/**
 * TIM3_IRQHandler: TRANSMIT
 * 	Encodes tx_buffer and writes half-bits every HALF_BIT_PERIOD,
 * 	 	over Tx_data (PC11).
 *	Does nothing in COLLISION state or if there is nothing to transmit.
 * race conditions:
 * 	self - IRQ will not occur if runtime is less than HALF_BIT_PERIOD.
 * 	others - this ISR is simple and should not be affected by overlapping ISRs.
 */
void TIM3_IRQHandler(void) {
	// clear UIF pending bit
	tim3->SR &= ~(1<<0);

	// reset after random delay
	if (tim3->ARR != HALF_BIT_PERIOD) {
		tim3->ARR = HALF_BIT_PERIOD;
	}

	// do nothing?
	if (state == COLLISION || tx_pending == 0) {
		gpioc->ODR |= (1<<PC11);	// idle at logic-1
		return;
	}

	// end of message?
	if (tx_buffer[tx_index / (2*8)] == '\0') {
		gpioc->ODR |= (1<<PC11);	// idle at logic-1
		tx_index = 0;
		tx_pending = 0;
		return;
	}

	/* Encode and Transmit Half-bit */
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
 * TIM5_IRQHandler: SAMPLING
 * 	Decodes half-bit pairs from Rx_data (PC12) into bits and writes rx_buffer,
 * 		every (2 * HALF_BIT_PERIOD).
 * 	Does nothing in IDLE/COLLISION states.
 * race conditions:
 * 	self - IRQ will not occur if runtime is less than HALF_BIT_PERIOD.
 * 	others - this ISR is simple and should not be affected by overlapping ISRs.
 */
void TIM5_IRQHandler(void) {
	// clear UIF pending bit
	tim5->SR &= ~(1<<0);

	// ignore idle/collision states
	if (state == COLLISION || state == IDLE) {
		return;
	}

	// ignore data if rx_buffer is full
	if (rx_buffer_offset >= (MAX_MESSAGES * MSG_SIZE)) {
		return;
	}

	// every other half-bit...
	if (rx_index % 2 == 1) {
		// store second half-bit
		char rx_hb1 = (gpioc->IDR & (1<<PC12)) ? 1 : 0;

		/* Decode Half-bit Pair */
		if (rx_hb0 && !rx_hb1) {
			// half-bits '10' become '0'
			rx_buffer[rx_buffer_offset + rx_index / (2*8)] &= ~(MSB >> ((rx_index / 2) % 8));
//			rx_buffer[rx_index / (2*8)] &= ~(MSB >> ((rx_index / 2) % 8));
		} else if (!rx_hb0 && rx_hb1) {
			// half bits '01' become '1'
			rx_buffer[rx_buffer_offset + rx_index / (2*8)] |= (MSB >> ((rx_index / 2) % 8));
//			rx_buffer[rx_index / (2*8)] |= (MSB >> ((rx_index / 2) % 8));
		} else if (rx_hb0 && rx_hb1) {
			error_state = 1;
			return;
			// TODO in idle state. do nothing?
		} else if (!rx_hb0 && !rx_hb1) {
			error_state = 2;
			return;
			// TODO in collision state. do nothing?
		}
	} else {
		// store first half-bit
		rx_hb0 = (gpioc->IDR & (1<<PC12)) ? 1 : 0;
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


















