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
static volatile GPIOX* const gpiob = (GPIOX*)GPIOB_ADR;
static volatile GPIOX* const gpioc = (GPIOX*) GPIOC_ADR;
static volatile SYSCFG* const syscfg = (SYSCFG*) SYSCFG_ADR;
static volatile EXTI* const exti = (EXTI*) EXTI_ADR;
static volatile RCC* const rcc = (RCC*) RCC_ADR;
static volatile NVIC* const nvic = (NVIC*) NVIC_ADR;

// state of the input signal Rx_data (pin PC12)
static STATE state;

static char rx_buffer_lock;

// buffer of decoded bytes to be encoded and sent on Tx_data (pin PC11)
static unsigned char tx_buffer[TX_BUFFER_SIZE+1];
// buffer of encoded half-bytes read from Rx_data (pin PC12)
static unsigned char rx_buffer[RX_BUFFER_SIZE+1];

// index of next half-bit to be read from decoded buffer
static int tx_index = 0;
// index of next half-bit to be written to encoded buffer
static int rx_index = 0;

/******************** Public (non-init) Methods ********************/

/**
 * tx_string:
 * Transmits up to 100 bytes over Tx_data (PC11) using Manchester
 * 		encoding.
 * parameters: a string of bytes to encode and transmit over Tx_data
 * returns: none
 */
void tx_string(const unsigned char input[]) {
	strncpy(tx_buffer, input, TX_BUFFER_SIZE);

	// busy wait for IDLE state
	while (state != IDLE) {};

	tx_index = 0;

	// start timer
	tim3->CNT = 0;			// clear TIM3
	tim3->CR1 |= (1<<0);	// enable TIM3
}

/**
 * rx_string:
 * Receives up to 100 bytes over Rx_data (PC12) using Manchester
 * 		decoding and stores them in provided string location.
 * parameters: a string destination to be written with decoded
 * received bytes
 * returns: none
 */
void rx_string(unsigned char output[]) {
	memset(rx_buffer, 0, RX_BUFFER_SIZE+1);
	rx_index = 0;
	rx_buffer_lock = 0;

	// busy wait until rx_buffer NOT being written
	while (rx_buffer_lock == 0) {};

	// account for first half bit being 1
	if (rx_index % 2) {
		// shift buffer by 1 bit and prepend '1'
		for (int i = 0; i < RX_BUFFER_SIZE; i++) {
			char prev_lsb, curr_lsb;
			// store least significant bit
			curr_lsb = rx_buffer[i] & 1;
			// shift byte
			rx_buffer[i] = rx_buffer[i] >> 1;
			// first byte is prepended by '1' always
			if (i == 0) {
				prev_lsb = 1;
			}
			// prepend previous lsb as new msb
			if (prev_lsb) {
				rx_buffer[i] |= MSB;
			}
			prev_lsb = curr_lsb;
		}
	}

	char invalid = 0;
	int i = 0;
	// iterate over half bits in rx_buffer
	while (i < (RX_BUFFER_SIZE * 2 * 8) && !invalid) {
		// write decoded bits to temp
		char temp;
		for (int j = 0; j < 8; j++) {
			// temp decoded byte
			temp = 0;
			// first and second half bit of each decoded bit
			char hb0, hb1;

			// fetch half bits
			hb0 = rx_buffer[i / 8] & (MSB >> (i % 8));
			hb1 = rx_buffer[i / 8] & (MSB >> (++i % 8));

			// decode half bits
			if (hb0 && !hb1) {
				temp &= !(MSB >> j);
			} else if (!hb0 && hb1) {
				temp |= (MSB >> j);
			} else {
				invalid = 1;
				break;
			}
			i++;
		}
		// print decoded byte to console
		console_print_char(temp);
	}

	if (invalid) {
		printf("Invalid data\n");
	}
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

	/* Enable Interrupts */
	nvic->ISER0 |= (1<<TIM3n);	// enable TIM3 interrupt in NVIC
	tim3->DIER |= (1<<0);		// enable TIM3 interrupt

	/* Pre-load Timer Duration */
	tim3->ARR = HALF_BIT_PERIOD;

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
	rcc->APB2ENR |= (1<<14);	// SYSCFG = Bit 14

	/* Rx Pin */
	gpioc->MODER &= ~(0b11<<(PC12 * 2)); // set PC12 to input

	/* External Interrupt Config */
	syscfg->EXTICR4 |= 0b0010;	// enable EXTI for PC12
	exti->IMR |= (1<<12);		// enable EXTI line 12
	exti->RTSR |= (1<<12);		// enable rising edge detection for EXTI12
	exti->FTSR |= (1<<12);		// enable falling edge detection for EXTI12

	/* Enable Interrupts */
	nvic->ISER1 |= (1<<(EXTI15_10n - 32));	// enable EXTI interrupt in NVIC
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
 * TIM3_IRQHandler:
 * Transmits encoded buffer as individual half-bits over Tx_data (PC11)
 * parameters: none
 * returns: none
 */
void TIM3_IRQHandler(void) {
	// clear UIF pending bit
	tim3->SR &= ~(1<<0);

	// stop transmission on COLLISION state or end of encoded half-bits
	if ((state == COLLISION) ||
		(tx_index >= TX_BUFFER_SIZE * 2 * 8) ||
		(tx_buffer[tx_index / (2 * 8)] == '\0'))
	{
		tim3->CR1 &= ~(1<<0); 		// stop timer
		gpioc->ODR |= (1<<PC11);	// reset Tx_data to 1
		return;
	}

	/* Encode next half-bit */
	// calculate index: 2 half-bits per bit, 8 bits per byte
	char byte = tx_buffer[tx_index / (2 * 8)];
	// isolate bit: 2 half-bits per bit, bits 0-7 of byte
	char bit = byte & (MSB >> (tx_index / 2) % 8);

	/* Write to Tx_data */
	// write 1 if:
	// 		input bit is 1 and this is first half-bit
	// 		input bit is 0 and this is second half-bit
	// write 0 if:
	//		input bit is 0 and this is first half-bit
	//		input bit is 1 and this is second half-bit
	if ((bit && tx_index % 2) || (!bit && !(tx_index % 2))) {
		gpioc->ODR |= (1<<PC11);	// write 1 to Tx_data
	} else {
		gpioc->ODR &= !(1<<PC11);	// write 0 to Tx_data
	}

	tx_index++;
}

/**
 * TIM4_IRQHandler
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

/**
 * EXTI15_10_IRQHandler:
 * Handles Rx_data (PC12) state change on rising or falling edge,
 * 		and presets timers for state timeouts
 * parameters: none
 * returns: none
 */
void EXTI15_10_IRQHandler(void) {
	// clear EXTI12 pending bit
	exti->PR |= (1<<12);

	// pause and reset timeout timer
	tim2->CR1 &= ~(1<<0);
	tim2->CNT = 0;

	// update state
	switch (state) {
	case IDLE: {
		state = BUSY_LOW;
		tim2->ARR = COLL_TIME;
		if (!rx_buffer_lock) {
			rx_buffer[rx_index / 8] &= !(MSB >> (rx_index % 8));
			rx_index++;
		}
		break;
		}
	case BUSY_LOW: {
		state = BUSY_HIGH;
		tim2->ARR = IDLE_TIME;
		if (!rx_buffer_lock) {
			rx_buffer[rx_index / 8] |= (MSB >> (rx_index % 8));
			rx_index++;
		}
		break;
		}
	case BUSY_HIGH: {
		state = BUSY_LOW;
		tim2->ARR = COLL_TIME;
		if (!rx_buffer_lock) {
			rx_buffer[rx_index / 8] &= !(MSB >> (rx_index % 8));
			rx_index++;
		}
		break;
		}
	case COLLISION: {
		//TODO wait random amount of time before switching?
		state = BUSY_HIGH;
		tim2->ARR = IDLE_TIME;
		if (!rx_buffer_lock) {
			rx_buffer[rx_index / 8] |= (MSB >> (rx_index % 8));
			rx_index++;
		}
		break;
		}
	default: break;
	};

	// resume timeout timer
	tim2->CR1 |= (1<<0);
}

/**
 * TIM2_IRQHandler:
 * Handles Rx_data (PC12) timeouts and state change to IDLE/COLLISION
 * parameters: none
 * returns: none
 */
void TIM2_IRQHandler(void) {
	// clear UIF pending bit
	tim2->SR &= ~(1<<0);

	// stop timer (resets implicitly)
	tim2->CR1 &= ~(1<<0);

	// update state
	if (gpioc->IDR & (1<<PC12)) {
		state = IDLE;
		rx_buffer_lock = 1;
	} else {
		state = COLLISION;
	}
}






















