/******************************************************************************
 * @assignment	   : CPE3300 Project 1
 * @file           : network.h
 * @author         : Team Edward - Kenny Gifford and Jerico Skluzacek
 * @date		   : 02/06/24
 * @brief          : Channel Monitor and Signal Transmitter Header
 *****************************************************************************/
#ifndef NETWORK_H_
#define NETWORK_H_

/* Protocol Timings */
#define CPU_FREQ 	16000000UL 			// system clock speed is 16MHz
#define DELAY_TIME 	0.00111 * CPU_FREQ	// max time between edges is 1.11ms
#define COLL_TIME 	0.00110 * CPU_FREQ	// collision timeout is 1.10ms
#define IDLE_TIME	0.00113 * CPU_FREQ 	// idle timeout is 1.13ms
#define IDEAL_BIT_PERIOD 0.001 * CPU_FREQ		// period for each bit is 1ms
#define HALF_BIT_PERIOD	IDEAL_BIT_PERIOD / 2	// transmitted half-bit period

/* Bit Constants */
#define PC11 11 	// Tx pin
#define PC12 12 	// Rx Pin
#define MSB 0x80	// most sig. bit of a byte

/* Buffer Constraints */
#define BUFFER_SIZE 100 // user input string length
#define MAX_HALF_BITS BUFFER_SIZE * 2 * 8

/* LED Config */
#define OUTPUT_MASK 0x55155400
#define PB5TO10 0x3F
#define PB12TO15 0x3C0
#define LED_PERIOD 0.0001 * CPU_FREQ	// polling rate for LEDs is 0.1ms

/* LED Constants */
#define IDLE_LED 		0b00000001
#define BUSY_LOW_LED  	0b00000010
#define BUSY_HIGH_LED 	0b00000100
#define COLL_LED 		0b00001000

typedef enum {
	 IDLE,
	 BUSY_LOW,
	 BUSY_HIGH,
	 COLLISION
} STATE;

void tx_string(const unsigned char str[]);
void tx_init(void);
void rx_init(void);
void led_init(void);
void TIM2_IRQHandler(void);
void TIM3_IRQHandler(void);
void TIM4_IRQHandler(void);
void EXTI15_10_IRQHandler(void);

#endif /* NETWORK_H_ */
