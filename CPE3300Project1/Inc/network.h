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
#define DELAY_TIME 	(0.00111 * CPU_FREQ)// max time between edges is 1.11ms
#define COLL_TIME 	(0.00110 * CPU_FREQ)// collision timeout is 1.10ms
#define IDLE_TIME	(0.00113 * CPU_FREQ)// idle timeout is 1.13ms
#define IDEAL_BIT_PERIOD (0.001 * CPU_FREQ)		// period for each bit is 1ms
#define HALF_BIT_PERIOD	(IDEAL_BIT_PERIOD / 2)	// transmitted half-bit period
#define N_MAX	200	// n_max for random wait time calculation

/* Bit Constants */
#define PC11 11 	// Tx pin
#define PC12 12 	// Rx Pin
#define MSB 0x80	// most sig. bit of a byte

/* Buffer Constraints */
#define MSG_SIZE	(5 + 255 + 1 + 1) // header + msg + tail + null
#define TX_BUFFER_SIZE MSG_SIZE 	// input string length
#define MAX_MESSAGES 10
#define RX_BUFFER_SIZE (MSG_SIZE * MAX_MESSAGES)

/* Transmission */
#define MAX_ATTEMPTS 10
#define CRC_DIVISOR 0x107

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

typedef enum {
	LOCKED,
	UNLOCKED
} BUFFER_STATE;

void tx_string(const char input[]);
void rx_string();

void tx_init(void);
void rx_init(void);
void led_init(void);

#endif /* NETWORK_H_ */
