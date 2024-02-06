/******************************************************************************
 * @assignment	   : CPE3300 Project 1
 * @file           : transmit.c
 * @author         : Team Edward - Kenny Gifford and Jerico Skluzacek
 * @date		   : 02/04/24
 * @brief          : Data Transmission Header
 *****************************************************************************/

#ifndef TRANSMIT_H_
#define TRANSMIT_H_

#define PC11 11 // Tx pin

#define BUFFER_SIZE 100 // user input string length

/* Protocol Timings */
#define CPU_FREQ 16000000UL 				// system clock speed is 16MHz
#define HALF_BIT_PERIOD	0.0005 * CPU_FREQ	// max time between edges is 1.11ms

void tx_init(void);
void tx_string(const unsigned char str[]);

#endif /* TRANSMIT_H_ */
