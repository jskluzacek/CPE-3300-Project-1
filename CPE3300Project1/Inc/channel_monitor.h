/******************************************************************************
 * @assignment	   : CPE3300 Project 1
 * @file           : channel_monitor.h
 * @author         : Team Edward - Kenny Gifford and Jerico Skluzacek
 * @date		   : 01/30/24
 * @brief          : Channel Monitor Header
 *****************************************************************************/

#ifndef _interrupt_H
#define _interrupt_H

#define PC12 12 // Rx Pin

/* Protocol Timings */
#define CPU_FREQ 	16000000UL 			// system clock speed is 16MHz
#define DELAY_TIME 	0.00111 * CPU_FREQ	// max time between edges is 1.11ms
#define COLL_TIME 	0.00110 * CPU_FREQ	// collision timeout is 1.10ms
#define IDLE_TIME	0.00113 * CPU_FREQ 	// idle timeout is 1.13ms

typedef enum {
	 IDLE,
	 BUSY_LOW,
	 BUSY_HIGH,
	 COLLISION
} STATE;

STATE get_state(void);
void edge_detection_init(void);
void channel_monitor_init(void);

#endif /* interrupt.h */
