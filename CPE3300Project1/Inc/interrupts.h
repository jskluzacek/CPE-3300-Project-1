/******************************************************************************
 * @assignment	   : CPE3300 Project 1
 * @file           : interrupts.h
 * @author         : Team Edward - Kenny Gifford and Jerico Skluzacek
 * @date		   : 01/28/24
 * @brief          : Interrupt Configuration
 *****************************************************************************/

#ifndef _interrupt_H
#define _interrupt_H

typedef enum {
	 IDLE,
	 BUSY_LOW,
	 BUSY_HIGH,
	 COLLISION
} STATE;


STATE get_state(void);
void edge_detection_init(void);

#endif /* interrupt.h */
