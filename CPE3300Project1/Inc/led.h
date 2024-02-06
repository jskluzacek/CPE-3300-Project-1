/******************************************************************************
 * @assignment	   : CPE3300 Project 1
 * @file           : led.h
 * @author         : Team Edward - Kenny Gifford and Jerico Skluzacek
 * @date		   : 01/23/24
 * @brief          : LED Header
 *****************************************************************************/

#ifndef _LED_H
#define _LED_H

#define OUTPUT_MASK 0x55155400
#define PB5TO10 0x3F
#define PB12TO15 0x3C0

/* Timings */
#define CPU_FREQ 16000000UL
#define LED_PERIOD 0.0002 * CPU_FREQ

/* LEDs */
#define IDLE_LED 		0b00000001
#define BUSY_LOW_LED  	0b00000010
#define BUSY_HIGH_LED 	0b00000100
#define COLL_LED 		0b00001000

void led_init(void);
void led_on(int number);
void led_cycle(void);

#endif /* led.h */
