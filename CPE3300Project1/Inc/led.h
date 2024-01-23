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

void led_init(void);
void led_cycle(void);

#endif /* led.h */
