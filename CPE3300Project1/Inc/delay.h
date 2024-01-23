/******************************************************************************
 * @assignment	   : CPE3300 Project 1
 * @file           : delay.h
 * @author         : Team Edward - Kenny Gifford and Jerico Skluzacek
 * @date		   : 01/23/24
 * @brief          : Delay API Header File
 *****************************************************************************/

// Compile Guard
#ifndef _DELAY_H
#define _DELAY_H

// Systick Base Address
#define STK_BASE 0xE000E010

// Systick Register Offsets
#define CTRL_OFFSET 0
#define LOAD_OFFSET 1
#define VAL_OFFSET 2

// Systick CTRL Register Bit Offsets
#define COUNT_FLAG (1<<16)
#define CLK_SRC (1<<2)
#define TICK_INT (1<<1)
#define STK_EN (1<<0)

// Number of clock cycles for 1ms = 16MHz * 10^-3 = 16000
#define MS_CYCLES 16000

// Number of clock cycles for 1us = 16MHz * 10^-6 = 16
#define US_CYCLES 16

// Delay Functions
void delay_ms(int n);
void delay_us(int n);

#endif /* delay.h */
