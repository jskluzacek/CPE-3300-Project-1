/******************************************************************************
 * @assignment	   : CPE3300 Project 1
 * @file           : stm32regs.h
 * @author         : Team Edward - Kenny Gifford and Jerico Skluzacek
 * @date		   : 01/23/24
 * @brief          : STM32 Register Offset Structures
 *****************************************************************************/
#include <stdint.h>

#ifndef _STM32REGS_H
#define _STM32REGS_H

// CPU clock frequency
#define F_CPU 16000000UL

// Register Adresses
#define RCC_ADR 	0x4002 3800
#define GPIOB_ADR 	0x4002 0000
#define GPIOC_ADR 	0x4002 0800


/***** STM32 REGISTER ACCESS *****/
//RCC
typedef struct {
	uint32_t CR;
	uint32_t PLLCFGR;
	uint32_t CFGR;
	uint32_t CIR;
	uint32_t AHB1RSTR;
	uint32_t AHB2RSTR;
	uint32_t reserved0;
	uint32_t reserved1;
	uint32_t APB1RSTR;
	uint32_t APB2RSTR;
	uint32_t reserved2;
	uint32_t reserved3;
	uint32_t AHB1ENR;
	uint32_t AHB2ENR;
	uint32_t reserved4;
	uint32_t reserved5;
	uint32_t APB1ENR;
	uint32_t APB2ENR;
	uint32_t reserved6;
	uint32_t reserved7;
	uint32_t AHB1LPENR;
	uint32_t AHB2LPENR;
	uint32_t reserved8;
	uint32_t reserved9;
	uint32_t APB1LPENR;
	uint32_t APB2LPENR;
	uint32_t reserved10;
	uint32_t reserved11;
	uint32_t BDCR;
	uint32_t CSR;
	uint32_t reserved12;
	uint32_t reserved13;
	uint32_t SSCGR;
	uint32_t PLLI2SCFGR;
	uint32_t reserved14;
	uint32_t DCKCFGR;
} RCC;

// ADC
typedef struct {
	uint32_t SR;
	uint32_t CR1;
	uint32_t CR2;
	uint32_t SMPR1;
	uint32_t SMPR2;
	uint32_t JOFR1;
	uint32_t JOFR2;
	uint32_t JOFR3;
	uint32_t JOFR4;
	uint32_t HTR;
	uint32_t LTR;
	uint32_t SQR1;
	uint32_t SQR2;
	uint32_t SQR3;
	uint32_t JSQR;
	uint32_t JDR1;
	uint32_t JDR2;
	uint32_t JDR3;
	uint32_t JDR4;
	uint32_t DR;
} ADC;

//GPIOX
typedef struct {
	uint32_t MODER;
	uint32_t OTYPER;
	uint32_t OSPEEDR;
	uint32_t PUPDR;
	uint32_t IDR;
	uint32_t ODR;
	uint32_t BSRR;
	uint32_t LCKR;
	uint32_t AFRL;
	uint32_t AFRH;
} GPIOX;

//TIMX
typedef struct {
	uint32_t CR1;
	uint32_t CR2;
	uint32_t SMCR;
	uint32_t DIER;
	uint32_t SR;
	uint32_t EGR;
	uint32_t CCMR1;
	uint32_t CCMR2;
	uint32_t CCER;
	uint32_t CNT;
	uint32_t PSC;
	uint32_t ARR;
	uint32_t reserved0;
	uint32_t CCR1;
	uint32_t CCR2;
	uint32_t CCR3;
	uint32_t CCR4;
	uint32_t reserved1;
	uint32_t DCR;
	uint32_t DMAR;
	uint32_t TIM2_OR;
	uint32_t TIM5_OR;
} TIMX;

//EXTI
typedef struct {
	uint32_t IMR;
	uint32_t EMR;
	uint32_t RTSR;
	uint32_t FTSR;
	uint32_t SWIER;
	uint32_t PR;
} EXTI;

#endif // "stm32regs.h"
