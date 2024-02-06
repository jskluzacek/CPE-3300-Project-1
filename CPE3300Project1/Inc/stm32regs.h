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

// Register Adresses
#define TIM2_ADR	0x40000000
#define TIM3_ADR	0x40000400
#define TIM4_ADR	0x40000800
#define SYSCFG_ADR  0x40013800
#define EXTI_ADR 	0x40013C00
#define GPIOA_ADR 	0x40020000
#define GPIOB_ADR 	0x40020400
#define GPIOC_ADR 	0x40020800
#define RCC_ADR 	0x40023800
#define NVIC_ADR	0xE000E100

// IRQ Positions Numbers
#define EXTI15_10n 	40
#define TIM2n		28
#define TIM3n		29
#define TIM4n		30

/***** STM32 REGISTER ACCESS *****/
//SYSCFG
typedef struct {
	uint32_t MEMRMP;
	uint32_t PMC;
	uint32_t EXTICR1;
	uint32_t EXTICR2;
	uint32_t EXTICR3;
	uint32_t EXTICR4;
	uint32_t CMPCR;
} SYSCFG;

//EXTI
typedef struct {
	uint32_t IMR;
	uint32_t EMR;
	uint32_t RTSR;
	uint32_t FTSR;
	uint32_t SWIER;
	uint32_t PR;
} EXTI;

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

//NVIC
typedef struct {
	uint32_t ISER0;
	uint32_t ISER1;
	uint32_t ISER2;
	uint32_t ICER0;
	uint32_t ICER1;
	uint32_t ICER2;
	uint32_t ISPR0;
	uint32_t ISPR1;
	uint32_t ISPR2;
	uint32_t ICPR0;
	uint32_t ICPR1;
	uint32_t ICPR2;
	uint32_t IABR0;
	uint32_t IABR1;
	uint32_t IABR2;
	uint32_t IPR0;
	uint32_t IPR1;
	uint32_t IPR2;
	uint32_t IPR3;
	uint32_t IPR4;
	uint32_t IPR5;
	uint32_t IPR6;
	uint32_t IPR7;
	uint32_t IPR8;
	uint32_t IPR9;
	uint32_t IPR10;
	uint32_t IPR11;
	uint32_t IPR12;
	uint32_t IPR13;
	uint32_t IPR14;
	uint32_t IPR15;
	uint32_t IPR16;
	uint32_t IPR17;
	uint32_t IPR18;
	uint32_t IPR19;
	uint32_t IPR20;
} NVIC;

#endif // "stm32regs.h"
