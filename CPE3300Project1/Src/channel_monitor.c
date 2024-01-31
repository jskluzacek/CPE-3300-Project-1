/******************************************************************************
 * @assignment	   : CPE3300 Project 1
 * @file           : channel_monitor.c
 * @author         : Team Edward - Kenny Gifford and Jerico Skluzacek
 * @date		   : 01/30/24
 * @brief          : Channel Monitor API
 *****************************************************************************/
#include "stm32regs.h"
#include "channel_monitor.h"

static volatile GPIOX* const gpioc = (GPIOX*) GPIOC_ADR;
static volatile TIMX* const tim2 = (TIMX*) TIM2_ADR;
static volatile SYSCFG* const syscfg = (SYSCFG*) SYSCFG_ADR;
static volatile EXTI* const exti = (EXTI*) EXTI_ADR;
static volatile RCC* const rcc = (RCC*) RCC_ADR;
static volatile NVIC* const nvic = (NVIC*) NVIC_ADR;

static STATE state = IDLE;

/**
 * get_state:
 * Returns current channel state.
 * parameters: none
 * returns: the current channel state of idle, busy, or collision
 */
STATE get_state(void) {
	return state;
}

/**
 * channel_monitor_init:
 * Enables rising and falling edge detection for pin PC12
 * and resulting interrupt call on EXTI12.
 * parameters: none
 * returns: none
 */
void channel_monitor_init(void) {
	/* RCC */
	// enable RCC for GPIOC (PC11-12)
	rcc->AHB1ENR |= (1<<2); 	// GPIOC = BIT 2
	// enable RCC for SYSCFG
	rcc->APB2ENR |= (1<<14);	// SYSCFG = Bit 14
	// enable RCC for TIM2
	rcc->APB1ENR |= (1<<0);		// TIM2 = Bit 0

	/* Tx/Rx Pins */
	// set PC11 to output and PC12 to input (rmw)
	gpioc->MODER |= PINS_MASK;
	gpioc->MODER &= ~(PINS_MASK<<1);

	/* External Interrupt Config */
	// enable EXTI for PC12
	syscfg->EXTICR4 |= 0b0010;
	// enable EXTI line 12
	exti->IMR |= (1<<12);
	// enable rising edge detection for EXTI12
	exti->RTSR |= (1<<12);
	// enable falling edge detection for EXTI12
	exti->FTSR |= (1<<12);

	/* Enable Interrupts */
	// enable TIM2 interrupt in NVIC
	nvic->ISER0 |= (1<<TIM2n);
	// enable EXTI interrupt in NVIC
	nvic->ISER1 |= (1<<(EXTI15_10n - 32));
	// enable TIM2 interrupts
	tim2->DIER |= (1<<0);
}

/**
 * TIM2_IRQHandler:
 * Interrupt handler for idle/collision timer timeout whilst in busy states.
 * parameters: none
 * returns: none
 */
void TIM2_IRQHandler(void) {
	// clear UIF pending bit
	tim2->SR &= ~(1<<0);

	// stop timer (resets implicitly)
	tim2->CR1 &= ~(1<<0);

	// update state
	switch (state) {
	case BUSY_LOW: {
		state = COLLISION;
		break;
		}
	case BUSY_HIGH: {
		state = IDLE;
		break;
		}
	default: break;
	};
}

/**
 * EXTI15_10_IRQHandler:
 * Interrupt handler for rising/falling edge detection on PC12 by EXTI12.
 * parameters: none
 * returns: none
 */
void EXTI15_10_IRQHandler(void) {
	// clear EXTI12 pending bit
	exti->PR |= (1<<12);

	//TODO temporarily disable interrupts to prevent race condition?

	// stop and reset timer
	tim2->CR1 &= ~(1<<0);
	tim2->CNT = 0;

	// update state
	switch (state) {
	case IDLE: {
		state = BUSY_LOW;
		// preset collision timer
		tim2->ARR = COLL_TIME;
		break;
		}
	case BUSY_LOW: {
		state = BUSY_HIGH;
		// preset idle timer
		tim2->ARR = IDLE_TIME;
		break;
		}
	case BUSY_HIGH: {
		state = BUSY_LOW;
		// preset collision timer
		tim2->ARR = COLL_TIME;
		break;
		}
	case COLLISION: {
		//TODO wait random amount of time before switching?
		state = BUSY_HIGH;
		// preset idle timer
		tim2->ARR = IDLE_TIME;
		break;
		}
	default: break;
	};

	// start timer
	tim2->CR1 |= (1<<0);
}
