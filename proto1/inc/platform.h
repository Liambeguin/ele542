#ifndef PLATFORM_H_
#define PLATFORM_H_
/*
 * This file contains global definitions used accross the project
 */


/* Global inclusions */
#include <stdint.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

/* Global definitions */
#define EXIT_SUCCESS 0
#define EXIT_FAILURE 1

#ifndef F_CPU
	#define F_CPU 16000000UL // 16.000 MHz
#endif /* F_CPU */
#ifndef BAUD
	#define BAUD 9600UL
#endif /* BAUD */

/* LEDs */
#define LED_0 ((uint8_t)(1 << 0))
#define LED_1 ((uint8_t)(1 << 1))
#define LED_2 ((uint8_t)(1 << 2))
#define LED_3 ((uint8_t)(1 << 3))
#define LED_4 ((uint8_t)(1 << 4))
#define LED_5 ((uint8_t)(1 << 5))
#define LED_6 ((uint8_t)(1 << 6))
#define LED_7 ((uint8_t)(1 << 7))

/* UART */
#define UART_BAUDRATE_9600 103 // see datasheet p166 for details

/* prototypes */
uint8_t platform_init(void);

#endif /* _PLATFORM_H_ */
/* vim: set cc=80 : */
