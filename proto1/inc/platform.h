#ifndef PLATFORM_H_
#define PLATFORM_H_
/*
 * This file contains global definitions used accross the project
 */


/* Global inclusions */
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

/* Global definitions */
#define EXIT_SUCCESS 0
#define EXIT_FAILURE 1

#ifndef F_CPU
	#define F_CPU 16000000 // 16.000 MHz
#endif /* F_CPU */
#ifndef BAUD
	#define BAUD 9600
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

#define LED_HEARTBEAT     LED_0
#define LED_CMD_RECV      LED_1
#define LED_SONAR_R_OBST  LED_2
#define LED_SONAR_R_PING  LED_3
#define LED_SONAR_L_OBST  LED_4
#define LED_SONAR_L_PING  LED_5
#define LED_ROBOT_IDLE    LED_6
#define LED_ROBOT_ACTIVE  LED_7

/* Timer */
#define PIN_TIM1_CHA ((uint8_t)(1 << 5))
#define PIN_TIM1_CHB ((uint8_t)(1 << 4))

/* UART */
#define UART_BAUDRATE_9600 103 // see datasheet p166 for details

/* ADC */
#define MOTEUR_G 0
#define MOTEUR_D 1

#define LOG_LEVEL 1
#if (LOG_LEVEL == 1)
	#define DEBUG(...) \
	do{ printf(" * in %s() : ",__FUNCTION__); \
		printf(__VA_ARGS__); printf("\n");} while(0);
#else
	#define DEBUG(...) (void)(__VA_ARGS__)
#endif

/* prototypes */
uint8_t platform_init(void);

void leds_on(uint8_t leds);
void leds_off(uint8_t leds);
void leds_toggle(uint8_t leds);


#endif /* _PLATFORM_H_ */
/* vim: set cc=80 : */
