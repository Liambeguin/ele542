/*
 * This file is part of Lab1 ELE542
 *
 * "THE BEER-WARE LICENSE" (Revision 42):
 *  As long as you retain this notice you can do whatever you want with this
 *  stuff. If we meet some day, and you think this stuff is worth it,
 *  you can buy us a beer in return.
 *  If you use this at ETS, beware of the shool's policy against copying
 *  and fraud.
 *
 *   Filename : platform.h
 * Created on : Jul 11, 2016
 *    Authors : Jeremie Venne <jeremie.k.venne@gmail.com>
 *              Liam Beguin <liambeguin@gmail.com>
 *
 */

#ifndef PLATFORM_H_
#define PLATFORM_H_
/*
 * This file contains global definitions used accross the project
 */


/* Global inclusions */
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>

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

/* Used to avoid warning when parameter are unused */
#define UNUSED(x) (void)(x)

#define Pi      (3.1415926535897932384626433832795)

#if (LOG_LEVEL == 1)
	#define DEBUG(f_, ...) \
	do{ printf(" * in %s() : ",__FUNCTION__); \
		printf((f_), ##__VA_ARGS__); printf("\r\n");} while(0);
#elif (LOG_LEVEL == 2)
	#define DEBUG(f_, ...) \
	do{ uart_send_byte(0xFE); printf((f_), ##__VA_ARGS__); \
		uart_send_byte(0xFF);} while (0);
#else
	#define DEBUG(f_, ...) {}
#endif

enum {
	ROBOT_IDLE = 0,
	ROBOT_HOT  = 1,
};
 

/* prototypes */
void platform_init(void);

void leds_on(uint8_t leds);
void leds_off(uint8_t leds);
void leds_toggle(uint8_t leds);

/* helper functions */
void ftoa(float num, char *buffer, uint8_t size);

#endif /* _PLATFORM_H_ */
/* vim: set cc=80 : */
