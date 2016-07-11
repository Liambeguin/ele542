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
 *   Filename : platform.c
 * Created on : Jul 11, 2016
 *    Authors : Jeremie Venne <jeremie.k.venne@gmail.com>
 *              Liam Beguin <liambeguin@gmail.com>
 *
 */

#include "inc/platform.h"
#include "inc/motors.h"
#include "inc/timer.h"
#include "inc/uart.h"
#include "inc/gpio.h"
#include "inc/adc.h"
#include "inc/twi.h"
#include "avr/wdt.h"

/* @brief: for clarity lets use more functions */
void leds_on(uint8_t leds) {
	gpio_pin_clear(GPIO_PORT_B, leds);
}

void leds_off(uint8_t leds) {
	gpio_pin_set(GPIO_PORT_B, leds);
}
void leds_toggle(uint8_t leds) {
	gpio_pin_toggle(GPIO_PORT_B, leds);
}

void platform_leds_init(void) {
	/* Set all PORTB as output */
	gpio_set_output(GPIO_PORT_B, 0xff);
	leds_off(0xff);
}

/* @brief: set switches as inputs */
void platform_init_switches(void) {
	gpio_set_input(GPIO_PORT_A, GPIO_PIN_6 | GPIO_PIN_7);
}

/* @brief: initialize the whole platform */
void platform_init(void) {
	/* Disable interrupts */
	cli();

	/* Init */
	platform_leds_init();
	platform_init_switches();
	uart_init(UART_BAUDRATE_9600);
	timer1_init();
	adc_init();
	motors_init();
	twi_init();

	/* Enable interrupts */
	sei();

	/* Restart if no command is recv after 500ms */
	wdt_enable(5);
}

/* @brief: atof^{-1} */
/* @return: if given 3.14 returns ['3', '.', '1', '4', '0'] */
void ftoa(float num, char *buffer, uint8_t size) {
	uint8_t i = 0;
	buffer[0] = (char)(((int)(num * 1   ) % 10) + 48) & 0xFF ;
	buffer[1] = '.';

	for(i=2; i < size; i++)
		buffer[i] = (char)(((int)(num * (int)(10^i)  ) % 10) + 48) & 0xFF ;
}
