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
 *   Filename : main.c
 * Created on : Jul 11, 2016
 *    Authors : Jeremie Venne <jeremie.k.venne@gmail.com>
 *              Liam Beguin <liambeguin@gmail.com>
 *
 */

/*
 * This is the main application
 */

#include <stdlib.h>
#include "inc/platform.h"
#include "inc/motors.h"
#include "inc/timer.h"
#include "inc/gpio.h"
#include "inc/uart.h"
#include "inc/adc.h"
#include "inc/twi.h"

/* global definitions */
extern uint8_t ovf;
uint8_t robot_state = ROBOT_IDLE;
uint8_t emergency_stop = 1;
uint8_t emergency_stop_buttons = 1;

int main(void) {

	/* Desired speed and angle received from uart */
	float speed_d = 0, angle_d = 0;
	/* main loop Utilitues */
	uint8_t counter = 0;
	/* Sonar readings */
	uint16_t sonar_dist_g = 0xffff, sonar_dist_d = 0xffff;
	/* angle correction for obstacle avoidance */
	float delta = 0;
	// FIXME: debug
	char buffer[5] ;



	platform_init();

	/* Definitions to use printf with our UART */
	FILE uart_str = FDEV_SETUP_STREAM(uart_putc, NULL, _FDEV_SETUP_RW);
	stdout = &uart_str;

	DEBUG("\nInit Done!\n");

	motors_calibration();
	// Clear calibration BIT
	gpio_pin_clear(GPIO_PORT_A, GPIO_PIN_4);
	sonar_init(16, 24);

	while(1) {

		if (!ovf)
			continue;
		else
			ovf = 0;

		/* If emergency stop */
		if (!gpio_pin_read(GPIO_PORT_A, GPIO_PIN_6))
			emergency_stop_buttons = 1;
		else if (!gpio_pin_read(GPIO_PORT_A, GPIO_PIN_7))
			emergency_stop_buttons = 0;
		emergency_stop = emergency_stop_buttons | !(uart_get_state());

		// Handle switches
		switch(robot_state) {
			case ROBOT_IDLE:
				leds_on(LED_ROBOT_IDLE);
				leds_off(LED_ROBOT_ACTIVE);
				if (!emergency_stop)
						robot_state = ROBOT_HOT;
				else {
					motor_set_direction(FREIN, FREIN);
					robot_state = ROBOT_IDLE;
					continue;
				}
				break;
			case ROBOT_HOT:
				leds_off(LED_ROBOT_IDLE);
				leds_on(LED_ROBOT_ACTIVE);
				if (emergency_stop)
						robot_state = ROBOT_IDLE;
				break;
		}

		if (counter >= 10) {
			counter = 0;
			/* get distances */
			sonar_get_obstacle(&sonar_dist_g, &sonar_dist_d);
			/* Avoid obstacles */
			sonar_avoid_obstacles(sonar_dist_g, sonar_dist_d, &delta);
			/* Change range fased on desired speed */
			sonar_update_range();
		}

		/* Get values from UART teleguidage */
		speed_d = uart_get_speed();
		angle_d = uart_get_angle() + delta;
		uart_echo();

		/* compute new speeds and update the timer */
		motors_update(speed_d, angle_d);

		counter++;
	}
	return EXIT_FAILURE;
}
