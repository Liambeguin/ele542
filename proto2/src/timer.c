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
 *   Filename : timer.c
 * Created on : Jul 11, 2016
 *    Authors : Jeremie Venne <jeremie.k.venne@gmail.com>
 *              Liam Beguin <liambeguin@gmail.com>
 *
 */
#include "inc/includes.h"
#include "inc/platform.h"
#include "inc/timer.h"
#include "inc/gpio.h"
#include "inc/uart.h"

static uint16_t OCR1A_value = 0x0000;
static uint16_t OCR1B_value = 0x0000;
uint8_t ovf = 0;

extern OS_EVENT	*TimerOVF_Sem;

/* @brief: Initialise and configure the timer 1 to use 2 PWM */
void timer1_init(void) {
	/* Datasheeet: Timer1 detailed on Page 84 */
	/* Timer1 Control Register A P105 */
	TCCR1A = (1 << COM1A1) | (0 << COM1A0) | (1 << COM1B1) | (0 << COM1B0) | \
			 (0 << FOC1A)  | (0 << FOC1B)  | (1 << WGM11)  | (0 << WGM10);
	/* Timer1 Control Register B P108 */
	TCCR1B = (0 << ICNC1) | (0 << ICES1) | (1 << WGM13) | (1 << WGM12) | \
			 (0 << CS12)  | (1 << CS11)  | (1 << CS10);

	/* Timer Interrupt Mask Register */
	TIMSK = (1 << TOIE1);

	/* ICR1 (=TOP) calculated using function P99 (Prescaler is 64, F=200Hz) */
	ICR1 = 0x04E1;

	/* Timer1 Channel A ouputs on PD5 and Channel B on PD4 */
	gpio_set_output(GPIO_PORT_D, (PIN_TIM1_CHA | PIN_TIM1_CHB) );

	timer1_update_channel_compare(0, 0);
}

/*
 * @brief: Write the 2 PWM channels (channel A and B) to be updated
 *
 * @param ch_a: Duty cycle of channel A from 0 to ICR1
 * @param ch_b: Duty cycle of channel B from 0 to ICR1
 */
void timer1_update_channel_compare(uint16_t ch_a, uint16_t ch_b) {
	OCR1A_value = ch_a;
	OCR1B_value = ch_b;
}

/* @brief: Timer 1 overflow interrupt */
ISR(TIMER1_OVF_vect, ISR_NAKED) {
	OS_INT_ENTER();
	OCR1A = OCR1A_value;
	OCR1B = OCR1B_value;

	// Used to see if 5ms have passed
	ovf = 1; // keep flag for motor calibration
	OSSemPost(TimerOVF_Sem);

	OS_INT_EXIT();
}
