#include "inc/platform.h"
#include "inc/timer.h"
#include "inc/gpio.h"

/* Datasheeet: Timer1 detailed on Page 84 */
void timer1_init(void) {

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

	OCR1A = 0;
	OCR1B = 0;
}


ISR(TIMER1_OVF_vect) {
	leds_toggle(LED_4);
}
