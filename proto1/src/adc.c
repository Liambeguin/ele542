#include "inc/platform.h"
#include "inc/gpio.h"
#include "inc/adc.h"
/*
 * Datasheet page 199
 */

uint8_t n_acquisitions = 0;
uint8_t ready = false;

/* used to accumulate ADC measure */
uint16_t acc_D = 0, acc_G = 0;
/* used as the actual value */
float avg_D = 0, avg_G = 0;


uint8_t adc_init(void) {

	/* Set pins */
	gpio_set_input(GPIO_PORT_A, GPIO_PIN_0 | GPIO_PIN_1);
	gpio_pin_clear(GPIO_PORT_A, GPIO_PIN_0 | GPIO_PIN_1);

	/* Disable analog comp */
	/* ACSR = ( 1 << ACD); */
	/* AREF, turn off internat Vref */
	ADMUX   = 0x00;
	ADCSRA  = (1 << ADEN) | (1 << ADSC) | (1 << ADATE) | (1 << ADIE);
	/* Setting prescaler to 128 */
	ADCSRA |= 0x07;
	/* Setting FreeRunning mode */
	SFIOR   = 0x00;

	/* This is used to indicate if an averaged value is available */
	ready = false;

	return EXIT_SUCCESS;
}

uint8_t adc_ready(void) {
	return ready;
}

void adc_new_measure(void) {
	n_acquisitions = 0;
	acc_G = 0;
	acc_D = 0;
	ready = false;
}


void adc_get_averaged_values(float *Vg, float *Vd) {
	while(!ready);
	*Vg = avg_G;
	*Vd = acc_D;
	adc_new_measure();
}

ISR(ADC_vect) {
	if (n_acquisitions < MAX_ACQUISITIONS) {
		/* If MOTEUR_D */
		if (ADMUX) {
			/* add to left motor because of delay */
			acc_G += ADC;
			n_acquisitions++;
		} else {
			acc_D += ADC;
			n_acquisitions++;
		}
		/* Switch channel */
		ADMUX ^= 0x01;
	} else {
		/* Conversion ready */
		ready = true;
		n_acquisitions = 0;

		/* calc average */
		avg_G = (acc_G << 1) / MAX_ACQUISITIONS;
		avg_D = (acc_D << 1) / MAX_ACQUISITIONS;
	}

}
