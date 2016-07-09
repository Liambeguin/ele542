/*
 * This is the main application
 */

#include "inc/platform.h"
#include "inc/motors.h"
#include "inc/timer.h"
#include "inc/uart.h"
#include "inc/adc.h"
#include "inc/twi.h"

int main(void) {

	uint8_t foo = 0;

	platform_init();

	/* Definitions to use printf with our UART */
	FILE uart_str = FDEV_SETUP_STREAM(printCHAR, NULL, _FDEV_SETUP_RW);
	stdout = &uart_str;

	printf("\nInit Done!\n");

	motors_calibration();
	sonar_init();

	while(1){

		leds_toggle(LED_0);
		_delay_ms(1000);

		twi_read(SONAR_GAUCHE, REG_MSB, &foo);
		DEBUG("%d", foo);
		twi_read(SONAR_DROIT, REG_GAIN, &foo);
		DEBUG("%d", foo);
		twi_read(SONAR_DROIT, REG_PORTEE, &foo);
		DEBUG("%d", foo);
	}

	return EXIT_FAILURE;
}
