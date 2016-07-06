/*
 * This is the main application
 */

#include "inc/platform.h"
#include "inc/motors.h"
#include "inc/timer.h"
#include "inc/uart.h"
#include "inc/adc.h"

int main(void) {

	platform_init();

	/* Definitions to use printf with our UART */
	FILE uart_str = FDEV_SETUP_STREAM(printCHAR, NULL, _FDEV_SETUP_RW);
	stdout = &uart_str;

	printf("\nInit Done!\n");

	motors_calibration();

	while(1){

		leds_toggle(LED_0);
		timer1_update_channel_compare(0, 0);
		_delay_ms(1000);
		timer1_update_channel_compare(1000, 0);
		_delay_ms(1000);
	}

	return EXIT_FAILURE;
}
