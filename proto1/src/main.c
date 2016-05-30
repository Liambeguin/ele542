/*
 * This is the main application
 */

#include "inc/platform.h"
#include "inc/uart.h"

int main(void) {


	platform_init();
	uart_send((uint8_t*)"\r\nInit Done!\r\n", 14);

	while(1){


		uart_send((uint8_t*)"a", 1);
		leds_toggle(LED_0);
		_delay_ms(1000);
	}

	return EXIT_FAILURE;
}
