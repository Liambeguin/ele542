/*
 * This is the main application
 */

#include "inc/platform.h"
#include "inc/uart.h"
#include "inc/timer.h"

int main(void) {


	platform_init();
	uart_send((uint8_t*)"\r\nInit Done!\r\n", 14);

	while(1){

		leds_toggle(LED_0);
		timer1_update_channel_compare(0, 0);
		_delay_ms(1000);
		timer1_update_channel_compare(1000, 0);
		_delay_ms(1000);
	}

	return EXIT_FAILURE;
}
