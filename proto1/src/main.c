/*
 * This is the main application
 */

#include "inc/platform.h"
#include "inc/uart.h"

int main(void) {

	uint8_t i=0;
	// Use all pins of PORTB as outputs
	DDRB = 0xff;


	platform_init();

	PORTB = ~(0xa);
	while(1){
		uart_send('a');
		_delay_ms(10);
		i++;
	}

	return EXIT_FAILURE;
}
