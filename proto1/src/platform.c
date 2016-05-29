
#include "inc/platform.h"
#include "inc/uart.h"



uint8_t platform_init(void) {
	/* Disable interrupts */
	cli();

	/* Init */

	DDRB = 0xff;
	PORTB = ~(0x00);

	uart_init(UART_BAUDRATE_9600);

	/* Enable interrupts */
	sei();

	return EXIT_SUCCESS;
}

