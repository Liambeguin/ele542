#include "inc/platform.h"
#include <util/setbaud.h>


uint8_t uart_init(uint8_t baudrate) {
	/* setting baudrate */
	UBRRH = (uint8_t) (baudrate >> 8);
	UBRRL = (uint8_t) baudrate;



	/* setting frame format */
	UCSRC = ((1 << URSEL) | (1 << UCSZ1) | (1 << UCSZ0));

	/* enabling Transmitter and Receiver */
	UCSRB = ((1 << RXCIE) | (1 << TXCIE) | (1 << RXEN) | (1 << TXEN));

	return EXIT_SUCCESS;
}

void uart_send (uint8_t data) {
	while (!(UCSRA &  (1<<UDRE)) );
	UDR = data;
}
