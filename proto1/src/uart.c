#include "inc/platform.h"
#include "inc/circular_buff.h"

/* Private uart buffer */
circular_buff_t uart_buffer = {0};


uint8_t uart_init(uint8_t baudrate) {
	/* setting baudrate */
	UBRRH = (uint8_t) (baudrate >> 8);
	UBRRL = (uint8_t) baudrate;


	/* setting frame format */
	UCSRC = ((1 << URSEL) | (1 << UCSZ1) | (1 << UCSZ0));

	/* enabling Transmitter and Receiver */
	UCSRB = ((1 << RXCIE) | (1 << TXCIE) | (1 << RXEN) | (1 << TXEN));

	/* Setting up Circular Buffer */
	uart_buffer = cb_init();

	/* Clearing UDR */
	UDR = 0x00;

	return EXIT_SUCCESS;
}


void uart_send_byte (uint8_t data) {
	while (!(UCSRA &  (1<<UDRE)) );
	UDR = data;
}
void uart_send (uint8_t* data, uint8_t size) {
	register uint8_t i = 0;
	for (i = 0; i < size; i++) {
		if (uart_buffer.isFull)
			return;
		cb_write(&uart_buffer, &data[i]);
	}
}


/* Interrupt when transmission is complete */
ISR(USART_TXC_vect) {
	uint8_t byte = 0;

	while (!(UCSRA &  (1<<UDRE)) );

	PORTB &= ~LED_UART_RX;
	cb_read(&uart_buffer, &byte);
	UDR = byte;
}



ISR(USART_RXC_vect) {

	uint8_t byte = UDR;
	PORTB |= ~LED_UART_RX;
	cb_write(&uart_buffer, &byte);
	/* uart_send_byte(byte); */
}

