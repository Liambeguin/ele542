/*
 * This file is part of Lab1 ELE542
 *
 * "THE BEER-WARE LICENSE" (Revision 42):
 *  As long as you retain this notice you can do whatever you want with this
 *  stuff. If we meet some day, and you think this stuff is worth it,
 *  you can buy us a beer in return.
 *  If you use this at ETS, beware of the shool's policy against copying
 *  and fraud.
 *
 *   Filename : uart.c
 * Created on : Jul 11, 2016
 *    Authors : Jeremie Venne <jeremie.k.venne@gmail.com>
 *              Liam Beguin <liambeguin@gmail.com>
 *
 */

#include "inc/platform.h"
#include "inc/gpio.h"
#include "inc/uart.h"
#include "avr/wdt.h"

/* uart globals */
char *uart_data_ptr;
uint8_t uart_counter;
uint8_t RX_data[3];
uint8_t echoData[3];
uint8_t RX_dataIndex = 0;
uint8_t RXcompleteFlag = 0;

/* @brief: initialize uart */
void uart_init(uint8_t baudrate) {
	/* setting baudrate */
	UBRRH = (uint8_t) (baudrate >> 8);
	UBRRL = (uint8_t) baudrate;

	/* setting frame format */
	UCSRC = ((1 << URSEL) | (1 << UCSZ1) | (1 << UCSZ0));

	/* enabling Transmitter and Receiver */
	UCSRB = ((1 << RXCIE) | (1 << TXCIE) | (1 << RXEN) | (1 << TXEN));

	/* Clearing UDR */
	UDR = 0x00;
}

/* @return desired 'teleguidage' speed [-1, 1] */
float uart_get_speed(void) {
	if (RX_data[1] == 0)
		return 0.0;
	else
		return (((float)RX_data[1] - 100.0) / 100.0);
}

/* @return desired 'teleguidage' speed [0, 200] */
uint8_t uart_get_raw_speed(void) {
	return RX_data[1];
}

/* @return desired 'teleguidage' angle [0, 2pi] */
float uart_get_angle(void) {
	if (RX_data[2] == 0)
		return 0.0;
	else
		return (((float)RX_data[2] * 2.0 * Pi) / 180.0);
}

/* @return: state of 'teleguidage' command {0; 1} */
uint8_t uart_get_state(void) {
	/* If this returns 0 -> emergency_stop */
	return (RX_data[0] & 0x01);
}

/* @brief: send a single byte to the uart using printf function */
int uart_putc(char character, FILE *stream) {
	UNUSED(stream);
	while (!(UCSRA &  (1<<UDRE)) );
	UDR = character;
	return 0;
} 

/* @brief: send a single byte to the uart */
void uart_send_byte(uint8_t data) { 
	while (!(UCSRA &  (1<<UDRE)) );
	UDR = data;
} 

/* @brief: send buffer <buf> to uart */
void uart_send(char *buf, uint8_t size) {
	if (!uart_counter) {
		/* write first byte to data buffer */
		uart_data_ptr = buf;          
		uart_counter  = size;
		UDR = *buf;
	}
}

/* @brief: send back command */
void uart_echo(void) {
	echoData[0] = RX_data[0];
	echoData[1] = RX_data[1];
	echoData[2] = RX_data[2];

	uart_send((char*)echoData, 3);
}

/* Interrupt when transmission is complete */
ISR(USART_TXC_vect) {
	uart_data_ptr++;
	if (--uart_counter)
		/* write byte to data buffer */
		UDR = *uart_data_ptr;
}

/* signal handler for receive complete interrupt */
ISR(USART_RXC_vect) {
	uint8_t UDR_data = UDR;
   	if (UDR_data == 0xF1 || UDR_data == 0xF0 || RX_dataIndex == 0) {
		RX_dataIndex = 0;
		RX_data[0]   = UDR_data;
		RX_dataIndex++;
		leds_toggle(LED_CMD_RECV);
		if (UDR_data == 0xF1 || UDR_data == 0xF0)
			wdt_reset();
   	} else {
   		RX_data[RX_dataIndex] = UDR_data;
		RX_dataIndex++;
		if(RX_dataIndex == 3) {
			RXcompleteFlag = 1;
			RX_dataIndex = 0;
		}
	}
}
