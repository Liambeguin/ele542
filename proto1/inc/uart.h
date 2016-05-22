//uart definitioons

/*
 * UDR:  UART Data Register
 * USR:  UART Status Register
 * UCR:  UART Control Register
 * UBRR: UART Baud Rate Register
 *
 */

#ifndef _UART_H
#define _UART_H

#include "inc/platform.h"

uint8_t uart_init(uint8_t baudrate);
void uart_send (uint8_t data);
#endif /* _UART_H */
