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
 *   Filename : uart.h
 * Created on : Jul 11, 2016
 *    Authors : Jeremie Venne <jeremie.k.venne@gmail.com>
 *              Liam Beguin <liambeguin@gmail.com>
 *
 */

#ifndef _UART_H
#define _UART_H

#include "inc/platform.h"

void uart_init(uint8_t baudrate);
int  uart_putc(char character, FILE *stream);
void uart_send_byte(uint8_t data);
void uart_send(char *buf, uint8_t size);

float uart_get_speed(void);
uint8_t uart_get_raw_speed(void);
float uart_get_angle(void);
uint8_t uart_get_state(void);

#endif /* _UART_H */
