#include "inc/platform.h"
#include "inc/gpio.h"


/*
 * More info:
 * https://cours.etsmtl.ca/ele542/labo/Ref-STK500-ATMega32/ATMega32-Complete.pdf
 * Page 49
 */

uint8_t gpio_set_input(gpio_port_t port, uint8_t gpio_mask) {

	switch(port) {
		case GPIO_PORT_A:
			DDRA &= ~(gpio_mask);
			break;
		case GPIO_PORT_B:
			DDRB &= ~(gpio_mask);
			break;
		case GPIO_PORT_C:
			DDRC &= ~(gpio_mask);
			break;
		case GPIO_PORT_D:
			DDRD &= ~(gpio_mask);
			break;
	}
	return EXIT_SUCCESS;
}
uint8_t gpio_set_output(gpio_port_t port, uint8_t gpio_mask) {

	switch(port) {
		case GPIO_PORT_A:
			DDRA |= gpio_mask;
			break;
		case GPIO_PORT_B:
			DDRB |= gpio_mask;
			break;
		case GPIO_PORT_C:
			DDRC |= gpio_mask;
			break;
		case GPIO_PORT_D:
			DDRD |= gpio_mask;
			break;
	}
	return EXIT_SUCCESS;
}
void gpio_pin_set(gpio_port_t port, uint8_t gpio_mask) {

	switch(port) {
		case GPIO_PORT_A:
			PORTA |= gpio_mask;
			break;
		case GPIO_PORT_B:
			PORTB |= gpio_mask;
			break;
		case GPIO_PORT_C:
			PORTC |= gpio_mask;
			break;
		case GPIO_PORT_D:
			PORTD |= gpio_mask;
			break;
	}
}

void gpio_pin_clear(gpio_port_t port, uint8_t gpio_mask) {

	switch(port) {
		case GPIO_PORT_A:
			PORTA &= ~(gpio_mask);
			break;
		case GPIO_PORT_B:
			PORTB &= ~(gpio_mask);
			break;
		case GPIO_PORT_C:
			PORTC &= ~(gpio_mask);
			break;
		case GPIO_PORT_D:
			PORTD &= ~(gpio_mask);
			break;
	}
}

void gpio_pin_toggle(gpio_port_t port, uint8_t gpio_mask) {

	switch(port) {
		case GPIO_PORT_A:
			PORTA ^= gpio_mask;
			break;
		case GPIO_PORT_B:
			PORTB ^= gpio_mask;
			break;
		case GPIO_PORT_C:
			PORTC ^= gpio_mask;
			break;
		case GPIO_PORT_D:
			PORTD ^= gpio_mask;
			break;
	}
}
uint8_t gpio_port_read(gpio_port_t port) {

	uint8_t data = 0x00;

	switch(port) {
		case GPIO_PORT_A:
			data = PINA;
			break;
		case GPIO_PORT_B:
			data = PINB;
			break;
		case GPIO_PORT_C:
			data = PINC;
			break;
		case GPIO_PORT_D:
			data = PIND;
			break;
	}
	return data;
}
/* inline uint8_t gpio_pin_read(gpio_port_t port, uint8_t pin) { */
/* 	return gpio_port_read(port) & pin; */
/* } */
