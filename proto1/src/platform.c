#include "inc/platform.h"
#include "inc/motors.h"
#include "inc/timer.h"
#include "inc/uart.h"
#include "inc/gpio.h"
#include "inc/adc.h"
#include "inc/twi.h"


void leds_on(uint8_t leds) {
	gpio_pin_clear(GPIO_PORT_B, leds);
}

void leds_off(uint8_t leds) {
	gpio_pin_set(GPIO_PORT_B, leds);
}
void leds_toggle(uint8_t leds) {
	gpio_pin_toggle(GPIO_PORT_B, leds);
}



void platform_leds_init(void) {
	/* Set all PORTB as output */
	gpio_set_output(GPIO_PORT_B, 0xff);
	leds_off(0xff);
}

uint8_t platform_init(void) {
	/* Disable interrupts */
	cli();

	/* Init */
	platform_leds_init();
	uart_init(UART_BAUDRATE_9600);
	timer1_init();
	adc_init();
	motors_init();
	twi_init();

	/* Enable interrupts */
	sei();

	return EXIT_SUCCESS;
}

