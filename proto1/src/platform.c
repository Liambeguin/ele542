
#include "inc/platform.h"
#include "inc/uart.h"



uint8_t platform_init(void){
	/* Disable interrupts */

	/* Init */
	/* led_init(); */
	uart_init(UART_BAUDRATE_9600);

	/* Enable interrupts */
	/* sei(); */

	return EXIT_SUCCESS;
}

