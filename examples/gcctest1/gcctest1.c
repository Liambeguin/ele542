#include <stdint.h>
#include <avr/io.h>
#include <util/delay.h>


int main( void )
{

	uint8_t i=0;
	// Use all pins of PORTB as outputs
	DDRB = 0xff;



	while(1){
		PORTB = (i);
		_delay_ms(100);
		i++;
	}

	return 1;
}
