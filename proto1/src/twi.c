#include "inc/platform.h"
#include "inc/gpio.h"
#include "inc/twi.h"
/*
 * Datasheet page 167
 */




/* Global variables used for transfering data */
uint8_t transfer_addr, transfer_reg, transfer_data;
/* Global variable used for receiving data */
uint8_t transfer_data_read = 0;

uint8_t _ready = true;

uint8_t twi_init(void) {
	/* selects the division factor for the bit rate generator */
	TWBR = 0xC6;
	/* control the operation of the TWI */
	TWCR = (1 << TWEN) | (1 << TWIE);
	/* Set the prescaler to 1 */
	TWSR = (0 << TWPS1) | (0 << TWPS0);

	_ready = true;

	return EXIT_SUCCESS;
}
void sonar_init(void) {
	twi_write(SONAR_GAUCHE, REG_COMMAND, 0x51);
	twi_write(SONAR_GAUCHE, REG_GAIN,    0x08);
	twi_write(SONAR_GAUCHE, REG_PORTEE,  0x20);

	twi_write(SONAR_DROIT, REG_COMMAND, 0x51);
	twi_write(SONAR_DROIT, REG_GAIN,    0x08);
	twi_write(SONAR_DROIT, REG_PORTEE,  0x20);
}


uint8_t twi_ready(void) {
	return _ready;
}
void twi_wait(void) {
	while(!_ready);
}

void twi_transfer_start(void) {
	TWCR = (1 << TWINT) | (1 << TWSTA) | (0 << TWSTO) | (1 << TWEN) | (1 << TWIE);
	_ready = false;
}
void twi_transfer_continue(void) {
	TWCR = (1 << TWINT) | (0 << TWSTA) | (0 << TWSTO) | (1 << TWEN) | (1 << TWIE);
}
void twi_transfer_stop(void) {
	TWCR = (1 << TWINT) | (0 << TWSTA) | (1 << TWSTO) | (1 << TWEN) | (1 << TWIE);
	_ready = true;
}


void twi_read(uint8_t address, uint8_t reg, uint8_t *data) {
	/* Setting read bit */
	address |= 0x01;

	twi_write(address, reg, 0);
	/* twi_wait(); */
	*data = transfer_data_read;
}
void twi_write(uint8_t address, uint8_t reg, uint8_t data) {
	if (_ready) {
		transfer_addr = address;
		transfer_reg  = reg;
		transfer_data = data;

		twi_transfer_start();
	} else {
		DEBUG("not ready 0x%x 0x%x 0x%x", address, reg, data);
	}
	/* twi_wait(); */
}


ISR(TWI_vect) {

	uint8_t status  = TWSR & 0xF8;

	switch (status) {
		case 0x08: /* Start Condition */
		case 0x10: /* Restart Condition */
			TWDR = transfer_addr;
			twi_transfer_continue();
			break;
		case 0x18: /* Address Write Ack */
		case 0x28: /* Data Write Ack */
		case 0x30: /* Date Write NoAck */
			if (transfer_reg != 0) {
				TWDR = transfer_reg;
				transfer_reg = 0;
				twi_transfer_continue();
			} else if(transfer_data != 0) {
				TWDR = transfer_data;
				transfer_data = 0;
				twi_transfer_continue();
			} else {
				twi_transfer_stop();
			}
			break;
		case 0x50: /* Data Read Ack */
		case 0x58: /* Data Read NoAck */
			transfer_data_read = TWDR;
			twi_transfer_stop();
			break;
		case 0x40: /* Address Read Ack */
			twi_transfer_continue();
			break;
		case 0x48: /* Address Read NoAck */
		case 0x20: /* Address Write NoAck */
			twi_transfer_stop();
			break;
		default :
			/* This should never happen ...  */
			twi_transfer_continue();
			break;
	}
}
