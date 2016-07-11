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
 *   Filename : twi.c
 * Created on : Jul 11, 2016
 *    Authors : Jeremie Venne <jeremie.k.venne@gmail.com>
 *              Liam Beguin <liambeguin@gmail.com>
 *
 */

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
/* Global variable used to check if the interface is busy */
uint8_t _ready = true;

void twi_init(void) {
	/* selects the division factor for the bit rate generator */
	TWBR = 0xC6;
	/* control the operation of the TWI */
	TWCR = (1 << TWEN) | (1 << TWIE);
	/* Set the prescaler to 1 */
	TWSR = (0 << TWPS1) | (0 << TWPS0);

	_ready = true;
}

/* @brief: configure the sonars to use cm */
void sonar_init(uint8_t gain, uint8_t range) {
	/* 0x51 is to get measures in cm */
	twi_write(SONAR_GAUCHE, REG_COMMAND, 0x51);
	twi_write(SONAR_GAUCHE, REG_GAIN,    gain);
	twi_write(SONAR_GAUCHE, REG_PORTEE,  range);

	twi_write(SONAR_DROIT, REG_COMMAND, 0x51);
	twi_write(SONAR_DROIT, REG_GAIN,    gain);
	twi_write(SONAR_DROIT, REG_PORTEE,  range);
}

/* @brief: update sonar ranges based on the desired speed */
void sonar_update_range(void) {
	uint8_t range = uart_get_raw_speed();
	if (range > 100)
		range -= 100;
	else
		range = 0;

	range += 24;

	twi_write(SONAR_GAUCHE, REG_PORTEE,  range);
	twi_write(SONAR_DROIT,  REG_PORTEE,  range);
}

/* @brief: update obstacle distance detection */
void sonar_get_obstacle(uint16_t *obstacle_g, uint16_t *obstacle_d) {
	uint8_t gauche = 0;
	uint8_t droite = 0;

	leds_toggle(LED_SONAR_L_PING);
	twi_read(SONAR_GAUCHE, REG_MSB, &gauche);
	*obstacle_g = (gauche << 4); 
	twi_read(SONAR_GAUCHE, REG_LSB, &gauche);
	*obstacle_g |= gauche; 

	leds_toggle(LED_SONAR_R_PING);
	twi_read(SONAR_DROIT, REG_MSB, &droite);
	*obstacle_d = (droite << 4); 
	twi_read(SONAR_DROIT, REG_LSB, &droite);
	*obstacle_d |= droite; 
}

/* @brief: add a delta to the desired angle to avoid obstacles */
void sonar_avoid_obstacles(uint16_t obstacle_g, uint16_t obstacle_d, float *delta_angle) {

	static int8_t delta_right = 0, delta_left = 0; 

	/* the closer we get, the more we turn */
	if (obstacle_g < 1) {
		leds_on(LED_SONAR_L_OBST);
		delta_left++; 
	/* if the object is far, adjust just a bit */
	} else if (obstacle_g < 15) {
		leds_on(LED_SONAR_L_OBST);
		delta_left = 5; 
	} else {
		leds_off(LED_SONAR_L_OBST);
		delta_left = 0; 
	}

	if (obstacle_d < 1) {
		leds_on(LED_SONAR_R_OBST);
		delta_right++; 
	} else if (obstacle_d < 15) {
		leds_on(LED_SONAR_R_OBST);
		delta_right = 5; 
	} else {
		leds_off(LED_SONAR_R_OBST);
		delta_right = 0; 
	}
	// NOTE: if both sonars detect someting the effect is canceled
	if (obstacle_g <= 1 && obstacle_d <= 1) {
		delta_right = 0;
		delta_left  = 90;
	}

	/* sum right and left error */
	*delta_angle = delta_right - delta_left;
	/* convert value to rad */
	*delta_angle = *delta_angle * Pi / 180;
	//DEBUG("OBSTACLES: %d %d - %d %d", obstacle_g, obstacle_d, delta_right, delta_left);
}


/* @brief: convenience functions */
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

/* @brief: read a given register over twi */
void twi_read(uint8_t address, uint8_t reg, uint8_t *data) {
	/* Setting read bit */
	address |= 0x01;
	twi_write(address, reg, 0);

	*data = transfer_data_read;
}
/* @brief: write a given register over twi */
void twi_write(uint8_t address, uint8_t reg, uint8_t data) {
	if (_ready) {
		transfer_addr = address;
		transfer_reg  = reg;
		transfer_data = data;

		twi_transfer_start();
	} else {
		DEBUG("not ready 0x%x 0x%x 0x%x", address, reg, data);
	}
	twi_wait();
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
