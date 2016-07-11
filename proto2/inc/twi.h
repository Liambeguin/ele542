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
 *   Filename : twi.h
 * Created on : Jul 11, 2016
 *    Authors : Jeremie Venne <jeremie.k.venne@gmail.com>
 *              Liam Beguin <liambeguin@gmail.com>
 *
 */

#ifndef _TWI_H
#define _TWI_H

#include "inc/platform.h"


enum {
	SONAR_GAUCHE = 0xE0,
	SONAR_DROIT  = 0xE2,
};

typedef enum {
	/* Read mode */
	REG_REVISION = 0,
	REG_MSB      = 2,
	REG_LSB      = 3,
	/* Write mode */
	REG_COMMAND  = 0,
	REG_GAIN     = 1,
	REG_PORTEE   = 2,
} sonar_reg_t;



void twi_init(void);
void twi_read(uint8_t address, uint8_t reg, uint8_t *data);
void twi_write(uint8_t address, uint8_t reg, uint8_t data);
void twi_wait(void);

// This should be in sonar.c but meh...
void sonar_init(uint8_t gain, uint8_t range);
void sonar_update_range(void);
void sonar_get_obstacle(uint16_t *obstacle_g, uint16_t *obstacle_d);
void sonar_avoid_obstacles(uint16_t obstacle_g, uint16_t obstacle_d, float *delta_angle);
#endif /* _TWI_H */
