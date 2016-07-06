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



uint8_t twi_init(void);
void twi_read(uint8_t address, uint8_t reg, uint8_t *data);
void twi_write(uint8_t address, uint8_t reg, uint8_t data);
void twi_wait(void);

// This should be in sonar.c but meh...
void sonar_init(void);
#endif /* _TWI_H */
