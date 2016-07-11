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
 *   Filename : circular_buff.h
 * Created on : Jul 11, 2016
 *    Authors : Jeremie Venne <jeremie.k.venne@gmail.com>
 *              Liam Beguin <liambeguin@gmail.com>
 *
 */

#ifndef CIRCULAR_BUFF_H_
#define CIRCULAR_BUFF_H_

#include <stdbool.h>
#include <stdint.h>

#define CB_MAX_SIZE 60

typedef struct {
	uint8_t size;
	uint8_t head;
	uint8_t tail;

	bool	isFull;
	bool	isEmpty;

	uint8_t buffer[CB_MAX_SIZE];

}circular_buff_t;


circular_buff_t cb_init(void);
void cb_write(circular_buff_t *cb, uint8_t *elt);
void cb_read(circular_buff_t *cb, uint8_t *elt);

#endif /* CIRCULAR_BUFF_H_ */
