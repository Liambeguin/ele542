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
 *   Filename : circular_buff.c
 * Created on : Jul 11, 2016
 *    Authors : Jeremie Venne <jeremie.k.venne@gmail.com>
 *              Liam Beguin <liambeguin@gmail.com>
 *
 */

#include "inc/circular_buff.h"

circular_buff_t cb_init(void){

	circular_buff_t circular_buff;
	circular_buff.size = CB_MAX_SIZE;
	circular_buff.head = 0;
	circular_buff.tail = 0;

	circular_buff.isEmpty = true;
	circular_buff.isFull  = false;

	return circular_buff;
}

void cb_write(circular_buff_t *cb, uint8_t *elt){

	/*TODO: asign action if buffer is full overwrite ? */
	if (cb->isFull)
		return;

	cb->isEmpty = false;
	cb->buffer[cb->head] = *elt;
	cb->head = (cb->head + 1) % cb->size;

	if (cb->head == cb->tail)
		cb->isFull = true;
}

void cb_read(circular_buff_t *cb, uint8_t *elt){

	if (cb->isEmpty){
		return;
	}

	cb->isFull = false;
	*elt = cb->buffer[cb->tail];
	cb->tail = (cb->tail + 1) % cb->size;

	if (cb->tail == cb->head)
		cb->isEmpty = true;
}
