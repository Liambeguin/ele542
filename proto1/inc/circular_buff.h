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
