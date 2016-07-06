#ifndef _ADC_H
#define _ADC_H

#include "inc/platform.h"

#define MAX_ACQUISITIONS 10

uint8_t adc_init(void);
void adc_get_averaged_values(float *Vg, float *Vd);

#endif /* _ADC_H */
