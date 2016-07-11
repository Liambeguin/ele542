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
 *   Filename : adc.c
 * Created on : Jul 11, 2016
 *    Authors : Jeremie Venne <jeremie.k.venne@gmail.com>
 *              Liam Beguin <liambeguin@gmail.com>
 *
 */

#include "inc/platform.h"
#include "inc/gpio.h"
#include "inc/adc.h"

uint32_t ADC_values[2];
uint16_t ADC_counts[2];
uint8_t ADC_channelSelection = 0;

/* @brief: Analog to digital converter initialisation */
void adc_init(void){

	gpio_set_input(GPIO_PORT_A, GPIO_PIN_0 | GPIO_PIN_1 );

	ADMUX  = 0x00;
	ADCSRA = (1 << ADEN) | (1 << ADSC) | (1 << ADATE) | (1 << ADIF) | (1 << ADIE) | \
			 (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

/*
 * @brief: Perform an average of the 2 global ADC values 
 * @param ADC1_value: A pointer to an ADC value to be store
 * @param ADC0_value: A pointer to an ADC value to be store
 */
void adc_read(uint16_t* ADC1_value, uint16_t* ADC0_value){
	ADCSRA &= ~(1 << ADIE); //Disable Interrupt

	*ADC1_value = ADC_values[1] / ADC_counts[1];
	ADC_values[1] = 0;
	ADC_counts[1] = 0;

	*ADC0_value = ADC_values[0] / ADC_counts[0];
	ADC_values[0] = 0;
	ADC_counts[0] = 0;

	ADCSRA |= (1 << ADIE); //Enable Interrupt
}

/* @brief: ADC end of conversion interrupt */
ISR(ADC_vect){
	ADC_values[ADC_channelSelection & 1] += ADC;
	ADC_counts[ADC_channelSelection & 1] ++;
	ADC_channelSelection ^= 0x01;
	ADMUX = ADC_channelSelection;
}
