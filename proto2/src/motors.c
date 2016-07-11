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
 *   Filename : motors.c
 * Created on : Jul 11, 2016
 *    Authors : Jeremie Venne <jeremie.k.venne@gmail.com>
 *              Liam Beguin <liambeguin@gmail.com>
 *
 */

#include "inc/platform.h"
#include "inc/motors.h"
#include "inc/timer.h"
#include "inc/gpio.h"
#include "inc/adc.h"

#define NB_CALIBRATION_ITERATION 150

uint32_t Positive_Vmax_Left   = 0;
uint32_t Positive_Vzero_Left  = 0;
uint32_t Negative_Vzero_Left  = 0;
uint32_t Negative_Vmax_Left   = 0;
uint32_t Positive_Vmax_Right  = 0;
uint32_t Positive_Vzero_Right = 0;
uint32_t Negative_Vzero_Right = 0;
uint32_t Negative_Vmax_Right  = 0;

uint16_t Speed_L_raw;
uint16_t Speed_R_raw;

extern uint8_t ovf;


/* @brief: Initialisation of ports needed to use the left and right motor */
void motors_init(void) {
	/* Set pins as outputs */
	gpio_set_output(GPIO_PORT_D, GPIO_PIN_6 | GPIO_PIN_7); // Motor dir D
	gpio_set_output(GPIO_PORT_D, GPIO_PIN_2 | GPIO_PIN_3); // Motor dir G
	gpio_set_output(GPIO_PORT_A, GPIO_PIN_4);              // Calibration

	gpio_set_input(GPIO_PORT_A, GPIO_PIN_2 | GPIO_PIN_3);  // Calib dir
}


/*
 * @brief: Motor calibration procedure
 *
 * @return: SUCCESS or FAIL
 */
uint8_t motors_calibration(void) {

	uint8_t Calibration_procedure_block = 1;
	uint8_t calibration_steps = 0;
	uint16_t delay_counter = 0;

	/* set both PWM output to 0 */
	timer1_update_channel_compare(0, 0);


	while(Calibration_procedure_block) {
		/* Get Vmax+ */
		if(ovf && calibration_steps == 0){
			gpio_pin_set(GPIO_PORT_A, GPIO_PIN_4);
			motor_set_direction(AVANT, AVANT);
			delay_counter ++;
			ovf = 0;
			adc_read(&Speed_L_raw, &Speed_R_raw);
			Positive_Vmax_Left += Speed_L_raw;
			Positive_Vmax_Right += Speed_R_raw;

			if(delay_counter >= NB_CALIBRATION_ITERATION){
				Positive_Vmax_Left /= NB_CALIBRATION_ITERATION;
				Positive_Vmax_Right /= NB_CALIBRATION_ITERATION;
				delay_counter = 0;
				calibration_steps ++;
			}
		}
		/* Get Vzero+ */
		if(ovf && calibration_steps == 1){
			gpio_pin_clear(GPIO_PORT_A, GPIO_PIN_4);
			motor_set_direction(AVANT, AVANT);
			delay_counter ++;
			ovf = 0;
			adc_read(&Speed_L_raw, &Speed_R_raw);
			Positive_Vzero_Left += Speed_L_raw;
			Positive_Vzero_Right += Speed_R_raw;

			if(delay_counter >= NB_CALIBRATION_ITERATION){
				Positive_Vzero_Left /= NB_CALIBRATION_ITERATION;
				Positive_Vzero_Right /= NB_CALIBRATION_ITERATION;
				delay_counter = 0;
				calibration_steps ++;
			}
		}
		/* Get Vmax- */
		if(ovf && calibration_steps == 2){
			gpio_pin_set(GPIO_PORT_A, GPIO_PIN_4);
			motor_set_direction(ARRIERE, ARRIERE);
			delay_counter ++;
			ovf = 0;
			adc_read(&Speed_L_raw, &Speed_R_raw);
			Negative_Vmax_Left += Speed_L_raw;
			Negative_Vmax_Right += Speed_R_raw;

			if(delay_counter >= NB_CALIBRATION_ITERATION){
				Negative_Vmax_Left /= NB_CALIBRATION_ITERATION;
				Negative_Vmax_Right /= NB_CALIBRATION_ITERATION;
				delay_counter = 0;
				calibration_steps ++;
			}
		}
		/* Get Vzero- */
		if(ovf && calibration_steps == 3){
			gpio_pin_clear(GPIO_PORT_A, GPIO_PIN_4);
			motor_set_direction(ARRIERE, ARRIERE);
			delay_counter ++;
			ovf = 0;
			adc_read(&Speed_L_raw, &Speed_R_raw);
			Negative_Vzero_Left += Speed_L_raw;
			Negative_Vzero_Right += Speed_R_raw;

			if(delay_counter >= NB_CALIBRATION_ITERATION){
				Negative_Vzero_Left /= NB_CALIBRATION_ITERATION;
				Negative_Vzero_Right /= NB_CALIBRATION_ITERATION;
				delay_counter = 0;
				Calibration_procedure_block = 0;
			}
		}
	}
	// FIXME
	DEBUG("%d %d", Positive_Vmax_Left,  Positive_Vmax_Right);
	DEBUG("%d %d", Positive_Vzero_Left, Positive_Vzero_Right);
	DEBUG("%d %d", Negative_Vzero_Left, Negative_Vzero_Right);
	DEBUG("%d %d", Negative_Vmax_Left,  Negative_Vmax_Right);

	return EXIT_SUCCESS;
}

/* @brief: Configure the direction of both left and right motor
 *
 * @param dir_D: Right motor direction
 * @param dir_G: Left motor direction
 */
void motor_set_direction(motor_dir_t dir_D, motor_dir_t dir_G) {
	switch(dir_D) {
		case NEUTRE:
			gpio_pin_clear(GPIO_PORT_D, GPIO_PIN_6 | GPIO_PIN_7);
			break;
		case AVANT:
			gpio_pin_set(GPIO_PORT_D, GPIO_PIN_6);
			gpio_pin_clear(GPIO_PORT_D, GPIO_PIN_7);
			break;
		case ARRIERE:
			gpio_pin_set(GPIO_PORT_D, GPIO_PIN_7);
			gpio_pin_clear(GPIO_PORT_D, GPIO_PIN_6);
			break;
		case FREIN:
			gpio_pin_set(GPIO_PORT_D, GPIO_PIN_6 | GPIO_PIN_7);
			break;
	}

	switch(dir_G) {
		case NEUTRE:
			gpio_pin_clear(GPIO_PORT_D, GPIO_PIN_2 | GPIO_PIN_3);
			break;
		case AVANT:
			gpio_pin_set(GPIO_PORT_D, GPIO_PIN_2);
			gpio_pin_clear(GPIO_PORT_D, GPIO_PIN_3);
			break;
		case ARRIERE:
			gpio_pin_set(GPIO_PORT_D, GPIO_PIN_3);
			gpio_pin_clear(GPIO_PORT_D, GPIO_PIN_2);
			break;
		case FREIN:
			gpio_pin_set(GPIO_PORT_D, GPIO_PIN_2 | GPIO_PIN_3);
			break;
	}
}

/* @brief: Update the speed of both the left and right motor from
 *		   a desired speed and angle
 *
 * @param speed: speed command (0 to 200)
 * @param angle: angle command (0 to 180)
 */
void motors_update(float speed, float angle) {
	float PWMg, PWMd;
	float speed_L_temp, speed_R_temp;
	float speedA, speedB;

	adc_read(&Speed_L_raw, &Speed_R_raw);

	if((PINA >> PA2) & 1) {
		speed_L_temp = (float)Speed_L_raw - (float)Negative_Vzero_Left;
		speed_L_temp = speed_L_temp < 0 ? 0 : speed_L_temp;
		speed_L_temp /= (float)(Negative_Vmax_Left-Negative_Vzero_Left);
		speed_L_temp = speed_L_temp > 1 ? 1 : speed_L_temp;
		speed_L_temp *= -1;
	} else {
		speed_L_temp = (float)Speed_L_raw - (float)Positive_Vzero_Left;
		speed_L_temp = speed_L_temp < 0 ? 0 : speed_L_temp;
		speed_L_temp /= (float)(Positive_Vmax_Left-Positive_Vzero_Left);
		speed_L_temp = speed_L_temp > 1 ? 1 : speed_L_temp;
	}

	if((PINA >> PA3) & 1) {
		speed_R_temp = (float)Speed_R_raw - (float)Negative_Vzero_Right;
		speed_R_temp = speed_R_temp < 0 ? 0 : speed_R_temp;
		speed_R_temp /= (float)(Negative_Vmax_Right-Negative_Vzero_Right);
		speed_R_temp = speed_R_temp > 1 ? 1 : speed_R_temp;
		speed_R_temp *= -1;
	} else {
		speed_R_temp = (float)Speed_R_raw - (float)Positive_Vzero_Right;
		speed_R_temp = speed_R_temp < 0 ? 0 : speed_R_temp;
		speed_R_temp /= (float)(Positive_Vmax_Right-Positive_Vzero_Right);
		speed_R_temp = speed_R_temp > 1 ? 1 : speed_R_temp;
	}


	CalculPWM(speed, angle, speed_L_temp, speed_R_temp, &PWMg, &PWMd);

	if(PWMg < 0) {
		PWMg *= -1.0;
		gpio_pin_set(GPIO_PORT_D, GPIO_PIN_3);
		gpio_pin_clear(GPIO_PORT_D, GPIO_PIN_2);
	} else {
		gpio_pin_set(GPIO_PORT_D, GPIO_PIN_2);
		gpio_pin_clear(GPIO_PORT_D, GPIO_PIN_3);
	}

	if(PWMd < 0) {
		PWMd *= -1.0;
		gpio_pin_set(GPIO_PORT_D, GPIO_PIN_7);
		gpio_pin_clear(GPIO_PORT_D, GPIO_PIN_6);
	} else {
		gpio_pin_set(GPIO_PORT_D, GPIO_PIN_6);
		gpio_pin_clear(GPIO_PORT_D, GPIO_PIN_7);
	}

	if (speed == 0) {
		speedA = 0;
		speedB = 0;
	} else {
		speedA = ICR1;
		speedA *= PWMg;

		speedB = ICR1;
		speedB *= PWMd;
	}

	timer1_update_channel_compare(speedB, speedA);
}

/* @brief: Fonction given by Bruno De Kelper and Louis-Bernard Lagueux.
		   See below for description.
 */
void CalculPWM(float Vitesse_D, float Angle_D, float Vg, float Vd, \
		float *Duty_G, float *Duty_D) {
	/*
	 * Dans cette fonction, la valeur des duty cycle pour chaque moteur est
	 * calcule. Ce calcul est effectue a l'aide de la vitesse desiree, de
	 * l'angle desrire ainsi que de la vitesse (mesuree avec CalculVitesses)
	 * et l'angle actuel.
	 *
	 * Vitesse_D = Vitesse desiree
	 * Angle_D   = Angle   desire
	 * Vg        = Vitesse du Moteur Gauche
	 * Vd        = Vitesse du Moteur Droit
	 * Duty_D    =
	 */

	static float Angle       = 0.0;
	static float ErreurAngle = 0.0;
	static float W           = 0.0;
	static float Old_W       = 0.0;
	static float Vt          = 0.0;
	static float Ut          = 0.0;
	static float Ua          = 0.0;
	static int 	 Signe_Ua    = 0;
	static int 	 Signe_Ut    = 0;

	/* Regarde les limites (-1.0 à 1.0) */
	Vg = (Vg > 1.0) ? 1.0 : ((Vg < -1.0) ? -1.0 : Vg);
	Vd = (Vd > 1.0) ? 1.0 : ((Vd < -1.0) ? -1.0 : Vd);

	Old_W = W;
	W     = 0.5 * (Vmax / RAYON) * (Vd - Vg);
	Vt    = 0.5 * (Vd + Vg);

	Angle = Angle + (0.5) * TS * (W + Old_W);
	Angle = (Angle > 2.0 * Pi) ? Angle - 2*Pi : ((Angle < 0.0) ? \
			Angle + 2*Pi : Angle); /* Angle entre 0 et 2 pi */

	ErreurAngle = ((Angle_D >= Pi + Angle) ? Angle_D - 2*Pi : \
			((Angle_D <= -Pi + Angle) ? Angle_D + 2*Pi : Angle_D))-Angle;

	Ut = -H11*Vt + H12*Vitesse_D;
	Ua = H21*ErreurAngle - H22*W;

	Signe_Ua = (Ua >= 0.0)   ? 1 : -1;
	Signe_Ut = (Ut >= 0.0) ? 1 : -1;

	Ua = (Signe_Ua*Ua > 1.0) ? Signe_Ua*1.0 : \
		 ((Signe_Ua*Ua <= 0.05) ? 0.0 : Ua);

	Ut = (Signe_Ut*Ut > 1.0) ? Signe_Ut*1.0 : Ut;

	Ut = ((Signe_Ut*Ut) > (1.0 - Signe_Ua*Ua)) ? \
		 Signe_Ut*(1.0 - Signe_Ua*Ua) : Ut;

	*Duty_D = (Ut+Ua);
	*Duty_G = (Ut-Ua);

	*Duty_D = (*Duty_D > 0.99) ? 0.99 : ((*Duty_D < -0.99) ? -0.99 : *Duty_D);
	*Duty_G = (*Duty_G > 0.99) ? 0.99 : ((*Duty_G < -0.99) ? -0.99 : *Duty_G);
}

