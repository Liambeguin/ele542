#include "inc/platform.h"
#include "inc/motors.h"
#include "inc/timer.h"
#include "inc/gpio.h"
#include "inc/adc.h"


// define global variables containing the calibration values
motor_t motor_cal_g, motor_cal_d;

void motors_init(void) {

	/* Set pins as outputs */
	gpio_set_output(GPIO_PORT_D, GPIO_PIN_6 | GPIO_PIN_7); // Motor dir D
	gpio_set_output(GPIO_PORT_D, GPIO_PIN_2 | GPIO_PIN_3); // Motor dir G
	gpio_set_output(GPIO_PORT_A, GPIO_PIN_4);              // Calibration

	gpio_set_input(GPIO_PORT_A, GPIO_PIN_2 | GPIO_PIN_3);  // Calib dir
}

/* calibrate motors following specs */
uint8_t motors_calibration(void) {

	/* set both PWM output to 0 */
	timer1_update_channel_compare(0, 0);

	/* Get Vmax+ */
	gpio_pin_set(GPIO_PORT_A, GPIO_PIN_4);
	motor_set_direction(AVANT, AVANT);
	adc_get_averaged_values(&motor_cal_g.Vmax_p, &motor_cal_d.Vmax_p);

	/* Get Vzero+ */
	gpio_pin_clear(GPIO_PORT_A, GPIO_PIN_4);
	motor_set_direction(AVANT, AVANT);
	adc_get_averaged_values(&motor_cal_g.Vzero_p, &motor_cal_d.Vzero_p);

	/* Get Vzero- */
	gpio_pin_clear(GPIO_PORT_A, GPIO_PIN_4);
	motor_set_direction(ARRIERE, ARRIERE);
	adc_get_averaged_values(&motor_cal_g.Vzero_m, &motor_cal_d.Vzero_m);

	/* Get Vmax- */
	gpio_pin_set(GPIO_PORT_A, GPIO_PIN_4);
	motor_set_direction(ARRIERE, ARRIERE);
	adc_get_averaged_values(&motor_cal_g.Vmax_m, &motor_cal_d.Vmax_m);

	DEBUG("max_p  G: %d     D: %d \n", (int)(motor_cal_g.Vmax_p*1000), (int)(motor_cal_d.Vmax_p*1000));
	DEBUG("zero_p G: %d     D: %d \n", (int)(motor_cal_g.Vzero_p*1000), (int)(motor_cal_d.Vzero_p*1000));
	DEBUG("zero_m G: %d     D: %d \n", (int)(motor_cal_g.Vzero_m*1000), (int)(motor_cal_d.Vzero_m*1000));
	DEBUG("max_m  G: %d     D: %d \n", (int)(motor_cal_g.Vmax_m*1000), (int)(motor_cal_d.Vmax_m*1000));

	return EXIT_SUCCESS;
}

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

