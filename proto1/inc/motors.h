#ifndef _MOTORS_H
#define _MOTORS_H

#include "inc/platform.h"


#define Pi      (3.1415926535897932384626433832795)
#define RAYON   (9.525)
#define TS      (0.005)
#define Vmax    (80.0)
#define Tau     (0.5)

#define H11		(3.90148347975678)
#define H12		(4.90148347975678)

#define H21		(1.1613504)
#define H22		(0.5806746734)

#define Fpwm (10000)



typedef enum {
	NEUTRE,
	AVANT,
	ARRIERE,
	FREIN,
} motor_dir_t;

typedef struct {
	float Vmax_p;
	float Vzero_p;
	float Vzero_m;
	float Vmax_m;
} motor_t;

void CalculPWM(float Vitesse_D, float Angle_D, float Vg, float Vd, \
		float *Duty_G, float *Duty_D);

void motors_init(void);
uint8_t motors_calibration(void);
void motor_set_direction(motor_dir_t dir_D, motor_dir_t dir_G);



#endif /* _MOTORS_H */
