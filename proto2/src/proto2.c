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
 *   Filename : main.c
 * Created on : Jul 11, 2016
 *    Authors : Jeremie Venne <jeremie.k.venne@gmail.com>
 *              Liam Beguin <liambeguin@gmail.com>
 *
 */

/*
 * This is the main application
 */

#include <stdlib.h>
#include <stdio.h>
//#include "includes.h"
#include <includes.h>

#include "platform.h"
#include "motors.h"
#include "timer.h"
#include "gpio.h"
#include "uart.h"
#include "adc.h"
#include "twi.h"

/* global definitions */
extern uint8_t ovf;
volatile uint8_t robot_state = ROBOT_IDLE;
volatile uint8_t emergency_stop = 1;
volatile uint8_t emergency_stop_buttons = 1;
extern uint8_t RXcompleteFlag;
/* Desired speed and angle received from uart */
volatile float speed_d = 0, angle_d = 0;

#define STANDARD_STACK_SIZE 150
//#define DEBUG_LED

OS_STK UARTComm_Stk[STANDARD_STACK_SIZE];
static  void  UARTCommTask(void *p_arg);

OS_STK MotorControl_Stk[300];
static  void  MotorControlTask(void *p_arg);

OS_STK IO_Control_Stk[150];
static  void  IO_ControlTask(void *p_arg);

OS_STK Sonar_Stk[150];
static  void  SonarTask(void *p_arg);

extern void InitOSTimer(void);

OS_EVENT	*TimerOVF_Sem;
OS_EVENT	*UARTRx_Sem;
OS_EVENT	*Check_eStop_Sem;
OS_EVENT	*ExecuteSonar_Sem;
OS_EVENT	*TWI_Transfer_Sem;

int main(void) {
	/* Definitions to use printf with our UART */
	INT8U	err;
	FILE uart_str = FDEV_SETUP_STREAM(uart_putc, NULL, _FDEV_SETUP_RW);
	stdout = &uart_str;
	platform_init();
	
	cli();
	OSInit();
		OSTaskCreate(UARTCommTask, NULL, (OS_STK *)&UARTComm_Stk[150-1], 5); 
		OSTaskCreate(MotorControlTask, NULL, (OS_STK *)&MotorControl_Stk[300-1], 7);
		OSTaskCreate(IO_ControlTask, NULL, (OS_STK *)&IO_Control_Stk[150-1], 4);
		OSTaskCreate(SonarTask, NULL, (OS_STK *)&Sonar_Stk[150-1], 6);
		TimerOVF_Sem = OSSemCreate(1);
		UARTRx_Sem = OSSemCreate(1);
		Check_eStop_Sem = OSSemCreate(1);
		ExecuteSonar_Sem = OSSemCreate(1);
		TWI_Transfer_Sem = OSSemCreate(1);
	OSStart();
}

static  void  IO_ControlTask(void *p_arg) {
	INT8U	err;
    (void)p_arg;          // Prevent compiler warnings
	int counter = 0;
	//OSTimeDly(5000);
    while(1){
		OSSemPend(Check_eStop_Sem, 0, &err);
		/* Check e-stop button state*/
		if (!gpio_pin_read(GPIO_PORT_A, GPIO_PIN_6))
			emergency_stop_buttons = 1;
		else if (!gpio_pin_read(GPIO_PORT_A, GPIO_PIN_7))
			emergency_stop_buttons = 0;
		
		/* Check the uart e-stop state*/
		emergency_stop = emergency_stop_buttons | !(uart_get_state());
		
		/* Apply the e-stop state to the robot*/
		if(emergency_stop){
			leds_on(LED_ROBOT_IDLE);
			leds_off(LED_ROBOT_ACTIVE);
		}
		else{
			leds_off(LED_ROBOT_IDLE);
			leds_on(LED_ROBOT_ACTIVE);
		}
		#ifdef DEBUG_LED
		if(counter == 20){
			counter = 0;
			leds_toggle(LED_SONAR_R_PING);
		}
		counter++;
		#endif
	}
}

static  void  MotorControlTask(void *p_arg) {
	INT8U	err;
	/* Sonar readings */
	volatile uint16_t sonar_dist_g = 0xffff, sonar_dist_d = 0xffff;
	/* angle correction for obstacle avoidance */
	volatile float delta = 0;
	// FIXME: debug
	volatile char buffer[5] ;
	sei();
	motors_calibration();
	int counter = 0;
	// Clear calibration BIT
	gpio_pin_clear(GPIO_PORT_A, GPIO_PIN_4);
	
    (void)p_arg; // Prevent compiler warnings
    while(1){

		OSSemPend(TimerOVF_Sem, 0, &err);

		if(emergency_stop){
			motor_set_direction(FREIN, FREIN);
		}
		else{
			motors_update(speed_d, angle_d);
		}
		#ifdef DEBUG_LED
		if(counter == 100){
			counter = 0;
			leds_toggle(LED_SONAR_R_OBST);
		}
		counter++;
		#endif
    }
}

volatile float delta = 0;

static  void  SonarTask(void *p_arg) {
	INT8U	err;
	/* Sonar readings */
	volatile uint16_t sonar_dist_g = 0xffff, sonar_dist_d = 0xffff;
	/* angle correction for obstacle avoidance */
    (void)p_arg; // Prevent compiler warnings
    while(1){
		OSSemPend(ExecuteSonar_Sem, 0, &err);
			/* get distances */
			sonar_get_obstacle(&sonar_dist_g, &sonar_dist_d);
			/* Avoid obstacles */
			sonar_avoid_obstacles(sonar_dist_g, sonar_dist_d, &delta);
			/* Change range fased on desired speed */
			sonar_update_range();
			#ifndef DEBUG_LED
			leds_toggle(LED_HEARTBEAT);
			#endif
    }
}

static  void  UARTCommTask(void *p_arg) {
	INT8U	err;
    (void)p_arg; // Prevent compiler warnings
    while(1){

		/* Get values from UART teleguidage */
		OSSemPend(UARTRx_Sem, 0, &err);
			RXcompleteFlag = 0;
			#ifdef DEBUG_LED
			leds_toggle(LED_HEARTBEAT);
			#endif
			speed_d = uart_get_speed();
			angle_d = uart_get_angle() + delta;
			uart_echo();
    }
}
