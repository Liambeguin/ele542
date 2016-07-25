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
#include "inc/includes.h"
#include "inc/platform.h"
#include "inc/motors.h"
#include "inc/timer.h"
#include "inc/gpio.h"
#include "inc/uart.h"
#include "inc/adc.h"
#include "inc/twi.h"

/* global definitions */
volatile uint8_t emergency_stop = 1;
volatile uint8_t emergency_stop_buttons = 1;
/* Desired speed and angle received from uart */
volatile float speed_d = 0, angle_d = 0;
float delta = 0;

/* Tasks declaration */
#define STANDARD_STACK_SIZE 200
OS_STK UARTComm_Stk[STANDARD_STACK_SIZE];
OS_STK MotorControl_Stk[STANDARD_STACK_SIZE];
OS_STK IO_Control_Stk[STANDARD_STACK_SIZE];
OS_STK Sonar_Stk[STANDARD_STACK_SIZE];
static void UARTCommTask(void *p_arg);
static void MotorControlTask(void *p_arg);
static void IO_ControlTask(void *p_arg);
static void SonarTask(void *p_arg);

extern void InitOSTimer(void);

/* Synchronisation Semaphores */
OS_EVENT *TimerOVF_Sem;
OS_EVENT *UARTRx_Sem;
OS_EVENT *TWI_Transfer_Sem;

/* Access Protection Semaphores */
OS_EVENT *E_StopAccess_Sem;
OS_EVENT *UARTDataAcces_Sem;
OS_EVENT *TWIDataAccess_Sem;

/* TODO: move init stuff inside platform_init */
int main(void) {
	/* Definitions to use printf with our UART */
	FILE uart_str = FDEV_SETUP_STREAM(uart_putc, NULL, _FDEV_SETUP_RW);
	stdout = &uart_str;
	platform_init();
	cli();
	OSInit();
	/* Tasks */
	OSTaskCreate(UARTCommTask,     NULL, \
			(OS_STK *)&UARTComm_Stk[STANDARD_STACK_SIZE-1], 5);
	OSTaskCreate(MotorControlTask, NULL, \
			(OS_STK *)&MotorControl_Stk[STANDARD_STACK_SIZE-1], 7);
	OSTaskCreate(IO_ControlTask,   NULL, \
			(OS_STK *)&IO_Control_Stk[STANDARD_STACK_SIZE-1], 4);
	OSTaskCreate(SonarTask,        NULL, \
			(OS_STK *)&Sonar_Stk[STANDARD_STACK_SIZE-1], 6);

	/* Synchronisation Semaphores */
	TimerOVF_Sem = OSSemCreate(1);
	UARTRx_Sem = OSSemCreate(1);
	TWI_Transfer_Sem = OSSemCreate(1);
	/* Access Protection Semaphores */
	E_StopAccess_Sem = OSSemCreate(1);
	UARTDataAcces_Sem = OSSemCreate(1);
	TWIDataAccess_Sem = OSSemCreate(1);
	OSStart();
}

/* @brief: Task that monitors emergency state and controls status LEDs */
static void IO_ControlTask(void *p_arg) {
    UNUSED(p_arg);
	uint8_t err;
    while(1) {
		OSSemPend(E_StopAccess_Sem, 0, &err);
		/* Check e-stop button state*/
		if (!gpio_pin_read(GPIO_PORT_A, GPIO_PIN_6))
			emergency_stop_buttons = 1;
		else if (!gpio_pin_read(GPIO_PORT_A, GPIO_PIN_7))
			emergency_stop_buttons = 0;

		/* Check the uart e-stop state*/
		emergency_stop = emergency_stop_buttons | !(uart_get_state());

		/* Apply the e-stop state to the robot*/
		if(emergency_stop) {
			leds_on(LED_ROBOT_IDLE);
			leds_off(LED_ROBOT_ACTIVE);
		} else {
			leds_off(LED_ROBOT_IDLE);
			leds_on(LED_ROBOT_ACTIVE);
		}
		OSSemPost(E_StopAccess_Sem);
		OSTimeDly(25);
	}
}

/* @brief: Task used to control motors */
static  void  MotorControlTask(void *p_arg) {
    UNUSED(p_arg);
	uint8_t err;
	sei();
	motors_calibration();
	uint8_t emergency_stop_temp = 0;
	float speed_d_temp, angle_d_temp;
	// Clear calibration BIT
	gpio_pin_clear(GPIO_PORT_A, GPIO_PIN_4);

    while(1) {
		OSSemPend(TimerOVF_Sem, 0, &err);
		OSSemPend(E_StopAccess_Sem, 0, &err);
		emergency_stop_temp = emergency_stop;
		OSSemPost(E_StopAccess_Sem);

		if(emergency_stop_temp) {
			motor_set_direction(FREIN, FREIN);
		} else {
			OSSemPend(UARTDataAcces_Sem, 0, &err);
			speed_d_temp = speed_d;
			angle_d_temp = angle_d;
			OSSemPost(UARTDataAcces_Sem);
			motors_update(speed_d_temp, angle_d_temp);
		}
	}
}

/* @brief: Task used to detect and avoid obstacles using sonars */
static void SonarTask(void *p_arg) {
    UNUSED(p_arg);
	/* Sonar readings */
	uint16_t sonar_dist_g = 0xffff, sonar_dist_d = 0xffff;
    while(1) {
		/* get distances */
		sonar_get_obstacle(&sonar_dist_g, &sonar_dist_d);
		/* Avoid obstacles */
		sonar_avoid_obstacles(sonar_dist_g, sonar_dist_d, &delta);
		/* Change range fased on desired speed */
		sonar_update_range();
		OSTimeDly(10);
    }
}

/* @brief: Task used to communicate with the remote control system */
static void UARTCommTask(void *p_arg) {
    UNUSED(p_arg);
	uint8_t err;
	InitOSTimer();
    while(1) {
		/* Get values from UART teleguidage */
		OSSemPend(UARTRx_Sem, 0, &err);
		OSSemPend(UARTDataAcces_Sem, 0, &err);
		speed_d = uart_get_speed();
		OSSemPend(TWIDataAccess_Sem, 0, &err);
		angle_d = uart_get_angle() + delta;
		OSSemPost(TWIDataAccess_Sem);
		OSSemPost(UARTDataAcces_Sem);
		uart_echo();
    }
}
