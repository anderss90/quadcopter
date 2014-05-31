/*
 * pid_controller.c
 *
 * Created: 23.05.2014 20:16:20
 *  Author: Anders
 */ 
#include <avr/io.h>
#include "pid_controller.h"
#define RATE_KP 1
#define RATE_KD 0
#define RATE_KI 0
#define STAB_KP 1
#define STAB_KD 0
#define STAB_KI 0
#define ELEV_KP 1
#define ELEV_KD 0
#define ELEV_KI 0
#define TIME_INTERVAL 1

double error;
double derivative;

double last_stab_error;
double last_rate_error;
double last_elev_error;

double stab_integral;
double rate_integral;
double elev_integral;


void pid_compute_output(double setpoint,double feedback,uint8_t type, double* output){
	error=setpoint-feedback;
	if (type==STAB){
		stab_integral=stab_integral+(TIME_INTERVAL*error);
		derivative= (error-last_stab_error)/TIME_INTERVAL;
		*output= STAB_KP*error+STAB_KI*stab_integral+STAB_KD*derivative;
		last_stab_error=error;
	}
	else if(type==RATE){
		rate_integral=rate_integral+(TIME_INTERVAL*error);
		derivative= (error-last_rate_error)/TIME_INTERVAL;
		*output= RATE_KP*error+RATE_KI*rate_integral+RATE_KD*derivative;
		last_rate_error=error;
	}
	else if(type==ELEV){
		elev_integral=elev_integral+(TIME_INTERVAL*error);
		derivative= (error-last_elev_error)/TIME_INTERVAL;
		*output= ELEV_KP*error+ELEV_KI*elev_integral+ELEV_KD*derivative;
		last_elev_error=error;
	}
	
}

void pid_compute_output_proportional(double setpoint,double feedback,uint8_t type, double* output){
	error=setpoint-feedback;
	if (type==STAB){
		*output= STAB_KP*error;
	}
	else if(type==RATE){
		*output= RATE_KP*error;
	}
	else if(type==ELEV){
		*output= ELEV_KP*error;
	}
}