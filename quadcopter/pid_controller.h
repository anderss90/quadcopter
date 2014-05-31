/*
 * pid_controller.h
 *
 * Created: 23.05.2014 20:16:29
 *  Author: Anders
 */ 


#ifndef PID_CONTROLLER_H_
#define PID_CONTROLLER_H_

enum CONTROLLER_TYPE{STAB,RATE,ELEV};

void pid_compute_output_proportional(double setpoint,double feedback,uint8_t type, double* output);


#endif /* PID_CONTROLLER_H_ */