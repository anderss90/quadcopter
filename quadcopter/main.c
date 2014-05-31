#define F_CPU 16000000
#define __AVR_ATmega168__

#include <stdlib.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <math.h>  //include libm

#include "mpu6050/mpu6050.h"

#define UART_BAUD_RATE 19200
//#define UART_UBBR_VALUE F_CPU/((UART_BAUD_RATE*16)-1)
#define UART_UBBR_VALUE 51
#include "uart/uart.h"
#include "pid_controller.h"
#include "input.h"




#if MPU6050_GETATTITUDE == 1 || MPU6050_GETATTITUDE == 2 || MPU6050_GETATTITUDE == 0
long *ptr = 0;
double qw = 1.0f;
double qx = 0.0f;
double qy = 0.0f;
double qz = 0.0f;
double roll = 0.0f;
double pitch = 0.0f;
double yaw = 0.0f;
double e_acc=0.0f;
double axg = 0;
double ayg = 0;
double azg = 0;
double gxds = 0;
double gyds = 0;
double gzds = 0;
int16_t ax = 0;
int16_t ay = 0;
int16_t az = 0;
int16_t gx = 0;
int16_t gy = 0;
int16_t gz = 0;
#endif

double sp_roll;
double sp_pitch;
double sp_yaw;
double sp_roll_dot;
double sp_pitch_dot;
double sp_yaw_dot;
double output_roll; //positive is counter clockwise
double output_pitch; //positive is counter clockwise
double output_yaw; //positive is counter clockwise

int motor_fl_speed; //front left
int motor_fr_speed; //front right
int motor_bl_speed; //back left
int motor_br_speed; //back right

uint8_t combined_thrust;
char itmp[10];



void calculate_elevation_force(double *e_acc,double roll,double pitch,double axg,double ayg,double azg);

void debug_printing(){
	//sensor readings
	
	uart_puts("sensors\n");
	dtostrf(roll, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
	dtostrf(pitch, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
	dtostrf(yaw, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
	dtostrf(gxds, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
	dtostrf(gyds, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
	dtostrf(gzds, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
	uart_puts("\r\n");
	
	/*
	// rates
	uart_puts("rates\n");
	dtostrf(sp_roll_dot, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
	dtostrf(sp_pitch_dot, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
	dtostrf(sp_yaw_dot, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
	uart_puts("\r\n");
	
	//acc outputs
	uart_puts("outputs\n");
	dtostrf(output_roll, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
	dtostrf(output_pitch, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
	dtostrf(output_yaw, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
	uart_puts("\r\n");
	uart_puts("\r\n");
	//_delay_ms(1000);
	//_delay_ms(400);
	*/
}

double rad_to_deg(double rad){
	return (rad/3.14)*180;
}

double deg_to_rad(double deg){
	return (deg/180)*3.14;
}
void controller_procedure(){
	
	//all these are euler angles
	
	//read sensors
	mpu6050_getRollPitchYaw(&roll, &pitch, &yaw);  //euler angles in radians
	//mpu6050_getConvData(&axg, &ayg, &azg, &gxds, &gyds, &gzds); //degrees /s
	//mpu6050_getQuaternion(&qw, &qx, &qy, &qz); 
	
	
	
	//with ==2
	/*
	if(mpu6050_getQuaternionWait(&qw, &qx, &qy, &qz)) {
		mpu6050_getRollPitchYaw(qw, qx, qy, qz, &roll, &pitch, &yaw);
	}
	*/
	
	_delay_ms(10);
	
	
	
	
	//get setpoints
	input_get_setpoints(&sp_roll,&sp_pitch,&sp_yaw);
	
	//limit setpoints
	
	//calculate rates. output is in dg/s(?) output will be in range [-1.5,1.5]*KP
	pid_compute_output_proportional(sp_roll,roll,STAB,&sp_roll_dot);
	pid_compute_output_proportional(sp_pitch,pitch,STAB,&sp_pitch_dot);
	pid_compute_output_proportional(sp_yaw,yaw,STAB,&sp_yaw_dot);
	
	
	//calculate manipulation values (pådrag). output in dg/s^2.
	pid_compute_output_proportional(sp_roll_dot,deg_to_rad(gxds),RATE,&output_roll);
	pid_compute_output_proportional(sp_pitch_dot,deg_to_rad(gyds),RATE,&output_pitch);
	pid_compute_output_proportional(sp_yaw_dot,deg_to_rad(gzds),RATE,&output_yaw);
	
	//calculate combined thrust
	combined_thrust=10;
	
	//convert outputs to sensible values
	
	//convert outputs to propeller force
	motor_fl_speed=combined_thrust-output_roll+output_pitch+output_yaw;
	motor_fr_speed=combined_thrust+output_roll+output_pitch-output_yaw;
	motor_bl_speed=combined_thrust-output_roll-output_pitch-output_yaw;
	motor_br_speed=combined_thrust+output_roll-output_pitch+output_yaw;
	
	//control upper/lower limits
	
	//send values to motors
	debug_printing();
	
}


void old_loop(){
		#if MPU6050_GETATTITUDE == 0
		mpu6050_getRawData(&ax, &ay, &az, &gx, &gy, &gz);
		mpu6050_getConvData(&axg, &ayg, &azg, &gxds, &gyds, &gzds);
		#endif

		#if MPU6050_GETATTITUDE == 1
		mpu6050_getQuaternion(&qw, &qx, &qy, &qz);
		mpu6050_getRollPitchYaw(&roll, &pitch, &yaw);
		_delay_ms(10);
		#endif

		#if MPU6050_GETATTITUDE == 2
		if(mpu6050_getQuaternionWait(&qw, &qx, &qy, &qz)) {
			mpu6050_getRollPitchYaw(qw, qx, qy, qz, &roll, &pitch, &yaw);
		}
		_delay_ms(10);
		#endif

		#if MPU6050_GETATTITUDE == 0
		char itmp[10];
		ltoa(ax, itmp, 10); uart_putc(' '); uart_puts(itmp); uart_putc(' ');
		ltoa(ay, itmp, 10); uart_putc(' '); uart_puts(itmp); uart_putc(' ');
		ltoa(az, itmp, 10); uart_putc(' '); uart_puts(itmp); uart_putc(' ');
		ltoa(gx, itmp, 10); uart_putc(' '); uart_puts(itmp); uart_putc(' ');
		ltoa(gy, itmp, 10); uart_putc(' '); uart_puts(itmp); uart_putc(' ');
		ltoa(gz, itmp, 10); uart_putc(' '); uart_puts(itmp); uart_putc(' ');
		uart_puts("\r\n");

		dtostrf(axg, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
		dtostrf(ayg, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
		dtostrf(azg, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
		dtostrf(gxds, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
		dtostrf(gyds, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
		dtostrf(gzds, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
		uart_puts("\r\n");

		uart_puts("\r\n");

		_delay_ms(1000);
		#endif

		#if MPU6050_GETATTITUDE == 1 || MPU6050_GETATTITUDE == 2

		//quaternion
		ptr = (long *)(&qw);
		uart_putc(*ptr);
		uart_putc(*ptr>>8);
		uart_putc(*ptr>>16);
		uart_putc(*ptr>>24);
		ptr = (long *)(&qx);
		uart_putc(*ptr);
		uart_putc(*ptr>>8);
		uart_putc(*ptr>>16);
		uart_putc(*ptr>>24);
		ptr = (long *)(&qy);
		uart_putc(*ptr);
		uart_putc(*ptr>>8);
		uart_putc(*ptr>>16);
		uart_putc(*ptr>>24);
		ptr = (long *)(&qz);
		uart_putc(*ptr);
		uart_putc(*ptr>>8);
		uart_putc(*ptr>>16);
		uart_putc(*ptr>>24);

		//roll pitch yaw
		ptr = (long *)(&roll);
		uart_putc(*ptr);
		uart_putc(*ptr>>8);
		uart_putc(*ptr>>16);
		uart_putc(*ptr>>24);
		ptr = (long *)(&pitch);
		uart_putc(*ptr);
		uart_putc(*ptr>>8);
		uart_putc(*ptr>>16);
		uart_putc(*ptr>>24);
		ptr = (long *)(&yaw);
		uart_putc(*ptr);
		uart_putc(*ptr>>8);
		uart_putc(*ptr>>16);
		uart_putc(*ptr>>24);

		uart_putc('\n');
		#endif
}

void new_loop(){
		#if MPU6050_GETATTITUDE == 1 
		
		//mpu6050_getQuaternion(&qw, &qx, &qy, &qz);
		mpu6050_getRawData(&ax, &ay, &az, &gx, &gy, &gz);
		_delay_ms(10);
		mpu6050_getConvData(&axg, &ayg, &azg, &gxds, &gyds, &gzds);
		_delay_ms(10);
		//mpu6050_getRollPitchYaw(&roll, &pitch, &yaw);
		//calculate_elevation_force(&e_acc,roll,pitch,axg,ayg,azg);
		
		char itmp[10];
		dtostrf(axg, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
		dtostrf(ayg, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
		dtostrf(azg, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
		dtostrf(gxds, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
		dtostrf(gyds, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
		dtostrf(gzds, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
		uart_puts("\r\n");
		//_delay_ms(1000);
		#endif
		
		#if MPU6050_GETATTITUDE == 0
		
		mpu6050_getConvData(&axg, &ayg, &azg, &gxds, &gyds, &gzds);
		mpu6050_getRollPitchYaw(&roll, &pitch, &yaw);
		//calculate_elevation_force(&e_acc,roll,pitch,axg,ayg,azg);
		
		char itmp[10];
		ltoa(ax, itmp, 10); uart_putc(' '); uart_puts(itmp); uart_putc(' ');
		ltoa(ay, itmp, 10); uart_putc(' '); uart_puts(itmp); uart_putc(' ');
		ltoa(az, itmp, 10); uart_putc(' '); uart_puts(itmp); uart_putc(' ');
		ltoa(gx, itmp, 10); uart_putc(' '); uart_puts(itmp); uart_putc(' ');
		ltoa(gy, itmp, 10); uart_putc(' '); uart_puts(itmp); uart_putc(' ');
		ltoa(gz, itmp, 10); uart_putc(' '); uart_puts(itmp); uart_putc(' ');
		uart_puts("\r\n");

		dtostrf(axg, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
		dtostrf(ayg, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
		dtostrf(azg, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
		dtostrf(gxds, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
		dtostrf(gyds, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
		dtostrf(gzds, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
		uart_puts("\r\n");

		uart_puts("\r\n");

		_delay_ms(1000);
		#endif
}

int main(void) {

	

    //init uart
	//uart_init(UART_BAUD_SELECT(UART_BAUD_RATE,F_CPU));
	uart_init(UART_UBBR_VALUE);

	//init interrupt
	sei();

	//init mpu6050
	mpu6050_init();
	_delay_ms(50);

	//init mpu6050 dmp processor
	#if MPU6050_GETATTITUDE == 2
	mpu6050_dmpInitialize();
	mpu6050_dmpEnable();
	_delay_ms(10);
	#endif
	for(;;){
		controller_procedure();
		//new_loop();
		//old_loop();
	}
	
}

void calculate_elevation_force(double *e_acc,double roll,double pitch,double axg,double ayg,double azg){
	*e_acc=axg*sin(roll)+ayg*sin(pitch)+azg*sin(roll)*sin(pitch);
}