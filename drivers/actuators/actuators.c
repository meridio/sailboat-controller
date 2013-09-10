/*
 * This code is made for Cyber sailing class spring'13
 * input is an angle between -45 deg og 45 deg
 * Interface:
 * RUDDER:
 * 	2 pwm's
 * 	2 outputs for Enable pin on the dual-h-bridge
 * 	1 adc in for Rudder angular sensor
 * 	SAIL:
 * 	2 pwm's
 * 	2 inputs from hall module (LA36)
 * 	2 outputs for Enable pin on the dual-h-bridge
 */

/* TO DO
 - Calibrate CONVERTION_VALUE
 - implement hall counter module or function
 - Enable = 0 when in neutral and ramp'd down
 */

#include "actuators.h"

int desired_angle, desired_length = 0;
int adc_value = 0; 
int actual_angle = 0;
int write_delay = 0;

FILE* file;
enum directions {
	RIGHT, LEFT, NEUTRAL, IN, OUT
};
int rudder_direction, sail_direction, last_rudder_direction = NEUTRAL;

//#define 	LIMIT_VALUE 	20
#define 	CONVERTION_VALUE 	0.11	// Needs to be calibrated!
#define 	FEEDBACK_CENTER 	1020 	// <-- ~1800/2
#define 	ERROR_MARGIEN 		2
#define 	MAX_DUTY 			60

void init_io();
void initFiles();
void read_desired_rudder_angle_values();
void read_desired_sail_length_values();
void sleep_ms();

int main() {

	initFiles();
	init_io();
	for (;;) {
		read_desired_rudder_angle_values();
		read_desired_sail_length_values();

		//read from rudder ADC
		/*ADC READ*/
		fprintf(stdout, "adc reading\n");
		file = fopen("/sys/class/hwmon/hwmon0/device/in3_input", "r");
		fscanf(file, "%d", &adc_value);
		fclose(file);

		//do convertion adc -> angle
		actual_angle = (FEEDBACK_CENTER - adc_value) * CONVERTION_VALUE; //maybe the other way around?
		fprintf(stdout, "actual_angle: %d\n", actual_angle);
		//write to disk
		if (write_delay > 10) {
			write_delay = 0;
			file = fopen("/tmp/sailboat/Rudder_Feedback", "w");
			fprintf(file, "%d", actual_angle);
			fclose(file);
		} 
		else
		write_delay++;

		fprintf(stdout, "ABS delta angle %d\n", (actual_angle - desired_angle)); 				//print out difference to desired angle
		if ((actual_angle - desired_angle) > ERROR_MARGIEN || (actual_angle - desired_angle) < ERROR_MARGIEN) {	//check if it is logical to move

			//decide direction of movement
			if (actual_angle < desired_angle) {
				fprintf(stdout, "actual_angle < desired_angle\n");					//print info
				if (rudder_direction != LEFT) {								//check if directions was changed
					rudder_direction = LEFT;							//set new direction
					ramp_up_rudder_left(find_rudder_duty(actual_angle, desired_angle));		//ramp up rudder left to desired duty
				}
				else{
					move_rudder_left(find_rudder_duty(actual_angle, desired_angle));		//set new speed of rudder to desired duty
				}
				
			} 
			else if (actual_angle > desired_angle) {
				fprintf(stdout, "actual_angle > desired_angle\n");					//print info
				if (rudder_direction != RIGHT) {							//check if directions was changed
					rudder_direction = RIGHT;							//set new direction
					ramp_up_rudder_right(find_rudder_duty(actual_angle, desired_angle));		//ramp up rudder right to desired duty
				}
				else{
					move_rudder_right(find_rudder_duty(actual_angle, desired_angle));		//set new speed of rudder to desired duty
				}	
			} 
			else {
				fprintf(stdout, "actual_angle = desired_angle!!!\n");					//print info
				if (rudder_direction != NEUTRAL) {							//check if rudder were in neutral
					rudder_direction = NEUTRAL;							//set rudder to neutral
					fprintf(stdout, "Going NEUTRAL\n");
'					stop_rudder();									//stop the rudder and prepare for ramp up
				}
			}
		} 
		else {
			//set outputs low/high? 
			fprintf(stdout, "delta angle < ERROR MARGIEN\n");
			stop_rudder();											//stop rudder in case of error margien

		}
		fprintf(stdout, "::::::::::END LOOP::::::::::\n\n");							//end
		sleep_ms(100);
	}
	return 0;
}

/*
 *	Create Empty files
 */
void initFiles() {
	fprintf(stdout, "file init\n");
	system("mkdir /tmp/actuators");
	system("echo 0 > /tmp/sailboat/Rudder_Feedback");
	system("echo 0 > /tmp/sailboat/Navigation_System_Rudder");
	system("echo 0 > /tmp/sailboat/Navigation_System_Sail");
}

void init_io() {
	//configure io and pwm
	fprintf(stdout, "IO init\n");
	/*
	 * 	2 pwm's
	 * 	-connected to EN-pins
	 * 	1 adc
	 * 	-connected to rudder feedback
	 * 	2 inputs from hall module (LA36)
	 * 	-hall A and B ___ THIS NEED TO BE IN A KERNEL MODULE! fast signals!
	 * 	4 outputs for dual-h-bridge
	 * 	-RPWM, LPWM for A and B motor
	 */
	//guide used: wiki.gumstix.org//index,php?title=GPIO
	//the most faulty non-informative-ish wiki guide
	/*                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 
	 * 
	 */
	system("devmem2 0x480021c8 h 0x10c"); //GPIO 171
	system("devmem2 0x480021ca h 0x10c"); //GPIO 172
	//system("devmem2 0x480021cc h 0x10c"); //GPIO 173
	//system("devmem2 0x480021ce h 0x10c"); //GPIO 174
	system("echo 171 > /sys/class/gpio/export");
	system("echo 172 > /sys/class/gpio/export");
	system("echo out > /sys/class/gpio/gpio171/direction");
	system("echo out > /sys/class/gpio/gpio172/direction");

	system("echo 1 > /sys/class/gpio/gpio171/value");
	system("echo 1 > /sys/class/gpio/gpio172/value");

	//install pwm module
	system("insmod pwm.ko");

}

void read_desired_rudder_angle_values() {
	fprintf(stdout, "reading desired angle\n");
	file = fopen("/tmp/sailboat/Navigation_System_Rudder", "r");
	fscanf(file, "%d", &desired_angle);
	fclose(file);
	fprintf(stdout, "desired angle: %d\n", desired_angle);
}

void read_desired_sail_length_values() {
	fprintf(stdout, "reading desired length\n");
	file = fopen("/tmp/sailboat/Navigation_System_Sail", "r");
	fscanf(file, "%d", &desired_length);
	fclose(file);
	fprintf(stdout, "desired length: %d\n", desired_length);
}
void move_rudder_left(int duty){
	file = fopen("/dev/pwm10", "w");
	fprintf(file, "%d", 0);
	fclose(file);
	file = fopen("/dev/pwm11", "w");
	fprintf(file, "%d", duty);
	fclose(file);
	fprintf(stdout, "going left, duty: %d\n", duty);
}
void move_rudder_right(int duty){
	file = fopen("/dev/pwm11", "w");
	fprintf(file, "%d", 0);
	fclose(file);
	file = fopen("/dev/pwm10", "w");
	fprintf(file, "%d", duty);
	fclose(file);
	fprintf(stdout, "going right, duty: %d\n", duty);
}
void ramp_up_rudder_left(int final_duty){
	for(int duty = 0; duty <= final_duty; duty += 10) {
		fprintf(stdout, "ramping up ~ duty: %d\n", duty);
		move_rudder_left(duty)
		sleep_ms(5);
	}
}
void ramp_up_rudder_right(int final_duty){
	for(int duty = 0; duty <= final_duty; duty += 10) {
		fprintf(stdout, "ramping up ~ duty: %d\n", duty);
		move_rudder_right(duty)
		sleep_ms(5);
	}
}
void stop_rudder(void){
	file = fopen("/dev/pwm11", "w");
	fprintf(file, "%d", 0);
	fclose(file);
	file = fopen("/dev/pwm10", "w");
	fprintf(file, "%d", 0);
	fclose(file);
	fprintf(stdout, "rudder stopped");
	sleep_ms(5);
}

int find_rudder_duty(int actual_angle, int desired_angle){
	int duty = MAX_DUTY;										//setting duty to highest speed
	if ((actual_angle - desired_angle) < 5 || (actual_angle - desired_angle) < -5) {		//check if within 5 degrees of desired angle 
		duty = 20;										//setting duty to lowerst speed
	} 
	else if ((actual_angle - desired_angle) < 10 || (actual_angle - desired_angle) < -10) {		//check if within 10 degrees of desired angle 
		duty = 30;										//setting duty to lower speed 
	}
}

void sleep_ms(int ms) {
	usleep(ms * 1000); //convert to microseconds
	return;
}
