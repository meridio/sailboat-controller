/*
 * This code is made for Cyber sailing class spring'13
 * input is an angle between -45 deg og 45 deg
 * Interface:
 * RUDDER:
 * 	2 pwm's
 * 	1 outputs for Enable pin on the dual-h-bridge
 * 	1 adc in for Rudder angular sensor
 * 	SAIL:
 * 	2 pwm's
 * 	2 inputs from hall module (LA36)
 * 	1 outputs for Enable pin on the dual-h-bridge
 */

/* TO DO
 - Calibrate CONVERTION_VALUE
 - implement hall counter module or function
 - Enable = 1 when in neutral and ramp'd down ? only an idea.. lets see on the current flow
 */

#include "actuators.h"

int desired_angle, desired_length = 0;
int adc_value = 0;
int actual_angle = 0;
int actual_length = 0;
int write_delay = 0;
int duty = 0;


FILE* file;
enum directions {
	RIGHT, LEFT, NEUTRAL, IN, OUT
};

int rudder_direction = NEUTRAL;
int sail_direction = NEUTRAL;

//#define 	LIMIT_VALUE 	20
#define 	CONVERTION_VALUE 	0.11	// Needs to be calibrated!
#define 	FEEDBACK_CENTER 	1020 	// <-- ~1800/2
#define 	ERROR_MARGIN_RUDDER 	2
#define 	ERROR_MARGIN_SAIL 	5
#define 	MAX_DUTY_RUDDER		60
#define 	MAX_DUTY_SAIL		90

void init_io();
void initFiles();
void read_desired_rudder_angle_values();
void read_desired_sail_length_values();
void sleep_ms();
int find_rudder_duty(int delta_angle);
int find_sail_duty(int delta_lenght);
void stop_rudder(void);
void stop_sail(void);
void ramp_up_rudder_right(int final_duty);
void ramp_up_rudder_left(int final_duty);
void ramp_up_sail_left(int final_duty);
void ramp_up_sail_right(int final_duty);
void move_rudder_right(int duty_loc);
void move_rudder_left(int duty_loc);
void move_sail_left(int duty_loc);
void move_sail_right(int duty_loc);

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

		//read from sail actual lenght
		/*File READ*/
		fprintf(stdout, "reading actual lenght\n");
		file = fopen("/dev/hall", "r");
		fscanf(file, "%d", &actual_length);
		fclose(file);

		//do convertion adc -> angle
		actual_angle = (FEEDBACK_CENTER - adc_value) * CONVERTION_VALUE; //maybe the other way around?
		fprintf(stdout, "actual_angle: %d\n", actual_angle);


		//write to disk
		if (write_delay > 10) {
			write_delay = 0;
			//write rudder
			file = fopen("/tmp/sailboat/Rudder_Feedback", "w");
			fprintf(file, "%d", actual_angle);
			fclose(file);
			//write sail
			file = fopen("/tmp/sailboat/Sail_Feedback", "w");
			fprintf(file, "%d", actual_length);
			fclose(file);

		} else
			write_delay++;
		int delta_angle = actual_angle - desired_angle;
		if (delta_angle < 0)
			delta_angle = -delta_angle;
		fprintf(stdout, "ABS delta angle %d\n", delta_angle); //print out difference to desired angle
		if (delta_angle > ERROR_MARGIN_RUDDER) { //check if it is logical to move

			//decide direction of movement
			if (actual_angle < desired_angle) {
				fprintf(stdout, "actual_angle < desired_angle\n"); //print info
				if (rudder_direction != LEFT) { //check if directions was changed
					rudder_direction = LEFT; //set new direction
					ramp_up_rudder_left(find_rudder_duty(delta_angle)); //ramp up rudder left to desired duty
				} else {
					move_rudder_left(find_rudder_duty(delta_angle)); //set new speed of rudder to desired duty
				}

			} else if (actual_angle > desired_angle) {
				fprintf(stdout, "actual_angle > desired_angle\n"); //print info
				if (rudder_direction != RIGHT) { //check if directions was changed
					rudder_direction = RIGHT; //set new direction
					ramp_up_rudder_right(find_rudder_duty(delta_angle)); //ramp up rudder right to desired duty
				} else {
					move_rudder_right(find_rudder_duty(delta_angle)); //set new speed of rudder to desired duty
				}
			} else {
				fprintf(stdout, "actual_angle = desired_angle!!!\n"); //print info
				if (rudder_direction != NEUTRAL) { //check if rudder were in neutral
					rudder_direction = NEUTRAL; //set rudder to neutral
					fprintf(stdout, "Going NEUTRAL\n");
					stop_rudder(); //stop the rudder and prepare for ramp up
				}
			}
		} else {
			//set outputs low/high? 
			fprintf(stdout, "delta angle < ERROR MARGIN\n");
			stop_rudder(); //stop rudder in case of error margin

		}

		////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		int delta_lenght = actual_length - desired_length;
		if (delta_lenght < 0)
			delta_lenght = -delta_lenght;
		if (delta_lenght > ERROR_MARGIN_SAIL) { //check if it is logical to move

			//decide direction of movement
			if (actual_length < desired_length) {
				fprintf(stdout, "actual_length < desired_lenght\n"); //print info
				if (sail_direction != OUT) { //check if directions was changed
					sail_direction = OUT; //set new direction
					ramp_up_sail_left(find_sail_duty(delta_lenght)); //ramp up sail left to desired duty
				} else {
					move_sail_left(find_sail_duty(delta_lenght)); //set new speed of sail to desired duty
				}

			} else if (actual_length > desired_length) {
				fprintf(stdout, "actual_length > desired_lenght\n"); //print info
				if (sail_direction != IN) { //check if directions was changed
					sail_direction = IN; //set new direction
					ramp_up_sail_right(find_sail_duty(delta_lenght)); //ramp up sail right to desired duty
				} else {
					move_sail_right(find_sail_duty(delta_lenght)); //set new speed of sail to desired duty
				}
			} else {
				fprintf(stdout, "actual_length = desired_lenght!!!\n"); //print info
				if (sail_direction != NEUTRAL) { //check if sail were in neutral
					sail_direction = NEUTRAL; //set sail to neutral
					fprintf(stdout, "Going NEUTRAL\n");
					stop_rudder(); //stop the sail and prepare for ramp up
				}
			}
		} else {
			//set outputs low/high? 
			fprintf(stdout, "delta angle < ERROR MARGIN\n");
			stop_sail(); //stop rudder in case of error margin

		}

		////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		fprintf(stdout, "::::::::::END LOOP::::::::::\n\n"); //end
		sleep_ms(100);
	}
	return 0;
}

/*
 *	Create Empty files
 */
void initFiles() {
	fprintf(stdout, "file init\n");
	system("mkdir -p /tmp/actuators");
	system("echo 0 > /tmp/sailboat/Rudder_Feedback");
	system("echo 0 > /tmp/sailboat/Navigation_System_Rudder");
	system("echo 0 > /tmp/sailboat/Sail_Feedback");
	system("echo 0 > /tmp/sailboat/Navigation_System_Sail");
}

void init_io() {
	//configure io and pwm
	fprintf(stdout, "IO init\n");
	/*
	 * 	2 pwm's
	 * 	-connected to EN-pins FAIL FAIL FAIL... redo this section later acording to actual IO setup
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

	/* Change the pinmux to GPIO instead of default SPI */
	system("devmem2 0x480021c6 h 0x10c"); //GPIO 170
	system("devmem2 0x480021c8 h 0x10c"); //GPIO 171
	system("devmem2 0x480021ca h 0x10c"); //GPIO 172
	system("devmem2 0x480021cc h 0x10c"); //GPIO 173
	system("devmem2 0x480021ce h 0x10c"); //GPIO 174
	system("devmem2 0x480021d0 h 0x10c"); //GPIO 175
	system("echo 170 > /sys/class/gpio/export");
	system("echo 171 > /sys/class/gpio/export");
	system("echo out > /sys/class/gpio/gpio170/direction");
	system("echo out > /sys/class/gpio/gpio171/direction");
	
	//install pwm module
	system("insmod pwm.ko");
	//install hall module
	system("insmod hall.ko");

	system("echo 0 > /sys/class/gpio/gpio170/value");	//Disable LOW, thereby working actuator.
	system("echo 0 > /sys/class/gpio/gpio171/value");	//Enable HIGH (inversed signal), meaning motor driver is on.

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
void move_rudder_left(int duty_loc) {
	file = fopen("/dev/pwm8", "w");
	fprintf(file, "%d", 0);
	fclose(file);
	file = fopen("/dev/pwm9", "w");
	fprintf(file, "%d", duty_loc);
	fclose(file);
	fprintf(stdout, "rudder going left, duty: %d\n", duty_loc);
}
void move_rudder_right(int duty_loc) {
	file = fopen("/dev/pwm9", "w");
	fprintf(file, "%d", 0);
	fclose(file);
	file = fopen("/dev/pwm8", "w");
	fprintf(file, "%d", duty_loc);
	fclose(file);
	fprintf(stdout, "rudder going right, duty: %d\n", duty_loc);
}
void move_sail_right(int duty_loc) {
	file = fopen("/dev/pwm10", "w");
	fprintf(file, "%d", 0);
	fclose(file);
	file = fopen("/dev/pwm11", "w");
	fprintf(file, "%d", duty_loc);
	fclose(file);
	fprintf(stdout, "sail going left, duty: %d\n", duty_loc);
}
void move_sail_left(int duty_loc) {
	file = fopen("/dev/pwm11", "w");
	fprintf(file, "%d", 0);
	fclose(file);
	file = fopen("/dev/pwm10", "w");
	fprintf(file, "%d", duty_loc);
	fclose(file);
	fprintf(stdout, "sail going right, duty: %d\n", duty_loc);
}
void ramp_up_rudder_left(int final_duty) {	
	for (duty = 0; duty <= final_duty; duty += 10) {
		fprintf(stdout, "ramping up rudder ~ duty: %d\n", duty);
		move_sail_left(duty);
		sleep_ms(5);
	}
}
void ramp_up_rudder_right(int final_duty) {
	for (duty = 0; duty <= final_duty; duty += 10) {
		fprintf(stdout, "ramping up rudder ~ duty: %d\n", duty);
		move_rudder_right(duty);
		sleep_ms(5);
	}
}
void ramp_up_sail_left(int final_duty) {	
	for (duty = 0; duty <= final_duty; duty += 10) {
		fprintf(stdout, "ramping up sail ~ duty: %d\n", duty);
		move_sail_left(duty);
		sleep_ms(5);
	}
}
void ramp_up_sail_right(int final_duty) {
	for (duty = 0; duty <= final_duty; duty += 10) {
		fprintf(stdout, "ramping up sail ~ duty: %d\n", duty);
		move_rudder_right(duty);
		sleep_ms(5);
	}
}
void stop_rudder(void) {
	file = fopen("/dev/pwm9", "w");
	fprintf(file, "%d", 0);
	fclose(file);
	file = fopen("/dev/pwm8", "w");
	fprintf(file, "%d", 0);
	fclose(file);
	fprintf(stdout, "rudder stopped");
	sleep_ms(5);
}
void stop_sail(void) {
	file = fopen("/dev/pwm11", "w");
	fprintf(file, "%d", 0);
	fclose(file);
	file = fopen("/dev/pwm10", "w");
	fprintf(file, "%d", 0);
	fclose(file);
	fprintf(stdout, "sail stopped");
	sleep_ms(5);
}

int find_rudder_duty(int delta_angle) {
	duty = MAX_DUTY_RUDDER; //setting duty to highest speed
	fprintf(stdout, "MD: %d\n", MAX_DUTY_RUDDER);
	if ((delta_angle) < 5) { //check if within 5 degrees of desired angle
		duty = 20; //setting duty to lowerst speed
	} else if ((delta_angle) < 10) { //check if within 10 degrees of desired angle
		duty = 30; //setting duty to lower speed
	}
	fprintf(stdout, "MD result: %d\n", duty);
	return duty;
}
int find_sail_duty(int delta_lenght) {
	duty = MAX_DUTY_SAIL; //setting duty to highest speed
	fprintf(stdout, "MD: %d\n", MAX_DUTY_SAIL);
	if ((delta_lenght) < 1) { //check if within 5 degrees of desired angle
		duty = 20; //setting duty to lowerst speed
	} else if ((delta_lenght) < 2) { //check if within 10 degrees of desired angle
		duty = 30; //setting duty to lower speed
	}
	fprintf(stdout, "MD result: %d\n", duty);
	return duty;
}

void sleep_ms(int ms) {
	usleep(ms * 1000); //convert to microseconds
	return;
}
