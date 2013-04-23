/*
 * This code is made for Cyber sailing class spring'13
 * input is an angle between -45 deg og 45 deg
 * Interface:
 * 	2 pwm's
 * 	1 adc
 * 	2 inputs from hall module (LA36)
 * 	4 outputs for dual-h-bridge
 */

/* TO DO
 - set a write file timeout
 - init function
 - Calibrate CONVERTION_VALUE
 */

#include "actuators.h"

int desired_angle = 0;
int adc_value = 0; //big enough ?
int actual_angle = 0;
int duty = 0;
FILE* file;
enum directions {
	RIGHT, LEFT, NEUTRAL
};
int direction = NEUTRAL;

#define 	CONVERTION_VALUE 	0.1		// Needs to be calibrated!
#define 	FEEDBACK_CENTER 	2048 	// <-- 4096/2
#define 	ERROR_MARGIEN 		5

void init_io();
void initFiles();
void read_desired_rudder_angle_values();

int main() {

	initFiles();
	init_io();

	for (;;) {
		read_desired_rudder_angle_values();

		//read from ADC
		/*ADC READ*/
		fprintf(stdout, "adc reading\n");
		//adc_value = ???
		//write to system("echo 0 > /tmp/sailboat/Rudder_Feedback");

		//do convertion adc -> angle
		adc_value = 2048;
		actual_angle = (FEEDBACK_CENTER - adc_value) * CONVERTION_VALUE; //maybe the other way around?
		fprintf(stdout, "actual_angle: %d\n", actual_angle);
		//decide direction of movement

		int delta_angle = actual_angle - desired_angle;
		if (delta_angle < 0)
			delta_angle = -delta_angle;
		fprintf(stdout, "ABS delta angle %d\n", delta_angle);

		if (delta_angle > ERROR_MARGIEN) {
			//set outputs
			if (actual_angle < desired_angle) {
				if (direction != RIGHT) {
					direction = RIGHT;
					duty = 0;
				}

				/*L bridge high*/
				/*R bridge low*/
				fprintf(stdout, "actual_angle < desired_angle\n");
			} else if (actual_angle > desired_angle) {
				if (direction != LEFT) {
					direction = LEFT;
					duty = 0;
				}
				/*L bridge low*/
				/*R bridge high*/
				fprintf(stdout, "actual_angle > desired_angle\n");
			} else {
				fprintf(stdout, "actual_angle = desired_angle!!!\n");
				if (direction != NEUTRAL) {
					direction = NEUTRAL;
					duty = 0;
				}

			}

			//ramp PWM up slowly
			if (direction == NEUTRAL) {
				/*set pwm duty = duty variable*/
			} else if (duty != 100 && direction != NEUTRAL) {
				for (duty = 0; duty < 90; duty += 20) {
					/*set pwm duty = duty variable*/
					fprintf(stdout, "duty: %d\n", duty);
					sleep(1);
				}
			}
		} else {
			//set outputs low/high?
			/*PWM duty = 0*/
			fprintf(stdout, "else\n");
		}

		fprintf(stdout, "::::::::::END LOOP::::::::::\n\n");
		sleep(5);

	}
	return 0;
}

/*
 *	Create Empty files
 */
void initFiles() {
	fprintf(stdout, "file init\n");
	system("mkdir /tmp/actuators");
	system("echo 0 > /tmp/actuators/rudder_angle");
	system("echo 0 > /tmp/sailboat/Rudder_Feedback");
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

}

void read_desired_rudder_angle_values() {
	fprintf(stdout, "reading desired angle\n");

	file = fopen("/tmp/actuators/rudder_angle", "r");
	fscanf(file, "%d", &desired_angle);
	fclose(file);

	fprintf(stdout, "desired angle: %d\n", desired_angle);
}

