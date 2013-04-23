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
 */

#include "actuators.h"

int rudder_angle = 0;
int adc_value = 0; //big enough ?
int actual_angle;
int duty = 0

const CONVERTION_VALUE 1
const FEEDBACK_CENTER 4096/2
const ERROR_MARGIEN 5

void initfiles();
void read_rudder_angle_values();

int main(int argc, char ** argv) {
	char * device = argv[1];
	
	initfiles();
	init_io();

	for (;;) {
		read_rudder_angle_values();

		//read from ADC
		/*ADC READ*/

		//adc_value = ???
		//do convertion adc -> angle
		actual_angle = (FEEDBACK_CENTER - adc_value) * CONVERTION_VALUE; //maybe the other way around?
		//decide direction of movement
		if ((actual_angle - rudder_angle) <= ERROR_MARGIEN) {
			//set outputs
			if (actual_angle < rudder_angle) {
				/*L bridge high*/
				/*R bridge low*/
			} else {
				/*L bridge low*/
				/*R bridge high*/
			}
			//ramp PWM up slowly
			if (duty != 100) {
				for (duty = 0; duty < 90; duty += 20) {
					/*set pwm duty = duty variable*/
					sleep(0.2);
				}
			}
		} else {
			//set outputs low/high?
			/*PWM duty = 0*/
		}

		sleep(1);

	}
	return 0;
}

/*
 *	Create Empty files
 */
void initFiles() {
	//system("mkdir /tmp/{127251,127250,127257,129025,129026,130306}");
	system("mkdir /tmp/actuators");

	system("echo 0 > /tmp/actuators/rudder_angle");
}

void init_io() {
	//configure io and pwm

}

void read_rudder_angle_values() {
	file = fopen("/tmp/actuators/rudder_angle", "r");
	fscanf(file, "%f", &rudder_angle);
	fclose (file);
}

