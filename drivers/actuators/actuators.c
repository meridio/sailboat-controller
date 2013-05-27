/*
 * This code is made for Cyber sailing class spring'13
 * input is an angle between -45 deg og 45 deg
 * Interface:
 * RUDDER:
 * 	2 pwm's
 * 	1 outputs for Enable pin on the dual-h-bridge
 * 	1 adc <- Rudder angular sensor
 * 	SAIL:
 * 	2 pwm's
 * 	2 inputs from hall module (LA36)
 * 	1 outputs for Enable pin on the dual-h-bridge
 */

/* TO DO
 - Calibrate CONVERTION_VALUE
 - implement hall counter module or function
 - Enable = 0 when in neutral and ramp'd down
 */

#include "actuators.h"

int desired_angle, desired_length = 0;
int adc_value = 0; //big enough ?
int actual_angle = 0;
int duty = 0;
int write_delay = 0;

FILE* file;
enum directions {
	RIGHT, LEFT, NEUTRAL, IN, OUT
};
int rudder_direction, sail_direction = NEUTRAL;

//#define 	LIMIT_VALUE 	20
#define 	CONVERTION_VALUE 	0.11	// Needs to be calibrated!
#define 	FEEDBACK_CENTER 	1020 	// <-- ~1800/2
#define 	ERROR_MARGIEN 		2

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
		} else
			write_delay++;

		//decide direction of movement

		int delta_angle = actual_angle - desired_angle;
		if (delta_angle < 0)
			delta_angle = -delta_angle;
		fprintf(stdout, "ABS delta angle %d\n", delta_angle);

		if (delta_angle > ERROR_MARGIEN) {
			//set outputs
			if (actual_angle < desired_angle) {
				if (rudder_direction != LEFT) {
					rudder_direction = LEFT;
					duty = 0;
					fprintf(stdout, "Going LEFT\n");
				}
				/*EN bridge A*/
				system("echo 1 > /sys/class/gpio/gpio171/value");
				system("echo 1 > /sys/class/gpio/gpio172/value");

				fprintf(stdout, "actual_angle < desired_angle\n");
			} else if (actual_angle > desired_angle) {
				if (rudder_direction != RIGHT) {
					rudder_direction = RIGHT;
					duty = 0;
					fprintf(stdout, "Going RIGHT\n");
				}
				/*EN bridge A*/
				system("echo 1 > /sys/class/gpio/gpio171/value");
				system("echo 1 > /sys/class/gpio/gpio172/value");

				fprintf(stdout, "actual_angle > desired_angle\n");
			} else {
				fprintf(stdout, "actual_angle = desired_angle!!!\n");
				if (rudder_direction != NEUTRAL) {
					rudder_direction = NEUTRAL;
					duty = 0;
					fprintf(stdout, "Going NEUTRAL\n");

					fprintf(stdout, "duty: %d\n", duty);

					file = fopen("/dev/pwm10", "w");
					fprintf(file, "%d", duty);
					fclose(file);
					file = fopen("/dev/pwm11", "w");
					fprintf(file, "%d", duty);
					fclose(file);
				}

			}

			//ramp PWM up slowly
			if (rudder_direction == NEUTRAL) {
				/*set pwm duty = duty variable*/
			} else if (duty != 20 && rudder_direction != NEUTRAL) {
				duty = 0;
				if (rudder_direction == LEFT) {
					file = fopen("/dev/pwm11", "w");
					fprintf(file, "%d", 0);
					fclose(file);
					while (duty < 20) {
						/*set pwm duty = duty variable*/
						duty += 10;
						fprintf(stdout, "duty: %d\n", duty);

						file = fopen("/dev/pwm10", "w");
						fprintf(file, "%d", duty);
						fclose(file);

						sleep_ms(30);
					}
				} else if (rudder_direction == RIGHT) {
					file = fopen("/dev/pwm10", "w");
					fprintf(file, "%d", 0);
					fclose(file);
					while (duty < 20) {
						/*set pwm duty = duty variable*/
						duty += 10;
						fprintf(stdout, "duty: %d\n", duty);

						file = fopen("/dev/pwm11", "w");
						fprintf(file, "%d", duty);
						fclose(file);

						sleep_ms(30);
					}
				}
			}
		} else {
			//set outputs low/high?
			/*PWM duty = 0*/
			fprintf(stdout, "delta angle < ERROR MARGIEN\n");
			duty = 0;
			fprintf(stdout, "duty: %d\n", duty);

			file = fopen("/dev/pwm11", "w");
			fprintf(file, "%d", duty);
			fclose(file);
			file = fopen("/dev/pwm10", "w");
			fprintf(file, "%d", duty);
			fclose(file);

		}

		fprintf(stdout, "::::::::::END LOOP::::::::::\n\n");
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

void sleep_ms(int ms) {
	usleep(ms * 1000); //convert to microseconds
	return;
}
