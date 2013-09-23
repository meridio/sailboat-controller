/*
 * This code is made for Cyber sailing class spring'13
 * It is made for reading the energy consuption of the Cyber sailing boat
 * Interface:	
 * 	1 adc in for complete system sensor 
 * 	1 adc in for electronic system sensor
 * 	1 adc in for battery voltage
 */

/* TO DO
 * 	add adc readers
 * 	implement logging
 */

#include "energy.h"

int complete_watt = 0;						//used for storring the full system consuption
int complete_current = 0;					//used for storring the full system current 
int electronic_current = 0;					//used for storring the basic electronic current
int electronic_watt = 0;					//used for storring the basic electronic consuption
int voltage_battey = 0;						//used for storring the battery voltage level
int average_voltage_battey = 0;				 	//used for storring the average battery voltage level

int loop_counter = 0;
int logEntry = 0;
char logfile[50];

FILE* file;

#define 	I_complete_gain 			9	// Needs to be calibrated
#define 	I_electronic_gain 			9	// Needs to be calibrated
#define 	I_complete_offset 			0	// Needs to be calibrated
#define 	I_electronic_offset			0	// Needs to be calibrated
#define 	V_Gain 					3	// Needs to be calibrated
#define 	electronic_voltage_level		4.85	// check if value stays here
#define 	number_of_measurements_per_average 	10
#define 	measurements_measurering_speed 		100	//in ms
#define		MAXLOGLINES				10000

void init_io();
void initFiles();
int read_complete_system_current(); 
int read_electronic_system_current(); 
int read_battery_voltage_level(); 
void write_log_file();


int main() {
	initFiles();
	init_io();
	for (;;) {
		complete_watt = 0;
		electronic_watt = 0;  
		average_voltage_battey = 0;
		for(loop_counter = 0; loop_counter<number_of_measurements_per_average; loop_counter++){
			//get converted inputs from the adc
			complete_current = read_complete_system_current(); 
	 		electronic_current = read_electronic_system_current(); 
	 		voltage_battey = read_battery_voltage_level();
			//calculate the energi usage 
			average_voltage_battey += voltage_battey;
			complete_watt += complete_current*voltage_battey;
			electronic_watt += electronic_current*electronic_voltage_level;  
			//sleep between measurements
			usleep(measurements_measurering_speed * 1000); // sleep value in in microseconds
		}
		complete_watt = complete_watt/number_of_measurements_per_average;
		electronic_watt = electronic_watt/number_of_measurements_per_average;
		average_voltage_battey = average_voltage_battey/number_of_measurements_per_average;
		fprintf(stdout, "-------------------------------------------- \n");
		fprintf(stdout, "Average battery voltage:   %d [W] \n", average_voltage_battey);
		fprintf(stdout, "Complete comsuption:   %d [W] \n", complete_watt);
		fprintf(stdout, "Electronic comsuption: %d [W] \n", electronic_watt);
		fprintf(stdout, "-------------------------------------------- \n");
		//store in file
		write_log_file();
	}
	return 0;
}

/*
 *	Create Empty files
 */
void initFiles() {
	fprintf(stdout, "file init\n");
	system("mkdir /tmp/current_sensors");
	system("mkdir -p sailboat-energy/");
}

void init_io() {
	//configure io
}

int read_complete_system_current(){
	int adc = 0;

	/*ADC READ*/
	fprintf(stdout, "adc reading\n");
	file = fopen("/sys/class/hwmon/hwmon0/device/in4_input", 			"r");
	fscanf(file, "%d", &adc);
	fclose(file);
	fprintf(stdout, "complete system in4: %d\n", adc);

	return (adc-I_complete_offset)*I_complete_gain;	
}

int read_electronic_system_current(){
	int adc = 15;	//do sensor stuff here

	/*ADC READ*/
	fprintf(stdout, "adc reading\n");
	file = fopen("/sys/class/hwmon/hwmon0/device/in5_input", 			"r");
	fscanf(file, "%d", &adc);
	fclose(file);
	fprintf(stdout, "electric system in5: %d\n", adc);

	return (adc-I_electronic_offset)*I_electronic_gain ;
} 

int read_battery_voltage_level(){
	int adc = 10;	//do sensor stuff here

	/*ADC READ*/
	fprintf(stdout, "adc reading\n");
	file = fopen("/sys/class/hwmon/hwmon0/device/in6_input", 			"r");
	fscanf(file, "%d", &adc);
	fclose(file);

	fprintf(stdout, "voltage in6: %d\n", adc);

	return adc*V_Gain;
}


void write_log_file() {

	FILE* file2;
	DIR * dirp;
	char  logline[1000];

	time_t rawtime;
	struct tm * timeinfo;
	struct dirent * entry;

	// crate a new file every MAXLOGLINES
	if(logEntry==0 || logEntry>=MAXLOGLINES) {
	
		//count files in log folder
		int file_count = 1;

		dirp = opendir("sailboat-energy/"); 

		while ((entry = readdir(dirp)) != NULL) {
			if (entry->d_type == DT_REG) { 
				 file_count++;
			}
		}
		closedir(dirp);

		// calculate filename
		sprintf(logfile,"sailboat-energy/logfile_%.4d",file_count);

		// create the new file with a verbose time format as first line
		time ( &rawtime );
		timeinfo = localtime ( &rawtime );
		file2 = fopen(logfile, "w");
		if (file2 != NULL) {
			fprintf(file2, "Local time and date: %s", asctime (timeinfo));
			fclose(file2);
		}
		logEntry=1;
	}

	// generate CSV log line
	sprintf(logline, "Complete_watt : Electronic_watt : Battery_voltage : timestamp %d:%d:%d:%u \n",complete_watt,electronic_watt,average_voltage_battey, (unsigned) time(NULL));

	// write to file
	file2 = fopen(logfile, "a");
	if (file2 != NULL) {
		fprintf(file2, "%s\n", logline);
		fclose(file2);
	}
	logEntry++;
}
