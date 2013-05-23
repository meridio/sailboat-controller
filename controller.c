/* 
 *	SAILBOAT-CONTROLLER
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>
#include <time.h>
#include <dirent.h>
#include <complex.h>
#include <stdbool.h>

#define PI 3.14159265
#define GAIN_P 1
#define GAIN_I 0

#define TACKINGRANGE 	200		//meters
#define RADIUSACCEPTED	20		//meters
#define CONVLON			64078	//meters per degree
#define CONVLAT			110742	//meters per degree

#define RATEOFTURN_MAX	0.1745	//PI/18
#define dHEADING_MAX	0.6
#define INTEGRATOR_MAX	0.2

FILE* file;
float Rate, Heading, Deviation, Variation, Yaw, Pitch, Roll, Latitude, Longitude, COG, SOG, Wind_Speed, Wind_Angle;
float Point_Start_Lat, Point_Start_Lon, Point_End_Lat, Point_End_Lon;
float integratorSum=0, theta_d=0, Guidance_Heading=0;
int   Rudder_Desired_Angle, Manual_Control_Rudder, Manual_Control_Sail;
int   Navigation_System, Manual_Control;
int   logEntry = 0;
char  logfile[50];

void initfiles();
void check_system_status();
void read_manual_control_values();
void read_weather_station();
void move_rudder(int angle);
void read_coordinates();
void guidance();
void calculate_rudder_angle();
void write_log_file();

int main(int argc, char ** argv) {
	initfiles();
	fprintf(stdout, "\nSailboat-controller running..\n");
	read_weather_station();

	while (1) {
		
		check_system_status();

		if (Manual_Control) {
			// MANUAL CONTROL IS ON
			// Values for the RUDDER and SAIL positions are received from the User Interface.
			// Use [Manual_Control_Rudder] and [Manual_Control_Sail] files to control the actuators.

			// Read data from the Weather station: Gps, Wind, Yaw, Roll, Rate of turn, ...
			read_weather_station();

			// Read desired values set by the Graphical User Interface
			read_manual_control_values();

			// Move the rudder to the desired position
			move_rudder(Manual_Control_Rudder);
			//printf("MANUAL: Rudder desired angle: %d \n",Manual_Control_Rudder);

		} else {

			if (Navigation_System) {
				// "AUTOPILOT" is activated.

				// Read data from the Weather station: Gps, Wind, Yaw, Roll, Rate of turn, ...
				read_weather_station();

				// Read START and TARGET point coordinates
				read_coordinates();

				// Calculate the desired boat heading
				guidance();

				// Calculate the desired rudder angle
				calculate_rudder_angle();

				// Move the rudder to the desired angle
				move_rudder(Rudder_Desired_Angle);
				// printf("NAVSYS: Rudder desired angle: %d \n",Rudder_Desired_Angle);

				// If the desired point is reached (20m tollerance), switch the coordinates and come back home
				// calculate_distance();
				// switch_coordinates();

			} else {
				// NAVIGATION SYSTEM IS IDLE
				// do nothing
			}

		}

		// log
		write_log_file();

		sleep(1);
	}
	return 0;
}

/*
 *	Create empty files: Manual Control, Starting Point, Ending Point
 *	Navigation system starts in IDLE mode, waiting for target point coordinates
 */
void initfiles() {
	system("mkdir -p /tmp/sailboat");
	system("mkdir -p /var/tmp/sailboat-log/");

	system("[ ! -f /tmp/sailboat/Navigation_System ] 		&& echo 0 > /tmp/sailboat/Navigation_System");
	system("[ ! -f /tmp/sailboat/Navigation_System_Rudder ] && echo 0 > /tmp/sailboat/Navigation_System_Rudder");
	system("[ ! -f /tmp/sailboat/Navigation_System_Sail ] 	&& echo 0 > /tmp/sailboat/Navigation_System_Sail");
	system("[ ! -f /tmp/sailboat/Manual_Control ] 			&& echo 0 > /tmp/sailboat/Manual_Control");
	system("[ ! -f /tmp/sailboat/Manual_Control_Rudder ] 	&& echo 0 > /tmp/sailboat/Manual_Control_Rudder");
	system("[ ! -f /tmp/sailboat/Manual_Control_Sail ] 		&& echo 0 > /tmp/sailboat/Manual_Control_Sail");
	system("[ ! -f /tmp/sailboat/Point_Start_Lat ] 			&& echo 0 > /tmp/sailboat/Point_Start_Lat");
	system("[ ! -f /tmp/sailboat/Point_Start_Lon ] 			&& echo 0 > /tmp/sailboat/Point_Start_Lon");
	system("[ ! -f /tmp/sailboat/Point_End_Lat ] 			&& echo 0 > /tmp/sailboat/Point_End_Lat");
	system("[ ! -f /tmp/sailboat/Point_End_Lon ] 			&& echo 0 > /tmp/sailboat/Point_End_Lon");
	system("[ ! -f /tmp/sailboat/Guidance_Heading ] 		&& echo 0 > /tmp/sailboat/Guidance_Heading");
}

/*
 *	Check Navigation System Status
 *	[Navigation System] 
 *		- [0] Boat in IDLE status
 *		- [1] Control System ON, Rudder under control of MCU
 *	[Manual Control]
 *		- [0] OFF
 *		- [1] User takes control of the rudder position
 */
void check_system_status() {
	file = fopen("/tmp/sailboat/Navigation_System", "r");
	fscanf(file, "%d", &Navigation_System);
	fclose(file);
	file = fopen("/tmp/sailboat/Manual_Control", "r");
	fscanf(file, "%d", &Manual_Control);
	fclose(file);
}

/*
 *	Read Manual_Control values
 *	[Manual_Control_Rudder] : user value for desired RUDDER angle [-30.0 to 30.0]
 *	[Manual_Control_Sail] : user value for desired SAIL angle [ignored]
 */
void read_manual_control_values() {
	file = fopen("/tmp/sailboat/Manual_Control_Rudder", "r");
	fscanf(file, "%d", &Manual_Control_Rudder);
	fclose(file);
	file = fopen("/tmp/sailboat/Manual_Control_Sail", "r");
	fscanf(file, "%d", &Manual_Control_Sail);
	fclose(file);
}

/*
 *	Move the rudder to the desired position.
 *	Write the desired angle to a file [] to be handled by another process 
 */
void move_rudder(int angle) {
	file = fopen("/tmp/sailboat/Navigation_System_Rudder", "w");
	fprintf(file, "%d", angle);
	fclose(file);
}

/*
 *	Read START and END point coordinated (latitude and longitude)
 *	Coordinates format is DD
 */
void read_coordinates() {
	file = fopen("/tmp/sailboat/Point_Start_Lat", "r");
	fscanf(file, "%f", &Point_Start_Lat);
	fclose(file);
	file = fopen("/tmp/sailboat/Point_Start_Lon", "r");
	fscanf(file, "%f", &Point_Start_Lon);
	fclose(file);
	file = fopen("/tmp/sailboat/Point_End_Lat", "r");
	fscanf(file, "%f", &Point_End_Lat);
	fclose(file);
	file = fopen("/tmp/sailboat/Point_End_Lon", "r");
	fscanf(file, "%f", &Point_End_Lon);
	fclose(file);
}


/*
 *	GUIDANCE:
 *
 *	Calculate the desired Heading based on the WindDirection, StartPoint, EndPoint and current position
 */
void guidance() {

/*
	// GUIDANCE v0: dummy solution
	float dx, dy, guidance_heading;
	dx = Point_End_Lon - Longitude;
	dy = Point_End_Lat - Latitude;
	guidance_heading = atan2(dx, dy) * 180 / PI;
*/


	// GUIDANCE v1: nogozone and tack capable solution
	float x, y, theta_wind, theta_d_b, a_x, b_x;
	float theta_LOS, theta_l, theta_r, theta_d1, theta_d1_b;
	float _Complex X, X0, X_T, Geo_X, Geo_X0, Geo_X_T, X_T_b, X_b, Xl, Xr;
	bool inrange;

	x=Longitude;
	y=Latitude;

	theta_wind=Wind_Angle*PI/180;
	
	// complex notation for x,y position of the starting point
	Geo_X0 = Point_Start_Lon + 1*I*Point_Start_Lat;
	X0 = 0 + 1*I*0;

	// complex notation for x,y position of the boat
	Geo_X = x + 1*I*y;
	X=(Geo_X-Geo_X0);
	X=creal(X)*CONVLON + I*cimag(X)*CONVLAT;


	// complex notation for x,y position of the target point
	Geo_X_T = Point_End_Lon + 1*I*Point_End_Lat;
	X_T=(Geo_X_T-Geo_X0);
	X_T=creal(X_T)*CONVLON + I*cimag(X_T)*CONVLAT;


	// ** turning matrix **

	// The calculations in the guidance system are done assuming constant wind
	// from above. To make this system work, we need to 'turn' it according to
	// the winddirection. It is like looking on a map, you have to find the
	// north before reading it.
	
	// Using theta_wind to transfer X_T. Here theta_wind is expected to be zero
	// when coming from north, going clockwise in radians.
	
	//X_T_b =	cexp(1*I*(atan2((endy-starty) *CONVLAT, (endx-startx) *CONVLON))+theta_wind)*cabs(X_T-X0);
	X_T_b = ccos(atan2(cimag(X_T),creal(X_T))+theta_wind)*cabs(X_T) + 1*I*(csin(atan2(cimag(X_T),creal(X_T))+theta_wind)*cabs(X_T));
	
	//X_b = cexp(1*I*(atan2((y-starty)    *CONVLAT, (x-startx)    *CONVLON))+theta_wind)*cabs(X-X0);
	X_b = ccos(atan2(cimag(X),creal(X))+theta_wind)*cabs(X) + 1*I*(csin(atan2(cimag(X),creal(X))+theta_wind)*cabs(X));
	
	theta_d_b = theta_d + theta_wind;



	// ** Guidance system **

	// deadzone limit direction
	Xl = -2 + 2*1*I;
	Xr = 2 + 2*1*I;	

	// definition of the different angles
	theta_LOS = atan2(cimag(X_T_b)-cimag(X_b),creal(X_T_b)-creal(X_b));
	theta_l = atan2(cimag(cexp(-1*I*theta_LOS)*Xl),creal(cexp(-1*I*theta_LOS)*Xl));
	theta_r = atan2(cimag(cexp(-1*I*theta_LOS)*Xr),creal(cexp(-1*I*theta_LOS)*Xr));

	// tacking boundaries
	// Line: x = a_x*y +/- b_x
	
	if (creal(X_T_b-X0) != 0)
	{
		a_x = creal(X_T_b-X0)/cimag(X_T_b-X0);
	}
	else
    {
		a_x=0;
	}
	
	b_x = TACKINGRANGE / (2 * sin(theta_LOS));

	//DEBUG

	printf("\n\n");
	//printf("[x: %8.2f], [y: %8.2f]\n[startx: %8.2f], [starty: %8.2f]\n[endx: %8.2f], [endy: %8.2f]\n",x, y, startx, starty, endx, endy);
	//printf("[X:     %7.5f + i%7.5f] \n", creal(X), cimag(X));
	printf("[X_T:   %7.5f + i%7.5f] \n", creal(X_T), cimag(X_T));
	printf("[X_T_b: %7.5f + i%7.5f] \n[X_b:   %7.5f + i%7.5f] \n", creal(X_T_b), cimag(X_T_b), creal(X_b), cimag(X_b) );
	printf("[theta_d: %5.2f] \n", theta_d);
	printf("[theta_wind: %5.2f] \n", theta_wind);
	printf("[theta_d_b: %5.2f] \n", theta_d_b);
	printf("[theta_LOS: %4.2f] [cond1: %5.3f] [cond2: %5.3f]\n", theta_LOS, atan2(cimag(Xr),creal(Xr)), atan2(cimag(Xl),creal(Xl)) );


	// stop signal
	if (cabs(X_T - X) < RADIUSACCEPTED)	{ inrange = true; }
	else { inrange = false; }
	

	// compute the next theta_d, ie at time t+1
	// (main algorithm)
	if (inrange) { 			// if stopping signal is sent, then head up agains the wind
	    theta_d1_b = PI/2;
		printf(">> debug 1 \n");
	}
	else
	{
		if (!(  atan2(cimag(Xr),creal(Xr))<=theta_LOS  &&  theta_LOS<=atan2(cimag(Xl),creal(Xl))  ))
		{
			// if theta_LOS is outside of the deadzone
			theta_d1_b = theta_LOS;
			printf(">> debug 2 : Out of nogozone\n");
		}
		else
		{
			if (theta_d_b >= atan2(cimag(Xl),creal(Xl))-PI/36  && theta_d_b <= atan2(cimag(Xl),creal(Xl))+PI/36 )
			{
				// If going upwind, left direction.
				if (creal(X_b) < a_x*cimag(X_b)-b_x) { theta_d1_b = atan2(cimag(Xr),creal(Xr)); printf(">> debug 3 \n"); }     
				else { theta_d1_b = theta_d_b; printf(">> debug 4 \n");}
		    } 
			else
			{
				if (theta_d_b >= atan2(cimag(Xr),creal(Xr))-PI/36  && theta_d_b <= atan2(cimag(Xr),creal(Xr))+PI/36 )
				{
				// If going upwind, right direction.
					if (creal(X_b) > a_x*cimag(X_b)+b_x) { theta_d1_b = atan2(cimag(Xl),creal(Xl)); printf(">> debug 5 \n");}
					else { theta_d1_b = theta_d_b; printf(">> debug 6 \n");}
				}
				else
				{
				// If the target is in nogozone, not going upwind.
					if(cabs(theta_l) < cabs(theta_r)) { theta_d1_b = atan2(cimag(Xl),creal(Xl)); printf(">> debug 7 \n");}
					else { theta_d1_b = atan2(cimag(Xr),creal(Xr)); printf(">> debug 8 \n");}
				}
			}
		}
	}

	// Inverse turning matrix
	theta_d1 = theta_d1_b-theta_wind;
	Guidance_Heading = (PI/2 - theta_d1) * 180/PI; 
	theta_d = theta_d1;

	// write guidance_heading to file to be displayed in GUI 
	file = fopen("/tmp/sailboat/Guidance_Heading", "w");
	fprintf(file, "%4.1f", Guidance_Heading);
	fclose(file);
	
}


/*
 *	RUDDER PID CONTROLLER:
 *
 *	Calulate the desired RUDDER ANGLE position based on the Target Heading and Current Heading.
 *	The result ia a rounded value of the angle stored in the [Rudder_Desired_Angle] global variable
 */
void calculate_rudder_angle() {

	float dHeading, pValue, integralValue;
	dHeading = Guidance_Heading - Heading; // in degrees
	// fprintf(stdout,"targetHeafing: %f, deltaHeading: %f\n",targetHeading, dHeading);

	// P controller
	pValue = GAIN_P * dHeading;

	// Integration part
	// The following checks, will keep integratorSum within -0.2 and 0.2
	if (integratorSum < -INTEGRATOR_MAX && dHeading > 0) {
		integratorSum = dHeading + integratorSum;
	} else if (integratorSum > INTEGRATOR_MAX && dHeading < 0) {
		integratorSum = dHeading + integratorSum;
	} else {
		integratorSum = integratorSum;
	}
	integralValue = GAIN_I * integratorSum;

	// result
	if (abs(dHeading) > dHEADING_MAX && abs(Rate) > RATEOFTURN_MAX) // Limit control statement
	{
		Rudder_Desired_Angle = round(pValue + integralValue);
	}
	// fprintf(stdout,"pValue: %f, integralValue: %f\n",pValue,integralValue);
	// fprintf(stdout,"Rudder_Desired_Angle: %d\n\n",Rudder_Desired_Angle);

}


/*
 *	Read data from the Weather Station
 */
void read_weather_station() {
	//RATE OF TURN
	file = fopen("/tmp/u200/Rate", "r");
	if (file != 0) {
		fscanf(file, "%f", &Rate);
		fclose(file);
	} else {
		printf("ERROR: Files from Weather Station are missing.\n");
		exit(1);
	}

	//VESSEL HEADING
	file = fopen("/tmp/u200/Heading", "r");
	fscanf(file, "%f", &Heading);
	fclose(file);
	file = fopen("/tmp/u200/Deviation", "r");
	fscanf(file, "%f", &Deviation);
	fclose(file);
	file = fopen("/tmp/u200/Variation", "r");
	fscanf(file, "%f", &Variation);
	fclose(file);

	//ATTITUDE
	file = fopen("/tmp/u200/Yaw", "r");
	fscanf(file, "%f", &Yaw);
	fclose(file);
	file = fopen("/tmp/u200/Pitch", "r");
	fscanf(file, "%f", &Pitch);
	fclose(file);
	file = fopen("/tmp/u200/Roll", "r");
	fscanf(file, "%f", &Roll);
	fclose(file);

	//GPS_DATA
	file = fopen("/tmp/u200/Latitude", "r");
	fscanf(file, "%f", &Latitude);
	fclose(file);
	file = fopen("/tmp/u200/Longitude", "r");
	fscanf(file, "%f", &Longitude);
	fclose(file);
	file = fopen("/tmp/u200/COG", "r");
	fscanf(file, "%f", &COG);
	fclose(file);
	file = fopen("/tmp/u200/SOG", "r");
	fscanf(file, "%f", &SOG);
	fclose(file);

	//WIND_DATA
	file = fopen("/tmp/u200/Wind_Speed", "r");
	fscanf(file, "%f", &Wind_Speed);
	fclose(file);
	file = fopen("/tmp/u200/Wind_Angle", "r");
	fscanf(file, "%f", &Wind_Angle);
	fclose(file);

}

/*
 *	Save all the variables of the navigation system in a log file in /var/tmp/sailboat-log/
 *	Create a new log file every 10 minutes
 */
void write_log_file() {

	char  logline[500];

	time_t rawtime;
	struct tm * timeinfo;

	// crate a new file at startup or every 10 minutes
	if(logEntry==0 || logEntry>600) {
		
		//count files in log folder
		int file_count = 1;
		DIR * dirp;
		struct dirent * entry;

		dirp = opendir("/var/tmp/sailboat-log/"); 

		while ((entry = readdir(dirp)) != NULL) {
			if (entry->d_type == DT_REG) { 
				 file_count++;
			}
		}
		closedir(dirp);

		// calculate filename
		sprintf(logfile,"/var/tmp/sailboat-log/logfile_%.4d",file_count);

		// create the new file with a verbose time format as first line
		time ( &rawtime );
		timeinfo = localtime ( &rawtime );
		file = fopen(logfile, "w");
		fprintf(file, "Local time and date: %s", asctime (timeinfo));
		fclose(file);
	
		logEntry=1;
	}

	// generate CSV log line
	sprintf(logline, "%u,%d,%d,%4.1f,%d,%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f" \
		, (unsigned)time(NULL) \
		, Navigation_System \
		, Manual_Control \
		, Guidance_Heading \
		, Rudder_Desired_Angle \
		, Manual_Control_Rudder \
		, Rate ,Heading ,Deviation ,Variation ,Yaw ,Pitch ,Roll ,Latitude ,Longitude ,COG ,SOG ,Wind_Speed ,Wind_Angle \
		, Point_Start_Lat ,Point_Start_Lon ,Point_End_Lat ,Point_End_Lon \
	);
	
	// write to file
	file = fopen(logfile, "a");
	fprintf(file, "%s\n", logline);
	fclose(file);
	
	logEntry++;
}
