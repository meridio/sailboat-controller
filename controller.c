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

#define PI 3.14159265
#define GAIN_P 1
#define GAIN_I 0

#define RATEOFTURN_MAX	0.1745	//PI/18
#define dHEADING_MAX	0.6
#define INTEGRATOR_MAX	0.2

FILE* file;
float Rate, Heading, Deviation, Variation, Yaw, Pitch, Roll, Latitude, Longitude, COG, SOG, Wind_Speed, Wind_Angle;
float Point_Start_Lat, Point_Start_Lon, Point_End_Lat, Point_End_Lon;
float integratorSum = 0;
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

			// Read desired values set by the Graphical User Interface
			read_manual_control_values();

			// Move the rudder to the desired position
			move_rudder(Manual_Control_Rudder);
			printf("Rudder desired angle: %d \n",Manual_Control_Rudder);

		} else {

			if (Navigation_System) {
				// "AUTOPILOT" is activated.

				// Read data from the Weather station: Gps, Wind, Yaw, Roll, Rate of turn, ...
				read_weather_station();

				// Read START and TARGET point coordinates
				read_coordinates();

				// Calculate the rudder desired angle based on environmental conditions and target position
				calculate_rudder_angle();

				// Move the rudder to the desired angle
				move_rudder(Rudder_Desired_Angle);
				printf("Rudder desired angle: %d \n",Rudder_Desired_Angle);

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

	system("echo 0 > /tmp/sailboat/Navigation_System");
	system("echo 0 > /tmp/sailboat/Navigation_System_Rudder");
	system("echo 0 > /tmp/sailboat/Navigation_System_Sail");
	system("echo 0 > /tmp/sailboat/Manual_Control");
	system("echo 0 > /tmp/sailboat/Manual_Control_Rudder");
	system("echo 0 > /tmp/sailboat/Manual_Control_Sail");
	system("echo 0 > /tmp/sailboat/Point_Start_Lat");
	system("echo 0 > /tmp/sailboat/Point_Start_Lon");
	system("echo 0 > /tmp/sailboat/Point_End_Lat");
	system("echo 0 > /tmp/sailboat/Point_End_Lon");
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
 *	GUIDANCE + RUDDER PID CONTROLLER
 *	
 *	GUIDANCE:
 *		- Calculate the Target Heading based on the target position and current position
 *	RUDDER PID CONTROLLER:
 *		- Calulate the desired RUDDER ANGLE position based on the Target Heading and Current Heading
 *
 *	The result ia a rounded value of the angle stored in the [Rudder_Desired_Angle] global variable
 */
void calculate_rudder_angle() {

	// ------------------------------------
	// GUIDANCE, calculate Target Heading:
	// ------------------------------------

	// targetHeading range is -180(to the left) to +180(to the right)
/*
	// GUIDANCE v1: point to target direction
	float dx, dy, targetHeading;
	dx = Point_End_Lon - Longitude;
	dy = Point_End_Lat - Latitude;
	targetHeading = atan2(dx, dy) * 180 / PI;
*/

	// GUIDANCE v2: jibe and tack capable
	float x, y, targetHeading, theta_wind;
	float _Complex X, X_T, X_T_g, X_g, Xl, Xr, theta_LOS, theta_l, theta_r; //float?? //X_T??

	x=Latitude;
	y=Longitude;
	theta_wind=Wind_Angle;

	// complex notation for x,y position
	X = x + 1I*y;

	// ** Turning matrix **
	// The calculations in the guidance system are done assuming constant wind
	// from above. To make this system work, we need to 'turn' it according to
	// the winddirection. It is like looking on a map, you have to find the
	// north before reading it.

	// Using theta_wind to transfer X_T. Here theta_wind is expected to be zero
	// when coming from north, going clockwise in radians.
	X_T_g = cexp(1I*((X_T*180/PI)+theta_wind))*cabs(X_T);
	// 'g' for guidance frame.
	X_g = cexp(1I*((X*180/PI)+theta_wind))*cabs(X);

	// deadzone limit direction
	Xl = -2 + 2*1I;
	Xr =  2 + 2*1I;
/*
	// tacking boundaries
	int xl = -10;
	int xr = 10;

	// definition of the different angles
	theta_LOS = angle(X_T_g - X_g);
	theta_l = angle(exp(-1I*theta_LOS)*Xl);
	theta_r = angle(exp(-1I*theta_LOS)*Xr);

	// compute the next theta_d, ie at time t+1
	// (main algorithm)
	if s==1 // if stopping signal is sent, then head up agains the wind
		theta_d1_g= pi/2;
	else
		if not(angle(Xr)<=theta_LOS & theta_LOS<=angle(Xl)) 
		    // if theta_LOS is outside of the deadzone
		    theta_d1_g = theta_LOS;
		else
		    if theta_d == angle(Xl)
		        if x <= xl 
		            theta_d1_g = angle(Xr)
		        else
		            theta_d1_g = theta_d
		        end
		    elseif theta_d == angle(Xr)
		        if xr <= x 
		            theta_d1_g = angle(Xl)
		        else
		            theta_d1_g = theta_d
		        end
		    else
		        ThetaLOSNorm = [ abs(theta_l) ; abs(theta_r) ];
		        Xdir = [ Xl ; Xr ]
		        [~,index_min] = min(ThetaLOSNorm);
		        theta_d1_g = angle(Xdir(index_min));
		    end
		end
	end

	// Inverse turning matrix
	theta_d1 = theta_d1_g-theta_wind
*/


	// ----------------------------------------------
	// RUDDER PID CONTROLLER, calculate Rudder angle:
	// ----------------------------------------------
	float dHeading, pValue, integralValue;
	dHeading = targetHeading - Heading; // in degrees
	// WARNING: calculating the dHeading consider the jump from 0 to 359 degrees
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
	sprintf(logline, "%u,%d,%d,%d,%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f" \
		, (unsigned)time(NULL) \
		, Navigation_System \
		, Manual_Control \
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
