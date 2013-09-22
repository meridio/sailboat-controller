/* 
 *	SAILBOAT-CONTROLLER
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <dirent.h>
#include <complex.h>
#include <stdbool.h>

#define MAINSLEEP_SEC	0		// seconds
#define MAINSLEEP_MSEC	250		// milliseconds
#define MAXLOGLINES	10000

#define PI 		3.14159265

#define TACKINGRANGE 	100		// meters
#define RADIUSACCEPTED	20		// meters
#define CONVLON		64078		// meters per degree
#define CONVLAT		110742		// meters per degree

#define JIBE_ANGLE	40		// [degrees]  Rudder angle while jibing
#define theta_nogo	55*PI/180	// [radiants] Angle of nogo zone, compared to wind direction
#define v_min 		0.1	  	// [Km/h] Min velocity for tacking

#define INTEGRATOR_MAX	20		// [degrees], influence of the integrator
#define RATEOFTURN_MAX	36		// [degrees/second]
#define dHEADING_MAX	10		// [degrees] deviation, before rudder PI acts
#define GAIN_P 		-1
#define GAIN_I 		0


FILE* file;
float Rate=0, Heading=0, Deviation=0, Variation=0, Yaw=0, Pitch=0, Roll=0;
float Latitude=0, Longitude=0, COG=0, SOG=0, Wind_Speed=0, Wind_Angle=0;
float Point_Start_Lat=0, Point_Start_Lon=0, Point_End_Lat=0, Point_End_Lon=0, Area_Center_Lat=0, Area_Center_Lon=0;
int   Rudder_Desired_Angle=0, Manual_Control_Rudder=0, Manual_Control_Sail=0, Rudder_Feedback=0, Area_Side=0, Area_Interval=0;
int   Navigation_System=0, Manual_Control=0;
int   logEntry=0, logCount=0, fa_debug=0;
char  logfile[50];

void initfiles();
void check_system_status();
void read_weather_station();
void move_rudder(int angle);
void read_coordinates();
void write_log_file();


//guidance
float _Complex X, X_T, X_T_b, X_b, X0;
float integratorSum=0, Guidance_Heading=0;
float theta=0, theta_b=0, theta_d=0, theta_d_b=0, theta_d1=0, theta_d1_b=0;
int   sig = 0, sig1 = 0, sig2 = 0, sig3 = 0; // coordinating the guidance

void guidance();
void findAngle();
void chooseManeuver();
void performManeuver();
void rudder_pid_controller();


struct timespec timermain;


int main(int argc, char ** argv) {
	
	// set timers
	timermain.tv_sec  = MAINSLEEP_SEC;
	timermain.tv_nsec = MAINSLEEP_MSEC * 1000000L;

	initfiles();
	fprintf(stdout, "\nSailboat-controller running..\n");
	read_weather_station();

	// MAIN LOOP
	while (1) {

		// read GUI configuration files (on/off and manual control values)
		check_system_status();

		if (Manual_Control) {
			// MANUAL CONTROL IS ON
			// Values for the RUDDER and SAIL positions are received from the User Interface.
			// Use [Manual_Control_Rudder] and [Manual_Control_Sail] files to control the actuators.

			// Move the rudder to the desired position
			move_rudder(Manual_Control_Rudder);

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
				rudder_pid_controller();

				// Move the rudder to the desired angle
				printf("Desired rudder angle: %d \n\n", Rudder_Desired_Angle);
				move_rudder(Rudder_Desired_Angle);
			
				// If the desired point is reached (20m tollerance), switch the coordinates and come back home
				// calculate_distance();
				// switch_coordinates();

				// write a log line every N samples
				logCount++;
				if(logCount>=1) {write_log_file(); logCount=0;}

			} else {
				// NAVIGATION SYSTEM IS IDLE
				// do nothing
			}

		}

		//sleep
		nanosleep(&timermain, (struct timespec *)NULL);
	}
	return 0;
}

/*
 *	Initialize system files and create folder structure
 */
void initfiles() {
	system("mkdir -p /tmp/sailboat");
	system("mkdir -p sailboat-log/");

	system("[ ! -f /tmp/sailboat/Navigation_System ] 	&& echo 0 > /tmp/sailboat/Navigation_System");
	system("[ ! -f /tmp/sailboat/Navigation_System_Rudder ] && echo 0 > /tmp/sailboat/Navigation_System_Rudder");
	system("[ ! -f /tmp/sailboat/Navigation_System_Sail ] 	&& echo 0 > /tmp/sailboat/Navigation_System_Sail");
	system("[ ! -f /tmp/sailboat/Manual_Control ] 		&& echo 0 > /tmp/sailboat/Manual_Control");
	system("[ ! -f /tmp/sailboat/Manual_Control_Rudder ] 	&& echo 0 > /tmp/sailboat/Manual_Control_Rudder");
	system("[ ! -f /tmp/sailboat/Manual_Control_Sail ] 	&& echo 0 > /tmp/sailboat/Manual_Control_Sail");
	system("[ ! -f /tmp/sailboat/Point_Start_Lat ] 		&& echo 0 > /tmp/sailboat/Point_Start_Lat");
	system("[ ! -f /tmp/sailboat/Point_Start_Lon ] 		&& echo 0 > /tmp/sailboat/Point_Start_Lon");
	system("[ ! -f /tmp/sailboat/Point_End_Lat ] 		&& echo 0 > /tmp/sailboat/Point_End_Lat");
	system("[ ! -f /tmp/sailboat/Point_End_Lon ] 		&& echo 0 > /tmp/sailboat/Point_End_Lon");
	system("[ ! -f /tmp/sailboat/Area_Center_Lat ] 		&& echo 0 > /tmp/sailboat/Area_Center_Lat");
	system("[ ! -f /tmp/sailboat/Area_Center_Lon ] 		&& echo 0 > /tmp/sailboat/Area_Center_Lon");
	system("[ ! -f /tmp/sailboat/Area_Side ] 		&& echo 0 > /tmp/sailboat/Area_Side");
	system("[ ! -f /tmp/sailboat/Area_Interval ] 		&& echo 0 > /tmp/sailboat/Area_Interval");
	system("[ ! -f /tmp/sailboat/Guidance_Heading ] 	&& echo 0 > /tmp/sailboat/Guidance_Heading");
	system("[ ! -f /tmp/sailboat/Rudder_Feedback ] 		&& echo 0 > /tmp/sailboat/Rudder_Feedback");
}

/*
 *	Check Navigation System Status
 *
 *	[Navigation System] 
 *		- [0] Boat in IDLE status
 *		- [1] Control System ON, Sailing to the waypoint
 *		- [2] Control System ON, Scanning Area
 *		- [3] Control System ON, Mantaining upwind position
 *	[Manual Control]
 *		- [0] OFF
 *		- [1] User takes control of sail and rudder positions
 *
 *	if Manual_Control is ON, read the following values:
 *		- [Manual_Control_Rudder] : user value for desired RUDDER angle [-30.0 to 30.0]
 *		- [Manual_Control_Sail]   : user value for desired SAIL angle 
 */
void check_system_status() {

	file = fopen("/tmp/sailboat/Navigation_System", "r");
	if (file != NULL) {
		fscanf(file, "%d", &Navigation_System);	fclose(file);
	}
	file = fopen("/tmp/sailboat/Manual_Control", "r");
	if (file != NULL) {	
		fscanf(file, "%d", &Manual_Control); fclose(file);
	}

	if(Manual_Control) {

		file = fopen("/tmp/sailboat/Manual_Control_Rudder", "r");
		if (file != NULL) {	
			fscanf(file, "%d", &Manual_Control_Rudder); fclose(file);
		}
		file = fopen("/tmp/sailboat/Manual_Control_Sail", "r");
		if (file != NULL) {	
			fscanf(file, "%d", &Manual_Control_Sail); fclose(file);
		}

	}
}


/*
 *	Move the rudder to the desired position.
 *	Write the desired angle to a file [Navigation_System_Rudder] to be handled by another process 
 */
void move_rudder(int angle) {
	file = fopen("/tmp/sailboat/Navigation_System_Rudder", "w");
	if (file != NULL) {	
		fprintf(file, "%d", angle);	fclose(file);
	}
}

/*
 *	Read START and END point coordinated (latitude and longitude)
 *	Coordinates format is DD
 */
void read_coordinates() {

	file = fopen("/tmp/sailboat/Point_Start_Lat", "r");
	if (file != NULL) {	
		fscanf(file, "%f", &Point_Start_Lat); fclose(file);
	}
	file = fopen("/tmp/sailboat/Point_Start_Lon", "r");
	if (file != NULL) {	
		fscanf(file, "%f", &Point_Start_Lon); fclose(file);
	}
	file = fopen("/tmp/sailboat/Point_End_Lat", "r");
	if (file != NULL) {
		fscanf(file, "%f", &Point_End_Lat);	fclose(file);
	}
	file = fopen("/tmp/sailboat/Point_End_Lon", "r");
	if (file != NULL) {	
		fscanf(file, "%f", &Point_End_Lon);	fclose(file);
	}

}


/*
 *	GUIDANCE V2:
 *
 *	Calculate the desired Heading based on the WindDirection, StartPoint, EndPoint, CurrentPosition and velocity.
 *
 *	This version is able to perform tacking and jibing. Compared to V1, it needs another input theta, which is the
 *	current heading of the vessel. Also it has two more outputs: 'sig' and 'dtheta'. They are used in tacking situations,
 *	putting the rudder on the desired angle. This leads to changes in the rudder-pid-controller, see below.
 *
 *	Daniel Wrede, May 2013
 */
void guidance() 
{
	// GUIDANCE V2: nogozone, tack and jibe capable solution
	//	- Let's try to keep the order using sig,sig1,sig2,sig3 and theta_d,theta_d1. theta_d_b is actually needed in the chooseManeuver function.
 	//	- lat and lon translation would be better on the direct input
	float x, y, theta_wind;
	float _Complex Geo_X, Geo_X0, Geo_X_T;
	
	printf("theta_d: %4.1f deg. \n",theta_d*180/PI);
		
	x=Longitude;
	y=Latitude;
	theta=Heading*PI/180;
	theta_wind=Wind_Angle*PI/180;

	// complex notation for x,y position of the starting point
	Geo_X0 = Point_Start_Lon + 1*I*Point_Start_Lat;
	X0 = 0 + 1*I*0;

	// complex notation for x,y position of the boat
	Geo_X = x + 1*I*y;
	X=(Geo_X-Geo_X0);
	X=creal(X)*CONVLON + I*cimag(X)*CONVLAT;

	// complex notation for x,y position of the target point
	Geo_X_T = Point_End_Lon + I*Point_End_Lat;
	X_T=(Geo_X_T-Geo_X0);
	X_T=creal(X_T)*CONVLON + I*cimag(X_T)*CONVLAT;


	// ** turning matrix **

	// The calculations in the guidance system are done assuming constant wind
	// from above. To make this system work, we need to 'turn' it according to
	// the wind direction. It is like looking on a map, you have to find the
	// north before reading it.

	// Using theta_wind to transfer X_T. Here theta_wind is expected to be zero
	// when coming from north, going clockwise in radians.
	X_T_b = ccos(atan2(cimag(X_T),creal(X_T))+theta_wind)*cabs(X_T) + 1*I*(csin(atan2(cimag(X_T),creal(X_T))+theta_wind)*cabs(X_T));
	
	X_b = ccos(atan2(cimag(X),creal(X))+theta_wind)*cabs(X) + 1*I*(csin(atan2(cimag(X),creal(X))+theta_wind)*cabs(X));

	// theta_d_b = theta_d + theta_wind; 		<- This is wrong; should be: theta_d_b = theta_d1_b at the end of guidance system.
	theta_b = theta_wind - theta + PI/2;
	
	//if (sig == 0)  { findAngle(); }
	//else {
		//theta_d1_b = theta_d_b;
		//sig1 = sig;
	//}
findAngle();
	printf("theta_d1: %4.1f deg. \n",theta_d1*180/PI);

	if (sig1 == 1) { chooseManeuver();  }
	else {	sig2 = sig1; }

	if (sig2 > 1)  { performManeuver(); }
	else {	sig3 = sig2; }

	printf("SIG1: [%d] - SIG2: [%d] - SIG3: [%d] \n",sig1, sig2, sig3);

	// Updating the history angle, telling the guidance heading from last iteration
	theta_d_b = theta_d1_b;
	
	// Inverse turning matrix
	theta_d1 = theta_d1_b-theta_wind;
	Guidance_Heading = (PI/2 - theta_d1) * 180/PI; 
	theta_d = theta_d1;		// Theta_d is the input, while theta_d1 is the output of this function. 
	sig = sig3;
	// When translating from Matlab code to C++, this seems an easy way of translation.


	// write guidance_heading to file to be displayed in GUI 
	file = fopen("/tmp/sailboat/Guidance_Heading", "w");
	if (file != NULL) {
		fprintf(file, "%4.1f", Guidance_Heading);
		fclose(file);
	}

}


void findAngle() 
{
	bool inrange;
	float a_x, b_x;
	float theta_LOS, theta_l, theta_r;
	float _Complex Xl, Xr;
	
	// deadzone limit direction
	Xl = -sin(theta_nogo)*2.8284 + I*cos(theta_nogo)*2.8284;	//-2 + 2*1*I;
	Xr = sin(theta_nogo)*2.8284 + I*cos(theta_nogo)*2.8284;		//2 + 2*1*I;	

	// definition of the different angles
	theta_LOS = atan2(cimag(X_T_b)-cimag(X_b),creal(X_T_b)-creal(X_b));
	theta_l = atan2(cimag(exp(-1*I*theta_LOS)*Xl),creal(cexp(-1*I*theta_LOS)*Xl));
	theta_r = atan2(cimag(exp(-1*I*theta_LOS)*Xr),creal(cexp(-1*I*theta_LOS)*Xr));

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
	
	printf("a_x: %f \n",a_x);
	b_x = TACKINGRANGE / (2 * sin(theta_LOS));

	// stop signal
	if (cabs(X_T - X) < RADIUSACCEPTED)	{ inrange = true; }
	else { inrange = false; }

	// compute the next theta_d, ie at time t+1
	// (main algorithm)
	if (inrange)	// if stopping signal is sent, then head up against the wind 
	{ 			
	    theta_d1_b = PI*3/2;
		sig1 = 0;
		printf(">> debug 1 \n");
		fa_debug=1;
	}
	else
	{
		if (!(  (atan2(cimag(Xr),creal(Xr))-PI/9)<=theta_LOS  &&  theta_LOS<=(atan2(cimag(Xl),creal(Xl))+PI/9)  ))
		{
			// if theta_LOS is outside of the deadzone
			theta_d1_b = theta_LOS;
			sig1 = 1;
			printf(">> debug 2 \n");
			fa_debug=2;
		}
		else
		{
			printf("theta_d_b: %f \n",theta_d_b);
			printf("atan2 Xl: %f \n",atan2(cimag(Xl),creal(Xl)));
			printf("atan2 Xr: %f \n",atan2(cimag(Xr),creal(Xr)));

			if (theta_d_b >= atan2(cimag(Xl),creal(Xl))-PI/36  && theta_d_b <= atan2(cimag(Xl),creal(Xl))+PI/36 )
			{
				if (creal(X_b) < a_x*cimag(X_b)-b_x) { theta_d1_b = atan2(cimag(Xr),creal(Xr)); printf(">> debug 3 \n"); fa_debug=3; sig1=1;}     
				else { theta_d1_b = theta_d_b; printf(">> debug 4 \n"); fa_debug=4; sig1=0;}
		    } 
			else
			{
				if (  (theta_d_b >= (atan2(cimag(Xr),creal(Xr))-(PI/36)))  &&  (theta_d_b <= (atan2(cimag(Xr),creal(Xr))+(PI/36))) )
				{
					if (creal(X_b) > a_x*cimag(X_b)+b_x) { theta_d1_b = atan2(cimag(Xl),creal(Xl)); printf(">> debug 5 \n"); fa_debug=5; sig1=1;}
					else { theta_d1_b = theta_d_b; printf(">> debug 6 \n"); fa_debug=6; sig1=0;}
				}
				else
				{
					if(cabs(theta_l) < cabs(theta_r)) { theta_d1_b = atan2(cimag(Xl),creal(Xl)); printf(">> debug 7 \n"); fa_debug=7; sig1=1;}
					else { theta_d1_b = atan2(cimag(Xr),creal(Xr)); printf(">> debug 8 \n"); fa_debug=8; sig1=1;}
				}
			}
		}
	}
}	

void chooseManeuver() 
{
	// The maneuver function does the maneuvers. This function performs each/every course change. 
	// Here it decides whether there is need for a tack, jibe or just a little course change. 
	// This decision incorporates two steps: 1. Is the desired heading on the other side of the deadzone? 
	// Then we need to jibe or tack. 2. Do we have enough speed for tacking? According to this it chooses sig.

	float dAngle, d1Angle;
	float _Complex X_d_b, X_d1_b;

	dAngle = PI/2 - theta_d_b;		// The two angle distances to wind direction clockwise.
	dAngle = atan2(sin(dAngle),cos(dAngle));
	d1Angle = PI/2 - theta_d1_b;		
	d1Angle = atan2(sin(d1Angle),cos(d1Angle));
	
	// Definition of X_d1_b and X_d_b
	X_d_b = ccos(theta_d_b) + I*(csin(theta_d_b));
	//X_d_b = exp(1i*theta_d_b);
	//X_d1_b = exp(1i*theta_d1_b);
	X_d1_b = ccos(theta_d1_b) + I*(csin(theta_d1_b));

	if (cimag(X_d1_b) > 0 && cimag(X_d_b) > 0 && SOG > v_min)
	{
		// If the old heading and the new heading is close to the wind, then tack.
		sig2 = 2;
	}
	else 
	{
		if (abs(d1Angle-dAngle) < PI/4) 
		{
			// For small deviations, course change.
			sig2 = 2;
		}
		else
		{
			if (d1Angle > dAngle)
			{
				// jibe over port (going left/counterclockwise around)
				sig2 = 3;		
			}
			else
			{
				if (d1Angle < dAngle)
				{
					// jibe over starboard (going right/clockwise around)
					sig2 = 4;	
				}
				else
				{
					// This else shouldn’t occur, but as a safety it is implemented.
					sig2 = 2;		
					printf("\n >>>>>> you can't see me! >>>>>>>>>>>>>> \n");
				}
			}
		}
	}
}	

void performManeuver()
{
	theta_b=atan2(sin(theta_b),cos(theta_b)); // avoiding singularity

	printf("theta_b: %f\n",theta_b);
	printf("theta_d1_b: %f\n",theta_d1_b);

	switch(sig2)
	{
		case 2:  // Tack / course change
			if (      ((theta_d1_b-PI/3) < theta_b) && (theta_b < (theta_d1_b+PI/3))  )  // If heading is close to desired heading
			{
				sig3 = 0; // As soon as we’re close to the desired heading, the jibe is finished and the boat will head for theta_d1.
			}
			else
			{
				sig3 = sig2;
			}
			break;
		case 3:  // Jibe right
			if ((theta_d1_b-PI/3) < theta_b && theta_b < (theta_d1_b+PI/3))  // If heading is close to desired heading
			{
				sig3 = 0; // As soon as we’re close to the desired heading, the jibe is finished and the boat will head for theta_d1.
				//disp('stop jibe left')
			}
			else			
			{
				sig3 = sig2;
			}
			break;
		case 4:  // Jibe left
			if ((theta_d1_b-PI/3) < theta_b && theta_b < (theta_d1_b+PI/3))  // If heading is close to desired heading
			{
				sig3 = 0; // As soon as we’re close to the desired heading, the jibe is finished and the boat will head for theta_d1.
				//disp('stop jibe right')
			}
			else			
			{
				sig3 = sig2;
			}
			break;
		default:
			sig3 = sig2;
			printf("\n default in perform manoeuvre function \n");
	}
}


/*
 *	RUDDER PID CONTROLLER:
 *
 *	Calculate the desired RUDDER ANGLE position based on the Target Heading and Current Heading.
 *	The result is a rounded value of the angle stored in the [Rudder_Desired_Angle] global variable
 */
void rudder_pid_controller() {
// Daniel Wrede, May 2013

	float dHeading, pValue, integralValue, temp_ang;
	
	dHeading = Guidance_Heading - Heading; // in degrees
	
	// Singularity translation
	dHeading = dHeading*PI/180;
	dHeading = atan2(sin(dHeading),cos(dHeading));
	dHeading = dHeading*180/PI;	
	
	printf("dHeading: %f\n",dHeading);

	// fprintf(stdout,"targetHeafing: %f, deltaHeading: %f\n",targetHeading, dHeading);
	//if (abs(dHeading) > dHEADING_MAX && abs(Rate) > RATEOFTURN_MAX) // Limit control statement
	//{
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
		temp_ang = pValue + integralValue; // Angle in radians
	//}
	// fprintf(stdout,"pValue: %f, integralValue: %f\n",pValue,integralValue);
	// fprintf(stdout,"Rudder_Desired_Angle: %d\n\n",Rudder_Desired_Angle);
	if ( sig == 3)	// Added by Daniel. When performing jibing manoeuvres, this is needed.
	{
		temp_ang = JIBE_ANGLE; // Angle in radians
	}
	if ( sig == 4)	// Added by Daniel. When performing jibing manoeuvres, this is needed.
	{
		temp_ang = -JIBE_ANGLE; // Angle in radians
	}
	
	Rudder_Desired_Angle = round(temp_ang);
	if(Rudder_Desired_Angle > 35) {Rudder_Desired_Angle=35; }
	if(Rudder_Desired_Angle < -35) {Rudder_Desired_Angle=-35; }
}




/*
 *	Read data from the Weather Station
 */
void read_weather_station() {

	//RATE OF TURN
	file = fopen("/tmp/u200/Rate", "r");
	if (file != NULL) {
		fscanf(file, "%f", &Rate);
		fclose(file);
	} else {
		printf("ERROR: Files from Weather Station are missing.\n");
		exit(1);
	}

	//VESSEL HEADING
	file = fopen("/tmp/u200/Heading", "r");
	if (file != NULL) { 
		fscanf(file, "%f", &Heading); fclose(file);
	}
	file = fopen("/tmp/u200/Deviation", "r");
	if (file != NULL) {
		fscanf(file, "%f", &Deviation);	fclose(file);
	}
	file = fopen("/tmp/u200/Variation", "r");
	if (file != NULL) {
		fscanf(file, "%f", &Variation);	fclose(file);
	}

	//ATTITUDE
	file = fopen("/tmp/u200/Yaw", "r");
	if (file != NULL) {
		fscanf(file, "%f", &Yaw); fclose(file);
	}
	file = fopen("/tmp/u200/Pitch", "r");
	if (file != NULL) {
		fscanf(file, "%f", &Pitch);	fclose(file);
	}
	file = fopen("/tmp/u200/Roll", "r");
	if (file != NULL) {
		fscanf(file, "%f", &Roll);	fclose(file);
	}

	//GPS_DATA
	file = fopen("/tmp/u200/Latitude", "r");
	if (file != NULL) {
		fscanf(file, "%f", &Latitude); fclose(file);
	}
	file = fopen("/tmp/u200/Longitude", "r");
	if (file != NULL) {
		fscanf(file, "%f", &Longitude);	fclose(file);
	}
	file = fopen("/tmp/u200/COG", "r");
	if (file != NULL) {
		fscanf(file, "%f", &COG); fclose(file);
	}
	file = fopen("/tmp/u200/SOG", "r");
	if (file != NULL) {
		fscanf(file, "%f", &SOG); fclose(file);
	}
	//WIND_DATA
	file = fopen("/tmp/u200/Wind_Speed", "r");
	if (file != NULL) {
		fscanf(file, "%f", &Wind_Speed); fclose(file);
	}
	file = fopen("/tmp/u200/Wind_Angle", "r");
	if (file != NULL) {
		fscanf(file, "%f", &Wind_Angle); fclose(file);
	}

}

/*
 *	Save all the variables of the navigation system in a log file in /var/tmp/sailboat-log/
 *	Create a new log file every 10 minutes
 */
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

		dirp = opendir("sailboat-log/"); 

		while ((entry = readdir(dirp)) != NULL) {
			if (entry->d_type == DT_REG) { 
				 file_count++;
			}
		}
		closedir(dirp);

		// calculate filename
		sprintf(logfile,"sailboat-log/logfile_%.4d",file_count);

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

	// read rudder feedback
	file2 = fopen("/tmp/sailboat/Rudder_Feedback", "r");
	if (file2 != NULL) {
		fscanf(file2, "%d", &Rudder_Feedback);	
		fclose(file2);
	}

	// generate CSV log line
	sprintf(logline, "%u,%d,%d,%4.1f,%d,%d,%d,%d,%d,%d,%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%4.1f,%f,%f,%f,%f,%f,%f,%f" \
		, (unsigned)time(NULL) \
		, Navigation_System \
		, Manual_Control \
		, Guidance_Heading \
		, Rudder_Desired_Angle \
		, Manual_Control_Rudder \
		, Rudder_Feedback \
		, sig1
		, sig2
		, sig3
		, fa_debug
		, theta_d1
		, theta_d
		, theta_d1_b
		, theta_b
		, Rate ,Heading ,Deviation ,Variation ,Yaw ,Pitch ,Roll ,Latitude ,Longitude ,COG ,SOG ,Wind_Speed ,Wind_Angle \
		, Point_Start_Lat ,Point_Start_Lon ,Point_End_Lat ,Point_End_Lon \
	);

	// write to file
	file2 = fopen(logfile, "a");
	if (file2 != NULL) {
		fprintf(file2, "%s\n", logline);
		fclose(file2);
	}

	fa_debug=0;
	logEntry++;
}
