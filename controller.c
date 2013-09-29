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
#define RADIUSACCEPTED	10		// meters
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

#define SIM_DISTANCE	8		// [meters] displacement in simulated coordinates at every iteration

#include "map_geometry.h"		// custom functions to handle geometry transformations on the map


FILE* file;
float Rate=0, Heading=0, Deviation=0, Variation=0, Yaw=0, Pitch=0, Roll=0;
float Latitude=0, Longitude=0, COG=0, SOG=0, Wind_Speed=0, Wind_Angle=0;
float Point_Start_Lat=0, Point_Start_Lon=0, Point_End_Lat=0, Point_End_Lon=0, Area_Center_Lat=0, Area_Center_Lon=0;
int   Rudder_Desired_Angle=0, Manual_Control_Rudder=0, Manual_Control_Sail=0, Rudder_Feedback=0, Area_Side=0, Area_Interval=0;
int   Navigation_System=0, Prev_Navigation_System=0, Manual_Control=0, simulation_stop=0;
int   logEntry=0, logCount=0, fa_debug=0, debug=0;
char  logfile1[50],logfile2[50];

void initfiles();
void check_navigation_system();
void onNavChange();
void read_weather_station();
void move_rudder(int angle);
void move_sail(int angle);
void read_endpoint_coordinates();
void write_log_file();

struct timespec timermain;


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


// area waypoints
int i=0, k=0, nwaypoints=0, current_waypoint=0;
Point Waypoints[1000];

void calculate_area_waypoints();
void move_to_simulated_position();



int main(int argc, char ** argv) {
	
	// set timers
	timermain.tv_sec  = MAINSLEEP_SEC;
	timermain.tv_nsec = MAINSLEEP_MSEC * 1000000L;

	initfiles();
	fprintf(stdout, "\nSailboat-controller running..\n");
	read_weather_station();

	// MAIN LOOP
	while (1) {

		// read GUI configuration files (navigation system and manual control values)
		check_navigation_system(); if (Navigation_System != Prev_Navigation_System) onNavChange();


		if (Manual_Control) {

			move_rudder(Manual_Control_Rudder);		// Move the rudder to user position
			move_sail(Manual_Control_Sail);			// Move the main sail to user position

		} else {

			// AUTOPILOT ON
			if ((Navigation_System==1)||(Navigation_System==2)||(Navigation_System==3)) {

				read_weather_station();			// Update sensors data
				guidance();				// Calculate the desired heading
				rudder_pid_controller();		// Calculate the desired rudder position
				move_rudder(Rudder_Desired_Angle);	// Move rudder

				// reaching the waypoint
				if (cabs(X_T - X) < RADIUSACCEPTED) {	

					if (Navigation_System==1) {
						// maintain position
						file = fopen("/tmp/sailboat/Navigation_System", "w");
						if (file != NULL) { fprintf(file, "3");	fclose(file); }
					}
					if (Navigation_System==2) {
						if (current_waypoint<nwaypoints-1) {
							// go to the next waypoint		
							current_waypoint++;
							Point_Start_Lon=Waypoints[current_waypoint-1].x;
							Point_Start_Lat=Waypoints[current_waypoint-1].y;			
							Point_End_Lon=Waypoints[current_waypoint].x;		
							Point_End_Lat=Waypoints[current_waypoint].y;
							file = fopen("/tmp/sailboat/Point_End_Lat", "w");
							if (file != NULL) { fprintf(file, "%f", Point_End_Lat); fclose(file); }
							file = fopen("/tmp/sailboat/Point_End_Lon", "w");
							if (file != NULL) { fprintf(file, "%f", Point_End_Lon); fclose(file); }
						}
						else{
							// maintain position
							file = fopen("/tmp/sailboat/Navigation_System", "w");
							if (file != NULL) { fprintf(file, "3");	fclose(file); }
						}
					}
				}
			}

			// SIMULATION
			if (Navigation_System==5 && simulation_stop==0) {

				guidance();				// Calculate the desired heading
				Heading=Guidance_Heading;		// Overwrite Heading
				move_to_simulated_position();		// Overwrite Latitude and Longitude with simulated coordinates
				
				// reaching the waypoint
				if (cabs(X_T - X) < RADIUSACCEPTED) {
					if (current_waypoint<nwaypoints-1) {
						// keep on simulating to the next waypoint		
						current_waypoint++;
						Point_Start_Lon=Waypoints[current_waypoint-1].x;
						Point_Start_Lat=Waypoints[current_waypoint-1].y;			
						Point_End_Lon=Waypoints[current_waypoint].x;		
						Point_End_Lat=Waypoints[current_waypoint].y;
						file = fopen("/tmp/sailboat/Point_End_Lat", "w");
						if (file != NULL) { fprintf(file, "%f", Point_End_Lat); fclose(file); }
						file = fopen("/tmp/sailboat/Point_End_Lon", "w");
						if (file != NULL) { fprintf(file, "%f", Point_End_Lon); fclose(file); }
						printf("SIM: Waypoint Reached\n");
					}
					else {
						// terminate path simulation on clients
						file = fopen("/tmp/sailboat/Simulated_Lat", "w");
						if (file != NULL) { fprintf(file, "0"); fclose(file); }
						file = fopen("/tmp/sailboat/Simulated_Lon", "w");
						if (file != NULL) { fprintf(file, "0"); fclose(file); }
						simulation_stop=1;
						printf("SIM: Simulation Stopped\n");
					}
				}
			}
			
			// write a log line every N samples
			if (Navigation_System!=0) {
				logCount++;
				if(logCount>=1) {write_log_file(); logCount=0;}	
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
	system("mkdir -p sailboat-log/debug/");

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
	system("[ ! -f /tmp/sailboat/Simulated_Lat ] 		&& echo 0 > /tmp/sailboat/Simulated_Lat");
	system("[ ! -f /tmp/sailboat/Simulated_Lon ] 		&& echo 0 > /tmp/sailboat/Simulated_Lon");
	system("[ ! -f /tmp/sailboat/Area_VertexNum ] 		&& echo 0 > /tmp/sailboat/VertexNum");
	system("[ ! -f /tmp/sailboat/Area_Interval ] 		&& echo 0 > /tmp/sailboat/Area_Interval");
	system("[ ! -f /tmp/sailboat/Guidance_Heading ] 	&& echo 0 > /tmp/sailboat/Guidance_Heading");
	system("[ ! -f /tmp/sailboat/Rudder_Feedback ] 		&& echo 0 > /tmp/sailboat/Rudder_Feedback");
}

/*
 *	Check Navigation System Status
 *
 *	[Navigation System] 
 *		- [0] Boat in IDLE status
 *		- [1] Control System ON, Sail to the waypoint
 *		- [2] Control System ON, Map area
 *		- [3] Control System ON, Mantain upwind position
 *		- [4] Simulation, Sail to Waypoint
 *		- [5] Simulation, Map Area
 *	[Manual Control]
 *		- [0] OFF
 *		- [1] User takes control of sail and rudder positions
 *
 *	if Manual_Control is ON, read the following values:
 *		- [Manual_Control_Rudder] : user value for desired RUDDER angle [-30.0 to 30.0]
 *		- [Manual_Control_Sail]   : user value for desired SAIL angle 
 */
void check_navigation_system() {

	file = fopen("/tmp/sailboat/Navigation_System", "r");
	if (file != NULL) { fscanf(file, "%d", &Navigation_System); fclose(file); }

	file = fopen("/tmp/sailboat/Manual_Control", "r");
	if (file != NULL) { fscanf(file, "%d", &Manual_Control); fclose(file); }

	if(Manual_Control) {

		file = fopen("/tmp/sailboat/Manual_Control_Rudder", "r");
		if (file != NULL) { fscanf(file, "%d", &Manual_Control_Rudder); fclose(file); }

		file = fopen("/tmp/sailboat/Manual_Control_Sail", "r");
		if (file != NULL) { fscanf(file, "%d", &Manual_Control_Sail); fclose(file);}
	}
}


/*
 *	Whenever a new Navigation_System state is entered:
 */
void onNavChange() {
	
	Point wp;

	read_weather_station();					// Update current position, wind angle, heading, ...

	// SWITCH AUTOPILOT OFF
	if(Navigation_System==0) {

		// do nothing
	}

	// START SAILING 
	if(Navigation_System==1) {

		if (Prev_Navigation_System==0) {		// START sailing after Simulation

			Point_Start_Lon=Latitude;		// Set START and END point coordinates	
			Point_Start_Lat=Longitude;
			read_endpoint_coordinates();
			wp.x=Point_End_Lon;			// Save End point in Waypoints array
			wp.y=Point_End_Lat;
			Waypoints[0]=wp;
		}

		if (Prev_Navigation_System==3) {		// RESUME sailing 
			
			Point_End_Lon=Waypoints[0].x;		
			Point_End_Lat=Waypoints[0].y;
		}
	}

	// START MAPPING AN AREA
	if(Navigation_System==2) {
		
		if (Prev_Navigation_System==5) {		// START MAPPING after simulation

			current_waypoint=0;
			Point_Start_Lon=Longitude;		// Set START and END point coordinates
			Point_Start_Lat=Latitude;			
			Point_End_Lon=Waypoints[0].x;		
			Point_End_Lat=Waypoints[0].y;
		}
		
		if (Prev_Navigation_System==3) {		// RESUME MISSION: Overwrite START and END point coordinates

			if ( current_waypoint != 0) {
				Point_Start_Lon = Waypoints[current_waypoint-1].x;
				Point_Start_Lat = Waypoints[current_waypoint-1].y;
			}
			Point_End_Lon   = Waypoints[current_waypoint].x;
			Point_End_Lat   = Waypoints[current_waypoint].y;
		}
	}

	// MAINTAIN POSITION
	if(Navigation_System==3) {				
		
		Point_End_Lon = Longitude;			// Overwrite temporarily the END point coord. using the current position
		Point_End_Lat = Latitude;
	}
	
	// SIMULATE - WAYPOINT
	if(Navigation_System==4) {

		// not implemented
	}

	// SIMULATE - MAP AREA
	if(Navigation_System==5) {

		simulation_stop=0;
		calculate_area_waypoints();		// Calculate the Waypoint array
		current_waypoint=0;
		Point_Start_Lon=Longitude;		// Set START and END point coordinates
		Point_Start_Lat=Latitude;			
		Point_End_Lon=Waypoints[0].x;		
		Point_End_Lat=Waypoints[0].y;
	}

	file = fopen("/tmp/sailboat/Point_End_Lat", "w");
	if (file != NULL) { fprintf(file, "%f", Point_End_Lat); fclose(file); }
	file = fopen("/tmp/sailboat/Point_End_Lon", "w");
	if (file != NULL) { fprintf(file, "%f", Point_End_Lon); fclose(file); }

	Prev_Navigation_System=Navigation_System;
}


/*
 *	Move the rudder to the desired position.
 *	Write the desired angle to a file [Navigation_System_Rudder] to be handled by another process 
 */
void move_rudder(int angle) {
	file = fopen("/tmp/sailboat/Navigation_System_Rudder", "w");
	if (file != NULL) { fprintf(file, "%d", angle);	fclose(file); }
}

/*
 *	Move the main sail to the desired position.
 *	Write the desired angle to a file [Navigation_System_Sail] to be handled by another process 
 */
void move_sail(int angle) {
	file = fopen("/tmp/sailboat/Navigation_System_Sail", "w");
	if (file != NULL) { fprintf(file, "%d", angle);	fclose(file); }
}

/*
 *	Read END point coordinates (latitude and longitude)
 *	Coordinates format is DD
 */
void read_endpoint_coordinates() {

	file = fopen("/tmp/sailboat/Point_End_Lat", "r");
	if (file != NULL) { fscanf(file, "%f", &Point_End_Lat);	fclose(file); }

	file = fopen("/tmp/sailboat/Point_End_Lon", "r");
	if (file != NULL) { fscanf(file, "%f", &Point_End_Lon);	fclose(file);}
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
	
	if (debug) printf("theta_d: %4.1f deg. \n",theta_d*180/PI);
		
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
	if (debug) printf("theta_d1: %4.1f deg. \n",theta_d1*180/PI);

	if (sig1 == 1) { chooseManeuver();  }
	else {	sig2 = sig1; }

	if (sig2 > 1)  { performManeuver(); }
	else {	sig3 = sig2; }

	if (debug) printf("SIG1: [%d] - SIG2: [%d] - SIG3: [%d] \n",sig1, sig2, sig3);

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
	
	if (debug) printf("a_x: %f \n",a_x);
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
		if (debug) printf(">> debug 1 \n");
		fa_debug=1;
	}
	else
	{
		if (!(  (atan2(cimag(Xr),creal(Xr))-PI/9)<=theta_LOS  &&  theta_LOS<=(atan2(cimag(Xl),creal(Xl))+PI/9)  ))
		{
			// if theta_LOS is outside of the deadzone
			theta_d1_b = theta_LOS;
			sig1 = 1;
			if (debug) printf(">> debug 2 \n");
			fa_debug=2;
		}
		else
		{
			if (debug) printf("theta_d_b: %f \n",theta_d_b);
			if (debug) printf("atan2 Xl: %f \n",atan2(cimag(Xl),creal(Xl)));
			if (debug) printf("atan2 Xr: %f \n",atan2(cimag(Xr),creal(Xr)));

			if (theta_d_b >= atan2(cimag(Xl),creal(Xl))-PI/36  && theta_d_b <= atan2(cimag(Xl),creal(Xl))+PI/36 )
			{
				if (creal(X_b) < a_x*cimag(X_b)-b_x) { theta_d1_b = atan2(cimag(Xr),creal(Xr)); if (debug) printf(">> debug 3 \n"); fa_debug=3; sig1=1;}     
				else { theta_d1_b = theta_d_b; if (debug) printf(">> debug 4 \n"); fa_debug=4; sig1=0;}
		    } 
			else
			{
				if (  (theta_d_b >= (atan2(cimag(Xr),creal(Xr))-(PI/36)))  &&  (theta_d_b <= (atan2(cimag(Xr),creal(Xr))+(PI/36))) )
				{
					if (creal(X_b) > a_x*cimag(X_b)+b_x) { theta_d1_b = atan2(cimag(Xl),creal(Xl)); if (debug) printf(">> debug 5 \n"); fa_debug=5; sig1=1;}
					else { theta_d1_b = theta_d_b; if (debug) printf(">> debug 6 \n"); fa_debug=6; sig1=0;}
				}
				else
				{
					if(cabs(theta_l) < cabs(theta_r)) { theta_d1_b = atan2(cimag(Xl),creal(Xl)); if (debug) printf(">> debug 7 \n"); fa_debug=7; sig1=1;}
					else { theta_d1_b = atan2(cimag(Xr),creal(Xr)); if (debug) printf(">> debug 8 \n"); fa_debug=8; sig1=1;}
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

	if (debug) printf("theta_b: %f\n",theta_b);
	if (debug) printf("theta_d1_b: %f\n",theta_d1_b);

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
			if (debug) printf("\n default in perform manoeuvre function \n");
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
	
	if (debug) printf("dHeading: %f\n",dHeading);

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
 *	Calculate the waypoint positions to map an area defined as a convex polygon
 */
void calculate_area_waypoints() {
	
	// local variable definitions
	int interval=0, nvertexes=0, next=0, d=-1, sw=0;
	double yMin, yMax, xMin, xMax, yRef;
	Point pw1, pw2, wayp, tmpW;
	Point tWaypoints[1000];
	Line lw,lp;
	char vfile[30];
	
	// reset previous waypoints
	nwaypoints=0;

	// read user values from files
	file = fopen("/tmp/sailboat/Area_Interval", "r");
	if (file != NULL) { fscanf(file, "%d", &interval); fclose(file); }
	file = fopen("/tmp/sailboat/Area_VertexNum", "r");
	if (file != NULL) { fscanf(file, "%d", &nvertexes); fclose(file); }
	
	Point Vertex[nvertexes];
	for (i=0; i<nvertexes; i++) {
		sprintf(vfile,"/tmp/sailboat/Area_v%d",i);
		file = fopen(vfile, "r");
		if (file != NULL) { fscanf(file, "%lf;%lf", &Vertex[i].x, &Vertex[i].y); fclose(file); }	
	}

	// convert vertex coordinates to XY plane
	for (i=0; i<nvertexes; i++) Vertex[i] = convert_xy(Vertex[i]);

	// rotate vertex coordinates accoringly to wind direction
	for (i=0; i<nvertexes; i++) Vertex[i] = rotate_point(Vertex[i],Wind_Angle);

	// find closest and farthest vertexes
	yMin=Vertex[0].y; yMax=Vertex[0].y;
	for (i=1; i<nvertexes; i++) {
		if (Vertex[i].y < yMin) yMin=Vertex[i].y;
		if (Vertex[i].y > yMax) yMax=Vertex[i].y;
	}

	// calculate how many intersecting lines
	int nlines=floor(abs(yMax-yMin)/interval)+1;

	// for each side of the polygon
	for (i=0; i<nvertexes; i++) {
			
		yRef=yMin;
		next=i+1; if (next==nvertexes) next=0;
		xMin=Vertex[i].x;
		xMax=Vertex[next].x;
		if (Vertex[i].x > xMax) { xMin=xMax; xMax=Vertex[i].x; }
		lp=new_line(Vertex[i],Vertex[next]);
			
		// for each intersecting line
		for (k=0; k<nlines; k++) {
			
			// find valid intersections
			pw1=new_point(xMin,yRef);
			pw2=new_point(xMax,yRef);
			lw=new_line(pw1,pw2);
			
			wayp=find_intersection(lp,lw);
			
			if (wayp.x >= xMin && wayp.x <= xMax) {		// in the segment
				if (wayp.x != 0 || wayp.y != 0) {	// non parallel lines
					tWaypoints[nwaypoints]=wayp;	// add to temp Array
					nwaypoints++;
				}
			}
			yRef+= (float) interval;			// rise the bar
		}
	}
		
	// sort the waypoints by Y ascending
	qsort(tWaypoints, nwaypoints, sizeof(Point), cmpfunc);
	
	// eliminate the first waypoint duplicated since in the vertex)
	for (i=1; i<nwaypoints; i++) Waypoints[i-1]=tWaypoints[i];
	nwaypoints-=1;

	// alternate waypoints by X coordinates (let's do a S-like zig-zag as left-left-right-right oh-yeah)
	// I hereby name this algorithm "the headless alternated paired bubble sorting" a.k.a. "the zig"	
	for (i=1; i<nwaypoints; i+=2) {

		if ((d<0) && (Waypoints[i].x > Waypoints[i+1].x)) sw=1;
		if ((d>0) && (Waypoints[i].x < Waypoints[i+1].x)) sw=1;

		if (sw==1) {
			tmpW = Waypoints[i];
			Waypoints[i]   = Waypoints[i+1];
			Waypoints[i+1] = tmpW;
		}
		sw=0; d*=-1;
	}

	// requested on 28 Sep 2013: Once mapped an area, map it backward again
	for (i=nwaypoints-2; i>=0; i--) {
		Waypoints[nwaypoints+(nwaypoints-i)-2]=Waypoints[i];
	}
	nwaypoints=(2*nwaypoints)-2;

	// rotate back the point coordinates for the map
	for (i=0; i<nwaypoints; i++) Waypoints[i] = rotate_point(Waypoints[i],-1*Wind_Angle);

	// convert coordinates back to Latitude-Longitude
	for (i=0; i<nwaypoints; i++) Waypoints[i] = convert_latlon(Waypoints[i]);
}


void move_to_simulated_position(){


	double alpha = (90-Guidance_Heading)*PI/180;
	
	double SimLon= ( (Longitude*CONVLON) + SIM_DISTANCE*cosf(alpha) )/CONVLON;
	double SimLat= ( (Latitude*CONVLAT)  + SIM_DISTANCE*sinf(alpha) )/CONVLAT;
	
	file = fopen("/tmp/sailboat/Simulated_Lat", "w");
	if (file != NULL) { fprintf(file, "%f", SimLat); fclose(file); }

	file = fopen("/tmp/sailboat/Simulated_Lon", "w");
	if (file != NULL) { fprintf(file, "%f", SimLon); fclose(file); }

	Latitude=(float)SimLat;
	Longitude=(float)SimLon;
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
 *	Create a new log file every MAXLOGLINES rows
 */
void write_log_file() {

	FILE* file2;
	DIR * dirp;
	char  logline[1000];
	char  timestp[25];

	time_t rawtime;
	struct tm  *timeinfo;
	struct dirent * entry;

	// crate a new file every MAXLOGLINES
	if(logEntry==0 || logEntry>=MAXLOGLINES) {

		// pin pointer
		file2 = fopen("sailboat-log/current_logfile", "w");
		if (file2 != NULL) { fprintf(file2, "init"); fclose(file2); }
				
		//count files in log folder
		int file_count = 0;
		dirp = opendir("sailboat-log/"); 

		while ((entry = readdir(dirp)) != NULL) {
			if (entry->d_type == DT_REG) { 
				 file_count++;
			}
		}
		closedir(dirp);

		// calculate new timestamp
		time(&rawtime);
		timeinfo = localtime(&rawtime);
		strftime(timestp, sizeof timestp, "%Y%m%d_%H%M", timeinfo);
		
		// save log filename for other processes
		file2 = fopen("sailboat-log/current_logfile", "w");
		if (file2 != NULL) { fprintf(file2, "logfile_%.4d_%s",file_count,timestp); fclose(file2); }
		
		// log filename
		sprintf(logfile1,"sailboat-log/logfile_%.4d_%s",file_count,timestp);
		sprintf(logfile2,"sailboat-log/debug/debug_%.4d_%s",file_count,timestp);

		logEntry=1;
	}

	// read rudder feedback
	file2 = fopen("/tmp/sailboat/Rudder_Feedback", "r");
	if (file2 != NULL) { fscanf(file2, "%d", &Rudder_Feedback); fclose(file2); }


	// generate csv LOG line
	sprintf(logline, "%u,%d,%d,%.1f,%d,%d,%d,%.4f,%.4f,%.4f,%.4f,%f,%f,%.1f,%.3f,%.2f,%.2f,%f,%f,%f,%f" \
		, (unsigned)time(NULL) \
		, Navigation_System \
		, Manual_Control \
		, Guidance_Heading \
		, Rudder_Desired_Angle \
		, Manual_Control_Rudder \
		, Rudder_Feedback \
		, Rate \
		, Heading \
		, Pitch \
		, Roll \
		, Latitude \
		, Longitude \
		, COG \
		, SOG \
		, Wind_Speed \
		, Wind_Angle \
		, Point_Start_Lat \
		, Point_Start_Lon \
		, Point_End_Lat \
		, Point_End_Lon \
	);
	// write to LOG file
	file2 = fopen(logfile1, "a");
	if (file2 != NULL) { fprintf(file2, "%s\n", logline); fclose(file2); }


	// generate csv DEBUG line
	sprintf(logline, "%u,%d,%d,%d,%d,%f,%f,%f,%f" \
		, (unsigned)time(NULL) \
		, sig1 \
		, sig2 \
		, sig3 \
		, fa_debug \
		, theta_d1 \
		, theta_d \
		, theta_d1_b \
		, theta_b \
	);
	// write to DEBUG file
	file2 = fopen(logfile2, "a");
	if (file2 != NULL) { fprintf(file2, "%s\n", logline); fclose(file2); }
	

	fa_debug=0;
	logEntry++;
}
