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
#define MAXLOGLINES	30000
#define SEC		4		// number of loops per second

#define PI 		3.14159265

#define TACKINGRANGE 	200		// meters
#define RADIUSACCEPTED	5		// meters
#define CONVLON		64078		// meters per degree
#define CONVLAT		110742		// meters per degree

#define JIBE_ANGLE	40		// [degrees] Rudder angle while jibing
#define theta_nogo	55*PI/180	// [radians] Angle of nogo zone, compared to wind direction
#define theta_down	30*PI/180 	// [radians] Angle of downwind zone, compared to wind direction.
#define v_min		0		// [meters/seconds] Min velocity for tacking
#define angle_lim 	5*PI/180	// [degrees] threshold for jibing. The heading has to be 10 degrees close to desired Heading.
#define ROLL_LIMIT 	5		// [degrees] Threshold for an automatic emergency sail release

#define INTEGRATOR_MAX	20		// [degrees], influence of the integrator
#define RATEOFTURN_MAX	36		// [degrees/second]
#define dHEADING_MAX	10		// [degrees] deviation, before rudder PI acts
#define GAIN_P 		-1
#define GAIN_I 		0

#define BoomLength	1.6 		// [meters] Length of the Boom
#define SCLength	1.43 		// [meters] horizontal distance between sheet hole and mast
#define SCHeight	0.6		// [meters] vertical distance between sheet hole and boom.
#define theta_tack	30 		// [degrees]
#define theta_reach	50 		// [degrees]
#define theta_running	80 		// [degrees]
#define strokelength	1		// [meters] actuator length

#define SAIL_ACT_TIME  3		// [seconds] actuation time of the sail hillclimbing algoritm
#define SAIL_OBS_TIME  20		// [seconds] observation time of the sail hillclimbing algoritm
#define ACT_MAX		870		// [ticks] the max number of actuator ticks
#define SAIL_LIMIT	50		// [ticks] max tolerated difference between desired and current actuator position

#define SIM_SOG		8		// [meters/seconds] boat speed over ground during simulation
#define SIM_ROT		5		// [degrees/seconds] rate of turn
#define SIM_ACT_INC	200		// [millimiters/seconds] sail actuator increment per second

#include "map_geometry.h"		// custom functions to handle geometry transformations on the map


FILE* file;
float Rate=0, Heading=0, Deviation=0, Variation=0, Yaw=0, Pitch=0, Roll=0;
float Latitude=0, Longitude=0, COG=0, SOG=0, Wind_Speed=0, Wind_Angle=0;
float Point_Start_Lat=0, Point_Start_Lon=0, Point_End_Lat=0, Point_End_Lon=0;
int   Rudder_Desired_Angle=0,   Manual_Control_Rudder=0, Rudder_Feedback=0;
int   Sail_Desired_Position=0,  Manual_Control_Sail=0,   Sail_Feedback=0;
int   Navigation_System=0, Prev_Navigation_System=0, Manual_Control=0, Simulation=0;
int   logEntry=0, fa_debug=0, debug=0, debug2=0;
char  logfile1[50],logfile2[50],logfile3[50];

void initfiles();
void check_navigation_system();
void onNavChange();
void read_weather_station();
void read_weather_station_essential();
void read_sail_position();
void move_rudder(int angle);
void move_sail(int position);
void write_log_file();
int  sign(float val);
void simulate_sailing();

struct timespec timermain;


//guidance
float _Complex X, X_T, X_T_b, X_b, X0;
float integratorSum=0, Guidance_Heading=0, override_Guidance_Heading=-1;
float theta=0, theta_b=0, theta_d=0, theta_d_b=0, theta_d1=0, theta_d1_b=0, a_x=0, b_x=0;
float theta_pM=0, theta_pM_b=0, theta_d_out=0;
int   sig = 0, sig1 = 0, sig2 = 0, sig3 = 0; // coordinating the guidance
int   roll_counter = 0, tune_counter = 0;
int   jibe_status = 1, actIn;
void guidance();
void findAngle();
void chooseManeuver();
void performManeuver();
void rudder_pid_controller();
void jibe_pass_fcn();


// sail hillclimbing algorithm
int sail_hc_periods=0, sail_hc_direction=1, sail_hc_val=0;
float sail_hc_ACC_V=0, sail_hc_OLD_V=0, sail_hc_MEAN_V=0;
void sail_controller();
void sail_hc_controller();


//waypoints
int nwaypoints=0, current_waypoint=0;
Point AreaWaypoints[1000], Waypoints[1000];
int calculate_area_waypoints();
int prepare_waypoint_array();




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
			read_weather_station_essential();

		} else {

			// AUTOPILOT ON
			if ((Navigation_System==1)||(Navigation_System==3))
			{
				read_weather_station();			// Update sensors data
				read_sail_position();			// Read sail actuator feedback
				guidance();				// Calculate the desired heading
				rudder_pid_controller();		// Calculate the desired rudder position
				sail_controller();			// Execute the default sail controller
				//sail_hc_controller();			// Execute the sail hillclimbing algorithm

				if(Simulation) simulate_sailing();

				// reaching the waypoint
				if  ( (cabs(X_T - X) < RADIUSACCEPTED) && (Navigation_System==1) )
				{	
					if (current_waypoint<nwaypoints-1)
					{
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
					else
					{
						// maintain position
						file = fopen("/tmp/sailboat/Navigation_System", "w");
						if (file != NULL) { fprintf(file, "3");	fclose(file); }
					}
				}
			}
			else
			{
				// AUTOPILOT OFF
				read_weather_station_essential();
			}
		}

		// write a log line
		write_log_file();

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

	system("cp /usr/share/wp_go     /tmp/sailboat/");
	system("cp /usr/share/wp_return /tmp/sailboat/");
	system("cp /usr/share/area_vx   /tmp/sailboat/");
	system("cp /usr/share/area_int  /tmp/sailboat/");

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
	system("[ ! -f /tmp/sailboat/Guidance_Heading ] 	&& echo 0 > /tmp/sailboat/Guidance_Heading");
	system("[ ! -f /tmp/sailboat/Rudder_Feedback ] 		&& echo 0 > /tmp/sailboat/Rudder_Feedback");
	system("[ ! -f /tmp/sailboat/Sail_Feedback ] 		&& echo 0 > /tmp/sailboat/Sail_Feedback");
	system("[ ! -f /tmp/sailboat/Simulation ] 		&& echo 0 > /tmp/sailboat/Simulation");
	system("[ ! -f /tmp/sailboat/Simulation_Wind ] 		&& echo 0 > /tmp/sailboat/Simulation_Wind");

	system("[ ! -f /tmp/sailboat/override_Guidance_Heading ] && echo -1 > /tmp/sailboat/override_Guidance_Heading");
}

/*
 *	Check Navigation System Status
 *
 *	[Navigation System] 
 *		- [0] Boat in IDLE status
 *		- [1] Control System ON, Sail to the waypoint
 *		- [3] Control System ON, Mantain position
 *		- [4] Calculate Route
 *	[Manual Control]
 *		- [0] OFF
 *		- [1] User takes control of sail and rudder positions
 *
 *	if Manual_Control is ON, read the following values:
 *		- [Manual_Control_Rudder] : user value for desired RUDDER angle [-30.0 to 30.0]
 *		- [Manual_Control_Sail]   : user value for desired SAIL position [0 to 500] 
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

	file = fopen("/tmp/sailboat/Simulation", "r");
	if (file != NULL) { fscanf(file, "%d", &Simulation); fclose(file); }
}


/*
 *	Whenever a new Navigation_System state is entered:
 */
void onNavChange() {

	// Update current position, wind angle, heading, ...
	read_weather_station();

	// SWITCH AUTOPILOT OFF
	if(Navigation_System==0) {

		// do nothing
	}


	// START SAILING
	if(Navigation_System==1) {

		if (Prev_Navigation_System==4) {		// START sailing

			current_waypoint=0;			
			Point_End_Lon=Waypoints[0].x;		
			Point_End_Lat=Waypoints[0].y;
		}
		
		if (Prev_Navigation_System==3) {		// RESUME sailing

			Point_End_Lon   = Waypoints[current_waypoint].x;
			Point_End_Lat   = Waypoints[current_waypoint].y;
		}

		Point_Start_Lon=Longitude;			// Update starting point
		Point_Start_Lat=Latitude;
	}


	// MAINTAIN POSITION
	if(Navigation_System==3) {				
		
		Point_End_Lon = Longitude;			// Overwrite temporarily the END point coord. using the current position
		Point_End_Lat = Latitude;
	}
	

	// CALCULATE ROUTE
	if(Navigation_System==4) {

		// Read WP_GO, WP_RETURN, calculate Area waypoints and put all together
		if (prepare_waypoint_array()) {

			// Start sailing	
			file = fopen("/tmp/sailboat/Navigation_System", "w");	
			if (file != NULL) { fprintf(file, "1"); fclose(file); }	
		}
	}
	

	file = fopen("/tmp/sailboat/Point_End_Lat", "w");
	if (file != NULL) { fprintf(file, "%f", Point_End_Lat); fclose(file); }
	file = fopen("/tmp/sailboat/Point_End_Lon", "w");
	if (file != NULL) { fprintf(file, "%f", Point_End_Lon); fclose(file); }

	Prev_Navigation_System=Navigation_System;
}




/*
 *	GUIDANCE V3:
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
	// GUIDANCE V3: nogozone, tack and jibe capable solution
	//  - Let's try to keep the order using sig,sig1,sig2,sig3 and theta_d,theta_d1. theta_d_b is actually needed in the chooseManeuver function.
	//  - lat and lon translation would be better on the direct input
	if (debug) printf("*********** Guidance **************** \n");
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
	theta_b = theta_wind - theta + PI/2;
	if (debug) printf("init SIG:[%d]\n",sig);

	if (sig == 0)  
	{
		findAngle();
		if (debug) printf("findAngle SIG1:[%d]\n",sig1);
                if (sig1 == 1) { chooseManeuver(); if (debug) printf("chooseManeuver SIG2:[%d]\n",sig2); }
		else { sig2 = sig1; }
	}
	else
	{
		theta_d1_b = theta_d_b;
                sig1 = sig;		
                sig2 = sig1;
	}

	//if (debug) printf("theta_d1: %4.1f deg. \n",theta_d1);

        if (sig2 > 0) { performManeuver(); if (debug) printf("performManeuver SIG3:[%d]\n",sig3); }
	else { sig3 = sig2; }

	if (debug) printf("SIG1: [%d] - SIG2: [%d] - SIG3: [%d] \n",sig1, sig2, sig3);

	// Updating the history angle, telling the guidance heading from last iteration
	theta_d_b = theta_d1_b;
	sig = sig3;
	//if (debug) printf("SIG1: [%d] - SIG2: [%d] - SIG3: [%d] \n",sig1, sig2, sig3);

	// Inverse turning matrix
	theta_d1 = theta_d1_b-theta_wind;
	theta_pM = theta_pM_b-theta_wind;

	if ( sig3>0 ) { theta_d_out = theta_pM; }
	else { theta_d_out = theta_d1; }
	Guidance_Heading = (PI/2 - theta_d_out) * 180/PI; 

	if (debug) printf("FA_DEBUG:[%d]\n",fa_debug);

	// write guidance_heading to file to be displayed in GUI 
	file = fopen("/tmp/sailboat/Guidance_Heading", "w");
	if (file != NULL) {
		fprintf(file, "%4.1f", Guidance_Heading);
		fclose(file);
	}
}

void findAngle() 
{
//	bool inrange;
	float theta_LOS, theta_l, theta_r, theta_dl, theta_dr;
	float _Complex Xl, Xr, Xdl, Xdr;

	// DEADzone limit direction
	Xl = -sin(theta_nogo)*2.8284 + I*cos(theta_nogo)*2.8284;	//-2 + 2*1*I;
	Xr = sin(theta_nogo)*2.8284 + I*cos(theta_nogo)*2.8284;		// 2 + 2*1*I;        

	// DOWNzone limit direction
        Xdl = -sin(theta_down)*2.8284 - I*cos(theta_down)*2.8284;        //-2 - 2*1*I;
        Xdr = sin(theta_down)*2.8284 - I*cos(theta_down)*2.8284;        // 2 - 2*1*I;        

	// definition of angles
	theta_LOS = atan2(cimag(X_T_b)-cimag(X_b),creal(X_T_b)-creal(X_b));
	theta_l = atan2(cimag(exp(-1*I*theta_LOS)*Xl),creal(cexp(-1*I*theta_LOS)*Xl));
	theta_r = atan2(cimag(exp(-1*I*theta_LOS)*Xr),creal(cexp(-1*I*theta_LOS)*Xr));
	// downwind angles
	theta_dl = atan2(cimag(exp(-1*I*theta_LOS)*Xdl),creal(cexp(-1*I*theta_LOS)*Xdl));
	theta_dr = atan2(cimag(exp(-1*I*theta_LOS)*Xdr),creal(cexp(-1*I*theta_LOS)*Xdr));

	// tacking boundaries
	// Line: x = a_x*y +/- b_x
	if (creal(X_T_b-X0) != 0)
	{
		a_x = creal(X_T_b-X0)/cimag(X_T_b-X0);
	}
	//******************************************************************
	// else // it should work without this.
	// {
	// 	a_x=0;
	// }
	//******************************************************************

	//if (debug) printf("theta_LOS: %f \n",theta_LOS);
	//if (debug) printf("angle(Xdr): %f \n",atan2(cimag(Xdr),creal(Xdr)));
	//if (debug) printf("angle(Xdl): %f \n",atan2(cimag(Xdl),creal(Xdl)));

	if (debug) printf("a_x: %f \n",a_x);
	b_x = TACKINGRANGE / (2 * sin(theta_LOS));


	// compute the next theta_d, ie at time t+1
	// (main algorithm)
		// Execution order:
		// 1. Is the LOS in deadzone?
		// 2. Is the LOS in downzone?
		// 3. the LOS is outside the zones, go straight.
		if ( (atan2(cimag(Xr),creal(Xr))-PI/9)<=theta_LOS  &&  theta_LOS<=(atan2(cimag(Xl),creal(Xl))+PI/9) )
		{
			if (debug) printf("theta_d_b: %f \n",theta_d_b);
			if (debug) printf("atan2(Xl): %f \n",atan2(cimag(Xl),creal(Xl)));
			if (debug) printf("atan2(Xr): %f \n",atan2(cimag(Xr),creal(Xr)));

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
		else
		{
			if ( atan2(cimag(Xdr),creal(Xdr)) >= theta_LOS  &&  theta_LOS >= atan2(cimag(Xdl),creal(Xdl)) )
			{
                                if (debug) printf("theta_d_b: %f \n",theta_d_b);
				if (debug) printf("atan2 Xdl: %f \n",atan2(cimag(Xdl),creal(Xdl)));
				if (debug) printf("atan2 Xdr: %f \n",atan2(cimag(Xdr),creal(Xdr)));

				if (theta_d_b >= atan2(cimag(Xdl),creal(Xdl))-PI/36  && theta_d_b <= atan2(cimag(Xdl),creal(Xdl))+PI/36 )
				{
					if (creal(X_b) > a_x*cimag(X_b)-b_x) { theta_d1_b = atan2(cimag(Xdr),creal(Xdr)); if (debug) printf(">> debug 13 \n"); fa_debug=13; sig1=1;}     
					else { theta_d1_b = theta_d_b; if (debug) printf(">> debug 14 \n"); fa_debug=14; sig1=0;}
				} 
				else
				{
					if (  (theta_d_b >= (atan2(cimag(Xdr),creal(Xdr))-(PI/36)))  &&  (theta_d_b <= (atan2(cimag(Xdr),creal(Xdr))+(PI/36))) )
					{
						if (creal(X_b) < a_x*cimag(X_b)+b_x) { theta_d1_b = atan2(cimag(Xdl),creal(Xdl)); if (debug) printf(">> debug 15 \n"); fa_debug=15; sig1=1;}
						else { theta_d1_b = theta_d_b; if (debug) printf(">> debug 16 \n"); fa_debug=16; sig1=0;}
					}
					else
					{
						if(cabs(theta_dl) < cabs(theta_dr)) { theta_d1_b = atan2(cimag(Xdl),creal(Xdl)); if (debug) printf(">> debug 17 \n"); fa_debug=17; sig1=1;}
						else { theta_d1_b = atan2(cimag(Xdr),creal(Xdr)); if (debug) printf(">> debug 18 \n"); fa_debug=18; sig1=1;}
						if (debug) printf("---- Downwind theta_d1_b: %f \n",theta_d1_b);
					}
				}
			}
			else
			{
				// if theta_LOS is outside of the deadzone
				theta_d1_b = theta_LOS;
				sig1 = 1;
				if (debug) printf(">> debug 2 \n");
				fa_debug=2;
			}
		}
	//if (debug) printf("theta_LOS = %f \n",theta_LOS);
	//if (debug) printf("X_T_b = %.1f + I*%.1f \n",creal(X_T_b),cimag(X_T_b));
	//if (debug) printf("X_b = %.1f + I*%.1f \n",creal(X_b),cimag(X_b));
	//if (debug) printf("Xl = %f + I*%f \n",creal(Xl),cimag(Xl));
	//if (debug) printf("Xr = %f + I*%f \n",creal(Xr),cimag(Xr));
	//if (debug) printf("Xdl = %f + I*%f \n",creal(Xdl),cimag(Xdl));
	//if (debug) printf("Xdr = %f + I*%f \n",creal(Xdr),cimag(Xdr));
}        

void chooseManeuver() 
{
	// The maneuver function does the maneuvers. This function performs each/every course change. 
	// Here it decides whether there is need for a tack, jibe or just a little course change. 
	// This decision incorporates two steps: 1. Is the desired heading on the other side of the deadzone? 
	// Then we need to jibe or tack. 2. Do we have enough speed for tacking? According to this it chooses sig.

	// float dAngle, d1Angle;
	float _Complex X_d_b, X_d1_b;

	// Definition of X_d1_b and X_d_b
	X_d_b = ccos(theta_d_b) + I*(csin(theta_d_b));
	X_d1_b = ccos(theta_d1_b) + I*(csin(theta_d1_b));

	if ( sign(creal(X_d_b)) == sign(creal(X_d1_b)) )
	{ sig2=0; }			//course change
	else
	{
		if ( cimag(X_d_b) > 0 && cimag(X_d1_b) > 0 && SOG > v_min )
		{ sig2=0; }	//tack
		else
		{		//jibe
			if ( creal(X_d_b) < 0 )
			{ sig2=1; }	//left
			else
			{ sig2=2; }	//right
		}
	}
	//if (debug) printf("X_d_b = %f + I*%f \n",creal(X_d_b),cimag(X_d_b));
	//if (debug) printf("X_d1_b = %f + I*%f \n",creal(X_d1_b),cimag(X_d1_b));
}

int sign(float val){
	if (val > 0) return 1;	// is greater then zero
	if (val < 0) return -1;	// is less then zero
	return 0;		// is zero
}



void performManeuver()
{
	// float v_b1, v_b2, v_d1_b1, v_d1_b2;
	float _Complex Xdl, Xdr;
	fa_debug=7353;
	// I claim, we don't need this any more! theta_b=atan2(sin(theta_b),cos(theta_b)); // avoiding singularity

	//if (debug) printf("theta_b: %f\n",theta_b);
	//if (debug) printf("theta_d1_b: %f\n",theta_d1_b);
	if (sig2==1) if (debug) printf("Jibe left ... ");
	if (sig2==2) if (debug) printf("Jibe right ... ");

	// DOWNzone limit direction ***** These variables are already defined in the findAngle-function. ***
        Xdl = -sin(theta_down)*2.8284 - I*cos(theta_down)*2.8284;
        Xdr = sin(theta_down)*2.8284 - I*cos(theta_down)*2.8284;
	
	//Jibe direction -> jibe status -> defines the headings during the maneuver.
	switch(sig2)
	{
		case 1: // Jibe left
			switch(jibe_status)
			{
				case 1: //begin Jibe: get on course
					theta_pM_b = atan2(cimag(Xdl),creal(Xdl));
					jibe_pass_fcn();
					break;
				case 2: //tighten sail (hold course)
					theta_pM_b = atan2(cimag(Xdl),creal(Xdl));
					actIn = 1;
					jibe_pass_fcn();
					break;
				case 3: //perform jibe (hold sail tight)
					theta_pM_b = atan2(cimag(Xdr),creal(Xdr));
					actIn = 1;
					jibe_pass_fcn();
					break;
				case 4: //release sail (hold course)
					theta_pM_b = atan2(cimag(Xdr),creal(Xdr));
					actIn = 0;
					jibe_pass_fcn();
					break;
				case 5: //find new course
					jibe_pass_fcn();
					break;
			}
			break;
		case 2: // Jibe right
			switch(jibe_status)
			{
				case 1: //begin Jibe: get on course
					theta_pM_b = atan2(cimag(Xdr),creal(Xdr));
					jibe_pass_fcn();
					break;
				case 2: //tighten sail (hold course)
					theta_pM_b = atan2(cimag(Xdr),creal(Xdr));
					actIn = 1;
					jibe_pass_fcn();
					break;
				case 3: //perform jibe (hold sail tight)
					theta_pM_b = atan2(cimag(Xdl),creal(Xdl));
					actIn = 1;
					jibe_pass_fcn();
					break;
				case 4: //release sail (hold course)
					theta_pM_b = atan2(cimag(Xdl),creal(Xdl));
					actIn = 0;
					jibe_pass_fcn();
					break;
				case 5: //find new course
					jibe_pass_fcn();
					break;
			}
			break;
	} // end switch(sig2)
	//if (debug) printf("theta_pM_b = %f \n",theta_pM_b);
	if (debug) printf("jibe status = %d \n",jibe_status);
	//if (debug) printf("Xdl = %f + I*%f \n",creal(Xdl),cimag(Xdl));
	//if (debug) printf("Xdr = %f + I*%f \n",creal(Xdr),cimag(Xdr));
}

/*	JIBE PASS FUNCTION
 *	increase the value of 'jibe_status' when needed and define sig3.
 */
void 	jibe_pass_fcn() {
	float _Complex X_h, X_pM;

	// defining direction unit vectors
	X_h = -sin(theta_b) + I*cos(theta_b);
	X_pM = -sin(theta_pM_b) + I*cos(theta_pM_b);

	if (debug) printf("Sail_Feedback: %d\n",Sail_Feedback);

	if ( cos(angle_lim) < (creal(X_h)*creal(X_pM) + cimag(X_h)*cimag(X_pM)) && jibe_status<5) // Here 'Sail_Feedback=0' means that the actuator is as short as possible, hence the sail is tight.
	{	// When the heading approaches the desired heading and the sail is tight, the jibe is performed.
		if ( actIn==0 )
		{
		jibe_status++;
		}
		if ( actIn && Sail_Feedback<10 )
		{
		jibe_status++;
		}
	}
	if (jibe_status==5 && Sail_Feedback>300)
	{
	sig3=0;
	jibe_status=1;
	}
	else
	{
	sig3=sig2;
	}
	if (debug) printf("X_h = %f + I*%f \n",creal(X_h),cimag(X_h));
	if (debug) printf("X_pM = %f + I*%f \n",creal(X_pM),cimag(X_pM));
}




/*
 *	RUDDER PID CONTROLLER:
 *
 *	Calculate the desired RUDDER ANGLE position based on the Target Heading and Current Heading.
 *	The result is a rounded value of the angle stored in the [Rudder_Desired_Angle] global variable.
 *	Daniel Wrede, May 2013
 */
void rudder_pid_controller() {

	float dHeading, pValue, temp_ang; //,integralValue;

	dHeading = Guidance_Heading - Heading; // in degrees

	// Singularity translation
	dHeading = dHeading*PI/180;
	dHeading = atan2(sin(dHeading),cos(dHeading));
	dHeading = dHeading*180/PI;        

	if (debug) printf("dHeading: %f\n",dHeading);

	//if (debug) fprintf(stdout,"targetHeafing: %f, deltaHeading: %f\n",targetHeading, dHeading);
	//if (abs(dHeading) > dHEADING_MAX && abs(Rate) > RATEOFTURN_MAX) // Limit control statement
	//{

		// P controller
		pValue = GAIN_P * dHeading;

		// Integration part
		// The following checks, will keep integratorSum within -0.2 and 0.2
		// if (integratorSum < -INTEGRATOR_MAX && dHeading > 0) {
		// 	integratorSum = dHeading + integratorSum;
		// } else if (integratorSum > INTEGRATOR_MAX && dHeading < 0) {
		// 	integratorSum = dHeading + integratorSum;
		// } else {
		// 	integratorSum = integratorSum;
		// }
		// integralValue = GAIN_I * integratorSum;

		// result
		temp_ang = pValue; //+ integralValue; // Angle in radians

	//}
	// fprintf(stdout,"pValue: %f, integralValue: %f\n",pValue,integralValue);
	// fprintf(stdout,"Rudder_Desired_Angle: %d\n\n",Rudder_Desired_Angle);

	Rudder_Desired_Angle = round(temp_ang);
	if(Rudder_Desired_Angle > 35) {Rudder_Desired_Angle=35; }
	if(Rudder_Desired_Angle < -35) {Rudder_Desired_Angle=-35; }

	// Move rudder
	move_rudder(Rudder_Desired_Angle);
}



/*
 *	SAIL CONTROLLER (default)
 *
 *	Controls the angle of the sail and implements an Emergency sail release when the boat's
 *	roll value exceeds a predefined threshold. Input to this function is [Wind_Angle]
 *        Daniel Wrede & Mikkel Heeboell Callesen, December 2013
 */
void sail_controller() {

	float C=0, C_zero=0; 		// sheet lengths
	float BWA=0, theta_sail=0; 	//theta_sail_feedback;
	float _Complex X_h, X_w;

	X_h = csin(Heading*PI/180) + I*(ccos(Heading*PI/180));
	X_w = csin(Wind_Angle*PI/180) + I*(ccos(Wind_Angle*PI/180));
	BWA = acos( cimag(X_h)*cimag(X_w) + creal(X_h)*creal(X_w) );

	// Deriving the function for theta_sail:
	// theta_sail(BWA) = a*BWA+b
	// Having two points, theta_sail(theta_nogo)=0 and theta_sail(3/4*PI)=1.23
	// a=1.23/(3/4*PI-theta_nogo) , b=-a*theta_nogo
	// This is inserted below.

	if ( BWA < theta_nogo ) { theta_sail = 0; }
	else
	{
		if ( BWA < PI*3/4 ) { theta_sail = 1.23/( 3/4*PI-theta_nogo )*BWA - 1.23/(3/4*PI-theta_nogo)*theta_nogo; }
		else
		{
			if ( BWA < (PI*17/18) ) { theta_sail=1.23; }
			else { theta_sail = 0; }
		}
	}

	C = sqrt( SCLength*SCLength + BoomLength*BoomLength -2*SCLength*BoomLength*cos(theta_sail) + SCHeight*SCHeight);
	C_zero = sqrt( SCLength*SCLength + BoomLength*BoomLength -2*SCLength*BoomLength*cos(0) + SCHeight*SCHeight);

	// Assuming the actuator to be outside at ACT_MAX and in at 0:
	//(C-C_zero)=0 -> sail tight when C=C_zero . ACT_MAX/500 is the ratio between ticks and stroke. 1000[mm]/3 is a unit change + 
	Sail_Desired_Position = (C-C_zero)/3*1000*ACT_MAX/500; 
	if ( Sail_Desired_Position > ACT_MAX ) Sail_Desired_Position=ACT_MAX; 
	if ( Sail_Desired_Position < 0 )   Sail_Desired_Position=0; 

	// If the boat tilts too much
	if(fabs(Roll*3.26)>ROLL_LIMIT) // we multiply by 3.26, to transform the input to degrees.
	{ 
                // Start Loosening the sail
		move_sail(ACT_MAX);                
		roll_counter=0;
		// if (debug) printf("max roll reached. \n" );
	} 
	else
	{
		if(roll_counter<10*SEC) { roll_counter++; }
	}

	// if (debug) printf("sail_controller readout: SIG3 = %d \n",sig3);
	// If it is time to jibe
	if (actIn) 
	{
		// Start Tightening the sail
		move_sail(0);
	}
	else
	{
		// sail tuning according to wind
		if(roll_counter > 5*SEC && (fabs(Sail_Desired_Position-Sail_Feedback)>SAIL_LIMIT || tune_counter>10*SEC) )
		{
			move_sail(Sail_Desired_Position);
			tune_counter=0;
			if (debug2) printf("- - - Sail tuning - - -\n");
		}
		else
		{
		tune_counter++;
		}
	}
	if (debug2) printf("Sail_Feedback: %d \n",Sail_Feedback);

}



/*
 *	SAIL CONTROLLER (based on hillclimbing function) [from "thesis" branch]
 *
 *	Actuates the sail in one direction for [SAIL_ACT_TIME] seconds
 *	Calculate the mean velocity of the boat on a period of [SAIL_OBS_TIME] seconds
 *	If the velocity is increasing keep moving in the same direction, otherwise change direction
 */
void sail_hc_controller() {

	// MEASURE
	sail_hc_ACC_V+=SOG;

	if(sail_hc_periods == SAIL_ACT_TIME*4) {

		// STOP MOVING THE ACTUATOR
		sail_hc_val=250;
		move_sail(sail_hc_val);
	}

	if(sail_hc_periods >= SAIL_OBS_TIME*4) {

		sail_hc_MEAN_V = sail_hc_ACC_V / sail_hc_periods;

		// TAKE DECISION
		if (sail_hc_MEAN_V < sail_hc_OLD_V) { sail_hc_direction*=-1; }

		// START MOVING THE ACTUATOR
		if (sail_hc_direction==1) sail_hc_val=500; else sail_hc_val=0;
		move_sail(sail_hc_val);

		sail_hc_OLD_V = sail_hc_MEAN_V;
		sail_hc_periods=0; sail_hc_ACC_V=0;
	}

	sail_hc_periods++;
}


void simulate_sailing() {
	
	// update boat heading
	double delta_Heading = (SIM_ROT/SEC)*(-(double)Rudder_Desired_Angle/30)*SIM_SOG;
	Heading = Heading + delta_Heading;


	// update boat position
	double displacement = ((double)SIM_SOG)/SEC;
	double SimLon= ( (Longitude*CONVLON) + displacement*sinf(Heading*PI/180) )/CONVLON;
	double SimLat= ( (Latitude*CONVLAT)  + displacement*cosf(Heading*PI/180) )/CONVLAT;
	Latitude=(float)SimLat;
	Longitude=(float)SimLon;


	// update sail actuator position
	int increment=SIM_ACT_INC/SEC;
	if (Sail_Feedback > Sail_Desired_Position) Sail_Feedback-=increment; 
	else {if(Sail_Feedback < Sail_Desired_Position)	{ Sail_Feedback+=increment; }}


	// update rudder position (for the GUI only)
	increment=16/SEC;	// 16 degrees/sec
	if (Rudder_Feedback > Rudder_Desired_Angle) Rudder_Feedback-=increment; 
	else Rudder_Feedback+=increment; 


	// change wind conditions
	// not implemented

	
	// debug
	// if (debug) printf("FA_DEBUG:[%d]\n",fa_debug);


	// Write new values to file
	file = fopen("/tmp/u200/Heading", "w");
	if (file != NULL) { fprintf(file, "%f", Heading); fclose(file); }
	file = fopen("/tmp/u200/Latitude", "w");
	if (file != NULL) { fprintf(file, "%.8f", Latitude); fclose(file); }
	file = fopen("/tmp/u200/Longitude", "w");
	if (file != NULL) { fprintf(file, "%.8f", Longitude); fclose(file); }
	file = fopen("/tmp/sailboat/Sail_Feedback", "w");
	if (file != NULL) { fprintf(file, "%d", Sail_Feedback); fclose(file); }
	file = fopen("/tmp/sailboat/Rudder_Feedback", "w");
	if (file != NULL) { fprintf(file, "%d", Rudder_Feedback); fclose(file); }

}



/*
 *	Calculate the waypoint positions to map an area defined as a convex polygon
 */
int calculate_area_waypoints() {

	// local variable definitions
	FILE* file_caw;
	int interval=0, nvertexes=0, next=0, d=-1, sw=0, iRef=0, i=0, k=0;
	double yMin, yMax, xMin, xMax, yRef, xRef;
	Point pw1, pw2, wayp, tmpW, p;
	Point tWaypoints[1000], Vertex[50];
	Line lw,lp;
	char str[50];
	
	// reset previous waypoints
	nwaypoints=0;

	// read Area interleav from file
	file_caw = fopen("/tmp/sailboat/area_int", "r");
	if (file_caw != NULL) { fscanf(file_caw, "%d", &interval); fclose(file_caw); }
	else return 0;
	
	// read Area vertexes from file
	file_caw = fopen("/tmp/sailboat/area_vx" , "r");
	if (file_caw == NULL) { perror("Error reading Area vertexes [/tmp/sailboat/area_vx]"); return 0; }
	while (fgets (str, 50, file_caw)!=NULL ) {
		sscanf(str, "%lf;%lf,", &p.y, &p.x);
		Vertex[nvertexes]=p;
		nvertexes++;
	}

	// convert vertex coordinates to XY plane
	for (i=0; i<nvertexes; i++) Vertex[i] = convert_xy(Vertex[i]);

	// rotate vertex coordinates accoringly to wind direction
	for (i=0; i<nvertexes; i++) Vertex[i] = rotate_point(Vertex[i],Wind_Angle);

	// find closest and farthest vertexes
	yMin=Vertex[0].y; yMax=Vertex[0].y;
	for (i=1; i<nvertexes; i++) {
		if (Vertex[i].y < yMin) { yMin=Vertex[i].y; xRef=Vertex[i].x; iRef=i; }
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
			pw1=new_point(xRef,yRef);
			pw2=new_point(xRef+1000,yRef);
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
	
	// remove one of the two waypoints in the main vertex (expected result)
	if (( abs(tWaypoints[0].x - tWaypoints[1].x) < 0.1 ) && ( abs(tWaypoints[0].y - tWaypoints[1].y) < 0.1 ) ) {
		for (i=1; i<nwaypoints; i++) AreaWaypoints[i-1]=tWaypoints[i];
		nwaypoints--;
	}
	// handle precision errors: only one waypoint in the main vertex 
	if (( abs(tWaypoints[0].x - tWaypoints[1].x) > 0.1 ) && ( abs(tWaypoints[0].y - tWaypoints[1].y) > 0.1 ) ) {
		for (i=0; i<nwaypoints; i++) AreaWaypoints[i]=tWaypoints[i];
	}
	// handle precision errors: no waypoints in the main vertex 
	if (( abs(tWaypoints[0].x - tWaypoints[1].x) > 0.1 ) && ( abs(tWaypoints[0].y - tWaypoints[1].y) < 0.1 ) ) {
		AreaWaypoints[0]=Vertex[iRef];
		for (i=0; i<nwaypoints; i++) AreaWaypoints[i+1]=tWaypoints[i];
		nwaypoints++;
	}

	// alternate waypoints by X coordinates (let's do a S-like zig-zag as left-left-right-right oh-yeah)
	// I hereby name this algorithm "the headless alternated paired bubble sorting" a.k.a. "the zig"	
	for (i=1; i<nwaypoints; i+=2) {

		if ((d<0) && (AreaWaypoints[i].x > AreaWaypoints[i+1].x)) sw=1;
		if ((d>0) && (AreaWaypoints[i].x < AreaWaypoints[i+1].x)) sw=1;

		if (sw==1) {
			tmpW = AreaWaypoints[i];
			AreaWaypoints[i]   = AreaWaypoints[i+1];
			AreaWaypoints[i+1] = tmpW;
		}
		sw=0; d*=-1;
	}
/*
	// requested on 28 Sep 2013: Once mapped an area, map it backward again
	for (i=nwaypoints-2; i>=0; i--) {
		AreaWaypoints[nwaypoints+(nwaypoints-i)-2]=AreaWaypoints[i];
	}
	nwaypoints=(2*nwaypoints)-2;

	// 06 Oct 2013: Mapping back the area using diagonals
	k=0; for (i=nwaypoints-3; i>=0; i-=2) {
		AreaWaypoints[nwaypoints+k]=AreaWaypoints[i];
		k++;
	}
	nwaypoints=nwaypoints+k;
*/
	// rotate back the point coordinates for the map
	for (i=0; i<nwaypoints; i++) AreaWaypoints[i] = rotate_point(AreaWaypoints[i],-1*Wind_Angle);

	// convert coordinates back to Latitude-Longitude
	for (i=0; i<nwaypoints; i++) AreaWaypoints[i] = convert_latlon(AreaWaypoints[i]);
	
	return 1;
}



/*
 *	Prepare waypoints array [Go_waypoints + Area_waypoints + Return_waypoints]
 */
int prepare_waypoint_array() {
	
	int n=0,i=0;
	char str[50];
	FILE* fp;
	Point p;

	// Read GO waypoints from file and add them to Waypoint Array
	fp = fopen("/tmp/sailboat/wp_go" , "r");
	if (fp == NULL) { perror("Error reading Waypoints_GO file [/tmp/sailboat/wp_go]"); return 0; }
	while (fgets (str, 50, fp)!=NULL ) {
		sscanf(str, "%lf;%lf,", &p.y, &p.x);
		Waypoints[n]=p;
		n++;
	}
	
	// Calculate the Area Waypoints and add them to the Waypoint array
	if (!calculate_area_waypoints()) return 0;
	nwaypoints=nwaypoints+n;
	for (i=n; i<nwaypoints; i++) {
		Waypoints[i]=AreaWaypoints[i-n];
	}

	// Read RETURN waypoints from file and add them to Waypoint Array
	n=0; fp = fopen("/tmp/sailboat/wp_return" , "r");
	if (fp == NULL) { perror("Error reading Waypoints_RETURN file [/tmp/sailboat/wp_return]"); return 0; }
	while (fgets (str, 50, fp)!=NULL ) {
		sscanf(str, "%lf;%lf,", &p.y, &p.x);
		Waypoints[nwaypoints]=p;
		nwaypoints++;
	}
	
	return 1;
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
/*	file = fopen("/tmp/u200/Deviation", "r");
	if (file != NULL) {
		fscanf(file, "%f", &Deviation);	fclose(file);
	}
	file = fopen("/tmp/u200/Variation", "r");
	if (file != NULL) {
		fscanf(file, "%f", &Variation);	fclose(file);
	}
*/
	//ATTITUDE
/*	file = fopen("/tmp/u200/Yaw", "r");
	if (file != NULL) {
		fscanf(file, "%f", &Yaw); fclose(file);
	}
*/	file = fopen("/tmp/u200/Pitch", "r");
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
 *	Read essential data from the Weather Station
 */
void read_weather_station_essential() {

	//VESSEL HEADING
	file = fopen("/tmp/u200/Heading", "r");
	if (file != NULL) { 
		fscanf(file, "%f", &Heading); fclose(file);
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
void move_sail(int position) {
	file = fopen("/tmp/sailboat/Navigation_System_Sail", "w");
	if (file != NULL) { fprintf(file, "%d", position); fclose(file); Sail_Desired_Position=position;}
}

/*
 *	Read Sail actuator feedback
 */
void read_sail_position() {

	file = fopen("/tmp/sailboat/Sail_Feedback", "r");
	if (file != NULL) { 
		fscanf(file, "%d", &Sail_Feedback); fclose(file);
	}
}



/*
 *	Save all the variables of the navigation system in a log file in sailboat-log/
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

		// write HEADERS in log files
		file2 = fopen(logfile1, "w");
		if (file2 != NULL) { fprintf(file2, "MCU_timestamp,Navigation_System,Manual_Control,Guidance_Heading,Manual_Ctrl_Rudder,Rudder_Desired_Angle,Rudder_Feedback,Manual_Ctrl_Sail,Sail_Desired_Pos,Sail_Feedback,Rate,Heading,Pitch,Roll,Latitude,Longitude,COG,SOG,Wind_Speed,Wind_Angle,Point_Start_Lat,Point_Start_Lon,Point_End_Lat,Point_End_Lon\n"); fclose(file2); }
		file2 = fopen(logfile2, "w");
		if (file2 != NULL) { fprintf(file2, "MCU_timestamp,sig1,sig2,sig3,fa_debug,theta_d1,theta_d,theta_d1_b,theta_b,a_x,b_x,X_b,X_T_b,sail_hc_periods,sail_hc_direction,sail_hc_val,sail_hc_MEAN_V\n"); fclose(file2); }
		
		logEntry=1;
	}

	// read rudder feedback
	file2 = fopen("/tmp/sailboat/Rudder_Feedback", "r");
	if (file2 != NULL) { fscanf(file2, "%d", &Rudder_Feedback); fclose(file2); }


	// generate csv LOG line
	sprintf(logline, "%u,%d,%d,%.1f,%d,%d,%d,%d,%d,%d,%.4f,%.4f,%.4f,%.4f,%f,%f,%.1f,%.3f,%.2f,%.2f,%f,%f,%f,%f" \
		, (unsigned)time(NULL) \
		, Navigation_System \
		, Manual_Control \
		, Guidance_Heading \
		, Manual_Control_Rudder \
		, Rudder_Desired_Angle \
		, Rudder_Feedback \
		, Manual_Control_Sail \
		, Sail_Desired_Position \
		, Sail_Feedback \
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
	sprintf(logline, "%u,%d,%d,%d,%d,%f,%f,%f,%f,%f,%f,%f_%fi,%f_%fi,%d,%d,%d,%.2f" \
		, (unsigned)time(NULL) \
		, sig1 \
		, sig2 \
		, sig3 \
		, fa_debug \
		, theta_d1 \
		, theta_d \
		, theta_d1_b \
		, theta_b \
		, a_x \
		, b_x \
		, creal(X_T), cimag(X_T) \
		, creal(X_T_b), cimag(X_T_b) \
		, sail_hc_periods \
		, sail_hc_direction \
		, sail_hc_val \
		, sail_hc_MEAN_V \
	);
	// write to DEBUG file
	file2 = fopen(logfile2, "a");
	if (file2 != NULL) { fprintf(file2, "%s\n", logline); fclose(file2); }
	

	fa_debug=0;
	logEntry++;
}
