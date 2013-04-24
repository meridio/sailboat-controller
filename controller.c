/* 
 *	SAILBOAT-CONTROLLER
 *
 */


#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>

#define PI 3.14159265
#define GAIN_P 1
#define GAIN_I 0

FILE* file;
float Rate,Heading,Deviation,Variation,Yaw,Pitch,Roll,Latitude,Longitude,COG,SOG,Wind_Speed,Wind_Angle;
float Point_Start_Lat, Point_Start_Lon, Point_End_Lat, Point_End_Lon;
float Manual_Control_Rudder, Manual_Control_Sail;
float Navigation_System_Rudder, Navigation_System_Sail;
float integratorSum=0;
int Rudder_Desired_Angle;
int Navigation_System, Manual_Control;

void initfiles();
void check_system_status();
void read_manual_control_values();
void read_weather_station();
void move_rudder(int angle);
void read_coordinates();
void calculate_rudder_angle();


int main(int argc, char ** argv)
{
	initfiles();
	fprintf(stdout,"\nSailboat-controller running..\n");
	read_weather_station();

	while (1) {
		
		check_system_status();
		
		if (Manual_Control) 
		{
			// MANUAL CONTROL IS ON
			// Values for the RUDDER and SAIL positions are received from the User Interface.
			// Use [Manual_Control_Rudder] and [Manual_Control_Sail] files to control the actuators.
			
			// Read desired values set by the Graphical User Interface
			read_manual_control_values();

			// Move the rudder to the desired position
			move_rudder(Manual_Control_Rudder); 		

		}
		else
		{

			if (Navigation_System) 
			{
				// "AUTOPILOT" is activated.

				// Read data from the Weather station: Gps, Wind, Yaw, Roll, Rate of turn, ...
				read_weather_station();
				
				// Read START and TARGET point coordinates
				read_coordinates();
			
				// Calculate the rudder desired angle based on environmental conditions and target position
				calculate_rudder_angle();

				// Move the rudder to the desired angle
				move_rudder(Rudder_Desired_Angle);

				// If the desired point is reached (20m tollerance), switch the coordinates and come back home
				// calculate_distance();
				// switch_coordinates();
				
			}
			else
			{
				// NAVIGATION SYSTEM IS IDLE
				// do nothing
			}

		}

		sleep(1);
	}
	return 0;
}




/*
 *	Create empty files: Manual Control, Starting Point, Ending Point
 *	Navigation system starts in IDLE mode, waiting for target point coordinates
 */
void initfiles()
{
	system("mkdir /tmp/sailboat");

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
void check_system_status()
{
	file = fopen ("/tmp/sailboat/Navigation_System", "r");
	fscanf (file, "%d", &Navigation_System);
	fclose (file);
	file = fopen ("/tmp/sailboat/Manual_Control", "r");
	fscanf (file, "%d", &Manual_Control);
	fclose (file); 
}


/*
 *	Read Manual_Control values
 *	[Manual_Control_Rudder] : user value for desired RUDDER angle [-30.0 to 30.0]
 *	[Manual_Control_Sail] : user value for desired SAIL angle [ignored]
 */
void read_manual_control_values()
{
	file = fopen ("/tmp/sailboat/Manual_Control_Rudder", "r");
	fscanf (file, "%f", &Manual_Control_Rudder);
	fclose (file);
	file = fopen ("/tmp/sailboat/Manual_Control_Sail", "r");
	fscanf (file, "%f", &Manual_Control_Sail);
	fclose (file); 
}


/*
 *	Move the rudder to the desired position.
 *	Write the desired angle to a file [] to be handled by another process 
 */
void move_rudder(int angle)
{
	file = fopen("/tmp/sailboat/Navigation_System_Rudder","w");
	fprintf(file,"%d",angle);
	fclose(file);
}


/*
 *	Read START and END point coordinated (latitude and longitude)
 *	Coordinates format is DD
 */
void read_coordinates()
{
	file = fopen ("/tmp/sailboat/Point_Start_Lat", "r");
	fscanf (file, "%f", &Point_Start_Lat);
	fclose (file);
	file = fopen ("/tmp/sailboat/Point_Start_Lon", "r");
	fscanf (file, "%f", &Point_Start_Lon);
	fclose (file);
	file = fopen ("/tmp/sailboat/Point_End_Lat", "r");
	fscanf (file, "%f", &Point_End_Lat);
	fclose (file);
	file = fopen ("/tmp/sailboat/Point_End_Lon", "r");
	fscanf (file, "%f", &Point_End_Lon);
	fclose (file); 
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
void calculate_rudder_angle()
{
	// GUIDANCE, calculate Target Heading:
		float dx, dy, targetHeading;
		dx=Point_End_Lon-Longitude;
		dy=Point_End_Lat-Latitude;
		// targetHeading range is -180(to the left) to +180(to the right)
		targetHeading = atan2(dx,dy) * 180 / PI;

	
	// RUDDER PID CONTROLLER, calculate Rudder angle:
		float dHeading, pValue, integralValue;	
		dHeading = targetHeading - Heading;
		// WARNING: calculating the dHeading consider the jump from 0 to 359 degrees
		// fprintf(stdout,"targetHeafing: %f, deltaHeading: %f\n",targetHeading, dHeading);

		// P controller
		pValue = GAIN_P * dHeading;

		// Integration part
		// integratorSum = dHeading + integratorSum; // WARNING: this keeps on growing at every loop
		integralValue = GAIN_I * integratorSum;
		
		// result
		//if ( dHeading < dHeading_max && rateofturn < rateofturn_max )  // not sure about the syntax
		//{
		Rudder_Desired_Angle = round(pValue + integralValue);
		//}
		// fprintf(stdout,"pValue: %f, integralValue: %f\n",pValue,integralValue);
		// fprintf(stdout,"Rudder_Desired_Angle: %d\n\n",Rudder_Desired_Angle);
}



/*
 *	Read data from the Weather Station
 */
void read_weather_station()
{
	//RATE OF TURN
	file = fopen ("/tmp/u200/Rate", "r");
	if(file != 0)
	{
  		fscanf (file, "%f", &Rate);
		fclose (file);
	}
	else
	{
		printf("ERROR: Files from Weather Station are missing.\n");
		exit (1);
	}

	//VESSEL HEADING
	file = fopen ("/tmp/u200/Heading", "r");
  	fscanf (file, "%f", &Heading);
	fclose (file);
	file = fopen ("/tmp/u200/Deviation", "r");
  	fscanf (file, "%f", &Deviation);
	fclose (file); 
	file = fopen ("/tmp/u200/Variation", "r");
  	fscanf (file, "%f", &Variation);
	fclose (file); 

	//ATTITUDE
	file = fopen ("/tmp/u200/Yaw", "r");
  	fscanf (file, "%f", &Yaw);
	fclose (file);
	file = fopen ("/tmp/u200/Pitch", "r");
  	fscanf (file, "%f", &Pitch);
	fclose (file); 
	file = fopen ("/tmp/u200/Roll", "r");
  	fscanf (file, "%f", &Roll);
	fclose (file); 

	//GPS_DATA
	file = fopen ("/tmp/u200/Latitude", "r");
  	fscanf (file, "%f", &Latitude);
	fclose (file);
	file = fopen ("/tmp/u200/Longitude", "r");
  	fscanf (file, "%f", &Longitude);
	fclose (file);
	file = fopen ("/tmp/u200/COG", "r");
  	fscanf (file, "%f", &COG);
	fclose (file);
	file = fopen ("/tmp/u200/SOG", "r");
  	fscanf (file, "%f", &SOG);
	fclose (file);

	//WIND_DATA
	file = fopen ("/tmp/u200/Wind_Speed", "r");
  	fscanf (file, "%f", &Wind_Speed);
	fclose (file);
	file = fopen ("/tmp/u200/Wind_Angle", "r");
  	fscanf (file, "%f", &Wind_Angle);
	fclose (file);
	
}
