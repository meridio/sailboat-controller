/* 
 *	SAILBOAT-CONTROLLER
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

FILE* file;
float Rate,Heading,Deviation,Variation,Yaw,Pitch,Roll,Latitude,Longitude,COG,SOG,Wind_Speed,Wind_Angle;
int Manual_Control;

void initfiles();
void check_manual_control();
void read_weather_station();


int main(int argc, char ** argv)
{
	initfiles();
	fprintf(stdout,"\nSailboat-controller running..\n");
	while (1) {
		
		check_manual_control();
		read_weather_station();
		
		if (Manual_Control) 
		{
			printf("Manual control is ON\n");
			// use the values from [Manual_Control_Rudder] and [Manual_Control_Sail] to control the actuators
		}
		else
		{
			printf("Manual control is OFF\n");
			// here comes the control logic based on the sensor values (0 means not_set)

			if (Heading > 40.6)
			{
				printf("Heading [%6.3f deg] is greater than 40.6 deg\n", Heading);
			}
			else
			{
				printf("Heading [%6.3f deg] is smaller than 40.6 deg\n", Heading);
			}

		}

		sleep(1);
	}
	return 0;
}




/*
 *	Create empty files: Manual Control, Starting Point, Ending Point
 */
void initfiles()
{
	system("mkdir /tmp/sailboat");

	system("echo 0 > /tmp/sailboat/Manual_Control");
	system("echo 0 > /tmp/sailboat/Manual_Control_Rudder");
	system("echo 0 > /tmp/sailboat/Manual_Control_Sail");
	system("echo 0 > /tmp/sailboat/point_start_lat");
	system("echo 0 > /tmp/sailboat/point_start_lon");
	system("echo 0 > /tmp/sailboat/point_end_lat");
	system("echo 0 > /tmp/sailboat/point_end_lon");
}


/*
 *	Read Manual_Control value
 */
void check_manual_control()
{
	file = fopen ("/tmp/sailboat/Manual_Control", "r");
	fscanf (file, "%d", &Manual_Control);
	fclose (file); 
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
