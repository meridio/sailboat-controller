/* 
 *	SAILBOAT-CONTROLLER
 *
 */

#include <stdio.h>
#include <unistd.h>

FILE* file;
float Rate,Heading,Deviation,Variation,Yaw,Pitch,Roll,Latitude,Longitude,COG,SOG,Wind_Speed,Wind_Angle;
int Manual_Control;


void check_manual_control();
void read_weather_station();


int main(int argc, char ** argv)
{
	fprintf(stdout,"Sailboat-controller running..\n");
	while (1) {
		
		check_manual_control();
		read_weather_station();
		
		if (Manual_Control) 
		{
			printf("Manual control is ON\n");
		}
		else
		{
			printf("Manual control is OFF\n");
		}

		if (Heading > 40.6)
		{
			printf("Heading [%6.3f deg] is greater than 40.6 deg\n", Heading);
		}
		else
		{
			printf("Heading [%6.3f deg] is smaller than 40.6 deg\n", Heading);
		}

		sleep(1);
	}
	return 0;
}







void check_manual_control()
{
	//MANUAL CONTROL
	file = fopen ("/tmp/sailboat/Manual_Control", "r");
  	fscanf (file, "%d", &Manual_Control);
	fclose (file); 
}

void read_weather_station()
{
	//RATE OF TURN
	file = fopen ("/tmp/u200/Rate", "r");
  	fscanf (file, "%f", &Rate);
	fclose (file); 

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
