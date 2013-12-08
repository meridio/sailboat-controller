/*
	COORDINATOR XBEE ttyUSB0 (2669 -> 5412) 
	Receive and split log lines from the MCU
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <xbee.h>

FILE* file;
char logline[1000],subbuff[100],tmp[100], AreaInfo[500], tmpArea[500], vfile[50];
char *found;
int  offset=0, len=0, i=0;

float Rate=0, Heading=0, Deviation=0, Variation=0, Yaw=0, Pitch=0, Roll=0;
float Latitude=0, Longitude=0, COG=0, SOG=0, Wind_Speed=0, Wind_Angle=0;
float Point_Start_Lat=0, Point_Start_Lon=0, Point_End_Lat=0, Point_End_Lon=0;
float Guidance_Heading=0,  lat=0, lon=0;
int   Rudder_Desired_Angle=0, Manual_Control_Rudder=0, Manual_Control_Sail=0, Rudder_Feedback=0, Navigation_System_Sail=0; 
int   timestp=0, Navigation_System=0, Manual_Control=0, interval=0, nvertexes=0;



void initLog(){
	// crea il fielsystem di base
	system("mkdir -m 777 -p /tmp/sailboat");
	system("mkdir -m 777 -p /tmp/u200");
}

void splitLogLine(){
	
	//sprintf(logline,"#1380879514,2,0,545.9,-35,0,20,-0.0292,95.6255,6.8213,0.4212,54.896885,9.784269,108.3,5.845,1.85,335.95,0.000000,0.000000,0.000000,0.000000,5,100,9.779463,54.898883,9.777317,54.895206,9.790492,54.893553,9.790664,54.896983,9.786887,54.898118");

	// integrity check
	if (logline[0]=='#') {
		
		// printf("\nLOGLINE: %s\n",logline);

		// scanf
		sscanf(logline, "#%d,%d,%d,%f,%d,%d,%d,%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d,%d,%s" \
			, &timestp \
			, &Navigation_System \
			, &Manual_Control \
			, &Guidance_Heading \
			, &Rudder_Desired_Angle \
			, &Navigation_System_Sail \
			, &Manual_Control_Rudder \
			, &Rudder_Feedback \
			, &Rate \
			, &Heading \
			, &Pitch \
			, &Roll \
			, &Latitude \
			, &Longitude \
			, &COG \
			, &SOG \
			, &Wind_Speed \
			, &Wind_Angle \
			, &Point_Start_Lat \
			, &Point_Start_Lon \
			, &Point_End_Lat \
			, &Point_End_Lon \
			, &nvertexes \
			, &interval \
			, AreaInfo \
		);
		
		// write to files
		file = fopen("/tmp/sailboat/Navigation_System", "w");
		if (file != NULL) { fprintf(file, "%d", Navigation_System); fclose(file); }
		
		file = fopen("/tmp/sailboat/Manual_Control", "w");
		if (file != NULL) { fprintf(file, "%d", Manual_Control); fclose(file); }

		file = fopen("/tmp/sailboat/Guidance_Heading", "w");
		if (file != NULL) { fprintf(file, "%f", Guidance_Heading); fclose(file); }

		file = fopen("/tmp/sailboat/Navigation_System_Rudder", "w");
		if (file != NULL) { fprintf(file, "%d", Rudder_Desired_Angle); fclose(file); }

		file = fopen("/tmp/sailboat/Navigation_System_Sail", "w");
		if (file != NULL) { fprintf(file, "%d", Navigation_System_Sail); fclose(file); }

		file = fopen("/tmp/sailboat/Manual_Control_Rudder", "w");
		if (file != NULL) { fprintf(file, "%d", Manual_Control_Rudder); fclose(file); }

		file = fopen("/tmp/sailboat/Rudder_Feedback", "w");
		if (file != NULL) { fprintf(file, "%d", Rudder_Feedback); fclose(file); }

		if (Navigation_System != 5) {
			file = fopen("/tmp/u200/Heading", "w");
			if (file != NULL) { fprintf(file, "%f", Heading); fclose(file); }
			file = fopen("/tmp/u200/Latitude", "w");
			if (file != NULL) { fprintf(file, "%f", Latitude); fclose(file); }
			file = fopen("/tmp/u200/Longitude", "w");
			if (file != NULL) { fprintf(file, "%f", Longitude); fclose(file); }
		}
		else {
			file = fopen("/tmp/sailboat/Simulated_Heading", "w");
			if (file != NULL) { fprintf(file, "%f", Heading); fclose(file); }
			file = fopen("/tmp/sailboat/Simulated_Lat", "w");
			if (file != NULL) { fprintf(file, "%f", Latitude); fclose(file); }
			file = fopen("/tmp/sailboat/Simulated_Lon", "w");
			if (file != NULL) { fprintf(file, "%f", Longitude); fclose(file); }
		}

		file = fopen("/tmp/u200/SOG", "w");
		if (file != NULL) { fprintf(file, "%f", SOG); fclose(file); }

		file = fopen("/tmp/u200/Wind_Speed", "w");
		if (file != NULL) { fprintf(file, "%f", Wind_Speed); fclose(file); }

		file = fopen("/tmp/u200/Wind_Angle", "w");
		if (file != NULL) { fprintf(file, "%f", Wind_Angle); fclose(file); }

		file = fopen("/tmp/sailboat/Point_Start_Lat", "w");
		if (file != NULL) { fprintf(file, "%f", Point_Start_Lat); fclose(file); }

		file = fopen("/tmp/sailboat/Point_Start_Lon", "w");
		if (file != NULL) { fprintf(file, "%f", Point_Start_Lon); fclose(file); }

		file = fopen("/tmp/sailboat/Point_End_Lat", "w");
		if (file != NULL) { fprintf(file, "%f", Point_End_Lat); fclose(file); }

		file = fopen("/tmp/sailboat/Point_End_Lon", "w");
		if (file != NULL) { fprintf(file, "%f", Point_End_Lon); fclose(file); }

		file = fopen("/tmp/sailboat/Area_VertexNum", "w");
		if (file != NULL) { fprintf(file, "%d", nvertexes); fclose(file); }

		file = fopen("/tmp/sailboat/Area_Interval", "w");
		if (file != NULL) { fprintf(file, "%d", interval); fclose(file); }

	
		// se presenti leggi anche le informazioni dell'area
		if(nvertexes>0) {		
			for (i=0;i<nvertexes;i++){
				
				sscanf(AreaInfo,"%f,%f,%s",&lon,&lat,tmpArea);
				
				sprintf(vfile,"/tmp/sailboat/Area_v%d",i);
				file = fopen(vfile, "w");
				if (file != NULL) { fprintf(file, "%f,%f", lon,lat); fclose(file); }

				sprintf(AreaInfo,"%s",tmpArea);			
			}
		}
	}	
}

void getLogLine(struct xbee *xbee, struct xbee_con *con, struct xbee_pkt **pkt, void **data) {
	if ((*pkt)->dataLen > 0) {

		sprintf(tmp,"%s",(*pkt)->data);
		printf("Receiving->[%s]\n",tmp);

		found = strchr(tmp, '\n');
		if (found) {
			len = strlen(tmp) - strlen(found);

			// close log line
			strncpy ( subbuff, tmp, len );
			subbuff[len] = '\0';	
			snprintf(logline+offset, sizeof (logline) - offset, "%s", subbuff);

			// call splitting function
			splitLogLine();

			// azzera e ricomincia a bufferizzare dalla posizione più uno
			logline[0]=0; offset=0;
		}
		else {
			// keep buffering log line
			offset+=snprintf(logline+offset, sizeof (logline) - offset, "%s", (*pkt)->data);
		}		
	}
	//printf("tx: %d\n", xbee_conTx(con, NULL, "Ack\r\n"));
}


int main(void) {
	struct xbee *xbee;
	struct xbee_con *con;
	struct xbee_conAddress address;
	xbee_err ret;

	logline[0]=0;
	initLog();

	if ((ret = xbee_setup(&xbee, "xbeeZB", "/dev/ttyUSB0", 57600)) != XBEE_ENONE) {
		printf("ret: %d (%s)\n", ret, xbee_errorToStr(ret));
		return ret;
	}

	memset(&address, 0, sizeof(address));
	address.addr64_enabled = 1;
	address.addr64[0] = 0x00;
	address.addr64[1] = 0x13;
	address.addr64[2] = 0xA2;
	address.addr64[3] = 0x00;
	address.addr64[4] = 0x40;
	address.addr64[5] = 0x9E;
	address.addr64[6] = 0x54;
	address.addr64[7] = 0x12;
	if ((ret = xbee_conNew(xbee, &con, "Data", &address)) != XBEE_ENONE) {
		xbee_log(xbee, -1, "xbee_conNew() returned: %d (%s)", ret, xbee_errorToStr(ret));
		return ret;
	}

	if ((ret = xbee_conDataSet(con, xbee, NULL)) != XBEE_ENONE) {
		xbee_log(xbee, -1, "xbee_conDataSet() returned: %d", ret);
		return ret;
	}

	if ((ret = xbee_conCallbackSet(con, getLogLine, NULL)) != XBEE_ENONE) {
		xbee_log(xbee, -1, "xbee_conCallbackSet() returned: %d", ret);
		return ret;
	}

	for (;;) {
		void *p;

		if ((ret = xbee_conCallbackGet(con, (xbee_t_conCallback*)&p)) != XBEE_ENONE) {
			xbee_log(xbee, -1, "xbee_conCallbackGet() returned: %d", ret);
			return ret;
		}

		if (p == NULL) break;

		sleep(1);
	}

	if ((ret = xbee_conEnd(con)) != XBEE_ENONE) {
		xbee_log(xbee, -1, "xbee_conEnd() returned: %d", ret);
		return ret;
	}

	xbee_shutdown(xbee);

	return 0;
}
