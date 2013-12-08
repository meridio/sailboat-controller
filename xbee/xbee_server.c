/*
	END_POINT XBEE ttyUSB1 (5412 -> 2669)
	build, split and send the log line to the client pc
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <xbee.h>

#define BUFSIZE 70

FILE* fp;
char logline[1000], logname[50];
char buf[BUFSIZE];
int  i=0,len=0,offset=0;

char * line = NULL;
size_t fl = 0;
ssize_t rr;

int main(void) {
	struct xbee *xbee;
	struct xbee_con *con;
	struct xbee_conAddress address;
	xbee_err ret;

	if ((ret = xbee_setup(&xbee, "xbeeZB", "/dev/ttyUSB1", 57600)) != XBEE_ENONE) {
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
	address.addr64[6] = 0x26;
	address.addr64[7] = 0x69;
	if ((ret = xbee_conNew(xbee, &con, "Data", &address)) != XBEE_ENONE) {
		xbee_log(xbee, -1, "xbee_conNew() returned: %d (%s)", ret, xbee_errorToStr(ret));
		return ret;
	}

	if ((ret = xbee_conDataSet(con, xbee, NULL)) != XBEE_ENONE) {
		xbee_log(xbee, -1, "xbee_conDataSet() returned: %d", ret);
		return ret;
	}

	printf("xBee Server running...\n");
	for (;;) {

		// build the logline
		// sprintf(logline,"#1380879500,2,0,545.9,-35,0,20,-0.0292,95.6255,6.8213,0.4212,54.896885,9.784269,108.3,5.845,1.85,335.95,0.000000,0.000000,0.000000,0.000000,1380879736,5,100,9.779463,54.898883,9.777317,54.895206,9.790492,54.893553,9.790664,54.896983,9.786887,54.898118,9.111111,54.222222\n");
		
		// prepare logline
		system("./prepare_logline.sh");

		fp = fopen("/tmp/logline", "r");
		if (fp != NULL) {
		
			if ((rr = getline(&line, &fl, fp)) != -1) {
				
				sprintf(logline,"%s",line);

				// split and transmit the logline
				len=strlen(logline);
				offset=0;
				for(i=1;i<len;i+=BUFSIZE){
			
					memcpy(buf,&logline[offset],BUFSIZE);
					buf[BUFSIZE]='\0';

					offset+=BUFSIZE;
					printf("[%s]\n",buf);

					// transmit the string
					printf("tx: %d\n",xbee_conTx(con, NULL, buf));
				}
			}
		}

		sleep(1);
	}

	if ((ret = xbee_conEnd(con)) != XBEE_ENONE) {
		xbee_log(xbee, -1, "xbee_conEnd() returned: %d", ret);
		return ret;
	}

	xbee_shutdown(xbee);

	return 0;
}
