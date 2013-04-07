/*

Read raw binary data from an AIRMAR U200 NMEA2000 to USB converter
(Actisense NGT-1 chip) and decode it into a readable text format.
A set of files is created in /tmp directory for each field of the
selected PGNs as interface for the sailboat-controller program.

Credits for reading data from a CANbus format and decoding the
NMEA2000 protocol from raw binary data into readable text go to
Kees Verruijt, Harlingen, The Netherlands.
https://github.com/canboat/canboat.git

*/



/*
	- code cleanup (remove all commented statements, per la parte decode almeno)
	- github push
	- file write timeout?
	- reinserire il parsing dei parametri: DEBUG -d
	- significato di FD1_Ready? si può eliminare e utilizzare solo R?
	- ripristinare messaggio di errore quando manca parametro device
	- fare del code cleaning in u200.c per la parte di read
*/


#include "u200.h"

// read
static int  debug = 1;
static bool isFile;
static long timeout = 0;
static unsigned char NGT_STARTUP_SEQ[] = { 0x11 , 0x02 , 0x00 };

enum MSG_State { MSG_START, MSG_ESCAPE, MSG_MESSAGE };
enum ReadyDescriptor { FD1_Ready = 0x0001, FD2_Ready = 0x0002 };

static char * now(void);
static void writeMessage(int handle, unsigned char command, const unsigned char * cmd, const size_t len);
static enum ReadyDescriptor isready(int fd1, int fd2);
static int  readNGT1(int handle);
static void readNGT1Byte(unsigned char c);
static void messageReceived(const unsigned char * msg, size_t msgLen);
static void n2kMessageReceived(const unsigned char * msg, size_t msgLen);
static void ngtMessageReceived(const unsigned char * msg, size_t msgLen);

// decode
DevicePackets * device[256];
size_t heapSize = 0;
int clockSrc = -1;

void msgdec(char * msg);
static int scanHex(char ** p, uint8_t * m);
static uint8_t scanNibble(char c);
bool printCanFormat(RawMessage * msg);
void printPacket(size_t index, RawMessage * msg);
bool printPgn(int index, int subIndex, RawMessage * msg);
static void extractNumber(Field * field, uint8_t * data, size_t startBit, size_t bits, int64_t * value, int64_t * maxValue);
static bool printNumber(char * fieldName, Field * field, uint8_t * data, size_t startBit, size_t bits);
static bool printLatLon(char * name, double resolution, uint8_t * data, size_t byte);

// write to disk
int i, pos=0, currentPgn=0;
char tmpchar[50];
ListItem currentList[20];
void initFiles();
void addtolist();
void writeondisk();



int main(int argc, char ** argv)
{
	int handle;
	char * device = argv[1];
	struct termios attr;
	struct stat statbuf;

	if (debug) fprintf(stderr, "Opening %s\n", device);
	handle = open(device, O_RDWR | O_NOCTTY);
	if (debug) fprintf(stderr, "fd = %d\n", handle);
	if (handle < 0)
	{
		fprintf(stderr, "Cannot open NGT-1-A device %s\n", device);
		exit(1);
	}
	if (fstat(handle, &statbuf) < 0)
	{
		fprintf(stderr, "Cannot determine device %s\n", device);
		exit(1);
	}

	isFile = S_ISREG(statbuf.st_mode);
	if (isFile)
	{
		if (debug) fprintf(stderr, "Device is a normal file, do not set the attributes.\n");
	}
	else
	{
		if (debug) fprintf(stderr, "Device is a serial port, set the attributes.\n");

		memset(&attr, 0, sizeof(attr));
		cfsetispeed(&attr, B115200);
		cfsetospeed(&attr, B115200);
		attr.c_cflag |= CS8 | CLOCAL | CREAD;

		attr.c_iflag |= IGNPAR;
		attr.c_cc[VMIN] = 1;
		attr.c_cc[VTIME] = 0;
		tcflush(handle, TCIFLUSH);
		tcsetattr(handle, TCSANOW, &attr);

		if (debug) fprintf(stderr, "Device is a serial port, send the startup sequence.\n");

		writeMessage(handle, NGT_MSG_SEND, NGT_STARTUP_SEQ, sizeof(NGT_STARTUP_SEQ));
		sleep(2);
	}

	initFiles();

	for (;;)
	{
		unsigned char msg[BUFFER_SIZE];
		enum ReadyDescriptor r;

		r = isready(handle, 0);

		if ((r & FD1_Ready) > 0)
		{
			if (!readNGT1(handle))
			{
				if (debug) fprintf(stdout, "DBG_01: loop break\n");
				break;
			}
		}
/*
		if ((r & FD2_Ready) > 0)
		{
			if (debug) fprintf(stdout, "DBG_02: FD2 - %s \n", msg);
			fflush(stdout);
		}
*/
	}

	close(handle);
	return 0;
}




static enum ReadyDescriptor isready(int fd1, int fd2)
{
	fd_set fds;
	struct timeval waitfor;
	int setsize;
	int r;
	enum ReadyDescriptor ret = 0;

	FD_ZERO(&fds);
	if (fd1 >= 0) { FD_SET(fd1, &fds); }
	if (fd2 >= 0) { FD_SET(fd2, &fds); }
	waitfor.tv_sec = timeout ? timeout : 10;
	waitfor.tv_usec = 0;
	if (fd1 > fd2) { setsize = fd1 + 1; }
	else { setsize = fd2 + 1; }

	r = select(setsize, &fds, 0, 0, &waitfor);
	if (r < 0)
	{
		fprintf(stderr,"I/O error; restart by quit\n");
	}
	if (r > 0)
	{
		if (fd1 >= 0 && FD_ISSET(fd1, &fds)) { ret |= FD1_Ready; }
		if (fd2 >= 0 && FD_ISSET(fd2, &fds)) { ret |= FD2_Ready; }
	}
	if (!ret && timeout)
	{
		fprintf(stderr,"Timeout %ld seconds; restart by quit\n", timeout);
	}
	return ret;
}


static void writeMessage(int handle, unsigned char command, const unsigned char * cmd, const size_t len)
{
	unsigned char bst[255];
	unsigned char *b = bst;
	unsigned char *lenPtr;
	unsigned char crc;
	int i;

	*b++ = DLE;
	*b++ = STX;
	*b++ = command;
	crc = command;
	lenPtr = b++;

	for (i = 0; i < len; i++)
	{
		if (cmd[i] == DLE) { *b++ = DLE; }
		*b++ = cmd[i];
		crc += (unsigned char) cmd[i];
	}

	*lenPtr = i;
	crc += i;

	*b++ = (unsigned char) (256 - (int)crc);
	*b++ = DLE;
	*b++ = ETX;

	if (write(handle, bst, b - bst) != b - bst)
	{
		fprintf(stderr,"Unable to write command '%.*s' to NGT-1-A device\n", (int) len, cmd);
	}
	fprintf(stderr,"Written command %X len %d\n", command, (int) len);
}


/*
 * Read serial data from the NGT1
 */
static int readNGT1(int handle)
{
	size_t i;
	ssize_t r;
	//bool printed = 0;
	unsigned char c;
	unsigned char buf[500];

	r = read(handle, buf, sizeof(buf));

	if (r <= 0) /* No char read, abort message read */
	{
		fprintf(stderr,"Unable to read from NGT1 device\n");
	}

	/* fprintf(stderr,"Read %d bytes from device\n", (int) r);*/
	/*  // if (debug) 
	if (0)
	{
		fprintf(stderr, "DEBUG read: ");
		for (i = 0; i < r; i++)
		{
			c = buf[i];
			fprintf(stderr, " %02X", c);
		}
		fprintf(stderr, "\n");
	}*/

	for (i = 0; i < r; i++)
	{
		c = buf[i];
		readNGT1Byte(c);
	}

	return r;
}


/*
 * Handle a byte coming in from the NGT1.
 */
static void readNGT1Byte(unsigned char c)
{
	static enum MSG_State state = MSG_START;
	//static bool startEscape = false;
	static bool noEscape = false;
	static unsigned char buf[500];
	static unsigned char * head = buf;

	// fprintf(stderr,"received byte %02x state=%d offset=%d\n", c, state, head - buf); 

	if (state == MSG_START)
	{
		if ((c == ESC) && isFile) { noEscape = true; }
	}

	if (state == MSG_ESCAPE)
	{
		if (c == ETX)
		{
			messageReceived(buf, head - buf);
			head = buf;
			state = MSG_START;
		}
		else if (c == STX)
		{
			head = buf;
			state = MSG_MESSAGE;
		}
		else if ((c == DLE) || ((c == ESC) && isFile) || noEscape)
		{
			*head++ = c;
			state = MSG_MESSAGE;
		}
		else
		{
			/*logError("DLE followed by unexpected char %02X, ignore message\n", c);*/
			fprintf(stderr,"DLE followed by unexpected char %02X, ignore message\n", c);
			state = MSG_START;
		}
	}
	else if (state == MSG_MESSAGE)
	{
		if (c == DLE) { state = MSG_ESCAPE; }
		else if (isFile && (c == ESC) && !noEscape) { state = MSG_ESCAPE; }
		else { *head++ = c; }
	}
	else
	{
		if (c == DLE) { state = MSG_ESCAPE; }
	}
}


/*
 * every time I receive a message termination byte:
 */
static void messageReceived(const unsigned char * msg, size_t msgLen)
{
	unsigned char command;
	unsigned char checksum = 0;
	//unsigned char * payload;
	unsigned char payloadLen;
	size_t i;

	if (msgLen < 3)
	{
		fprintf(stderr,"Ignore short command len = %zu\n", msgLen);
		return;
	}

	for (i = 0; i < msgLen; i++)
	{
		checksum += msg[i];
	}
	if (checksum)
	{
		fprintf(stderr,"Ignoring message with invalid checksum\n");
		return;
	}

	command = msg[0];
	payloadLen = msg[1];

	fprintf(stderr,"message command = %02x len = %u\n", command, payloadLen);

	if (command == N2K_MSG_RECEIVED)
	{
		n2kMessageReceived(msg + 2, payloadLen);
	}
	else if (command == NGT_MSG_RECEIVED)
	{
		ngtMessageReceived(msg + 2, payloadLen);
	}
}


/*
 * in case of NGT-1 specific message
 */
static void ngtMessageReceived(const unsigned char * msg, size_t msgLen)
{
	size_t i;
	char line[1000];
	char * p;

	if (msgLen < 12)
	{
		fprintf(stderr,"Ignore short msg len = %zu\n", msgLen);
		return;
	}

	sprintf(line, "%s,%u,%u,%u,%u,%u", now(), 0, 0x40000 + msg[0], 0, 0, (unsigned int) msgLen - 1);
	p = line + strlen(line);
	for (i = 1; i < msgLen && p < line + sizeof(line) - 5; i++)
	{
		sprintf(p, ",%02x", msg[i]);
		p += 3;
	}
	*p++ = 0;

	puts(line);
	fflush(stdout);
}


/*
 * In case of a NMEA2000 message
 */
static void n2kMessageReceived(const unsigned char * msg, size_t msgLen)
{
	unsigned int prio, src, dst;
	unsigned int pgn;
	size_t i;
	unsigned int len;
	char line[800];
	//char ckpgn[6];
	char * p;

	if (msgLen < 11)
	{
		fprintf(stderr,"Ignoring N2K message - too short\n");
		return;
	}
	prio = msg[0];
	pgn  = (unsigned int) msg[1] + 256 * ((unsigned int) msg[2] + 256 * (unsigned int) msg[3]);
	dst  = msg[4];
	src  = msg[5];
	/* Skip the timestamp logged by the NGT-1-A in bytes 6-9 */
	len  = msg[10];

	if (len > 223)
	{
		fprintf(stderr,"Ignoring N2K message - too long (%u)\n", len);
		return;
	}

	p = line;

	snprintf(p, sizeof(line), "%s,%u,%u,%u,%u,%u", now(), prio, pgn, src, dst, len);
	p += strlen(line);

	len += 11;
	for (i = 11; i < len; i++)
	{
		snprintf(p, line + sizeof(line) - p, ",%02x", msg[i]);
		p += strlen(p);
	}

	/* Send the line with exadecimal values to the decoder */
	//puts(line);
	//strncpy(ckpgn, line+26, 6);
	if(	pgn!=262386 && pgn!=130312 && pgn!=130313 && pgn!=130314)
	{
		msgdec(line);
	}

	fflush(stdout);
}


/*
 * CALCULATE NOW()
 */
static char * now(void)
{
	static char str[64];
	struct timeval tv;
	struct tm tm;

	str[0] = 0;

	if (gettimeofday(&tv, 0) != 0)
	{
		return "?";
	}

	gmtime_r(&tv.tv_sec, &tm);

	snprintf(str, sizeof(str), "%u", (unsigned int) tv.tv_sec);

	strftime(str, sizeof(str) - 5, "%F-%T", &tm);
	snprintf(str + strlen(str), 5, ".%03d", (int) (tv.tv_usec / 1000L));

	return str;
}



void msgdec(char * msg) 
{
	int r;

	RawMessage m;
	//unsigned int prio, pgn, dst, src, len, junk;
	unsigned int prio, pgn, dst, src, len;
	unsigned int i;
	char * p;
	

	// START FROM "MSG" STRING		
	// fprintf(stdout, "\n%s", msg);
	
	// MKS: using RAWFORMAT_FAST by default
	p = strchr(msg, ',');
	
	memcpy(m.timestamp, msg, p - msg);
	m.timestamp[p - msg] = 0;

	/* Moronic Windows does not support %hh<type> so we use intermediate variables */
	r = sscanf( p
      , ",%u,%u,%u,%u,%u "
      , &prio
      , &pgn
      , &src
      , &dst
      , &len
    );

	if (r < 5)	{
		fprintf(stdout, "Error reading message, scanned [%u] from [%s]", r, msg);
		return;
	}

	for (i = 0; *p && i < 5;)
	{
		if (*++p == ',') { i++; }
	}

	if (!p) {
		fprintf(stdout, "Error reading message, scanned [%zu] bytes from [%s]", p - msg, msg);
		return;
	}
  
	p++;

	for (i = 0; i < len; i++)
	{
		if (scanHex(&p, &m.data[i])) {
			fprintf(stdout,"Error(1) reading message\n");
			continue;
		}
		if (i < len)
		{
			if (*p != ',' && !isspace(*p)) {
				fprintf(stdout,"Error(2) reading message\n");
				continue;
			}
			p++;
		}
	}

	m.prio = prio;
	m.pgn  = pgn;
	m.dst  = dst;
	m.src  = src;
	m.len  = len;

	currentPgn=pgn; pos=0;
	printCanFormat(&m);
	writeondisk();

}

static int scanHex(char ** p, uint8_t * m)
{
	uint8_t hi, lo;

	if (!(*p)[0] || !(*p)[1]) { return 1; }

	hi = scanNibble((*p)[0]);
	if (hi > 15) { return 1; }

	lo = scanNibble((*p)[1]);
	if (lo > 15) { return 1;}

	(*p) += 2;
	*m = hi << 4 | lo;
	// printf("(b=%02X,p=%p) ", *m, *p);
	return 0;
}

static uint8_t scanNibble(char c)
{
	if (isdigit(c)) { return c - '0'; }
	if (c >= 'A' && c <= 'F') {	return c - 'A' + 10; }
	if (c >= 'a' && c <= 'f') {	return c - 'a' + 10; }
	return 16;
}

bool printCanFormat(RawMessage * msg)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(pgnList); i++)
	{
		if (msg->pgn == pgnList[i].pgn)
		{
			if (!pgnList[i].size) { return true; } // Determine size by raw packet first 

			/* Found the pgn that matches this particular packet */
			printPacket(i, msg);
			return true;
		}
	}

	if (i == ARRAY_SIZE(pgnList)) { printPacket(0, msg); }

	return false;
}



void printPacket(size_t index, RawMessage * msg)
{
	// size_t fastPacketIndex;  //unsued
	// size_t bucket;			//unused
	Packet * packet;
	Pgn * pgn = &pgnList[index];
	size_t subIndex;

	// allocate memory foreach new device
	if (!device[msg->src])
	{
		heapSize += sizeof(DevicePackets);
		device[msg->src] = calloc(1, sizeof(DevicePackets));
		if (!device[msg->src])
		{
			fprintf(stdout, "Error: Out of memory\n");
			exit(1);
		}
	}
	packet = &(device[msg->src]->packetList[index]);
	
	// allocate memory for data
	if (!packet->data)
	{
		packet->allocSize = max(min(pgn->size, 8) + FASTPACKET_BUCKET_N_SIZE, msg->len);
		heapSize += packet->allocSize;
		packet->data = malloc(packet->allocSize);
		if (!packet->data)
		{
			fprintf(stdout, "Out of memory\n");
			exit(1);
		}
	}

	/* assuming RAWFORMAT_FAST: */
	/*	if (msg->len > 0x8 || format != RAWFORMAT_PLAIN) 
	{ */
	if (packet->allocSize < msg->len)
	{
		heapSize += msg->len - packet->allocSize;
		packet->data = realloc(packet->data, msg->len);
		packet->data = realloc(packet->data, msg->len);
		if (!packet->data)
		{
			fprintf(stdout, "Out of memory\n");
			exit(1);
		}
		packet->allocSize = msg->len;
	}
	memcpy( packet->data
			, msg->data
			, msg->len
	);
	packet->size = msg->len;
	/* }  */

/* still assuming RAWFORMAT_FAST:  */

/* else if (pgn->size > 0x8)
  {
    fastPacketIndex = msg->data[FASTPACKET_INDEX];
    bucket = fastPacketIndex & FASTPACKET_MAX_INDEX;

    if (bucket == 0)
    {
      size_t newSize = msg->data[FASTPACKET_SIZE] + FASTPACKET_BUCKET_N_SIZE;

      if (packet->allocSize < newSize)
      {
        heapSize += newSize - packet->allocSize;
        logDebug("Resizing buffer for PGN %u device %u to accomodate %zu bytes (heap %zu bytes)\n", pgn->pgn, msg->src, newSize, heapSize);
        packet->data = realloc(packet->data, newSize);
        if (!packet->data)
        {
          die("Out of memory\n");
        }
        packet->allocSize = newSize;
      }
      packet->size = msg->data[FASTPACKET_SIZE];
      memcpy( packet->data
            , msg->data + FASTPACKET_BUCKET_0_OFFSET
            , FASTPACKET_BUCKET_0_SIZE
            );
    }
    else
    {
      if (packet->lastFastPacket + 1 != fastPacketIndex)
      {
        logError("PGN %u malformed packet for %u received; expected %zu but got %zu\n"
                , pgn->pgn, msg->src, packet->lastFastPacket + 1, fastPacketIndex
                );
        return;
      }
      memcpy( packet->data + FASTPACKET_BUCKET_0_SIZE + FASTPACKET_BUCKET_N_SIZE * (bucket - 1)
            , msg->data + FASTPACKET_BUCKET_N_OFFSET
            , FASTPACKET_BUCKET_N_SIZE
            );
    }
    packet->lastFastPacket = fastPacketIndex;

    if (FASTPACKET_BUCKET_0_SIZE + FASTPACKET_BUCKET_N_SIZE * bucket < packet->size)
    {
      // Packet is not complete yet 
      return;
    }
  }
  else // msg->len <= 8 && pgn->size <= 0x8 
  {
    packet->size = msg->len;
    memcpy( packet->data
          , msg->data
          , msg->len
          );
  }
*/

	subIndex = index;
	for (subIndex = index; subIndex < ARRAY_SIZE(pgnList) && (msg->pgn == pgnList[subIndex].pgn || !index); subIndex++)
	{
		if (printPgn(index, subIndex, msg)) // Only the really matching ones will actually return true 
		{
			if (index != subIndex)
			{
				// logDebug("PGN %d matches version %zu\n", msg->pgn, subIndex - index);//
			}
		// mwrite(stdout);
		break;
		}
		else
		{
			// mreset();
		}
	}

}




bool printPgn(int index, int subIndex, RawMessage * msg)
{
	uint8_t * dataStart;
	uint8_t * data;
	size_t size;
	uint8_t * dataEnd;
	size_t i;
	Field field;
	size_t bits;
	size_t bytes;
	size_t startBit;
	Pgn * pgn;
	int      repetition = 1;
	uint16_t valueu16;
	uint32_t valueu32;
	// uint16_t currentDate = UINT16_MAX;
	// uint32_t currentTime = UINT32_MAX;
	char fieldName[60];
	bool r;
	bool matchedFixedField;
	//bool hasFixedField;
	uint32_t refPgn = 0;

	if (!device[msg->src])	{ return false; }

	dataStart = device[msg->src]->packetList[index].data;
	if (!dataStart)	{ return false;	}
	size = device[msg->src]->packetList[index].size;
	dataEnd = dataStart + size;

	for (;(index < ARRAY_SIZE(pgnList)) && (msg->pgn == pgnList[index].pgn); index++)
	{
		matchedFixedField = true;
		//hasFixedField = false;

		// There is a next index that we can use as well. We do so if the 'fixed' fields don't match 

		pgn = &pgnList[index];

		for (i = 0, startBit = 0, data = dataStart; i < pgn->fieldCount; i++)
		{
			field = pgn->fieldList[i];
			if (!field.name || !field.size) { break; }

			bits = field.size;

			if (field.units && field.units[0] == '=')
			{
				int64_t value, desiredValue;
				int64_t maxValue;

				//hasFixedField = true;
				extractNumber(&field, data, startBit, field.size, &value, &maxValue);
				desiredValue = strtol(field.units + 1, 0, 10);
				if (value != desiredValue)
				{
					matchedFixedField = false;
					break;
				}
			}
			startBit += bits;
			data += startBit / 8;
			startBit %= 8;
		}
		if (matchedFixedField) { break; }
	}

	if ((index >= ARRAY_SIZE(pgnList)) || (msg->pgn != pgnList[index].pgn)) { index = 0; }

	pgn = &pgnList[index];


	//    mprintf("%s %u %3u %3u %6u %s:", msg->timestamp, msg->prio, msg->src, msg->dst, msg->pgn, pgn->description);
	//    sep = " ";
	fprintf(stdout,"\n----- PGN [%6u] -- %s -----------\n", msg->pgn, pgn->description);


	// parse all the bytes of the message body and translate them into numeric values
	for (i = 0, startBit = 0, data = dataStart; data < dataEnd; i++)
	{
		r = true;
		field = pgn->fieldList[i];
		if (!field.name)
		{
			if (pgn->repeatingFields)
			{
				i = i - pgn->repeatingFields;
				field = pgn->fieldList[i];
				repetition++;
			}
			else { break; }
		}

		if (repetition > 1)
		{
			sprintf(fieldName, "%s #%u", field.name, repetition);
		}
		else
		{
			strcpy(fieldName, field.name);
		}

		bits  = field.size;
		bytes = (bits + 7) / 8;
		bytes = min(bytes, (size_t) (dataEnd - data));
		bits  = min(bytes * 8, bits);


		if (strcmp(fieldName, "PGN") == 0)
		{
			refPgn = data[0] + (data[1] << 8) + (data[2] << 16);
		}

		if (field.resolution < 0.0)
		{
			int len;
			int k;

			// These fields have only been found to start on byte boundaries,
			// making their location easier

			if (field.resolution == RES_STRINGLZ)
			{
				len = *data++;
				bytes--;
				goto ascii_string;
			}

			if (field.resolution == RES_ASCII)
			{
				len = (int) bytes;
				char lastbyte = data[len - 1];

				if (lastbyte == 0xff || lastbyte == ' ' || lastbyte == 0 || lastbyte == '@')
				{
					while (len > 0 && (data[len - 1] == lastbyte)) { len--; }
				}

ascii_string:

				fprintf(stdout,"DBG_02: [%s] DATA: [", fieldName);

				for (k = 0; k < len; k++)
				{
					if (data[k] >= ' ' && data[k] <= '~')
					{
						int c = data[k];
						// mprintf("%c", c);
						fprintf(stdout,"(%c)", c);
					}
				}
				fprintf(stdout,"]\n");
			}
			else if (field.resolution == RES_STRING)
			{
				int len;
				if (*data == 0x02)
				{
					data++;
					for (len = 0; data + len < dataEnd && data[len] != 0x01; len++);
					bytes = len + 1;
				}
				else if (*data > 0x02)
				{
					bytes = *data++;
					bytes--; // Compensate for that we've already increased data by 1 
					if (*data == 0x01)
					{
						data++;
						bytes--;
					}
					len = bytes - 1;
				}
				else { bytes = 1; len = 0; }

				if (len) { fprintf(stdout,"**DBG_04: STRING: FIELD [%s], DATA [%s]\n", fieldName, data); }
				bits = BYTES(bytes);
			}
			else if (field.resolution == RES_LONGITUDE || field.resolution == RES_LATITUDE)
			{
				fprintf(stdout,"DBG_05: LAT-LON -> ");
				printLatLon(fieldName, field.resolution, data, bytes);
			}
			else if (field.resolution == RES_DATE)
			{
				memcpy((void *) &valueu16, data, 2);
				fprintf(stdout,"DBG_06: DATE -> (ignored)\n");
				//printDate(fieldName, valueu16);
				//currentDate = valueu16;
			}
			else if (field.resolution == RES_TIME)
			{
				memcpy((void *) &valueu32, data, 4);
				fprintf(stdout,"DBG_07: TIME -> (ignored)\n");
				//printTime(fieldName, valueu32);
				//currentTime = valueu32;
			}
			else if (field.resolution == RES_TEMPERATURE)
			{
				memcpy((void *) &valueu16, data, 2);
				fprintf(stdout,"DBG_08: TEMPERATURE: (ignored)\n");
				//printTemperature(fieldName, valueu16);
			}
			else if (field.resolution == RES_PRESSURE)
			{
				memcpy((void *) &valueu16, data, 2);
				fprintf(stdout,"DBG_09: PRESSURE: (ignored)\n");
				//printPressure(fieldName, valueu16);
			}
			else if (field.resolution == RES_6BITASCII)
			{
				//print6BitASCIIText(fieldName, data, startBit, bits);
				fprintf(stdout,"**DBG_10: ASCII TEXT [...]\n");
			}
			else if (bits == LEN_VARIABLE)
			{
				//printVarNumber(fieldName, pgn, refPgn, &field, data, startBit, &bits);
				fprintf(stdout,"DBG_11: VAR_NUM [...]\n");
			}
			else if (bits > BYTES(8))
			{
				//printHex(fieldName, data, startBit, bits);
				fprintf(stdout,"DBG_12: HEX [...]\n");
			}
			else if (field.resolution == RES_INTEGER
			|| field.resolution == RES_LOOKUP
			|| field.resolution == RES_BINARY
			|| field.resolution == RES_MANUFACTURER
			)
			{
				fprintf(stdout,"DBG_13: NUM -> ");
				printNumber(fieldName, &field, data, startBit, bits);
			}
			else
			{
				//logError("Unknown resolution %f for %s\n", field.resolution, fieldName);
				fprintf(stdout,"Unknown resolution %f for %s\n", field.resolution, fieldName);
			}

		}
		else if (field.resolution > 0.0)
		{
			fprintf(stdout,"DBG_14: NUM -> ");
			printNumber(fieldName, &field, data, startBit, bits);
		}
		if (!r)
		{
			fprintf(stdout,"DBG!! IGNORE THIS\n\n");
			return false;
		}

		startBit += bits;
		data += startBit / 8;
		startBit %= 8;
	}

	/*
	if (msg->pgn == 126992 && currentDate < UINT16_MAX && currentTime < UINT32_MAX && clockSrc == msg->src)
	{
		setSystemClock(currentDate, currentTime);
	}
	*/
	return r;
}


static void extractNumber(Field * field, uint8_t * data, size_t startBit, size_t bits, int64_t * value, int64_t * maxValue)
{
	bool hasSign = field->hasSign;

	size_t firstBit = startBit;
	size_t bitsRemaining = bits;
	size_t magnitude = 0;
	size_t bitsInThisByte;
	uint64_t bitMask;
	uint64_t allOnes;
	uint64_t valueInThisByte;

	*value = 0;
	*maxValue = 0;

	while (bitsRemaining)
	{
		bitsInThisByte = min(8 - firstBit, bitsRemaining);
		allOnes = (uint64_t) ((((uint64_t) 1) << bitsInThisByte) - 1);

		//How are bits ordered in bytes for bit fields? There are two ways, first field at LSB or first
		//field as MSB.
		//Experimentation, using the 129026 PGN, has shown that the most likely candidate is LSB.
		bitMask = allOnes << firstBit;
		valueInThisByte = (*data & bitMask) >> firstBit;

		*value |= valueInThisByte << magnitude;
		*maxValue |= (int64_t) allOnes << magnitude;

		magnitude += bitsInThisByte;
		bitsRemaining -= bitsInThisByte;
		firstBit += bitsInThisByte;
		if (firstBit >= 8)
		{
			firstBit -= 8;
			data++;
		}
	}

	if (hasSign)
	{
		*maxValue >>= 1;

		if (field->offset) /* J1939 Excess-K notation */
		{
			*value += field->offset;
		}
		else
		{
			bool negative = (*value & (((uint64_t) 1) << (bits - 1))) > 0;

			if (negative)
			{
				/* Sign extend value for cases where bits < 64 */
				/* Assume we have bits = 16 and value = -2 then we do: */
				/* 0000.0000.0000.0000.0111.1111.1111.1101 value    */
				/* 0000.0000.0000.0000.0111.1111.1111.1111 maxvalue */
				/* 1111.1111.1111.1111.1000.0000.0000.0000 ~maxvalue */
				*value |= ~*maxValue;
			}
		}
	}
}


static bool printNumber(char * fieldName, Field * field, uint8_t * data, size_t startBit, size_t bits)
{
	//bool ret = false;
	int64_t value;
	int64_t maxValue;
	int64_t notUsed;
	double a;

	extractNumber(field, data, startBit, bits, &value, &maxValue);

	if (maxValue >= 15)	{ notUsed = 2; }
	else if (maxValue > 1) { notUsed = 1; }
	else { notUsed = 0; }

	if (value <= maxValue - notUsed)
	{
		if (field->units && field->units[0] == '=')
		{
			char lookfor[20];
			char * s;

			sprintf(lookfor, "=%"PRId64, value);
			if (strcmp(lookfor, field->units) != 0)
			{
				fprintf(stdout,"RETURN FALSE ***\n");
				return false;
			}
			s = field->description;
			if (!s) { s = lookfor + 1; }

			// mprintf("%s %s = %s", getSep(), fieldName, s);
			fprintf(stdout,"A) FIELD [%s], DATA [%s]\n", fieldName, data);
		}
		else
		if (field->resolution == RES_LOOKUP && field->units)
		{
			char lookfor[20];
			char * s, * e;

			sprintf(lookfor, ",%"PRId64"=", value);
			s = strstr(field->units, lookfor);
			if (s)
			{
				s += strlen(lookfor);
				e = strchr(s, ',');
				e = e ? e : s + strlen(s);

				fprintf(stdout,"B) [%s] : %.*s\n", fieldName,  (int) (e - s), s);

				sprintf(tmpchar,"%.*s", (int) (e - s), s);
				addtolist(fieldName, tmpchar);
			}
			else
			{
				// mprintf("%s %s = %"PRId64"", getSep(), fieldName, value);
				fprintf(stdout,"C) FIELD [%s], DATA [%"PRId64"]\n", fieldName, value);
			}
		}
		else if (field->resolution == RES_BINARY)
		{
			//  mprintf("%s %s = 0x%"PRIx64, getSep(), fieldName, value);
			fprintf(stdout,"D) FIELD [%s], DATA [%"PRIx64"]\n", fieldName, value);
		}
		else if (field->resolution == RES_MANUFACTURER)
		{
			// I DONT'T CARE ABOUT MANUFACTURER
			fprintf(stdout,"MANUFACTURER ...\n");
		}
		else
		{

			if (field->resolution == RES_INTEGER)
			{
				//  mprintf("%s %s = %"PRId64, getSep(), fieldName, value);
				fprintf(stdout,"E) FIELD [%s], DATA = [%"PRId64"]", fieldName, data);
			}
			else
			{
				int precision = 0;
				double r;

				a = value * field->resolution;

				if (field->resolution == RES_DEGREES) { precision = 1; }
				else if (field->resolution == RES_DEGREES * 0.0001)	{ precision = 4; }
				else
				{
					for (r = field->resolution; (r > 0.0) && (r < 1.0); r *= 10.0) { precision++; }
				}

				if (field->units && strcmp(field->units, "m") == 0 && a >= 1000.0)
				{
					//mprintf("%s %s = %.*f km", getSep(), fieldName, precision + 3, a / 1000);
					fprintf(stdout,"F) FIELD [%s], DATA = [%.*f]Km \n", fieldName, precision + 3, a / 1000);
				}
				else
				{
					//mprintf("%s %s = %.*f", getSep(), fieldName, precision, a);
					fprintf(stdout,"G) [%s] : %.*f ", fieldName, precision, a);

					sprintf(tmpchar,"%.*f", precision, a);
					addtolist(fieldName, tmpchar);

					if (field->units)
					{
						//mprintf(" %s", field->units);
						fprintf(stdout,"(%s)\n", field->units);
					}
					else
					{
						fprintf(stdout,"\n");
					}
				}
			}
		}
	}
	else
	{
		// undefined value
		fprintf(stdout,"   [%s]: ???? \n",fieldName);
		sprintf(tmpchar,"?");
		addtolist(fieldName,tmpchar);
	}

	return true;
}



/*
 * There are three ways to print lat/long: DD, DM, DMS.
 * We print in the way set by the config. Default is DMS. DD is useful for Google Maps.
 */

static bool printLatLon(char * name, double resolution, uint8_t * data, size_t bytes)
{
	uint64_t absVal;
	int64_t value;

	value = 0;
	memcpy(&value, data, bytes);
	if (bytes == 4 && ((data[3] & 0x80) > 0))
	{
		value |= UINT64_C(0xffffffff00000000);
	}
	if (value > ((bytes == 8) ? INT64_C(0x7ffffffffffffffd) : INT64_C(0x7ffffffd)))
	{
		fprintf(stdout," [%s]: ???? **********\n",name);
		sprintf(tmpchar,"?");
		addtolist(name, tmpchar);
		return false;
	}

	if (bytes == 8)
	{
		value /= INT64_C(1000000000);
	}
	absVal = (value < 0) ? -value : value;


	// DD
		double dd = (double) value / (double) RES_LAT_LONG_PRECISION;
		//mprintf("%s %s = %010.7f", getSep(), name, dd);
		fprintf(stdout,"%s = %010.7f \n",name, dd);
		sprintf(tmpchar,"%010.7f", dd);
		addtolist(name, tmpchar);
   

/*
	// DM  //One degree = 10e6 
		uint64_t degrees = (absVal / RES_LAT_LONG_PRECISION);
		uint64_t remainder = (absVal % RES_LAT_LONG_PRECISION);
		double minutes = (remainder * 60) / (double) RES_LAT_LONG_PRECISION;

		mprintf((showJson ? "%s\"%s\":\"%02u&deg; %06.3f %c\"" : "%s %s = %02ud %06.3f %c")
				, getSep(), name, (uint32_t) degrees, minutes
				, ((resolution == RES_LONGITUDE)
					? ((value >= 0) ? 'E' : 'W')
					: ((value >= 0) ? 'N' : 'S')
					)
				);
*/
/*
	// DMS
		uint32_t degrees = (uint32_t) (absVal / RES_LAT_LONG_PRECISION);
		uint32_t remainder = (uint32_t) (absVal % RES_LAT_LONG_PRECISION);
		uint32_t minutes = (remainder * 60) / RES_LAT_LONG_PRECISION;
		double seconds = (((uint64_t) remainder * 3600) / (double) RES_LAT_LONG_PRECISION) - (60 * minutes);

		fprintf(stdout, "[%s] : %02ud %02u' %06.3f\"%c \n"
				, name, degrees, minutes, seconds
				, ((resolution == RES_LONGITUDE)
					? ((value >= 0) ? 'E' : 'W')
					: ((value >= 0) ? 'N' : 'S')
					)
				);
*/
		
  return true;
}



/*
 *	Add a new [NAME-VALUE] entry to the list of the current PGN
 */
void addtolist(char name[], char value[])
{

	//convert spaces into underscores
	char *p;
	while (1)  {
		p = strchr(name, ' ');
		if (p == NULL)  { break; }
		*p = '_';
	}

	// add the new entry and incremente the place holder 'pos'
	ListItem m;
	strcpy(m.name, name);
	strcpy(m.value, value);
	currentList[pos]=m;
	pos++;
}

void initFiles(){
	//system("mkdir /tmp/{127251,127250,127257,129025,129026,130306}");
	system("mkdir /tmp/u200");

	system("touch /tmp/u200/Rate");
	system("touch /tmp/u200/Heading");
	system("touch /tmp/u200/Deviation");
	system("touch /tmp/u200/Variation");
	system("touch /tmp/u200/Yaw");
	system("touch /tmp/u200/Pitch");
	system("touch /tmp/u200/Roll");
	system("touch /tmp/u200/Latitude");
	system("touch /tmp/u200/Longitude");
	system("touch /tmp/u200/COG");
	system("touch /tmp/u200/SOG");
	system("touch /tmp/u200/Wind_Speed");
	system("touch /tmp/u200/Wind_Angle");
}


/*
 *	Write each notnull [NAME-VALUE] entry of the current PGN to the relevant file
 */
void writeondisk()
{
	FILE *file; 

	fprintf(stdout,"\n");

	// For each Field decoded from the current message body, write the value to file if not null
	for(i=0;i<pos;i++) {

		if (
			// Rate of Turn 
			( currentPgn == 127251 && strcmp(currentList[i].name,"Rate")==0	)

			// Vessel Heading
			||  ( currentPgn == 127250 && strcmp(currentList[i].name,"Heading")==0 )
			||  ( currentPgn == 127250 && strcmp(currentList[i].name,"Deviation")==0 )
			||  ( currentPgn == 127250 && strcmp(currentList[i].name,"Variation")==0 )

			// Attitude
			||  ( currentPgn == 127257 && strcmp(currentList[i].name,"Yaw")==0 )
			||  ( currentPgn == 127257 && strcmp(currentList[i].name,"Pitch")==0 )
			||  ( currentPgn == 127257 && strcmp(currentList[i].name,"Roll")==0 )

			// Position, Rapid Update
			||  ( currentPgn == 129025 && strcmp(currentList[i].name,"Latitude")==0 )
			||  ( currentPgn == 129025 && strcmp(currentList[i].name,"Longitude")==0 )	

			// COG and SOG, Rapid Update
			||  ( currentPgn == 129026 && strcmp(currentList[i].name,"COG")==0 )
			||  ( currentPgn == 129026 && strcmp(currentList[i].name,"SOG")==0 )			

			// Wind Data
			||	( currentPgn == 130306 && strcmp(currentList[i].name,"Wind_Speed")==0 && strcmp(currentList[3].value,"True (ground referenced to North)")==0 )	
			||	( currentPgn == 130306 && strcmp(currentList[i].name,"Wind_Angle")==0 && strcmp(currentList[3].value,"True (ground referenced to North)")==0 )		
		){
			
			if (strcmp(currentList[i].value,"?")!=0) 
			{
				sprintf(tmpchar,"/tmp/u200/%s", currentList[i].name);
				fprintf(stdout,"  %s -> (%s)\n",tmpchar, currentList[i].value);

				// write to file		
				file = fopen(tmpchar,"w");
				fprintf(file,"%s",currentList[i].value);
				fclose(file);
			}
		}
	}
	fprintf(stdout,"\n");
}

