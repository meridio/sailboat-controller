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
	- reinserire il parsing dei parametri: DEBUG -d
	- significato di FD1_Ready? si può eliminare e utilizzare solo R?
	- ripristinare messaggio di errore quando manca parametro device
	- fare del code cleaning in u200.c per la parte di read
*/


#include "u200.h"


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
  if (isFile) 	// unused
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


  for (;;)
  {
    unsigned char msg[BUFFER_SIZE];
    enum ReadyDescriptor r;

		r = isready(handle, 0);

    if ((r & FD1_Ready) > 0)
    {
      if (!readNGT1(handle))
      {
				if (debug) fprintf(stdout, "DEBUG: loop break\n");
        break;
      }
    }

    if ((r & FD2_Ready) > 0)
    {
      if (debug) fprintf(stdout, "DEBUG: FD2 - %s \n", msg);
      fflush(stdout);
    }

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
  if (fd1 >= 0)
  {
    FD_SET(fd1, &fds);
  }
  if (fd2 >= 0)
  {
    FD_SET(fd2, &fds);
  }
  waitfor.tv_sec = timeout ? timeout : 10;
  waitfor.tv_usec = 0;
  if (fd1 > fd2)
  {
    setsize = fd1 + 1;
  }
  else
  {
    setsize = fd2 + 1;
  }
  r = select(setsize, &fds, 0, 0, &waitfor);
  if (r < 0)
  {
		fprintf(stderr,"I/O error; restart by quit\n");
  }
  if (r > 0)
  {
    if (fd1 >= 0 && FD_ISSET(fd1, &fds))
    {
      ret |= FD1_Ready;
    }
    if (fd2 >= 0 && FD_ISSET(fd2, &fds))
    {
      ret |= FD2_Ready;
    }
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
    if (cmd[i] == DLE)
    {
      *b++ = DLE;
    }
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
 * Read serial data from the NGT1, buffer=500?
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
    /* logAbort("Unable to read from NGT1 device\n"); */
	fprintf(stderr,"Unable to read from NGT1 device\n");
  }

  /*logDebug("Read %d bytes from device\n", (int) r);*/
  /* fprintf(stderr,"Read %d bytes from device\n", (int) r);*/
 /* if (debug) */
  if (0)
  {
    fprintf(stderr, "DEBUG read: ");
    for (i = 0; i < r; i++)
    {
      c = buf[i];
      fprintf(stderr, " %02X", c);
    }
    fprintf(stderr, "\n");
  }

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

  /* logDebug("received byte %02x state=%d offset=%d\n", c, state, head - buf);*/
  /* fprintf(stderr,"received byte %02x state=%d offset=%d\n", c, state, head - buf); */

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
    if (c == DLE)
    {
      state = MSG_ESCAPE;
    }
    else if (isFile && (c == ESC) && !noEscape)
    {
      state = MSG_ESCAPE;
    }
    else
    {
      *head++ = c;
    }
  }
  else
  {
    if (c == DLE)
    {
      state = MSG_ESCAPE;
    }
  }
}


/*
 * OGNI VOLTA CHE RICEVO UN BYTE DI TERMINAZIONE MESSAGGIO
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
    /*logError("Ignore short command len = %zu\n", msgLen);*/
	fprintf(stderr,"Ignore short command len = %zu\n", msgLen);
    return;
  }

  for (i = 0; i < msgLen; i++)
  {
    checksum += msg[i];
  }
  if (checksum)
  {
    /*logError("Ignoring message with invalid checksum\n");*/
	fprintf(stderr,"Ignoring message with invalid checksum\n");
    return;
  }

  command = msg[0];
  payloadLen = msg[1];

  /*logDebug("message command = %02x len = %u\n", command, payloadLen);*/
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
 * SE SI TRATTA DI UN MESSAGGIO SPECIFICO NGT-1
 */
static void ngtMessageReceived(const unsigned char * msg, size_t msgLen)
{
  size_t i;
  char line[1000];
  char * p;

  if (msgLen < 12)
  {
    /*logError("Ignore short msg len = %zu\n", msgLen);*/
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
 * SE SI TRATTA DI UNO DEI MESSAGGI NMEA2000
 */
static void n2kMessageReceived(const unsigned char * msg, size_t msgLen)
{
  unsigned int prio, src, dst;
  unsigned int pgn;
  size_t i;
  //unsigned int id;
  unsigned int len;
  //unsigned int data[8];
  char line[800];
  char * p;

  if (msgLen < 11)
  {
    /*logError("Ignoring N2K message - too short\n");*/
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
    /*logError("Ignoring N2K message - too long (%u)\n", len);*/
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

  /* scrive la stringa line su stdout */
  puts(line);
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

