
/* common includes */

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <sys/time.h>
#include <math.h>
#include <time.h>

#include <termios.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <sys/wait.h>
#include <unistd.h>

#include <stdint.h>
#include <stdarg.h>
#include <inttypes.h>
#include <ctype.h>

#include "pgn_list.h"


/* Actisense NGT-1 message structure is:

   DLE STX <command> <len> [<data> ...]  <checksum> DLE ETX

   <command> is a byte from the list below.
   In <data> any DLE characters are double escaped (DLE DLE).
   <len> encodes the unescaped length.
   <checksum> is such that the sum of all unescaped data bytes plus the command
              byte plus the length adds up to zero, modulo 256.
*/

#define STX (0x02)  // Start packet 
#define ETX (0x03)  // End packet 
#define DLE (0x10)  // Start pto encode a STX or ETX send DLE+STX or DLE+ETX 
#define ESC (0x1B)  // Escape 

#define N2K_MSG_RECEIVED (0x93)  /* Receive standard N2K message */
#define N2K_MSG_SEND     (0x94)  /* Send N2K message */
#define NGT_MSG_RECEIVED (0xA0)  /* Receive NGT specific message */
#define NGT_MSG_SEND     (0xA1)  /* Send NGT message */

#define BUFFER_SIZE 900


/* function definitions */

#ifndef ARRAY_SIZE
# define ARRAY_SIZE(x) (sizeof(x) / sizeof(x[0]))
#endif
#ifndef min
# define min(x,y) ((x)<=(y)?(x):(y))
#endif
#ifndef max
# define max(x,y) ((x)>=(y)?(x):(y))
#endif


typedef struct
{
  char timestamp[30 + 1];
  uint8_t prio;
  uint32_t pgn;
  uint8_t dst;
  uint8_t src;
  uint8_t len;
  uint8_t data[FASTPACKET_MAX_SIZE];
} RawMessage;

typedef struct
{
  size_t lastFastPacket;
  size_t size;
  size_t allocSize;
  uint8_t * data;
} Packet;

typedef struct
{
  Packet packetList[ARRAY_SIZE(pgnList)];
} DevicePackets;

