/*

Decode NMEA2000 protocol into a readable text format.
A set of files is created in /tmp directory for each field of the
selected PGNs as interface for the sailboat-controller program.

Credits for decoding the NMEA2000 protocol from raw binary data
into readable text go to: Kees Verruijt, Harlingen, The Netherlands.
https://github.com/canboat/canboat.git

*/


/*
*     - scrittura file multipli
*			- inizializzare i file vuoti
*			- fondere in u200.c
*/

#include "../u200.h"

DevicePackets * device[256];
char * manufacturer[1 << 12];

size_t heapSize = 0;
int clockSrc = -1;

//static void fillManufacturers(void);
//static void fillFieldCounts(void);
void msgdec(char * msg);
static int scanHex(char ** p, uint8_t * m);
static uint8_t scanNibble(char c);
bool printCanFormat(RawMessage * msg);
void printPacket(size_t index, RawMessage * msg);
bool printPgn(int index, int subIndex, RawMessage * msg);
static void extractNumber(Field * field, uint8_t * data, size_t startBit, size_t bits, int64_t * value, int64_t * maxValue);
static bool printNumber(char * fieldName, Field * field, uint8_t * data, size_t startBit, size_t bits);
static bool printLatLon(char * name, double resolution, uint8_t * data, size_t bytes);

int main(int argc, char ** argv)
{
	//char msg[] = "2013-03-30-15:09:38.449,3,127257,1,255,8,e7,ff,7f,a2,01,e4,fa,ff";	
	char msg[2000];
	char ckpgn[6];

	FILE * file;
	file = fopen("sample.log", "r");
      if (!file)
      {
        printf("Cannot open file ");
        exit(1);
      }
	while (fgets(msg, sizeof(msg) - 1, file))
  {
			strncpy(ckpgn, msg+26, 6);
			
			if( strcmp(ckpgn,"127250")!=0 
					&& strcmp(ckpgn,"262386")!=0
					&& strcmp(ckpgn,"130312")!=0 			
					&& strcmp(ckpgn,"130313")!=0
					&& strcmp(ckpgn,"130314")!=0 						
			) msgdec(msg);
	}

  return 0;
}



void msgdec(char * msg) 
{
	int r;

	RawMessage m;
	//unsigned int prio, pgn, dst, src, len, junk;
	unsigned int prio, pgn, dst, src, len;
	unsigned int i;
	char * p;
	

	//fillManufacturers();
	//fillFieldCounts();


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

	if (r < 5)
		{
      fprintf(stdout, "Error reading message, scanned [%u] from [%s]", r, msg);
      return;
    }

	/* ?? */
	for (i = 0; *p && i < 5;)
      {
        if (*++p == ',')
        {
          i++;
        }
      }

	/* ?? */
	if (!p)
    {
      fprintf(stdout, "Error reading message, scanned [%zu] bytes from [%s]", p - msg, msg);
      return;
    }
  
	p++;

	for (i = 0; i < len; i++)
    {
      if (scanHex(&p, &m.data[i]))
      {
				fprintf(stdout,"--> ERR1\n");
        //logError("Error reading message, scanned %zu bytes from %s/%s, index %u", p - msg, msg, p, i);
        //if (!showJson) fprintf(stdout, "%s", msg);
        continue;
      }
      if (i < len)
      {
        if (*p != ',' && !isspace(*p))
        {
					fprintf(stdout,"--> ERR2\n");
          //logError("Error reading message, scanned %zu bytes from %s", p - msg, msg);
          //if (!showJson) fprintf(stdout, "%s", msg);
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

	printCanFormat(&m);

}

static int scanHex(char ** p, uint8_t * m)
{
  uint8_t hi, lo;

  if (!(*p)[0] || !(*p)[1])
  {
    return 1;
  }

  hi = scanNibble((*p)[0]);
  if (hi > 15)
  {
    return 1;
  }
  lo = scanNibble((*p)[1]);
  if (lo > 15)
  {
    return 1;
  }
  (*p) += 2;
  *m = hi << 4 | lo;
  /* printf("(b=%02X,p=%p) ", *m, *p); */
  return 0;
}

static uint8_t scanNibble(char c)
{
  if (isdigit(c))
  {
    return c - '0';
  }
  if (c >= 'A' && c <= 'F')
  {
    return c - 'A' + 10;
  }
  if (c >= 'a' && c <= 'f')
  {
    return c - 'a' + 10;
  }
  return 16;
}

bool printCanFormat(RawMessage * msg)
{
	

  size_t i;
/*
  if (onlySrc >=0 && onlySrc != msg->src)
  {
    return false;
  }
*/

 
  for (i = 0; i < ARRAY_SIZE(pgnList); i++)
  {
    if (msg->pgn == pgnList[i].pgn)
    {
      if (!pgnList[i].size)
      {
        return true; /* Determine size by raw packet first */
      }

      /* Found the pgn that matches this particular packet */
      printPacket(i, msg);
      return true;
    }
  }

  if (i == ARRAY_SIZE(pgnList))
  {
    printPacket(0, msg);
  }

  return false;
}



void printPacket(size_t index, RawMessage * msg)
{
	/* da questa funzione in poi richiamato tutto senza ottimizzazione !!*/
	// size_t fastPacketIndex;  //unsued
  // size_t bucket;						//unused
  Packet * packet;
  Pgn * pgn = &pgnList[index];
  size_t subIndex;

	// alloca memoria per ogni nuovo device che trovi
	if (!device[msg->src])
  {
    heapSize += sizeof(DevicePackets);
/*    if (showBytes)
    {
      logInfo("New device at address %u (heap %zu bytes)\n", msg->src, heapSize);
    }*/
    device[msg->src] = calloc(1, sizeof(DevicePackets));
    if (!device[msg->src])
    {
      /*die("Out of memory\n");*/
			fprintf(stdout, "Out of memory\n");
			exit(1);
    }
  }
  packet = &(device[msg->src]->packetList[index]);
	
	if (!packet->data)
		{
		  packet->allocSize = max(min(pgn->size, 8) + FASTPACKET_BUCKET_N_SIZE, msg->len);
		  heapSize += packet->allocSize;
		  /*logInfo("New PGN %u for device %u (heap %zu bytes)\n", pgn->pgn, msg->src, heapSize);*/
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
      /*logDebug("Resizing buffer for PGN %u device %u to accomodate %u bytes (heap %zu bytes)\n", pgn->pgn, msg->src, msg->len, heapSize);*/
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
  // uint16_t currentDate = UINT16_MAX; //unused
  // uint32_t currentTime = UINT32_MAX;	//unused
  char fieldName[60];
  bool r;
  bool matchedFixedField;
  //bool hasFixedField;
  uint32_t refPgn = 0;

  if (!device[msg->src])
  {
    return false;
  }


  dataStart = device[msg->src]->packetList[index].data;
  if (!dataStart)
  {
    return false;
  }
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
      if (!field.name || !field.size)
      {
        break;
      }

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
    if (matchedFixedField)
    {
      break;
    }
  }

  if ((index >= ARRAY_SIZE(pgnList)) || (msg->pgn != pgnList[index].pgn))
  {
    index = 0;
  }

  pgn = &pgnList[index];
/*
  if (showData)
  {
    FILE * f = stdout;
    char c = ' ';

    if (showJson)
    {
      f = stderr;
    }

    fprintf(f, "%s %u %3u %3u %6u %s: ", msg->timestamp, msg->prio, msg->src, msg->dst, msg->pgn, pgn->description);
    for (i = 0; i < size; i++)
    {
      fprintf(f, " %2.02X", dataStart[i]);
    }
    putc('\n', f);

    fprintf(f, "%s %u %3u %3u %6u %s: ", msg->timestamp, msg->prio, msg->src, msg->dst, msg->pgn, pgn->description);
    for (i = 0; i < size; i++)
    {
      fprintf(f, "  %c", isalnum(dataStart[i]) ? dataStart[i] : '.');
    }
    putc('\n', f);
  }
*/
/*
  if (showJson)
  {
    mprintf("{\"timestamp\":\"%s\",\"prio\":\"%u\",\"src\":\"%u\",\"dst\":\"%u\",\"pgn\":\"%u\",\"description\":\"%s\"", msg->timestamp, msg->prio, msg->src, msg->dst, msg->pgn, pgn->description);
    sep = ",\"fields\":{";
  }
  else
  {*/

/* HERE msg->pgn */
//    mprintf("%s %u %3u %3u %6u %s:", msg->timestamp, msg->prio, msg->src, msg->dst, msg->pgn, pgn->description);
//    sep = " ";
	fprintf(stdout,"\n----- PGN [%6u] -----\n", msg->pgn);


 /* } */

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
      else
      {
        break;
      }
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
          while (len > 0 && (data[len - 1] == lastbyte))
          {
            len--;
          }
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
        else
        {
          bytes = 1;
          len = 0;
        }
        if (len)
        {
						fprintf(stdout,"** DBG_04: STRING: FIELD [%s], DATA [%s]\n", fieldName, data);
        }
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
				fprintf(stdout,"DBG_08: TEMPERATURE [...]\n");
        //printTemperature(fieldName, valueu16);
      }
      else if (field.resolution == RES_PRESSURE)
      {
        memcpy((void *) &valueu16, data, 2);
				fprintf(stdout,"DBG_09: PRESSURE [...]\n");
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
			// IGNORA
			fprintf(stdout,"DBG!! IGNORE THIS\n\n");
      return false;
    }

		// CORRETTO, STAMPA IL RISULTATO DEL CAMPO PER IL CORRENTE PGN
		// fprintf(stdout,"DBG!! OK\n\n");

    startBit += bits;
    data += startBit / 8;
    startBit %= 8;


  }

 
  //mprintf("\n");
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

	if (maxValue >= 15)
  {
    notUsed = 2;
  }
  else if (maxValue > 1)
  {
    notUsed = 1;
  }
  else
  {
    notUsed = 0;
  }

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
      if (!s)
      {
        s = lookfor + 1;
      }
     
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
        
        // mprintf("%s %s = %.*s", getSep(), fieldName, (int) (e - s), s);
				fprintf(stdout,"B) [%s] : %.*s\n", fieldName,  (int) (e - s), s);
        
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

        if (field->resolution == RES_DEGREES)
        {
          precision = 1;
        }
        else if (field->resolution == RES_DEGREES * 0.0001)
        {
          precision = 4;
        }
        else
        {
          for (r = field->resolution; (r > 0.0) && (r < 1.0); r *= 10.0)
          {
            precision++;
          }
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
			fprintf(stdout,"   ???? \n");
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
		fprintf(stdout," RETURN FALSE **********\n");
    return false;
  }

  if (bytes == 8)
  {
    value /= INT64_C(1000000000);
  }
  absVal = (value < 0) ? -value : value;

 /*
  if (showGeo == GEO_DD)
  {
    double dd = (double) value / (double) RES_LAT_LONG_PRECISION;

    if (showJson)
    {
      mprintf("%s\"%s\":\"%010.7f\"", getSep(), name, dd);
    }
    else
    {
      mprintf("%s %s = %010.7f", getSep(), name, dd);
    }
  }
  else if (showGeo == GEO_DM)
  {
    // One degree = 10e6 

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
  }
  else
  {
*/
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

		

 /* } */
  return true;
}
