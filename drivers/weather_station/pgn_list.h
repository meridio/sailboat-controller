/*

Analyzes NMEA 2000 PGNs.

Credits for reverse engineering NMEA2000 protocol go to
Kees Verruijt, Harlingen, The Netherlands.
https://github.com/canboat/canboat.git

The following is just the subset of PGN used by the
weather station AIRMAR WS-200WX.

*/

#define FASTPACKET_INDEX (0)
#define FASTPACKET_SIZE (1)
#define FASTPACKET_BUCKET_0_SIZE (6)
#define FASTPACKET_BUCKET_N_SIZE (7)
#define FASTPACKET_BUCKET_0_OFFSET (2)
#define FASTPACKET_BUCKET_N_OFFSET (1)
#define FASTPACKET_MAX_INDEX (0x1f)
#define FASTPACKET_MAX_SIZE (FASTPACKET_BUCKET_0_SIZE + FASTPACKET_BUCKET_N_SIZE * (FASTPACKET_MAX_INDEX - 1))

#define Pi (3.141592654)
#define RadianToDegree (360.0 / 2 / Pi)
#define BYTES(x) ((x)*(8))

#define LOOKUP_INDUSTRY_CODE ( \
						",4=Marine"\
						)
#define LOOKUP_DIRECTION_REFERENCE ( \
						",0=True"\
						",1=Magnetic"\
						)
#define LOOKUP_MAGNETIC_VARIATION ( \
            ",0=Manual" \
            ",1=Automatic Chart"\
            ",2=Automatic Table"\
            ",3=Automatic Calculation"\
            ",4=WMM 2000"\
            ",5=WMM 2005"\
            ",6=WMM 2010"\
            ",7=WMM 2015"\
            ",8=WMM 2020"\
            )
#define LOOKUP_GNS ( \
						",0=GPS"\
						",1=GLONASS"\
						",2=GPS+GLONASS"\
						",3=GPS+SBAS/WAAS"\
						",4=GPS+SBAS/WAAS+GLONASS"\
						",5=Chayka"\
						",6=integrated"\
						",7=surveyed"\
						",8=Galileo"\
						)
#define LOOKUP_GNS_METHOD ( \
						",0=no GNSS"\
						",1=GNSS fix"\
						",2=DGNSS fix"\
						",3=Precise GNSS"\
						",4=RTK Fixed Integer"\
						",5=RTK float"\
						",6=Estimated (DR) mode"\
						",7=Manual Input"\
						",8=Simulate mode"\
						)
#define LOOKUP_GNS_INTEGRITY ( \
						",0=No integrity checking"\
						",1=Safe"\
						",2=Caution"\
						)
#define LOOKUP_WIND_REFERENCE ( \
						",0=True (ground referenced to North)"\
						",1=Magnetic (ground referenced to Magnetic North)"\
						",2=Apparent"\
						",3=True (boat referenced)"\
						",4=True (water referenced)"\
						)


typedef struct
{
  char * name;
  uint32_t size;     /* Size in bits. All fields are contiguous in message; use 'reserved' fields to fill in empty bits. */
# define LEN_VARIABLE (0)
  double resolution; /* Either a positive real value or one of the following RES_ special values */
# define RES_NOTUSED (0)
# define RES_DEGREES (1e-4 * RadianToDegree)
# define RES_ROTATION (1e-3/32.0 * RadianToDegree)
# define RES_ASCII (-1.0)
# define RES_LATITUDE (-2.0)
# define RES_LONGITUDE (-3.0)
# define RES_DATE (-4.0)
# define RES_TIME (-5.0)
# define RES_TEMPERATURE (-6.0)
# define RES_6BITASCII (-7.0)            /* Actually not used in N2K, only in N183 AIS */
# define RES_INTEGER (-8.0)
# define RES_LOOKUP (-9.0)
# define RES_BINARY (-10.0)
# define RES_MANUFACTURER (-11.0)
# define RES_STRING (-12.0)
# define RES_FLOAT (-13.0)
# define RES_PRESSURE (-14.0)
# define RES_STRINGLZ (-15.0)            /* ASCII string starting with length byte and terminated by zero byte */
# define MAX_RESOLUTION_LOOKUP 15

  bool hasSign; /* Is the value signed, e.g. has both positive and negative values? */
  char * units; /* String containing the 'Dimension' (e.g. s, h, m/s, etc.) unless it starts with , in which
                 * case it contains a set of lookup values.
                 */
  char * description;
  int32_t offset; /* Only used for SAE J1939 values with sign; these are in Offset/Excess-K notation instead
                   * of two's complement as used by NMEA 2000.
                   * See http://en.wikipedia.org/wiki/Offset_binary
                   */
} Field;


typedef struct
{
  char     * description;
  uint32_t   pgn;
  bool       known;             /* Are we pretty sure we've got all fields with correct definitions? */
  uint32_t   size;              /* (Minimal) size of this PGN. Helps to determine fast/single frame and initial malloc */
  uint32_t   repeatingFields;   /* How many fields at the end repeat until the PGN is exhausted? */
  Field      fieldList[28];     /* Note fixed # of fields; increase if needed. RepeatingFields support means this is enough for now. */
  uint32_t   fieldCount;        /* Filled by C, no need to set in initializers. */
} Pgn;


Pgn pgnList[] =
{

{ "Unknown PGN", 0, false, 8, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, 0, "" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  , { 0 }
  }
}

/* 59392....... ISO Acknowledgement */
,
{ "ISO Acknowledgement", 59392, true, 8, 0,
  { { "Control", BYTES(1), RES_LOOKUP, false, ",0=ACK,1=NAK,2=Access Denied,3=Address Busy", "" }
  , { "Group Function", BYTES(1), 1, false, 0, "" }
  , { "Reserved", 24, 1, RES_BINARY, 0, "Reserved" }
  , { "PGN", 24, RES_INTEGER, false, 0, "Parameter Group Number of requested information" }
  , { 0 }
  }
}


/* 060928..... ISO Address Claim */
	/* not found */


/* 126208..... Acknowledge Group Function */
	/* http://www.maretron.com/support/manuals/DST100UM_1.2.pdf */
,
{ "NMEA - Request group function", 126208, true, 8, 2,
  { { "Function Code", BYTES(1), RES_INTEGER, false, "=0", "Request" }
  , { "PGN", BYTES(3), RES_INTEGER, false, 0, "Requested PGN" }
  , { "Transmission interval", BYTES(4), 1, false, 0, "" }
  , { "Transmission interval offset", BYTES(2), 1, false, 0, "" }
  , { "# of Requested Parameters", 8, 1, false, 0, "How many parameter pairs will follow" }
  , { "Parameter Index", 8, RES_INTEGER, false, 0, "Parameter index" }
  , { "Parameter Value", LEN_VARIABLE, RES_INTEGER, false, 0, "Parameter value, variable length" }
  , { 0 }
  }
}

,
{ "NMEA - Command group function", 126208, true, 8, 2,
  { { "Function Code", BYTES(1), RES_INTEGER, false, "=1", "Command" }
  , { "PGN", BYTES(3), RES_INTEGER, false, 0, "Commanded or requested PGN" }
  , { "Priority", 4, 1, false, 0, ",8=Leave priority unchanged" }
  , { "Reserved", 4, 1, false, 0, "" }
  , { "# of Commanded Parameters", BYTES(1), 1, false, 0, "How many parameter pairs will follow" }
  , { "Parameter Index", BYTES(1), RES_INTEGER, false, 0, "Parameter index" }
  , { "Parameter Value", LEN_VARIABLE, RES_INTEGER, false, 0, "Parameter value, variable length" }
  , { 0 }
  }
}

,
{ "NMEA - Acknowledge group function", 126208, true, 8, 1,
  { { "Function Code", BYTES(1), RES_INTEGER, false, "=2", "Acknowledge" }
  , { "PGN", 24, RES_INTEGER, false, 0, "Commanded or requested PGN" }
  , { "PGN error code", 4, 1, false, 0, "" }
  , { "Transmission interval/Priority error code", 4, 1, false, 0, "" }
  , { "# of Commanded Parameters", 8, 1, false, 0, "" }
  , { "Parameter Error", 8, RES_INTEGER, false, 0, "" }
  , { 0 }
  }
}

/* 126464..... PGN List */
,
{ "PGN List (Transmit and Receive)", 126464, false, 8, 1,
  { { "Function Code", BYTES(1), RES_LOOKUP, false, ",0=Transmit PGN list,1=Receive PGN list", "Transmit or receive PGN Group Function Code" }
  , { "PGN", 24, RES_INTEGER, false, 0, }
  , { 0 }
  }
}


/* 126992..... System Time */
	/* not found */

/* 126996..... Product Information */
	/* not found */

/* 126998..... Configuration Information */
,
{ "Configuration Information", 126998, false, 0x2a, 0,
  { { "Station ID", BYTES(2), 1, false, 0, "" }
  , { "Station Name", BYTES(2), 1, false, 0, "" }
  , { "A", BYTES(2), 1, false, 0, "" }
  , { "Manufacturer", BYTES(36), RES_ASCII, false, 0, "" }
  , { "Installation Description #1", BYTES(2), 1, false, 0, "" }
  , { "Installation Description #2", BYTES(2), 1, false, 0, "" }
  , { 0 }
  }
}


/* 127250..... Vessel Heading */
  /* NMEA + Simrad AT10 */
  /* http://www.maretron.com/support/manuals/SSC200UM_1.7.pdf */
  /* molly_rose_E80start.kees */
,
{ "Vessel Heading", 127250, true, 8, 0,
  { { "SID", BYTES(1), 1, false, 0, "" }
  , { "Heading", BYTES(2), RES_DEGREES, false, "deg", "" }
  , { "Deviation", BYTES(2), RES_DEGREES, true, "deg", "" }
  , { "Variation", BYTES(2), RES_DEGREES, true, "deg", "" }
  , { "Reference", 2, RES_LOOKUP, false, LOOKUP_DIRECTION_REFERENCE, "" }
  , { 0 }
  }
}


/* 127251..... Rate of Turn */
  /* http://www.maretron.com/support/manuals/SSC200UM_1.7.pdf */
  /* Lengths observed from Simrad RC42 */
,
{ "Rate of Turn", 127251, true, 5, 0,
  { { "SID", BYTES(1), 1, false, 0, "" }
  , { "Rate", BYTES(4), RES_ROTATION * 0.0001, true, "deg/s", "" }
  , { 0 }
  }
}


/* 127257..... Attitude */
,
{ "Attitude", 127257, true, 7, 0,
  { { "SID", BYTES(1), 1, false, 0, "" }
  , { "Yaw", BYTES(2), RES_ROTATION, true, "deg/s", "" }
  , { "Pitch", BYTES(2), RES_ROTATION, true, "deg/s", "" }
  , { "Roll", BYTES(2), RES_ROTATION, true, "deg/s", "" }
  , { 0 }
  }
}


/* 127258..... Magnetic Variation */
  /* NMEA + Simrad AT10 */
  /* http://www.maretron.com/support/manuals/GPS100UM_1.2.pdf */
,
{ "Magnetic Variation", 127258, true, 6, 0,
  { { "SID", BYTES(1), 1, false, 0, "" }
  , { "Source", 4, RES_LOOKUP, false, LOOKUP_MAGNETIC_VARIATION, "" }
  , { "Reserved", 4, RES_BINARY, false, 0, "Reserved" }
  , { "Age of service", BYTES(2), RES_DATE, false, "days", "Days since January 1, 1970" }
  , { "Variation", BYTES(2), RES_DEGREES, true, "deg", "" }
  , { 0 }
  }
}


/* 129025..... Position, Rapid Update */
,
{ "Position, Rapid Update", 129025, true, 8, 0,
  { { "Latitude", BYTES(4), RES_LATITUDE, true, "deg", "" }
  , { "Longitude", BYTES(4), RES_LONGITUDE, true, "deg", "" }
  , { 0 }
  }
}


/* 129026..... COG and SOG, Rapid Update */
  /* http://www.maretron.com/support/manuals/GPS100UM_1.2.pdf */
,
{ "COG & SOG, Rapid Update", 129026, true, 8, 0,
  { { "SID", BYTES(1), 1, false, 0, "" }
  , { "COG Reference", 2, RES_LOOKUP, false, LOOKUP_DIRECTION_REFERENCE, "" }
  , { "Reserved", 6, RES_BINARY, false, 0, "Reserved" }
  , { "COG", BYTES(2), RES_DEGREES, false, "deg", "" }
  , { "SOG", BYTES(2), 0.01, false, "m/s", "" }
  , { "Reserved", BYTES(2), RES_BINARY, false, 0, "Reserved" }
  , { 0 }
  }
}


/* 129029..... GNSS Position Data */
  /* http://www.maretron.com/support/manuals/GPS100UM_1.2.pdf */
,
{ "GNSS Position Data", 129029, true, 51, 3,
  { { "SID", BYTES(1), 1, false, 0, "" }
  , { "Date", BYTES(2), RES_DATE, false, "days", "Days since January 1, 1970" }
  , { "Time", BYTES(4), RES_TIME, false, "s", "Seconds since midnight" }
  , { "Latitude", BYTES(8), RES_LATITUDE, true, "deg", "" }
  , { "Longitude", BYTES(8), RES_LONGITUDE, true, "deg", "" }
  , { "Altitude", BYTES(8), 1e-6, true, "m", "" }
  , { "GNSS type", 4, RES_LOOKUP, false, LOOKUP_GNS, "" }
  , { "Method", 4, RES_LOOKUP, false, LOOKUP_GNS_METHOD, "" }
  , { "Integrity", 2, RES_LOOKUP, false, LOOKUP_GNS_INTEGRITY, "" }
  , { "Reserved", 6, RES_BINARY, false, 0, "Reserved" }
  , { "Number of SVs", BYTES(1), 1, false, 0, "Number of satellites used in solution" }
  , { "HDOP", BYTES(2), 0.01, true, 0, "Horizontal dilution of precision" }
  , { "PDOP", BYTES(2), 0.01, true, 0, "Probable dilution of precision" }
  , { "Geoidal Separation", BYTES(2), 0.01, false, "m", "Geoidal Separation" }
  , { "Reference Stations", BYTES(1), 1, false, 0, "Number of reference stations" }
  , { "Reference Station Type", 4, RES_LOOKUP, false, LOOKUP_GNS, "" }
  , { "Reference Station ID", 12, 1, false, "" }
  , { "Age of DGNSS Corrections", BYTES(2), 0.01, false, "s", "" }
  , { 0 }
  }
}


/* 129033..... Time and Date */
,
{ "Time & Date", 129033, true, 8, 0,
  { { "Date", BYTES(2), RES_DATE, false, "days", "Days since January 1, 1970" }
  , { "Time", BYTES(4), RES_TIME, false, "s", "Seconds since midnight" }
  , { "Local Offset", BYTES(2), RES_INTEGER, true, "minutes", "Minutes" }
  , { 0 }
  }
}


/* 129044..... Datum */
,
{ "Datum", 129044, true, 24, 0,
  { { "Local Datum", BYTES(4), RES_ASCII, false, 0, "defined in IHO Publication S-60, Appendices B and C."
              " First three chars are datum ID as per IHO tables."
              " Fourth char is local datum subdivision code." }
  , { "Delta Latitude", BYTES(4), RES_LATITUDE, true, "deg", "" }
  , { "Delta Longitude", BYTES(4), RES_LONGITUDE, true, "deg", "" }
  , { "Delta Altitude", BYTES(4), 1e-6, true, "m", "" }
  , { "Reference Datum", BYTES(4), RES_ASCII, false, 0, "defined in IHO Publication S-60, Appendices B and C."
              " First three chars are datum ID as per IHO tables."
              " Fourth char is local datum subdivision code." }
  , { 0 }
  }
}


/* 129538..... GNSS Control Status */
  /* http://www.maretron.com/support/manuals/GPS100UM_1.2.pdf */
  /* Haven't seen this yet (no way to send PGN 059904 yet) so lengths unknown */
,
{ "GNSS Control Status", 129538, false, 0, 0,
  { { "SV Elevation Mask", BYTES(2), 1, false, 0, "Will not use SV below this elevation" }
  , { "PDOP Mask", BYTES(2), 0.01, false, 0, "Will not report position above this PDOP" }
  , { "PDOP Switch", BYTES(2), 0.01, false, 0, "Will report 2D position above this PDOP" }
  , { "SNR Mask", BYTES(2), 0.01, false, 0, "Will not use SV below this SNR" }
  , { "GNSS Mode (desired)", 3, RES_LOOKUP, false, ",0=1D,1=2D,2=3D,3=Auto,4=Reserved,5=Reserved,6=Error", "" }
  , { "DGNSS Mode (desired)", 3, RES_LOOKUP, false, ",0=no SBAS,1=SBAS,3=SBAS", "" }
  , { "Position/Velocity Filter", 2, 1, false, 0, "" }
  , { "Max Correction Age", BYTES(2), 1, false, 0, "" }
  , { "Antenna Altitude for 2D Mode", BYTES(2), 0.01, false, "m", "" }
  , { "Use Antenna Altitude for 2D Mode", 2, RES_LOOKUP, false, ",0=use last 3D height,1=Use antenna altitude", "" }
  , { 0 }
  }
}


/* 129539..... GNSS DOPs */
  /* http://www.maretron.com/support/manuals/GPS100UM_1.2.pdf */
,
{ "GNSS DOPs", 129539, true, 8, 0,
  { { "SID", BYTES(1), 1, false, 0, "" }
  , { "Desired Mode", 3, RES_LOOKUP, false, ",0=1D,1=2D,2=3D,3=Auto,4=Reserved,5=Reserved,6=Error", "" }
  , { "Actual Mode", 3, RES_LOOKUP, false, ",0=1D,1=2D,2=3D,3=Auto,4=Reserved,5=Reserved,6=Error", "" }
  , { "Reserved", 2, RES_BINARY, false, 0, "Reserved" }
  , { "HDOP", BYTES(2), 0.01, true, 0, "Horizontal dilution of precision" }
  , { "VDOP", BYTES(2), 0.01, true, 0, "Vertical dilution of precision" }
  , { "TDOP", BYTES(2), 0.01, true, 0, "Time dilution of precision" }
  , { 0 }
  }
}


/* 129540..... GNSS Sats in View */
,
{ "GNSS Sats in View", 129540, true, 0xff, 7,
  { { "SID", BYTES(1), 1, false, 0, "" }
  , { "Mode", 2, RES_LOOKUP, false, ",3=Range residuals used to calculate position", "" }
  , { "Reserved", 6, RES_BINARY, false, 0, "Reserved" }
  , { "Sats in View", BYTES(1), 1, false, 0, "" }

  , { "PRN", BYTES(1), 1, false, 0, "" }
  , { "Elevation", BYTES(2), RES_DEGREES, true, "deg", "" }
  , { "Azimuth", BYTES(2), RES_DEGREES, true, "deg", "" }
  , { "SNR", BYTES(2), 0.01, false, "dB", "" }
  , { "Range residuals", BYTES(4), 1, true, 0, "" }
  , { "Status", 4, RES_LOOKUP, false, ",0=Not tracked,1=Tracked,2=Used,3=Not tracked+Diff,4=Tracked+Diff,5=Used+Diff", "" }
  , { "Reserved", 4, RES_BINARY, false, 0, "Reserved" }
  , { 0 }
  }
}


/* 130306..... Wind Data */
  /* http://askjackrabbit.typepad.com/ask_jack_rabbit/page/7/ */
,
{ "Wind Data", 130306, true, 6, 0,
  { { "SID", BYTES(1), 1, false, 0, "" }
  , { "Wind Speed", BYTES(2), 0.01, false, "m/s", "" }
  , { "Wind Angle", BYTES(2), RES_DEGREES, false, "deg", "" }
  , { "Reference", 3, RES_LOOKUP, false, LOOKUP_WIND_REFERENCE, "" }
  , { 0 }
  }
}


/* 130310..... Environmental Parameters */
  /* Water temperature, Transducer Measurement, Temperature, Atmospheric Pressure */
,
{ "Environmental Parameters", 130310, true, 7, 0,
  { { "SID", BYTES(1), 1, false, 0, "" }
  , { "Water Temperature", BYTES(2), RES_TEMPERATURE, false, "K", "" }
  , { "Outside Ambient Air Temperature", BYTES(2), RES_TEMPERATURE, false, "K", "" }
  , { "Atmospheric Pressure", BYTES(2), RES_PRESSURE, false, "hPa", "" }
  , { 0 }
  }
}


/* 130311..... Environmental Parameters */
	/* Humidity, Temperature, Atmospheric Pressure */
,
{ "Environmental Parameters", 130311, true, 8, 0,
  { { "SID", BYTES(1), 1, false, 0, "" }
  , { "Temperature Instance", 6, RES_LOOKUP, false, ",0=Sea,1=Outside,2=Inside,3=Engine room,4=Main Cabin", "" }
  , { "Humidity Instance", 2, RES_LOOKUP, false, ",0=Inside,1=Outside", "" }
  , { "Temperature", BYTES(2), RES_TEMPERATURE, false, "K", "" }
  , { "Humidity", BYTES(2), 100.0/25000, true, "%", "" }
  , { "Atmospheric Pressure", BYTES(2), RES_PRESSURE, false, "hPa", "" }
  , { 0 }
  }
}


/* 130323..... Meteorological Station Data */
,
{ "Meteorological Station Data", 130323, false, 0x1e, 0,
  { { "Mode", 4, 1, false, 0, "" }
  , { "Reserved", 4, RES_BINARY, false, 0, "" }
  , { "Measurement Date", BYTES(2), RES_DATE, false, "days", "Days since January 1, 1970" }
  , { "Measurement Time", BYTES(4), RES_TIME, false, "s", "Seconds since midnight" }
  , { "Station Latitude", BYTES(4), RES_LATITUDE, true, "deg", "" }
  , { "Station Longitude", BYTES(4), RES_LONGITUDE, true, "deg", "" }
  , { "Wind Speed", BYTES(2), 0.01, false, "m/s", "" }
  , { "Wind Direction", BYTES(2), RES_DEGREES, false, "deg", "" }
  , { "Wind Reference", 3, RES_LOOKUP, false, LOOKUP_WIND_REFERENCE, "" }
  , { "Reserved", 5, RES_BINARY, false, "", "reserved" }
  , { "Wind Gusts", BYTES(2), 0.01, false, "m/s", "" }
  , { "Atmospheric Pressure", BYTES(2), RES_PRESSURE, false, "hPa", "" }
  , { "Ambient Temperature", BYTES(2), RES_TEMPERATURE, false, "K", "" }
  , { "Station ID", BYTES(2), RES_STRING, false, 0, "" }
  , { "Station Name", BYTES(2), RES_STRING, false, 0, "" }
  , { 0 }
  }
}

}
;

