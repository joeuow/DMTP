// ----------------------------------------------------------------------------
// Copyright 2006-2007, Martin D. Flynn
// All rights reserved
// ----------------------------------------------------------------------------
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// 
// http://www.apache.org/licenses/LICENSE-2.0
// 
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// ----------------------------------------------------------------------------

#ifndef _GPSTOOLS_H
#define _GPSTOOLS_H
#ifdef __cplusplus
extern "C" {
#endif

#include "utctools.h"

// ----------------------------------------------------------------------------

/* standard constants */
/*#define PI                          (3.14159265358979323846) // math.h:M_PI */
#define PI		M_PI
#define TWO_PI		(PI * 2.0)
#define RADIANS		(PI / 180.0)
#define EPSILON		(1.0E-9)

#define POW2_24F	(1 << 24) // 2^24
#define POW2_31F	(LONG_MAX)       // 2^31
#define POW2_32F	(ULONG_MAX+1.0)       // 2^32 

/* distance conversions */
#define FEET_PER_MILE               (5280.0)
#define METERS_PER_FOOT             (0.3048)
#define FEET_PER_METER              (1.0 / METERS_PER_FOOT)
#define METERS_PER_MILE             (METERS_PER_FOOT * FEET_PER_MILE) // 1609.344
#define KILOMETERS_PER_KNOT         (1.85200000)
#define MILES_PER_KILOMETER         (0.621371192) // 0.621371192237334
#define KILOMETERS_TO_MILES(K)      ((K) * MILES_PER_KILOMETER)

/* average earth radius */
#define EARTH_EQUATORIAL_RADIUS_KM  (6378.1370) // Km: a
#define EARTH_POLOR_RADIUS_KM       (6356.7523) // Km: b
#define EARTH_RADIUS_KM             (6371.0088) // Km: (2a + b)/3 (average)
#define EARTH_RADIUS_METERS         (EARTH_RADIUS_KM * 1000.0) 
#define EARTH_RADIUS_MILES          (EARTH_RADIUS_METERS / METERS_PER_MILE) 

/* undefined GPS values */
#define GPS_UNDEFINED_LATITUDE      0.0
#define GPS_UNDEFINED_LONGITUDE     0.0
#define GPS_UNDEFINED_ACCURACY      0.0
#define GPS_UNDEFINED_MAG_VARIATION 0.0
#define GPS_UNDEFINED_GEOID_HEIGHT  0.0
#define GPS_UNDEFINED_DOP           0.0
#define GPS_UNDEFINED_SPEED         0.0
#define GPS_UNDEFINED_HEADING       0.0
#define GPS_UNDEFINED_ALTITUDE      0.0
#define GPS_UNDEFINED_DISTANCE      0.0
#define GPS_UNDEFINED_TEMPERATURE   0.0

// ----------------------------------------------------------------------------

typedef struct 
{
    double          latitude;
    double          longitude;
} GPSPoint_t;

// ----------------------------------------------------------------------------

#define NMEA0183_GPRMC              0x00000001L
#define NMEA0183_GPGGA              0x00000002L
#define NMEA0183_GPGSA              0x00000004L

typedef struct 
{
    // Note: 
    //  - Items added to this structure should also be copied to the event
    //    in the function "evSetEventGPS".
    //  - Ideally, 'systime' and 'fixtime' should be identical.  However, because
    //    this is unlikely (due to 'drift' in the system clock), 'systime' is used
    //    here to determine how recent a fix was based on comparison to the current
    //    system clock.
    // These items must be the same as GPSShort_t
    GPSPoint_t      point;      // <-- first entry (to allow casting to GPSPoint_t)
    time_t	fixtime;    // seconds (from gps record)
    // Other data follows
    TimerSec_t      ageTimer;   // age timer (not copied)
    float          accuracy;   // meters
    float          speedKPH;   // kph
    float          heading;    // degrees
    float          altitude;   // meters
    float          pdop;       // PDOP
    float          hdop;       // HDOP
    float          vdop;       // VDOP
    UInt32          fixtype;    // Quality [1=GPS, 2=DGPS]
    UInt32          nmea;       // NMEA-0183 record types in this sample
} GPS_t;

// ----------------------------------------------------------------------------

typedef struct 
{
    GPSPoint_t      point;      // <-- first entry
    UInt32          fixtime;    // seconds
} GPSShort_t;

typedef struct 
{
    GPSPoint_t      point;      // <-- first entry
    UInt32          fixtime;    // seconds
    UInt32          meters;     // odometer meters
} GPSOdometer_t;

// ----------------------------------------------------------------------------

// This structure must be able to be cast to a (UInt32[])
typedef struct
{
    UInt32          lastSampleTime;
    UInt32          lastValidTime;
    UInt32          sampleCount_A;
    UInt32          sampleCount_V;
    UInt32          restartCount;
} GPSDiagnostics_t;

// ----------------------------------------------------------------------------

void gpsPointClear(GPSPoint_t *gp);
GPSPoint_t *gpsPoint(GPSPoint_t *gp, double lat, double lon);
GPSPoint_t *gpsPointCopy(GPSPoint_t *d, const GPSPoint_t *s);
utBool gpsPointIsValid(const GPSPoint_t *gp);

GPS_t *gpsClear(GPS_t *gps);
GPS_t *gpsCopy(GPS_t *dest, const GPS_t *src);
utBool gpsIsValid(const GPS_t *gps);
void gpsInvalidate(GPS_t *gps);

// ----------------------------------------------------------------------------
// distance calcs

double gpsRadiansToPoint(const GPSPoint_t *gpS, const GPSPoint_t *gpE);
double gpsMetersToPoint(const GPSPoint_t *gpS, const GPSPoint_t *gpE);
double gpsKilometersToPoint(const GPSPoint_t *gpS, const GPSPoint_t *gpE);

// ----------------------------------------------------------------------------
// lat/lon encoding/decoding

UInt8 *gpsPointEncode6(UInt8 *buf, const GPSPoint_t *gps);
GPSPoint_t *gpsPointDecode6(GPSPoint_t *gps, const UInt8 *buf);

UInt8 *gpsPointEncode8(UInt8 *buf, const GPSPoint_t *gps);
GPSPoint_t *gpsPointDecode8(GPSPoint_t *gps, const UInt8 *buf);

// ----------------------------------------------------------------------------
// lat/lon string encoding

GPS_t *gpsParseString(GPS_t *gps, const char *s);
const char *gpsToString(GPS_t *gps, utBool all, char *s, int slen);

GPSOdometer_t *gpsOdomParseString(GPSOdometer_t *gps, const char *s);
const char *gpsOdomToString(GPSOdometer_t *gps, char *, int slen);

// ----------------------------------------------------------------------------

#ifdef __cplusplus
}
#endif
#endif
