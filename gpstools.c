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
// Description:
//  GPS tools 
//  Tools for managing GPS Latitude/Longitude and calculating distance.
// ---
// General References:
//  http://williams.best.vwh.net/avform.htm
//  http://www.boeing-727.com/Data/fly%20odds/distance.html    [excellent ref]
//  http://williams.best.vwh.net/intersect.htm
//  http://www.topcoder.com/tc?module=Static&d1=tutorials&d2=geometry1
//  http://www.topcoder.com/tc?module=Static&d1=tutorials&d2=geometry3#satellites
//  http://geometryalgorithms.com/Archive/algorithm_0102/algorithm_0102.htm#Distance%20to%20Ray%20or%20Segment
//  http://mathforum.org/library/drmath/sets/select/dm_lat_long.html
//  http://mathforum.org/library/drmath/view/51818.html
// Heading/Bearing:
//  http://mathforum.org/library/drmath/view/55417.html
//  http://williams.best.vwh.net/avform.htm
// Velocities:
//  http://www.xbow.com/Support/Support_pdf_files/NAV420AppNote.pdf
// ---
// Change History:
//  2006/01/04  Martin D. Flynn
//     -Initial release
//  2006/03/12  Martin D. Flynn
//     -Fixed bug where Longitude was not encoded/decoded properly in the Eastern Hemishpere.
//  2006/06/08  Martin D. Flynn
//     -Added 'pdop' to 'GPT_t'
//  2007/01/28  Martin D. Flynn
//     -WindowsCE port
//     -Added support for 'GPSOdometer_t'
// ----------------------------------------------------------------------------

#define SKIP_TRANSPORT_MEDIA_CHECK // only if TRANSPORT_MEDIA not used in this file 
#include "defaults.h"

#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include <limits.h>

#include "stdtypes.h"
#include "strtools.h"
#include "gpstools.h"
#include "utctools.h"

#include "log.h"

// ----------------------------------------------------------------------------

static double SQUARE(double X) { return X * X; }

#if defined(TARGET_WINCE)
#define IS_NAN(X)   _isnan(X)
#define SNPRINTF    _snprintf
#else
#define IS_NAN(X)   isnan(X)
#define SNPRINTF    snprintf
#endif

// ----------------------------------------------------------------------------

/* clear GPSPoint_t structure */
void gpsPointClear(GPSPoint_t *gp)
{
	gp->latitude = GPS_UNDEFINED_LATITUDE;
	gp->longitude = GPS_UNDEFINED_LONGITUDE;
}

/* init GPSPoint_t structure */
GPSPoint_t *gpsPoint(GPSPoint_t *gp, double lat, double lon)
{
	gp->latitude  = lat;
	gp->longitude = lon;
	return gp;
}

/* copy source point to destination structure */
GPSPoint_t *gpsPointCopy(GPSPoint_t *d, const GPSPoint_t *s)
{
    if (d && s) {
        memcpy(d, s, sizeof(GPSPoint_t));
        return d;
    } else {
        return (GPSPoint_t*)0;
    }
}

/* return true if the specified point is valid */
utBool gpsPointIsValid(const GPSPoint_t *gp)
{
    // return 'true' if this is a valid GPS point, 'false' otherwise
	if ((gp->latitude == 0.0) && (gp->longitude == 0.0)) {
		return utFalse;
	} else
	if ((gp->latitude >= 90.0) || (gp->latitude <= -90.0)) {
		return utFalse;
	} else
		return utTrue;
}

// ----------------------------------------------------------------------------

/* return the distance (in radian) between the specified points */
double gpsRadiansToPoint(
    const GPSPoint_t *gpS,            // From point
    const GPSPoint_t *gpE)            // To point
{
	/* validate both points */
	/*if ((gpS->latitude  >=  90.0) || (gpS->latitude  <=  -90.0) ||
	(gpS->longitude >= 180.0) || (gpS->longitude <= -180.0) ||
	(gpE->latitude  >=  90.0) || (gpE->latitude  <=  -90.0) ||
	(gpE->longitude >= 180.0) || (gpE->longitude <= -180.0)   ) {
	return 0.0;
	} */

	// Haversine formula (faster and less prone to rounding errors than 'Law of Cosines' algorithm
	// required math functions: sin, cos, atan2, sqrt
	double radLatS, radLonS, radLatE, radLonE, dlat, dlon, a, rad;
	radLatS = gpS->latitude  * RADIANS;  // Y
	radLonS = gpS->longitude * RADIANS;  // X
	radLatE = gpE->latitude  * RADIANS;  // Y
	radLonE = gpE->longitude * RADIANS;  // X
	dlat    = radLatE - radLatS;
	dlon    = radLonE - radLonS;
	a       = SQUARE(sin(dlat/2.0)) + (cos(radLatS) * cos(radLatE) * SQUARE(sin(dlon/2.0)));
	rad     = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
	return rad;
}

/* return the distance (in meters) between the specified points */
double gpsMetersToPoint(
    const GPSPoint_t *gpS,            // From point
    const GPSPoint_t *gpE)            // To point
{
	double rad = gpsRadiansToPoint(gpS, gpE);
	if (gpS->latitude == gpE->latitude && gpS->longitude == gpE->longitude)
		return 0.0;
	return rad * EARTH_RADIUS_METERS;
}

/* return the distance (in kilometers) between the specified points */
double gpsKilometersToPoint(
    const GPSPoint_t *gpS,            // From point
    const GPSPoint_t *gpE)            // To point
{
	double rad = gpsRadiansToPoint(gpS, gpE);
	if (gpS->latitude == gpE->latitude && gpS->longitude == gpE->longitude)
		return 0.0;
	return (rad * EARTH_RADIUS_METERS) / 1000.0;
}
// ----------------------------------------------------------------------------

/* copy source GPS structure to destination */
GPS_t *gpsCopy(GPS_t *dest, const GPS_t *src)
{
    if (dest) {
        if (src) {
            memcpy(dest, src, sizeof(GPS_t));
        } else {
            gpsClear(dest);
        }
        return dest;
    } else {
        return (GPS_t*)0;
    }
}

/* initialize GPS structure */
GPS_t *gpsClear(GPS_t *gps)
{
    if (gps) {
        memset(gps, 0, sizeof(GPS_t));
        // explicitly init all floating point values ...
        gps->point.latitude  = GPS_UNDEFINED_LATITUDE;
        gps->point.longitude = GPS_UNDEFINED_LONGITUDE;
        gps->accuracy        = GPS_UNDEFINED_ACCURACY;
        gps->speedKPH        = GPS_UNDEFINED_SPEED;
        gps->heading         = GPS_UNDEFINED_HEADING;
        gps->altitude        = GPS_UNDEFINED_ALTITUDE;
        gps->pdop            = GPS_UNDEFINED_DOP;
        gps->hdop            = GPS_UNDEFINED_DOP;
        gps->vdop            = GPS_UNDEFINED_DOP;
    }
    return gps;
}

/* return true if the specified GPS stgructure has been populated with a valid GPS fix */
utBool gpsIsValid(const GPS_t *gps)
{
	if (((gps->nmea & NMEA0183_GPRMC) != 0) && 
		((gps->nmea & NMEA0183_GPGGA) != 0) &&
		((gps->nmea & NMEA0183_GPGSA) != 0)) {
		return utTrue;
	} else {
		return utFalse;
	}
}

/* return true if the specified GPS stgructure has been populated with a valid GPS fix */
void gpsInvalidate(GPS_t *gps)
{
	gps->nmea &= ~(NMEA0183_GPRMC | NMEA0183_GPGGA | NMEA0183_GPGSA);
}

// ----------------------------------------------------------------------------

/* parse strng into GPS structure */
GPS_t *gpsParseString(GPS_t *gps, const char *s)
{
    // <fixtime>,<latitude>,<longitude>,<accuracy>,<speed>,<heading>,<altitude>
    if (!gps || !s) {
        
        // nothing to parse
        return (GPS_t*)0;
        
    } else {
        
        gpsClear(gps);
        int f;
        for (f = 0; f < 7; f++) { // 7 possible fields
            
            /* skip to field */
            while (*s && isspace(*s)) { s++; }
            if (!*s || (!isdigit(*s) && (*s != '-'))) { break; }
            
            /* parse field */
            switch (f) {
                case 0: gps->fixtime         =        strParseUInt32(s,      0L); break;
                case 1: gps->point.latitude  =        strParseDouble(s,     0.0); break;
                case 2: gps->point.longitude =        strParseDouble(s,     0.0); break;
                case 3: gps->accuracy        = (float)strParseDouble(s,     0.0); break;
                case 4: gps->speedKPH        = (float)strParseDouble(s,    -1.0); break;
                case 5: gps->heading         = (float)strParseDouble(s,    -1.0); break;
                case 6: gps->altitude        = (float)strParseDouble(s, -9999.0); break;
            }

            /* skip over comma */
            while (*s && (*s != ',')) { s++; }
            if (*s != ',') { break; }
            s++;

        }
        return gps;
        
    }
}

/* create a string from the specified GPS structure */
const char *gpsToString(GPS_t *gps, utBool all, char *s, int slen)
{
    // <fixtime>,<latitude>,<longitude>,<accuracy>,<speed>,<heading>,<altitude>
    if (s && (slen > 0)) {
        if (all) {
            SNPRINTF(s, slen, "%u,%.6lf,%.6lf,%.1f,%.1f,%.1f,%.1f", 
                (UInt32)gps->fixtime,
                gps->point.latitude, gps->point.longitude,
                gps->accuracy,
                gps->speedKPH,
                gps->heading,
                gps->altitude
                );
        } else {
            SNPRINTF(s, slen, "%u,%.6lf,%.6lf", 
                (UInt32)gps->fixtime,
                gps->point.latitude, gps->point.longitude
                );
        }
        return s;
    }
    return (char*)0;
}

// ----------------------------------------------------------------------------

/* parse GPSOdometer_t from string */
GPSOdometer_t *gpsOdomParseString(GPSOdometer_t *gps, const char *s)
{
    // <fixtime>,<latitude>,<longitude>,<meters>
    if (!gps || !s) {
        
        // nothing to parse
        return (GPSOdometer_t*)0;
        
    } else {
        
        memset(gps, 0, sizeof(GPSOdometer_t));
        gpsPointClear(&(gps->point));
        int f;
        for (f = 0; f < 4; f++) { // 4 possible fields
            
            /* skip to field */
            while (*s && isspace(*s)) { s++; }
            if (!*s || (!isdigit(*s) && (*s != '-'))) { break; }
            
            /* parse field */
            switch (f) {
                case 0: gps->fixtime         = strParseUInt32(s,  0L); break;
                case 1: gps->point.latitude  = strParseDouble(s, 0.0); break;
                case 2: gps->point.longitude = strParseDouble(s, 0.0); break;
                case 3: gps->meters          = strParseUInt32(s,  0L); break;
            }

            /* skip over comma */
            while (*s && (*s != ',')) { s++; }
            if (*s != ',') { break; }
            s++;

        }
        return gps;

    }
}

/* print GPSOdometer_t to string */
const char *gpsOdomToString(GPSOdometer_t *gps, char *s, int slen)
{
    // <fixtime>,<latitude>,<longitude>,<meters>
    if (s && (slen > 0)) {
        SNPRINTF(s, slen, "%u,%.6lf,%.6lf,%u", 
            gps->fixtime,
            gps->point.latitude, gps->point.longitude,
            gps->meters
            );
        return s;
    }
    return (char*)0;
}

// ----------------------------------------------------------------------------

/* encode GPS lat/lon to 6-byte buffer */
UInt8 *gpsPointEncode6(UInt8 *buf, const GPSPoint_t *gps)
{
    if (gpsPointIsValid(gps)) {
        int i;
        UInt32 rLat = (UInt32)((gps->latitude  -  90.0) * (POW2_24F / -180.0));
        UInt32 rLon = (UInt32)((gps->longitude + 180.0) * (POW2_24F /  360.0)); // BUGFIX_2006/03/12
        for (i = 2; i >= 0; i--) {
            buf[i    ] = (UInt8)(rLat & 0xFF);
            buf[i + 3] = (UInt8)(rLon & 0xFF);
            rLat >>= 8;
            rLon >>= 8;
        }
    } else {
        memset(buf, 0, 6);
    }
    return buf;
}

/* decode GPS lat/lon from 6-byte buffer */
GPSPoint_t *gpsPointDecode6(GPSPoint_t *gps, const UInt8 *buf)
{
    int i;
    UInt32 rLat = 0L, rLon = 0L;
    for (i = 0; i < 3; i++) {
        rLat = (rLat << 8) | buf[i    ];
        rLon = (rLon << 8) | buf[i + 3];
    }
    if (rLat || rLon) {
        gps->latitude  = ((double)rLat * (-180.0 / POW2_24F)) +  90.0;
        gps->longitude = ((double)rLon * ( 360.0 / POW2_24F)) - 180.0; // BUGFIX_2006/03/12
    } else {
        gps->latitude  = 0.0;
        gps->longitude = 0.0;
    }
    return gps;
}

// ----------------------------------------------------------------------------

/* encode GPS lat/lon to 8-byte buffer */
UInt8 *gpsPointEncode8(UInt8 *buf, const GPSPoint_t *gps)
{
    if (gpsPointIsValid(gps)) {
        int i;
        UInt32 rLat = (UInt32)((gps->latitude  -  90.0) * (POW2_32F / -180.0));
        UInt32 rLon = (UInt32)((gps->longitude + 180.0) * (POW2_32F /  360.0)); // BUGFIX_2006/03/12
        for (i = 3; i >= 0; i--) {
            buf[i    ] = (UInt8)(rLat & 0xFF);
            buf[i + 4] = (UInt8)(rLon & 0xFF);
            rLat >>= 8;
            rLon >>= 8;
        }
    } else {
        memset(buf, 0, 8);
    }
    return buf;
}

/* decode GPS lat/lon from 8-byte buffer */
GPSPoint_t *gpsPointDecode8(GPSPoint_t *gps, const UInt8 *buf)
{
    int i;
    UInt32 rLat = 0L, rLon = 0L;
    for (i = 0; i < 4; i++) {
        rLat = (rLat << 8) | buf[i    ];
        rLon = (rLon << 8) | buf[i + 4];
    }
    if (rLat || rLon) {
        gps->latitude  = ((double)rLat * (-180.0 / POW2_32F)) +  90.0;
        gps->longitude = ((double)rLon * ( 360.0 / POW2_32F)) - 180.0; // BUGFIX_2006/03/12
    } else {
        gps->latitude  = 0.0;
        gps->longitude = 0.0;
    }
    return gps;
}

// ----------------------------------------------------------------------------
