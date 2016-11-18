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

#ifndef _GEOFENCE_H
#define _GEOFENCE_H
#ifdef __cplusplus
extern "C" {
#endif

#include "gpstools.h"
#include "events.h"

// ----------------------------------------------------------------------------

// DEBUG: uncomment to print loaded geozones
#define GEOZ_INCL_PRINT_GEOZONE

// ----------------------------------------------------------------------------
// Other references:
//   http://mathforum.org/library/drmath/sets/select/dm_lat_long.html
//   http://www.faqs.org/faqs/geography/infosystems-faq/
// Google:"great circle" distance perpendicular

// ----------------------------------------------------------------------------
// Property GeoZone admin command types:

#define GEOF_CMD_ADD_STD            0x10
#define GEOF_CMD_ADD_HIGH           0x11
#define GEOF_CMD_REMOVE             0x20
#define GEOF_CMD_SAVE               0x30

// ----------------------------------------------------------------------------
// GeoFence types:

/* geofence types */
// client need not support all of these, however, it must generate an error if
// the server presents a format that the client does not support.
#define GEOF_DUAL_POINT_RADIUS      0       // 2 point/radius zones
#define GEOF_BOUNDED_RECT           1       // pt[0] is NorthWest, pt[1] is SouthEast
#define GEOF_SWEPT_POINT_RADIUS     2       // swept point/radius (point to point) (if implemented)
//#define GEOF_DELTA_RECT           3       // pt[0] is center, pt[1] is delta lat/lon

/* GeoZone ID definition */
// The size of the GeoZone ID may be either 'UInt32' or 'UInt16'.  Note however that
// this will effect the size of the internal GeoZone table.
typedef UInt16                      GeoZoneID_t;    // 2 bytes
//typedef UInt32                    GeoZoneID_t;    // 4 bytes
#define GeoZoneID_MASK              ((1L << (sizeof(GeoZoneID_t) * 8)) - 1L)

#define NO_ZONE                     ((GeoZoneID_t)0x0)

typedef struct
{
    // We use 'float' instead of 'double' to reduce the memory footprint.
    // (The increased resolution afforded by 'double' isn't needed here)
    float               latitude;   // 4 bytes
    float               longitude;  // 4 bytes
} GeoZonePoint_t;

typedef struct
{
    GeoZoneID_t         zoneID;     //  2 bytes (assuming GeoZoneID_t is UInt16)
    UInt16Pack          type:3;     // --   3 bits
    UInt16Pack          radius:13;  //  2  13 bits (max 8191 meters)
    GeoZonePoint_t      point[2];   // 16 bytes
} GeoZone_t;                        // 20 bytes

// ----------------------------------------------------------------------------

void geozInitialize(eventAddFtn_t queueEvent);

void geozCheckGPS(const GPS_t *oldFix, const GPS_t *newFix);

void geozSetVersion(const char *v);
const char *geozGetVersion();

GeoZoneID_t geozGetCurrentID();
void geozSetCurrentID(GeoZoneID_t zoneID);
GeoZone_t *geozInZone(const GPSPoint_t *newGP);

UInt16 geozGetGeoZoneCount();

// ----------------------------------------------------------------------------

utBool geozAddGeoZone(GeoZone_t *gz);
utBool geozRemoveGeoZone(GeoZoneID_t zoneID);
utBool geozSaveGeoZone();

// ----------------------------------------------------------------------------

#ifdef __cplusplus
}
#endif
#endif
