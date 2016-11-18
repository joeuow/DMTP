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
//  GPS rule module for checking for GeoZone arrival/departure
// ---
// Change History:
//  2006/05/07  Martin D. Flynn
//     -Initial release
//  2007/01/28  Martin D. Flynn
//     -WindowsCE port
//     -Receiving a 'GEOF_CMD_REMOVE' with no specific zone-id will remove
//      ALL existing geozones.  This is used for purposes of reloading the
//      entire geozone cache.
//     -Qualify 'GEOZONE_FILENAME' location to directory specified by the
//      'CONFIG_DIR' definition provided in 'defaults.h'
//  2007/04/28  Martin D. Flynn
//     -Changed to 'back-date' arrival/departure point to actual point of 
//      arrival/departure.
// ----------------------------------------------------------------------------
#if defined (ENABLE_GEOZONE)
#include "defaults.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "startup.h"
#include "log.h"

#include "stdtypes.h"
#include "gpstools.h"
#include "strtools.h"
#include "utctools.h"
#include "io.h"

#include "propman.h"
#include "statcode.h"
#include "events.h"

#include "geozone.h"

// ----------------------------------------------------------------------------

#define ARRIVE_PRIORITY             PRIORITY_NORMAL
#define DEPART_PRIORITY             PRIORITY_NORMAL

// ----------------------------------------------------------------------------

// where the geozone table will be saved
#define GEOZONE_FILENAME            (CONFIG_DIR_ "GEOZONE.DAT")

// ----------------------------------------------------------------------------

// pre-allocate the maximum number of possible geozones
#ifndef MAX_GEOZONES  // <-- a different value could be specified in 'defaults.h'
#  define MAX_GEOZONES              4000       // 80K bytes (@20 bytes/rcd)
#endif

// define to force all inserted ZoneIDs to be unique
//#define FORCE_UNIQUE_ZONE_IDS

// any ZoneID, other than 'NO_ZONE', is valid
#define IS_VALID_ZONE(Z)            ((Z) != NO_ZONE)

// true if arrival/departure should be set-back to actual arrival/departure point
#define SETBACK_POINT               (utTrue)

// ----------------------------------------------------------------------------

// This value represents the length of the data presented in the 'Add GeoZone' command
#define PACKED_GEOZONE_SIZE         (14 + sizeof(GeoZoneID_t))

// ----------------------------------------------------------------------------

#ifdef PROTOCOL_THREAD
#include "threads.h"
static threadMutex_t                geozMutex;
#define GEOZ_LOCK                   MUTEX_LOCK(&geozMutex);
#define GEOZ_UNLOCK                 MUTEX_UNLOCK(&geozMutex);
#else
#define GEOZ_LOCK
#define GEOZ_UNLOCK
#endif

// ----------------------------------------------------------------------------

static utBool           didInitialize   = utFalse;

static GeoZone_t        geoZoneList[MAX_GEOZONES];
static UInt16           maxZones        = MAX_GEOZONES;
static UInt16           usedZones       = 0;
static utBool           geozIsDirty     = utFalse;

static GPS_t            arrivePoint; // need initialization
static GPS_t            departPoint; // need initialization

static eventAddFtn_t    ftnQueueEvent   = 0;

// ----------------------------------------------------------------------------

/* add a geofence event to the event queue */
static void _queueGeofenceEvent(PacketPriority_t priority, StatusCode_t code, const GPS_t *gps, int geofID)
{
    if (gps && ftnQueueEvent) {
        Event_t evRcd;
        evSetEventDefaults(&evRcd, code, 0L, gps);
        evRcd.geofenceID[0] = geofID;
        (*ftnQueueEvent)(priority, DEFAULT_EVENT_FORMAT, &evRcd);
    }
}

// ----------------------------------------------------------------------------

void geozSetVersion(const char *v)
{
    propSetString(PROP_GEOF_VERSION, v);
}

const char *geozGetVersion()
{
    return propGetString(PROP_GEOF_VERSION, "");
}

// ----------------------------------------------------------------------------

/* check new GPS fix for various motion events */
void geozCheckGPS(const GPS_t *oldFix, const GPS_t *newFix)
{
    
    /* new fix required */
    if (!newFix) {
        //logDEBUG(LOGSRC,"No new fix! ...");
        return;
    }

    /* in GeoZone? */
    //logDEBUG(LOGSRC,"Zone checking in progress ...");
    GeoZone_t *inZone = geozInZone(&(newFix->point));
    //if (isDebugMode() && inZone) { logDEBUG(LOGSRC,"!!!!!! In Zone %d", inZone->zoneID); }
    
    /* get zones */
    GeoZoneID_t curZoneID = geozGetCurrentID();
    GeoZoneID_t newZoneID = inZone? inZone->zoneID : NO_ZONE;
    
    /* GeoZone changed? */
    // This zone-change-checking method only triggers an arrival/departure if
    // we're moving from inside the CURRENT zone to outside ALL zones,
    // or from outside ALL zones to inside ANY zone.
    // This allows creating unique ZoneIDs for overlapping sub-zones, and the 
    // Arrival event will then indicate which specific sub-zone was entered.
    utBool zoneChange = (IS_VALID_ZONE(curZoneID) != IS_VALID_ZONE(newZoneID))? utTrue : utFalse;
    // This zon-change-checking method only triggers an arrival/departure if
    // were moving from inside the CURRENT zone to outside the CURRENT zone,
    // or from outside ALL zones to inside ANY zone.
    // All overlapping sub-zones should have the same ZoneID.
    //utBool zoneChange = (curZoneID != newZoneID);
    
    /* check zone change */
    if (zoneChange) {
        // My 'Zone' state has changed
        //logDEBUG(LOGSRC,"GeoZone state changed: %u != %u", curZoneID, newZoneID);
        
        /* check departure */
        if (IS_VALID_ZONE(curZoneID)) {
            // I was in 'curZoneID', but now I am not (ie. departing 'curZoneID')
            if (!gpsIsValid(&departPoint)) {
                // This is the first 'departure' event
                gpsCopy(&departPoint, newFix); // a valid 'fixtime' is assumed
            }
            // check 'departure' delay
            UInt16 depDelay = (UInt16)propGetUInt32(PROP_GEOF_DEPART_DELAY, 0L);
            if ((depDelay == 0) || ((departPoint.fixtime + (UInt32)depDelay) <= utcGetTimeSec())) {
                // I've now departed the zone
                const GPS_t *departFix = SETBACK_POINT? &departPoint : newFix;
                _queueGeofenceEvent(DEPART_PRIORITY, STATUS_GEOFENCE_DEPART, departFix, curZoneID);
                geozSetCurrentID(NO_ZONE);
                gpsClear(&departPoint);
                logINFO(LOGSRC,"Departed %u [%u]\n", curZoneID, geozGetCurrentID());
                startupSaveProperties();
            } else {
                // not yet ready to mark as 'departed'
                //logDEBUG(LOGSRC,"Depart in %lu seconds", ((departPoint.fixtime + (UInt32)depDelay) - utcGetTimeSec()));
            }
        } else {
            // I wasn't in any known zone (I'm not departing)
            gpsClear(&departPoint);
        }
        
        /* check arrival */
        if (IS_VALID_ZONE(newZoneID)) {
            // I was not in 'newZoneID' before, but now I am (arriving 'newZoneID')
            if (!gpsIsValid(&arrivePoint)) {
                // This is the first 'arrival' event
                gpsCopy(&arrivePoint, newFix); // a valid 'fixtime' is assumed
            }
            // check 'arrival' delay
            UInt16 arrDelay = (UInt16)propGetUInt32(PROP_GEOF_ARRIVE_DELAY, 0L);
            if ((arrDelay == 0) || ((arrivePoint.fixtime + (UInt32)arrDelay) <= utcGetTimeSec())) {
                // I've now arrived in the zone
                const GPS_t *arriveFix = SETBACK_POINT? &arrivePoint : newFix;
                geozSetCurrentID(newZoneID);
                _queueGeofenceEvent(ARRIVE_PRIORITY, STATUS_GEOFENCE_ARRIVE, arriveFix, newZoneID);
                gpsClear(&arrivePoint);
                logINFO(LOGSRC,"Arrived %u [%u]\n", newZoneID, geozGetCurrentID());
                startupSaveProperties();
            } else {
                // not yet ready to mark as 'arrived'
                //logDEBUG(LOGSRC,"Arrive in %lu seconds", ((arrivePoint.fixtime + (UInt32)arrDelay) - utcGetTimeSec()));
            }
        } else {
            // I'm not arriving at any new zone
            gpsClear(&arrivePoint);
        }
        
    } else {
        // My 'Zone' state has not changed
        // (ie. If I'm 'out', I'm still 'out'.  If I'm 'in', I'm still 'in'.)
        //logDEBUG(LOGSRC,"GeoZone state NOT changed: %2u / %2u\n", curZoneID, newZoneID);
        gpsClear(&departPoint);
        gpsClear(&arrivePoint);
    }

}

// ----------------------------------------------------------------------------

/* return current geozone id */
GeoZoneID_t geozGetCurrentID()
{
    return (GeoZoneID_t)propGetUInt32(PROP_GEOF_CURRENT, (UInt32)NO_ZONE);
}

/* set current geozone id */
void geozSetCurrentID(GeoZoneID_t zoneID)
{
    propSetUInt32(PROP_GEOF_CURRENT, (UInt32)zoneID);
}

/* convert GeoZone point to GPS point */
static GPSPoint_t *_geozToGPSPoint(GPSPoint_t *gp, const GeoZonePoint_t *gzp)
{
    if (gzp && gp) {
        gp->latitude  = gzp->latitude;
        gp->longitude = gzp->longitude;
        return gp;
    } else {
        return (GPSPoint_t*)0;
    }
}

/* check new GPS fix for various motion events */
static utBool _geozInZone(GeoZone_t *geoz, const GPSPoint_t *newGP)
{
    utBool inZone = utFalse;
    if (geoz && newGP && IS_VALID_ZONE(geoz->zoneID)) {
        
        //logDEBUG(LOGSRC,"Target point: %.5lf / %.5lf\n", newGP->latitude, newGP->longitude);
        GPSPoint_t geozGP_0, geozGP_1;
        double deltaMeters, radiusMeters = (double)geoz->radius;
        switch (geoz->type) {

#ifdef GEOF_SWEPT_POINT_RADIUS
            case GEOF_SWEPT_POINT_RADIUS:
                // not supported in this implementation.
                // default to dual point/radius below
#endif
                
            case GEOF_DUAL_POINT_RADIUS:
                _geozToGPSPoint(&geozGP_0, &(geoz->point[0]));
                if (gpsPointIsValid(&geozGP_0)) {
                    deltaMeters = gpsMetersToPoint(newGP, &geozGP_0);
                    //logDEBUG(LOGSRC,"  Zone %d A (%.5lf / %.5lf) [%.1lf <= %.1lf]", geoz->zoneID, geozGP_0.latitude, geozGP_0.longitude, deltaMeters, radiusMeters);
                    if (deltaMeters <= radiusMeters) {
                        inZone = utTrue;
                        break;
                    }
                }
                _geozToGPSPoint(&geozGP_1, &(geoz->point[1]));
                if (gpsPointIsValid(&geozGP_1)) {
                    deltaMeters = gpsMetersToPoint(newGP, &geozGP_1);
                    //logDEBUG(LOGSRC,"  Zone %d B (%.5lf / %.5lf) [%.1lf <= %.1lf]", geoz->zoneID, geozGP_1.latitude, geozGP_1.longitude, deltaMeters, radiusMeters);
                    if (deltaMeters <= radiusMeters) {
                        inZone = utTrue;
                        break;
                    }
                }
                break;
                
            case GEOF_BOUNDED_RECT:
                // North of the top latitude
                if (newGP->latitude > (double)geoz->point[0].latitude) { 
                    break;
                }
                // South of the bottom latitude
                if (newGP->latitude < (double)geoz->point[1].latitude) { 
                    break;
                }
                // West of the left longitude (will fail if zone spans +/- 180 degrees)
                if (newGP->longitude < (double)geoz->point[0].longitude) { 
                    break;
                }
                // East of the right longitude (will fail if zone spans +/- 180 degrees)
                if (newGP->longitude > (double)geoz->point[1].longitude) { 
                    break;
                }
                inZone = utTrue;
                break;
                
#ifdef GEOF_DELTA_RECT
            case GEOF_DELTA_RECT:
                // North of the top latitude
                geozGP_0.latitude = (double)(geoz->point[0].latitude + geoz->point[1].latitude); // top
                if (newGP->latitude > geozGP_0.latitude) { 
                    break;
                }
                // South of the bottom latitude
                geozGP_0.latitude = (double)(geoz->point[0].latitude - geoz->point[1].latitude); // bottom
                if (newGP->latitude < geozGP_0.latitude) { 
                    break;
                }
                // West of the left longitude (will fail if zone spans +/- 180 degrees)
                geozGP_0.longitude = (double)(geoz->point[0].longitude - geoz->point[1].longitude); // left
                if (newGP->longitude < geozGP_0.longitude) { 
                    break;
                }
                // East of the right longitude (will fail if zone spans +/- 180 degrees)
                geozGP_0.longitude = (double)(geoz->point[0].longitude + geoz->point[1].longitude); // right
                if (newGP->longitude > geozGP_0.longitude) { 
                    break;
                }
                inZone = utTrue;
                break;
#endif

        } // switch (geoz->type)
        
    } // if (...)
    return inZone;
}

/* return the zone where the specified point is located */
GeoZone_t *geozInZone(const GPSPoint_t *newGP)
{
    /* is newGP inside GeoZone? */
    GeoZone_t *gz = (GeoZone_t*)0;
    UInt16 i;
    GEOZ_LOCK {
        for (i = 0; i < usedZones; i++) {
            if (_geozInZone(&geoZoneList[i], newGP)) {
                gz = &geoZoneList[i];
                break;
            }
        }
    } GEOZ_UNLOCK
    return gz;
}

// ----------------------------------------------------------------------------

static void _geozClearAll()
{
    memset(geoZoneList, sizeof(geoZoneList), 0);
    geozIsDirty = (usedZones > 0)? utTrue : utFalse;
    usedZones = 0;
}

static GeoZone_t *_geozDecodeGeoZone(Buffer_t *src, GeoZone_t *gz, utBool hiRes)
{
    // this method decodes a GeoZone as presented in the packet.
    UInt32      zoneID;
    UInt32      typeRadius;
    GPSPoint_t  pt0, pt1;
    int         fldCnt;
    if (hiRes) {
        // High resolution point
        fldCnt = binBufScanf(src, "%4u%2u%8g%8g", &zoneID, &typeRadius, &pt0, &pt1);
    } else {
        // Standard resolution point
        fldCnt = binBufScanf(src, "%2u%2u%6g%6g", &zoneID, &typeRadius, &pt0, &pt1);
    }
    memset(gz, 0, sizeof(GeoZone_t));
    if (fldCnt == 4) {
        gz->zoneID = (GeoZoneID_t)zoneID;
        gz->type   = (typeRadius >> 13) & 0x7;
        gz->radius = typeRadius & 0x1FFF;
        gz->point[0].latitude  = (float)pt0.latitude;
        gz->point[0].longitude = (float)pt0.longitude;
        gz->point[1].latitude  = (float)pt1.latitude;
        gz->point[1].longitude = (float)pt1.longitude;
        return gz;
    } else {
        return (GeoZone_t*)0;
    }
}

/*
#ifdef GEOZ_INCL_PRINT_GEOZONE
static void _geozPrintGeoZone(GeoZone_t *gz)
{
    if (gz) {
        logDEBUG(LOGSRC,"GeoZone: %u (typ=%u, rad=%u) 0=%.4f/%.4f, 1=%.4f/%.4f\n", 
            gz->zoneID, gz->type, gz->radius, 
            gz->point[0].latitude, gz->point[0].longitude, 
            gz->point[1].latitude, gz->point[1].longitude);
    }
}
#endif
*/

// ----------------------------------------------------------------------------

static CommandError_t _geozAddGeoZone(GeoZone_t *gz)
{
#ifdef GEOZ_INCL_PRINT_GEOZONE
    //_geozPrintGeoZone(gz);
#endif

    /* valid zone ID? */
    if (!IS_VALID_ZONE(gz->zoneID)) { // NO_ZONE
        // '0' is reserved for 'No GeoZone'
        return COMMAND_ZONE_ID;
    }

    /* validate radius */
    if (gz->radius <= 0) {
        // the radius must be > 0
        return COMMAND_RADIUS;
    }

    /* validate type */
    GPSPoint_t pt0, pt1;
    _geozToGPSPoint(&pt0, &(gz->point[0]));
    _geozToGPSPoint(&pt1, &(gz->point[1]));
#ifdef GEOF_DUAL_POINT_RADIUS
    // Two separate points with the same specified radius
    if (gz->type == GEOF_DUAL_POINT_RADIUS) {
        if (!gpsPointIsValid(&pt0)) {
            if (gpsPointIsValid(&pt1)) {
                // move the valid point into point[0]
                gz->point[0].latitude  = gz->point[1].latitude;
                gz->point[0].longitude = gz->point[1].longitude;
                gz->point[1].latitude  = 0.0;
                gz->point[1].longitude = 0.0;
            } else {
                // at least one point must be valid
                return COMMAND_LATLON;
            }
        }
    } else
#endif
#ifdef GEOF_BOUNDED_RECT
    // A simple rectangle bounded by an North-West point, and a South-East point
    if (gz->type == GEOF_BOUNDED_RECT) {
        // This GeoZone type will fail if the points spans the +/-180 deg meridian
        if (!gpsPointIsValid(&pt0) || !gpsPointIsValid(&pt1)) {
            // both points must be valid
            return COMMAND_LATLON;
        }
        if (gz->point[0].latitude < gz->point[1].latitude) {
            // move larger latitude to point[0]
            float tmp = gz->point[0].latitude;
            gz->point[0].latitude = gz->point[1].latitude;
            gz->point[1].latitude = tmp;
        }
        if (gz->point[0].longitude > gz->point[1].longitude) {
            // move smaller longitude to point[0]
            float tmp = gz->point[0].longitude;
            gz->point[0].longitude = gz->point[1].longitude;
            gz->point[1].longitude = tmp;
        }
    } else
#endif
#ifdef GEOF_SWEPT_POINT_RADIUS
    // A point radius swept over the globe from one point to another
    if (gz->type == GEOF_SWEPT_POINT_RADIUS) {
        if (!gpsPointIsValid(&pt0) || !gpsPointIsValid(&pt0)) {
            // both points must be valid
            return COMMAND_LATLON;
        }
        // The points should be a reasonable distance apart (ie. no more than a few miles)
    } else
#endif
#ifdef GEOF_DELTA_RECT
    // A center point with a +/- delta offset
    if (gz->type == GEOF_DELTA_RECT) {
        // This GeoZone type will fail if the points spans the +/-180 deg meridian
        if (!gpsPointIsValid(&pt0)) {
            // the center point must be valid
            return COMMAND_LATLON;
        }
        if (gz->point[1].latitude == 0.0) {
            // The delta latitude must be non zero
            //gz->point[1].latitude = 0.0200;
            return COMMAND_LATLON;
        }
        if (gz->point[1].longitude == 0.0) {
            // The delta latitude must be non zero
            //gz->point[1].longitude = 0.0200;
            return COMMAND_LATLON;
        }
        if (gz->point[1].latitude < 0.0) {
            // The delta latitude must be positive
            gz->point[1].latitude = -gz->point[1].latitude;
        }
        if (gz->point[1].longitude < 0.0) {
            // The delta latitude must be positive
            gz->point[1].longitude = -gz->point[1].longitude;
        }
    } else
#endif
    {
        // type not supported
        return COMMAND_TYPE;
    }
    
    /* unique ZoneIDs? */
#ifdef FORCE_UNIQUE_ZONE_IDS
    _geozRemoveGeoZone(gz->zoneID);
#endif

    /* get available insert point */
    UInt16 zoneNdx = 0;
    for (zoneNdx = 0; zoneNdx < usedZones; zoneNdx++) {
        if (!IS_VALID_ZONE(geoZoneList[zoneNdx].zoneID)) { // NO_ZONE
            break;
        }
    }
    if (zoneNdx >= maxZones) {
        // we've reached the maximum limit
        return COMMAND_OVERFLOW;
    }
    if (usedZones <= zoneNdx) {
        // we're allocating another unused zone
        usedZones = zoneNdx + 1;
    }

    /* add new geoZone */
    memcpy(&geoZoneList[zoneNdx], gz, sizeof(GeoZone_t));
    geozIsDirty = utTrue;
    return COMMAND_OK;
    
}

utBool geozAddGeoZone(GeoZone_t *gz)
{
    if (gz) {
        utBool addErr = utFalse;
        GEOZ_LOCK {
            addErr = (_geozAddGeoZone(gz) == COMMAND_OK)? utTrue : utFalse;
        } GEOZ_UNLOCK
        return addErr;
    } else {
        return utFalse;
    }
}

// ----------------------------------------------------------------------------

static utBool _geozRemoveGeoZone(GeoZoneID_t zoneID)
{

    /* remove all GeoZones */
    if (zoneID == NO_ZONE) {
        if (usedZones != 0) {
            usedZones = 0;
            geozIsDirty = utTrue;
            return utTrue;
        } else {
            return utFalse;
        }
    }
    
    /* remove all matching geoZones */
    utBool rtn = utFalse;
    UInt16 i;
    for (i = 0; i < usedZones; i++) {
        if (zoneID == geoZoneList[i].zoneID) {
            geoZoneList[i].zoneID = NO_ZONE;
            geozIsDirty = utTrue;
            rtn = utTrue;
        }
    }
    
    /* trim back invalid zones at end of list */
    for (; (usedZones > 0) && !IS_VALID_ZONE(geoZoneList[usedZones-1].zoneID); usedZones--);
    
    /* clear current zone, if this zone was deleted */
    GeoZoneID_t curZoneID = geozGetCurrentID();
    if (zoneID == curZoneID) {
        geozSetCurrentID(NO_ZONE);
    }
    
    return rtn;
}

utBool geozRemoveGeoZone(GeoZoneID_t zoneID)
{
    utBool rmvErr = utFalse;
    GEOZ_LOCK {
        rmvErr = _geozRemoveGeoZone(zoneID);
    } GEOZ_UNLOCK
    return rmvErr;
}

// ----------------------------------------------------------------------------

UInt16 geozGetGeoZoneCount()
{
    UInt16 i, count = 0;
    for (i = 0; i < usedZones; i++) {
        if (geoZoneList[i].zoneID != NO_ZONE) {
            count++;
        }
    }
    return count;
}

// ----------------------------------------------------------------------------

static utBool _geozSaveGeoZones(const char *geozName)
{
    
    /* invalid file name? */
    if (!geozName || !*geozName) {
        return utFalse;
    }
   
    /* file name */
    char geozFile[256];
    sprintf(geozFile, "%s", geozName);
    
    /* open file for writing */
    FILE *file = ioOpenStream(geozFile, IO_OPEN_WRITE);
    if (!file) {
        // error openning
        logERROR(LOGSRC,"Unable to open GeoZone file for writing: %s", geozFile);
        return utFalse;
    }

    /* write geozones to file */
    UInt16 i = 0, count = 0;
    for (i = 0; i < usedZones; i++) {
        if (geoZoneList[i].zoneID != NO_ZONE) {
            long wrtLen = ioWriteStream(file, (UInt8*)&geoZoneList[i], sizeof(GeoZone_t));
            if (wrtLen < sizeof(GeoZone_t)) {
                logERROR(LOGSRC,"Unable to write GeoZone: [%ld] %s", i, geozFile);
                ioCloseStream(file);
                return utFalse;
            }
            count++;
        }
    }
    
    /* close file */
    logINFO(LOGSRC,"Saved GeoZone file: %s [%u]", geozFile, count);
    ioCloseStream(file);
    geozIsDirty = utFalse;
    return utTrue;

}

utBool geozSaveGeoZone()
{
    utBool savErr = utFalse;
    GEOZ_LOCK {
        savErr = _geozSaveGeoZones(GEOZONE_FILENAME);
    } GEOZ_UNLOCK
    return savErr;
}

// ----------------------------------------------------------------------------

static utBool _geozLoadGeoZones(const char *geozName)
{
    
    /* reset terminals */
    _geozClearAll();
    geozIsDirty = utFalse;

    /* file name */
    char geozFile[256];
    sprintf(geozFile, "%s", geozName);
    
    /* make sure file exists */
    if (!ioIsFile(geozFile)) {
        logINFO(LOGSRC,"GeoZone file does not exist: %s", geozFile);
        return utFalse;
    }

    /* open file for writing */
    FILE *file = ioOpenStream(geozFile, IO_OPEN_READ);
    if (!file) {
        // error openning
        logERROR(LOGSRC,"Unable to open GeoZone file for reading: %s", geozFile);
        return utFalse;
    }
    
    /* read terminals */
    logINFO(LOGSRC,"Loading GeoZones: %s", geozFile);
    usedZones = 0;
    for (; usedZones < maxZones; usedZones++) {
        GeoZone_t *geoz = &geoZoneList[usedZones];
        /* read GeoZone record */
        long vlen = ioReadStream(file, geoz, sizeof(GeoZone_t));
        if (vlen < 0L) {
            logERROR(LOGSRC,"Error reading GeoZone: [rcd=%u] %s", usedZones, geozFile);
            ioCloseStream(file);
            return utFalse; // error
        } else
        if (vlen == 0L) {
            logINFO(LOGSRC,"Loaded GeoZones: [cnt=%u] %s", usedZones, geozFile);
            ioCloseStream(file);
            break;
        } else
        if (vlen < sizeof(GeoZone_t)) {
            logERROR(LOGSRC,"Unable to read GeoZone: [rcd=%u] %s", usedZones, geozFile);
            ioCloseStream(file);
            return utFalse; // error
        }
#ifdef GEOZ_INCL_PRINT_GEOZONE
        //_geozPrintGeoZone(geoz);
#endif
    }
    
    return utTrue;
}

// ----------------------------------------------------------------------------
// PROP_CMD_GEOF_ADMIN property handler

CommandError_t _cmdGeoZoneAdmin(int pi, Key_t key, const UInt8 *data, int dataLen)
{
    
    /* init buffer */
    Buffer_t bsrc, *src = binBuffer(&bsrc, (UInt8*)data, dataLen, BUFFER_SOURCE);
    if (BUFFER_DATA_LENGTH(src) < 1) {
        return COMMAND_ARGUMENTS;
    }
    
    /* admin command type */
    Int32 adminType = -1L;
    binBufScanf(src, "%1u", &adminType);
    CommandError_t cmdErr = COMMAND_FEATURE_NOT_SUPPORTED;
    switch ((Int16)adminType) {
        case GEOF_CMD_ADD_STD:  // Add GeoZone list to table (standard resolution)
        case GEOF_CMD_ADD_HIGH: // Add GeoZone list to table (high resolution)
            cmdErr = COMMAND_OK;
            GEOZ_LOCK {
                utBool hiRes = ((Int16)adminType == GEOF_CMD_ADD_HIGH)? utTrue : utFalse;
                while (BUFFER_DATA_LENGTH(src) >= PACKED_GEOZONE_SIZE) {
                    GeoZone_t gz;
                    if (_geozDecodeGeoZone(src, &gz, hiRes)) {
                        CommandError_t addErr = _geozAddGeoZone(&gz);
                        if (addErr != COMMAND_OK) {
                            cmdErr = addErr;
                        }
                    } else {
                        cmdErr = COMMAND_ARGUMENTS;
                    }
                }
                if (BUFFER_DATA_LENGTH(src) > 0) {
                    cmdErr = COMMAND_OVERFLOW;
                }
            } GEOZ_UNLOCK
            break;
        case GEOF_CMD_REMOVE: // Remove specified GeoZone(terminal) ID from table
            cmdErr = COMMAND_OK;
            GEOZ_LOCK {
                if (BUFFER_DATA_LENGTH(src) == 0) {
                    // this will remove ALL Geozones!
                    _geozRemoveGeoZone((GeoZoneID_t)NO_ZONE);
                } else {
                    while (BUFFER_DATA_LENGTH(src) >= sizeof(GeoZoneID_t)) {
                        UInt32 zoneID = 0L;
                        binBufScanf(src, "%*x", sizeof(GeoZoneID_t), &zoneID);
                        _geozRemoveGeoZone((GeoZoneID_t)zoneID);
                    }
                    if (BUFFER_DATA_LENGTH(src) > 0) {
                        cmdErr = COMMAND_OVERFLOW;
                    }
                }
            } GEOZ_UNLOCK
            break;
        case GEOF_CMD_SAVE: // Save GeoZone table to predefined location.
            GEOZ_LOCK {
                cmdErr = _geozSaveGeoZones(GEOZONE_FILENAME)?
                    COMMAND_OK_ACK :    // COMMAND_OK;
                    COMMAND_EXECUTION;  // not saved
            } GEOZ_UNLOCK
            break;
        default:
            cmdErr = COMMAND_FEATURE_NOT_SUPPORTED;
            break;
    }
    
    /* return error */
    return cmdErr;

}

// ----------------------------------------------------------------------------

/* initialize geofence module */
void geozInitialize(eventAddFtn_t queueEvent)
{
    
    /* already initialized? */
    if (didInitialize) {
        return; // init only once
    }
    didInitialize = utTrue; 

    /* event generator */
    ftnQueueEvent = queueEvent;
        
    /* init lock */
#ifdef ENABLE_THREADS
    threadMutexInit(&geozMutex);
#endif

    /* init vars */
    gpsClear(&arrivePoint);
    gpsClear(&departPoint);
    _geozLoadGeoZones(GEOZONE_FILENAME);
        
    /* set geozone property command handler */
    propSetCommandFtn(PROP_CMD_GEOF_ADMIN, &_cmdGeoZoneAdmin);
    
    /* debug stuff */
    //GeoZone_t _gz, *gz = &_gz;
    //logDEBUG(LOGSRC,"sizeof(GeoZone_t)        = %d", sizeof(_gz));
    //logDEBUG(LOGSRC,"sizeof(GeoZone_t.point)  = %d", sizeof(gz->point));
    //logDEBUG(LOGSRC,"addr(&(GeoZone_t.point)) = %ld", (UInt32)((UInt32)&(gz->point) - (UInt32)gz));

}

// ----------------------------------------------------------------------------
#endif //(ENABLE_GEOZONE)
