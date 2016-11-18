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
//  GPS fix checking for odometer events.
//  Examines changes in GPS fix data and generates odometer events accordingly.
// ---
// Change History:
//  2006/01/04  Martin D. Flynn
//     -Initial release
//  2007/01/28  Martin D. Flynn
//     -WindowsCE port
//     -This module has been updated to accomodate the odometer resolution change
//      from 0.1 meter unit to 1 meter units.
// ----------------------------------------------------------------------------

#include "defaults.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "log.h"
#include "gps.h"

#include "stdtypes.h"
#include "gpstools.h"
#include "utctools.h"

#include "propman.h"
#include "statcode.h"
#include "event.h"

#include "odometer.h"

// ----------------------------------------------------------------------------

#define ODOMETER_PRIORITY   PRIORITY_HIGH

static Int32 _round(double D) { return (D>=0.0)?(Int32)(D+0.5):(Int32)(D-0.5); }
#define ROUND(D) _round(D)   // ((UInt32)((D) + 0.5))
static utBool	odom_enable = utFalse;
// ----------------------------------------------------------------------------

static eventAddFtn_t  ftnQueueEvent = 0;

static struct {
    Key_t           value;
    Key_t           limit;
    Key_t           state;
    StatusCode_t    code;
} odomTable[] = {
   // Meters                 Alarm Limit            Odom State/GPS       Status Code
    { PROP_ODOMETER_0_VALUE, PROP_ODOMETER_0_LIMIT, PROP_ODOMETER_0_GPS, STATUS_ODOM_LIMIT_0 },
    { PROP_ODOMETER_1_VALUE, PROP_ODOMETER_1_LIMIT, PROP_ODOMETER_1_GPS, STATUS_ODOM_LIMIT_1 },
    { PROP_ODOMETER_2_VALUE, PROP_ODOMETER_2_LIMIT, PROP_ODOMETER_2_GPS, STATUS_ODOM_LIMIT_2 },
    { PROP_ODOMETER_3_VALUE, PROP_ODOMETER_3_LIMIT, PROP_ODOMETER_3_GPS, STATUS_ODOM_LIMIT_3 },
    { PROP_ODOMETER_4_VALUE, PROP_ODOMETER_4_LIMIT, PROP_ODOMETER_4_GPS, STATUS_ODOM_LIMIT_4 },
    { PROP_ODOMETER_5_VALUE, PROP_ODOMETER_5_LIMIT, PROP_ODOMETER_5_GPS, STATUS_ODOM_LIMIT_5 },
    { PROP_ODOMETER_6_VALUE, PROP_ODOMETER_6_LIMIT, PROP_ODOMETER_6_GPS, STATUS_ODOM_LIMIT_6 },
    { PROP_ODOMETER_7_VALUE, PROP_ODOMETER_7_LIMIT, PROP_ODOMETER_7_GPS, STATUS_ODOM_LIMIT_7 }
};

#define ODOMETER_COUNT          (sizeof(odomTable)/sizeof(odomTable[0]))

// ----------------------------------------------------------------------------

static UInt8 odomFirstInit[ODOMETER_COUNT];
#define ODOM_IsFirst(X)         (odomFirstInit[(X)])
#define ODOM_SetFirst(X,V)      {odomFirstInit[(X)] = (V);}

/* get odometer state for specified index */
static GPSOdometer_t *_odomGetState(UInt16 ndx)
{
    GPSOdometer_t *gps = (GPSOdometer_t*)0;
    if (ndx < ODOMETER_COUNT) {
        gps = (GPSOdometer_t*)propGetGPS(odomTable[ndx].state, (GPSOdometer_t*)0);
    }
    if (!gps) {
        logCRITICAL(LOGSRC,"Internal odometer error");
    }
    return gps;
}

// ----------------------------------------------------------------------------

/* initialize odometer */
void odomInitialize(eventAddFtn_t queueEvent)
{
	UInt32 odom_limit0;
	ftnQueueEvent = queueEvent;
	memset(odomFirstInit, 0, sizeof(odomFirstInit));
	odom_limit0 = propGetUInt32(odomTable[0].limit, 0L);
	if (odom_limit0 > 0)
		odom_enable = utTrue;
}

// ----------------------------------------------------------------------------

/* return actual odometer value for the vehicle to which the device is attached */
// will return '0' if the actual odometer is not available
double odomGetActualOdometerMeters()
{
    // implement 'actual' odometer value retrieval here (via OBC, etc).
    return 0.0;
}

/* return meters value at specified index */
double odomGetDistanceMetersAtIndex(int ndx)
{
    if (ndx == 0) {
        // vehicle distance (may need special handling)
        return (double)propGetUInt32(odomTable[0].value, 0L);
    } else
    if ((ndx > 0) && (ndx < ODOMETER_COUNT)) {
        // driver, etc
        return (double)propGetUInt32(odomTable[ndx].value, 0L);
    } else {
        // invalid index
        return 0.0;
    }
}

/* return total meters for device */
double odomGetDeviceDistanceMeters()
{
    // vehicle distance (may need special handling)
    return odomGetDistanceMetersAtIndex(0);
}

/* return meters value at specified index */
utBool odomResetDistanceMetersAtIndex(int ndx)
{
    if ((ndx >= 0) && (ndx < ODOMETER_COUNT)) {
        propSetUInt32(odomTable[ndx].value, 0L);
        return utTrue;
    } else {
        return utFalse;
    }
}

// ----------------------------------------------------------------------------

/* queue new event */
static void _queueOdometerEvent(PacketPriority_t priority, StatusCode_t code, const GPS_t *gps, UInt32 odomMeters)
{
    
    /* make sure we have a GPS fix */
    GPS_t gpsFix;
    if (!gps) {
        // all odometers events should have a GPS fix, so this should never occur
        gps = gpsGetLastGPS(&gpsFix, 0);
    }

    /* queue event */
    if (gps && ftnQueueEvent) {
        Event_t evRcd;
        evSetEventDefaults(&evRcd, code, 0L, gps);
        evRcd.distanceKM = (double)odomMeters / 1000.0;
        (*ftnQueueEvent)(priority, DEFAULT_EVENT_FORMAT, &evRcd);
    }
    
}

// ----------------------------------------------------------------------------

/* periodic check new GPS fix */
void odomCheckGPS(const GPS_t *oldFix, const GPS_t *newFix)
{
    // 'newFix' may be NULL
	if (!odom_enable) {
		return;
	}
    /* get actual odometer, if available */
    UInt32 actualOdomMeters = ROUND(odomGetActualOdometerMeters());
    /* loop through odometers */
    UInt32 minDeltaMeters = propGetUInt32(PROP_GPS_DISTANCE_DELTA, 500L);    
    if (minDeltaMeters < 10L) { minDeltaMeters = 10L; }
    int i;
    for (i = 0; i < ODOMETER_COUNT; i++) {
        GPSOdometer_t *gps = _odomGetState(i);
        if (!gps) { continue; } // error (invalid index)
        
        /* update odometer value */
        UInt32 oldOdomMeters = propGetUInt32(odomTable[i].value, 0L);
        UInt32 newOdomMeters = 0L;
        if ((!oldOdomMeters && !ODOM_IsFirst(i)) || // <-- I've reset the value
            (!gps->fixtime || !gpsPointIsValid(&(gps->point)))) {
            // First tripometer check (after a reset)
            if (i == 0) {
                // vehicle odometer
                newOdomMeters = (actualOdomMeters > 0L)? actualOdomMeters : oldOdomMeters;
                gps->meters = 0L; // vehicle odometer base is '0'
            } else {
                // tripometer
                newOdomMeters = 0L;
                gps->meters = ROUND(odomGetDeviceDistanceMeters());
            }
            propSetUInt32(odomTable[i].value, newOdomMeters);
            if (newFix) {
                memcpy((GPSShort_t*)gps, (GPSShort_t*)newFix, sizeof(GPSShort_t));
            }
            propSetGPS(odomTable[i].state, gps);
            ODOM_SetFirst(i,1); // this will be the first fix
        } else
        if (newFix) {
            // GPS based odometer
            UInt32 deltaMeters = ROUND(gpsMetersToPoint(&(newFix->point), &(gps->point)));
            if (deltaMeters >= minDeltaMeters) {
                // I've moved at least the minimum required distance to set a new steak in the ground
                newOdomMeters = ODOM_IsFirst(i)? deltaMeters : (deltaMeters + oldOdomMeters);
                propSetUInt32(odomTable[i].value, newOdomMeters);
                memcpy((GPSShort_t*)gps, (GPSShort_t*)newFix, sizeof(GPSShort_t));
                propSetGPS(odomTable[i].state, gps);
                ODOM_SetFirst(i,0); // no longer the first fix
            }
        }
        
        /* check odometer limits */
        // only sends an event at the crossing of the limit
        if (newOdomMeters > 0L) {
            UInt32 limitOdomMeters = propGetUInt32(odomTable[i].limit, 0L);
            if ((limitOdomMeters > oldOdomMeters) && (limitOdomMeters <= newOdomMeters)) {
                StatusCode_t code = (StatusCode_t)odomTable[i].code;
                _queueOdometerEvent(ODOMETER_PRIORITY, code, newFix, newOdomMeters);
            }
        }
        
    }
    
}

// ----------------------------------------------------------------------------
