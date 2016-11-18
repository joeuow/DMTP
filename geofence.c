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
//  GPS rule module for checking for geofence arrival/departure
// Notes:
//  - A simple point and radius is used to determine a GeoZone. A more sophisticated
//    implementation may be needed for commercial applications.
//  - For simplicity, this implementation stores a geofence in the property manager, 
//    which limits the number of useful geo-zones. This is most likely not what will 
//    be wanted in a commercial application.
//  - The property manager 'GPS' type includes the 'fixtime' of the GPS.  Since this
//    field in not needed for geofence arrival/departure detection, this field is
//    instead used to hold the value of the geofence radius.
// ---
// Change History:
//  2006/01/17  Martin D. Flynn
//     -Initial release
//  2006/05/07  Martin D. Flynn
//     -Retired, in favor of the full-featured GeoZone arrival/departure module
//      now found at "src/modules/geozone.*"
//  2007/01/28  Martin D. Flynn
//     -Moved to "src/modules/"
// ----------------------------------------------------------------------------

#if defined (ENABLE_GEOZONE)
#include "stdafx.h" // TARGET_WINCE
#include "custom/defaults.h"    // build defaults

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "custom/startup.h"
#include "custom/log.h"

#include "tools/stdtypes.h"
#include "tools/gpstools.h"
#include "tools/utctools.h"

#include "base/propman.h"
#include "base/statcode.h"
#include "base/events.h"

//#include "modules/odometer.h"
#include "modules/geofence.h"

// ----------------------------------------------------------------------------

#define DEFAULT_EVENT_FORMAT        PKT_CLIENT_FIXED_FMT_HIGH

// ----------------------------------------------------------------------------

#define ARRIVE_PRIORITY             PRIORITY_NORMAL
#define DEPART_PRIORITY             PRIORITY_NORMAL

// ----------------------------------------------------------------------------

static eventAddFtn_t  ftnQueueEvent = 0;

// ----------------------------------------------------------------------------

static UInt32 geofProps[] = {
    PROP_CUST_GEOFENCE_1,
    PROP_CUST_GEOFENCE_2,
    PROP_CUST_GEOFENCE_3,
    PROP_CUST_GEOFENCE_4,
};

static int _geofInTerminal(const GPSPoint_t *gpsFix)
{
    if (gpsPointIsValid(gpsFix)) {
        int i;
        for (i = 0; i < sizeof(geofProps)/sizeof(geofProps[0]); i++) {
            const GPSShort_t *geofGps = propGetGPS(geofProps[i], (GPSShort_t*)0);
            if (gpsPointIsValid((GPSPoint_t*)geofGps)) {
                double maxDeltaMeters = (double)(geofGps->fixtime);
                double deltaMeters = gpsMetersToPoint(gpsFix, (GPSPoint_t*)geofGps);
                if (deltaMeters <= maxDeltaMeters) {
                    return i + 1;
                }
            }
        }
    }
    return 0; // not in a geofence
}

// ----------------------------------------------------------------------------

/* initialize motion module */
void geofInitialize(eventAddFtn_t queueEvent)
{
    ftnQueueEvent = queueEvent;
}

// ----------------------------------------------------------------------------

/* add a motion event to the event queue */
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

/* check new GPS fix for arrival/departure */
void geofCheckGPS(const GPS_t *oldFix, const GPS_t *newFix)
{
    
    /* in terminal? */
    int newGeofIndex = _geofInTerminal(&(newFix->point));
    int oldGeofIndex = (int)propGetUInt32(PROP_GEOF_CURRENT, 0L);
    
    /* terminal changed? */
    if (oldGeofIndex != newGeofIndex) {
        if (oldGeofIndex > 0) {
            // departed 'oldGeofIndex'
            logINFO(LOGSRC,"Departed GeoFence %d ...", oldGeofIndex);
            _queueGeofenceEvent(DEPART_PRIORITY, STATUS_GEOFENCE_DEPART, newFix, oldGeofIndex);
        }
        if (newGeofIndex > 0) {
            // arrived 'newGeofIndex'
            logINFO(LOGSRC,"Arrived GeoFence %d ...", newGeofIndex);
            _queueGeofenceEvent(ARRIVE_PRIORITY, STATUS_GEOFENCE_ARRIVE, newFix, newGeofIndex);
        }
        propSetUInt32(PROP_GEOF_CURRENT, (UInt32)newGeofIndex);
    }

}
// ----------------------------------------------------------------------------
#endif //(ENABLE_GEOZONE)
