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
//  Functions for a single event structure
// ---
// Change History:
//  2007/01/28  Martin D. Flynn
//     -Split off from 'events.c'
//     -Changed to support elapsed time resolution of 1 second
//     -Added field support for OBC fields
//  2007/03/11  Martin D. Flynn
//     -Added support for 'obcFuelUsed'
//  2007/04/28  Martin D. Flynn
//     -Populate current geofenceID[0] in 'evSetEventDefaults'
// ----------------------------------------------------------------------------

#include "defaults.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <ctype.h>
#include <time.h>

#include "log.h"

#include "stdtypes.h"
#include "bintools.h"
#include "strtools.h"
#include "utctools.h"

#include "odometer.h"

#include "propman.h"
#include "pqueue.h"
#include "event.h"
#include "events.h"

// ----------------------------------------------------------------------------

/* clear event structure */
Event_t * evClearEvent(Event_t *er)
{
    if (er) {

        /* clear entire structure */
        memset(er, 0, sizeof(Event_t));

        /* default timestamp */
        er->timestamp[0]    = utcGetTimeSec();

        /* gps point(s) */
        int gi;
        for (gi = 0; gi < sizeof(er->gpsPoint)/sizeof(er->gpsPoint[0]); gi++) {
            gpsPointClear(&(er->gpsPoint[gi]));
        }

        /* gps attributes */
        er->speedKPH        = GPS_UNDEFINED_SPEED;
        er->heading         = GPS_UNDEFINED_HEADING;
        er->altitude        = GPS_UNDEFINED_ALTITUDE;
        er->distanceKM      = GPS_UNDEFINED_DISTANCE;
        er->odometerKM      = GPS_UNDEFINED_DISTANCE;

        /* sensor init */
#ifdef EVENT_INCL_TEMPERATURE
        int ai;
        for (ai = 0; ai < sizeof(er->tempLO)/sizeof(er->tempLO[0]); ai++) {
            er->tempLO[ai]   = GPS_UNDEFINED_TEMPERATURE;
            er->tempHI[ai]   = GPS_UNDEFINED_TEMPERATURE;
            er->tempAV[ai]   = GPS_UNDEFINED_TEMPERATURE;
        }
#endif

        /* misc */
        er->topSpeedKPH     = GPS_UNDEFINED_SPEED;

        /* gps */
#ifdef EVENT_INCL_GPS_STATS
        er->gpsHorzAccuracy = GPS_UNDEFINED_ACCURACY;
        er->gpsVertAccuracy = GPS_UNDEFINED_ACCURACY;
        er->gpsMagVariation = GPS_UNDEFINED_MAG_VARIATION;
        er->gpsGeoidHeight  = GPS_UNDEFINED_GEOID_HEIGHT;
        er->gpsPDOP         = GPS_UNDEFINED_DOP;
        er->gpsHDOP         = GPS_UNDEFINED_DOP;
        er->gpsVDOP         = GPS_UNDEFINED_DOP;
#endif

#ifdef EVENT_INCL_OBC
        er->obcDistanceKM   = GPS_UNDEFINED_DISTANCE;
        er->obcEngineHours  = 0.0;
        er->obcCoolantTemp  = GPS_UNDEFINED_TEMPERATURE;
        er->obcCoolantLevel = 0.0;
        er->obcOilLevel     = 0.0;
        er->obcOilPressure  = 0.0;
        er->obcFuelLevel    = 0.0;
        er->obcFuelEconomy  = 0.0;
        er->obcAvgFuelEcon  = 0.0;
        er->obcFuelUsed     = 0.0;
#endif

    }
    return er;
}

/* copy GPS information into the event */
Event_t *evSetEventGPS(Event_t *er, const GPS_t *gps)
{
    if (er && gps) {
        gpsPointCopy(&(er->gpsPoint[0]), &(gps->point));
      //er->timestamp[0]    = utcGetTimeSec();
        er->gpsAge          = er->timestamp[0] - gps->fixtime;
        er->speedKPH        = gps->speedKPH;
        er->heading         = gps->heading;
        er->altitude        = gps->altitude;
#ifdef EVENT_INCL_GPS_STATS
        er->gpsHorzAccuracy = gps->accuracy;
        er->gpsPDOP         = gps->pdop;
        er->gpsHDOP         = gps->hdop;
        er->gpsVDOP         = gps->vdop;
        er->gpsQuality      = gps->fixtype;
#endif
    }
    return er;
}

/* set default event values */
Event_t *evSetEventDefaults(Event_t *er, StatusCode_t code, UInt32 timestamp, const GPS_t *gps)
{
    if (er) {
        
        /* clear */
        evClearEvent(er);
        
        /* status code */
        er->statusCode = code;
        
        /* timestamp (if specified) */
        if (timestamp > 0L) {
            er->timestamp[0] = timestamp;
        }
        
        /* GPS */
        evSetEventGPS(er, gps);
        
        /* odometer */
        er->odometerKM = odomGetDeviceDistanceMeters() / 1000.0;
        
        /* geozone (if not already set) */
#if defined(ENABLE_GEOZONE)
        if (!er->geofenceID[0]) {
           er->geofenceID[0] = propGetUInt32(PROP_GEOF_CURRENT, 0L);
        }
#endif
        
    }
    return er;
}
