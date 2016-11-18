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
//  GPS module initialization and callback.
// ---
// Change History:
//  2006/01/04  Martin D. Flynn
//     -Initial release
//  2007/01/28  Martin D. Flynn
//     -WindowsCE port
// ----------------------------------------------------------------------------

#include "defaults.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "log.h"
#include "gps.h"
#include "gpsmods.h"

#include "motion.h"
#include "odometer.h"
#if defined(ENABLE_GEOZONE)
#  include "geozone.h"
#endif

#include "events.h"

#include "stdtypes.h"

// ----------------------------------------------------------------------------

/* GPS event module initialization */
void gpsModuleInitialize(eventAddFtn_t queueEvent)
{

    /* motion */
    motionInitialize(queueEvent);

    /* odometer */
    odomInitialize(queueEvent);

    /* geozone */
#if defined(ENABLE_GEOZONE)
    geozInitialize(queueEvent);
#endif

    // Add other module initialization to this list as needed

}

// ----------------------------------------------------------------------------

/* check for triggered events based on GPS fix changes */
void gpsModuleCheckEvents(GPS_t *lastFix, GPS_t *newFix)
{
    // NOTE: 'newFix' may be NULL if a new fix is not available
    
    /* motion */
    motionCheckGPS(lastFix, newFix);
    
    /* odometer */
    odomCheckGPS(lastFix, newFix);

    /* geozone (if enabled) */
#if defined(ENABLE_GEOZONE)
    geozCheckGPS(lastFix, newFix);
#endif

    // Add other modules to this list as needed

}

// ----------------------------------------------------------------------------

/* periodic invocation */
// (approximately once per second)
void gpsModulePeriodic()
{
    // Add other modules to this list as needed
}

// ----------------------------------------------------------------------------

