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
//  GPS fix checking for motion events.
//  Examines changes in GPS fix data and generates motion events accordingly.
// ---
// Change History:
//  2006/01/04  Martin D. Flynn
//     -Initial release
//  2006/01/23  Martin D. Flynn
//     -Init dormant timer at first non-motion GPS fix
//  2006/02/13  Martin D. Flynn
//     -Changed stop event checking to include an option to have the stop event
//      time set to the time the vehicle actually stopped.  This also effects the
//      behaviour of the in-motion message to sending in-motion events iff the 
//      vehicle is actually moving at the time the event is generated.  (See
//      property PROP_MOTION_STOP_TYPE).
//  2007/01/28  Martin D. Flynn
//     -WindowsCE port
//  2007/03/11  Martin D. Flynn
//     -Modified 'stop' checking to allow for detection of other 'stop' indicators,
//      such as an indication that the ignition has been turned off.
//  2007/04/28  Martin D. Flynn
//     -Added a 5kph setback to the excess-speed detection and event generation.
//      Example: If a 100 kph excess speed is triggered, the vehicle must slow to
//      below 95 kph to reset the excess speed indicator.
// ----------------------------------------------------------------------------
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>

#include "defaults.h"
#include "log.h"
#include "gps.h"

#include "stdtypes.h"
#include "gpstools.h"
#include "utctools.h"

#include "propman.h"
#include "statcode.h"
#include "events.h"

#include "motion.h"

// ----------------------------------------------------------------------------

#define MOTION_START_PRIORITY       PRIORITY_NORMAL
#define MOTION_STOP_PRIORITY        PRIORITY_NORMAL
#define IN_MOTION_PRIORITY          PRIORITY_NORMAL
#define DORMANT_PRIORITY            PRIORITY_NORMAL
#define EXCESS_SPEED_PRIORITY       PRIORITY_NORMAL
#define MOVING_PRIORITY             PRIORITY_NORMAL

// ----------------------------------------------------------------------------

#if defined(DEBUG_COMPILE)
// no minimum when compiling for testing purposes
#define MIN_IN_MOTION_INTERVAL      0L                  // seconds
#define MIN_DORMANT_INTERVAL        0L                  // seconds

#elif defined(TRANSPORT_MEDIA_NONE)
// no minimum when sending to a file
#define MIN_IN_MOTION_INTERVAL      0L                  // seconds
#define MIN_DORMANT_INTERVAL        0L                  // seconds

#elif defined(TRANSPORT_MEDIA_FILE)
// no minimum when sending to a file
#define MIN_IN_MOTION_INTERVAL      0L                  // seconds
#define MIN_DORMANT_INTERVAL        0L                  // seconds

#elif defined(TRANSPORT_MEDIA_SERIAL)
// small minimum when sending to bluetooth
#define MIN_IN_MOTION_INTERVAL      20L                 // seconds
#define MIN_DORMANT_INTERVAL        20L                 // seconds

#else
// set default minimum when sending to server
#define MIN_IN_MOTION_INTERVAL      60L                 // seconds
#define MIN_DORMANT_INTERVAL        MINUTE_SECONDS(5)   // seconds

#endif

#define EXCESS_SPEED_SETBACK        5.0                 // kph

// ----------------------------------------------------------------------------
static utBool	isInMotion  = utFalse;
static utBool	motionInit  = utFalse;
static utBool	isDormant  = utFalse;
static GPS_t	lastMotionFix;              // needs to be initialized
static UInt32	stopTimer = 0;
static UInt32	inMotionMessageCycle = 0;
static UInt32	dormantMessageCycle = 0;
static UInt32	dormantCount = 0;
static UInt32	rfidMovingCount = 0;
#if defined(TRANSPORT_MEDIA_SERIAL) || defined(SECONDARY_SERIAL_TRANSPORT)
static TimerSec_t           lastMovingMessageTimer      = 0L;
#endif
static eventAddFtn_t	ftnQueueEvent = 0;
static threadMutex_t	motionMutex;
/*static UInt32	motion_debug = 0; */
extern UInt32 gpsDistanceDelta;
#define MOTION_LOCK         MUTEX_LOCK(&motionMutex);
#define MOTION_UNLOCK       MUTEX_UNLOCK(&motionMutex);
// ----------------------------------------------------------------------------

void motionResetStatus(void)
{
	isInMotion = utFalse;
	stopTimer = 0;
	dormantCount = 0;
	rfidMovingCount = 0;
	gpsClear(&lastMotionFix);
}

/* initialize motion module */
void motionInitialize(eventAddFtn_t queueEvent)
{
    
    /* init motion */
    if (!motionInit) {
        motionInit = utTrue;
        
        /* event handler */
        ftnQueueEvent = queueEvent;
    
        /* init motion mutex */
        threadMutexInit(&motionMutex);
        
		motionResetStatus();
    }

}
	

#if defined(TRANSPORT_MEDIA_SERIAL)
/* reset moving message timer */
void motionResetMovingMessageTimer()
{
    MOTION_LOCK {
        lastMovingMessageTimer = 0L;
    } MOTION_UNLOCK
}
#endif
// ----------------------------------------------------------------------------

/* add a motion event to the event queue */
static void _queueMotionEvent(PacketPriority_t priority, StatusCode_t code, UInt32 timestamp, const GPS_t *gps)
{
	/* init event */
	Event_t evRcd;
	evSetEventDefaults(&evRcd, code, timestamp, gps);
	/* queue event */
	if (ftnQueueEvent) {
		(*ftnQueueEvent)(priority, DEFAULT_EVENT_FORMAT, &evRcd);
	}
}
// ----------------------------------------------------------------------------
/* motion in motion */
static void motion_in_motion(double speed, UInt32 now, const GPS_t *newFix)
{
	double speed_limit;
	speed_limit = propGetDouble(PROP_MOTION_EXCESS_SPEED, 120.0);
	if (speed > speed_limit)
		_queueMotionEvent(EXCESS_SPEED_PRIORITY, STATUS_MOTION_EXCESS_SPEED, now, newFix);
	else 
		_queueMotionEvent(IN_MOTION_PRIORITY, STATUS_MOTION_IN_MOTION, now, newFix);
}

/* check new GPS fix for various motion events */
void motionCheckGPS(const GPS_t *oldFix, const GPS_t *newFix)
{
	double speedKPH = 0.0, deltaMeters = 0.0;
	double defMotionDistance, defMotionStart;
	UInt32 defDormantInterval, maxDormantCount, nowTime;
	UInt32 defMotionStop, defStartType, defStopType, defMotionInterval, val1;
	utBool isMoving = utFalse;
	/* 'start' definition */
	// defStartType:
	//   0 - check GPS speed (kph)
	//   1 - check GPS distance (meters)
	defStartType = propGetUInt32(PROP_MOTION_START_TYPE, 0);
	speedKPH = newFix->speedKPH;
	if (defStartType == MOTION_START_GPS_METERS) {
		defMotionDistance = propGetDouble(PROP_MOTION_START, 0.0);
		deltaMeters = gpsMetersToPoint(&oldFix->point, &newFix->point);
		if (deltaMeters > defMotionDistance)
			isMoving = utTrue;
	}
	else {
		defMotionStart = propGetDouble(PROP_MOTION_START, 0.0);
		if (speedKPH > defMotionStart)
			isMoving = utTrue;
	}
	if (!gpsIsValid(&lastMotionFix)) {
		gpsCopy(&lastMotionFix, newFix); // save new motion fix for next check
	}
	if (isDormant) {
	/* check motion start/stop */
		deltaMeters = gpsMetersToPoint(&lastMotionFix.point, &newFix->point);
		if (deltaMeters >= gpsDistanceDelta)
			isMoving = utTrue;
	}
	//   - send start/stop/in-motion event
	// PROP_GPS_MIN_SPEED should already be accounted for
	nowTime = utcGetTimeSec();
	/* check for currently moving */
	if (isMoving) {
		// I am moving, reset stop timer
		if (!isInMotion) {
			// I wasn't moving before, but now I am
			isInMotion = utTrue;
			isDormant = utFalse;
			inMotionMessageCycle = 0; // start "in-motion" timer
			// send 'start' event
			_queueMotionEvent(MOTION_START_PRIORITY, STATUS_MOTION_START, nowTime, newFix);
		} 
		else {
			++inMotionMessageCycle;
			val1 = propGetUInt32(PROP_MOTION_IN_MOTION, 0);
			if (val1 == 0)
				val1 = USHRT_MAX;
			defMotionInterval = (val1 + gpsEventCycle - 1) / gpsEventCycle;
			// In-motion interval has been defined - we want in-motion events.
			if (inMotionMessageCycle >= defMotionInterval) {
					// we're moving, and the in-motion interval has expired
				motion_in_motion(speedKPH, nowTime, newFix);
				inMotionMessageCycle = 0;
			} 
		}
		gpsCopy(&lastMotionFix, newFix); // save new motion fix for next check
		stopTimer = 0;
	} // isMoving
	else {
		if (isInMotion) {
		// I was moving, but stops now.
			++stopTimer;
			defStopType = propGetUInt32(PROP_MOTION_STOP_TYPE, 0);
			if (defStopType == MOTION_STOP_WHEN_STOPPED)
				defMotionStop = 0;
			else 
				defMotionStop = propGetUInt32(PROP_MOTION_STOP, 0) / gpsEventCycle; 
			if (stopTimer >= defMotionStop) {
				_queueMotionEvent(MOTION_STOP_PRIORITY, STATUS_MOTION_STOP, nowTime, newFix);
				isInMotion = utFalse;
				dormantCount = 0;
				dormantMessageCycle = 0;
			}
			else {
				++inMotionMessageCycle;
				val1 = propGetUInt32(PROP_MOTION_IN_MOTION, 0);
				if (val1 == 0)
					val1 = USHRT_MAX;
				defMotionInterval = (val1 + gpsEventCycle - 1) / gpsEventCycle;
				if (inMotionMessageCycle >= defMotionInterval) {
					// we're moving, and the in-motion interval has expired
					motion_in_motion(speedKPH, nowTime, newFix);
					inMotionMessageCycle = 0;
				}
			} 
		} //isInMotion
		else {
			val1 = propGetUInt32(PROP_MOTION_DORMANT_INTRVL, 0);
			defDormantInterval = (val1 + gpsEventCycle - 1) / gpsEventCycle;
			maxDormantCount = propGetUInt32(PROP_MOTION_DORMANT_COUNT, 0L);
			if (dormantCount < maxDormantCount && defDormantInterval > 0) {
				if (++dormantMessageCycle >= defDormantInterval) {
					// send dormant message
					_queueMotionEvent(DORMANT_PRIORITY, STATUS_MOTION_DORMANT, nowTime, newFix);
					dormantCount++;
					dormantMessageCycle = 0;
					isDormant = utTrue;
					gpsCopy(&lastMotionFix, newFix); // save new motion fix for next check
				}
			}
		}
	} //isMoving (else)
}
