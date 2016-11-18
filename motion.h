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

#ifndef _MOTION_H
#define _MOTION_H
#ifdef __cplusplus
extern "C" {
#endif

#include "gpstools.h"
#include "event.h"

// ----------------------------------------------------------------------------

#define MOTION_START_GPS_KPH        0
#define MOTION_START_GPS_METERS     1
#define MOTION_START_OBC_KPH        2

#define MOTION_STOP_WHEN_STOPPED    0
#define MOTION_STOP_AFTER_DELAY     1

// ----------------------------------------------------------------------------

void motionInitialize(eventAddFtn_t queueEvent);
#if defined(TRANSPORT_MEDIA_SERIAL)
void motionResetMovingMessageTimer();
#endif

void motionCheckGPS(const GPS_t *oldFix, const GPS_t *newFix);
void motionResetStatus(void);

// ----------------------------------------------------------------------------

#ifdef __cplusplus
}
#endif
#endif
