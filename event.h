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

#ifndef _EVENT_H
#define _EVENT_H
#ifdef __cplusplus
extern "C" {
#endif

#include "defaults.h"

#include "stdtypes.h"
#include "gpstools.h"

#include "props.h"
#include "statcode.h"

// ----------------------------------------------------------------------------

#define EVENT_INCL_ENTITY
#define EVENT_INCL_STRING
#define EVENT_INCL_BINARY
#define EVENT_INCL_DIGITAL_INPUT
#define EVENT_INCL_ANALOG_INPUT
#define EVENT_INCL_TEMPERATURE
#define EVENT_INCL_GPS_STATS
#define EVENT_INCL_OBC

// ----------------------------------------------------------------------------
// MID : Message ID
//          128 = Engine
//          130 = Transmission
//          136 = Brakes (ABS)
// PID: Parameter ID

/* OBC/J1708 value */
#ifdef EVENT_INCL_OBC
typedef struct {
    UInt16          mid;
    UInt16          pid;
    UInt8           dataLen;
    UInt8           data[27];
} EvOBCValue_t;
typedef struct {
    UInt16          mid;        // 8-bits
    UInt16          pidSid;     // 9-bits
    UInt16          fault;      // 4-bits
    UInt16          count;      // 8-bits
} EvOBCFault_t;
#endif

// ----------------------------------------------------------------------------

/* Event structure */
typedef struct {
    
    UInt16          statusCode;
    UInt32          timestamp[1];
    UInt32          index;

    GPSPoint_t      gpsPoint[1];
    UInt32          gpsAge;
    double          speedKPH;
    double          heading;
    double          altitude;
    double          distanceKM;
    double          odometerKM;

    //UInt32          sequence; <-- handled at packet generation
    //UInt32          seqLen;

    UInt32          geofenceID[2];
    double          topSpeedKPH;

#ifdef EVENT_INCL_ENTITY
    char            entity[2][MAX_ID_SIZE + 1];
#endif

#ifdef EVENT_INCL_STRING
    char            string[2][MAX_ID_SIZE + 1];
#endif

#ifdef EVENT_INCL_BINARY
    UInt8           *binary;
    UInt8           binaryLen;
#endif

#ifdef EVENT_INCL_DIGITAL_INPUT
    UInt32          inputID;
    UInt32          inputState;
    UInt32          outputID;
    UInt32          outputState;
    UInt32          elapsedTimeSec[8];
    UInt32          counter[1];         // [8];
#endif

#ifdef EVENT_INCL_ANALOG_INPUT
    UInt32          supplyVoltageMV;
    UInt32          sensor32LO[1];      // [8];
    UInt32          sensor32HI[1];      // [8];
    UInt32          sensor32AV[1];      // [8];
#endif

#ifdef EVENT_INCL_TEMPERATURE
    double          tempLO[4];          // [8];
    double          tempHI[4];          // [8];
    double          tempAV[4];          // [8];
#endif

#ifdef EVENT_INCL_GPS_STATS
    UInt32          gpsDgpsUpdate;
    double          gpsHorzAccuracy;
    double          gpsVertAccuracy;
    UInt32          gpsSatellites;
    double          gpsMagVariation;
    UInt32          gpsQuality;         // 1=GPS, 2=DGPS
    UInt32          gps2D3D;            // 2=2D, 3=3D
    double          gpsGeoidHeight;
    double          gpsPDOP;
    double          gpsHDOP;
    double          gpsVDOP;
#endif

#ifdef EVENT_INCL_OBC
    EvOBCValue_t    obcValue[10];
    UInt32          obcGeneric[10];
    UInt32          obcJ1708Fault[2];
    double          obcDistanceKM;
    double          obcEngineHours;
    UInt32          obcEngineRPM;
    double          obcCoolantTemp;
    double          obcCoolantLevel;
    double          obcOilLevel;
    double          obcOilPressure;
    double          obcFuelLevel;
    double          obcFuelEconomy;
    double          obcAvgFuelEcon;
    double          obcFuelUsed;
#endif

} Event_t;

// ----------------------------------------------------------------------------

Event_t *evClearEvent(Event_t *er);
Event_t *evSetEventGPS(Event_t *er, const GPS_t *gps);
Event_t *evSetEventDefaults(Event_t *er, StatusCode_t code, UInt32 timestamp, const GPS_t *gps);

// ----------------------------------------------------------------------------

#ifdef __cplusplus
}
#endif
#endif
