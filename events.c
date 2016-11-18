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
//  GPS/Data event manager. 
//  Generates and manages events.
// ---
// Change History:
//  2006/01/04  Martin D. Flynn
//     -Initial release
//  2006/06/08  Martin D. Flynn
//     -Populate gpsPDOP in 'evSetEventGPS'
//  2007/01/28  Martin D. Flynn
//     -WindowsCE port
//     -Added custom fields FIELD_ENTITY, FIELD_OBC_DISTANCE
//     -Limit temperatures to +/-126 for LO_RES, and +/-3276.6 for HI_RES
//     -Changed to support elapsed time resolution of 1 second
//     -Added field support for OBC fields
//  2007/02/11  Martin D. Flynn
//     -Added FIELD_OBC_COOLANT_LEVEL, FIELD_OBC_OIL_PRESSURE
//     -Changed FIELD_OBC_ENGINE_TEMP to FIELD_OBC_COOLANT_TEMP and added hiRes mode
//  2007/02/25  Martin D. Flynn
//     -Changed 'FIELD_OBC_FAULT_CODE' to 'FIELD_OBC_J1708_FAULT'
//     -Changed 'obcFaultCode' to 'obcJ1708Fault'
//  2007/03/11  Martin D. Flynn
//     -Added support for 'FIELD_OBC_FUEL_USED'
// ----------------------------------------------------------------------------

#include "defaults.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <ctype.h>
#include <time.h>
//#include <sys/time.h>

#include "log.h"

#include "stdtypes.h"
#include "bintools.h"
#include "strtools.h"
#include "utctools.h"

#include "propman.h"
#include "pqueue.h"
#include "event.h"
#include "events.h"

// ----------------------------------------------------------------------------

#define TEMPERATURE_LO_RES_INVALID        127L  // 1 byte
#define TEMPERATURE_HI_RES_INVALID      32767L  // 2 bytes

// ----------------------------------------------------------------------------
pthread_mutex_t event_mutex = PTHREAD_MUTEX_INITIALIZER;
PacketQueue_DEFINE(eventQueue,EVENT_QUEUE_SIZE);

static UInt32	eventSequence = 0;
static UInt32	totalPacketCount = 0;

// ----------------------------------------------------------------------------

/* define standard resolution fixed packet type */
static FieldDef_t   FixedFields_30[] = {
    EVENT_FIELD(FIELD_STATUS_CODE   , LO_RES, 0, 2),
    EVENT_FIELD(FIELD_TIMESTAMP     , LO_RES, 0, 4),
    EVENT_FIELD(FIELD_GPS_POINT     , LO_RES, 0, 6),
    EVENT_FIELD(FIELD_SPEED         , LO_RES, 0, 1),
    EVENT_FIELD(FIELD_HEADING       , LO_RES, 0, 1),
    EVENT_FIELD(FIELD_ALTITUDE      , LO_RES, 0, 2),
    EVENT_FIELD(FIELD_DISTANCE      , LO_RES, 0, 3),
    EVENT_FIELD(FIELD_SEQUENCE      , LO_RES, 0, 1),
};
static CustomDef_t FixedPacket_30 = {
    PKT_CLIENT_FIXED_FMT_STD,
    (sizeof(FixedFields_30)/sizeof(FixedFields_30[0])),
    FixedFields_30
};

/* define high resolution fixed packet type */
static FieldDef_t   FixedFields_31[] = {
    EVENT_FIELD(FIELD_STATUS_CODE   , HI_RES, 0, 2),
    EVENT_FIELD(FIELD_TIMESTAMP     , HI_RES, 0, 4),
    EVENT_FIELD(FIELD_GPS_POINT     , HI_RES, 0, 8),
    EVENT_FIELD(FIELD_SPEED         , HI_RES, 0, 2),
    EVENT_FIELD(FIELD_HEADING       , HI_RES, 0, 2),
    EVENT_FIELD(FIELD_ALTITUDE      , HI_RES, 0, 3),
    EVENT_FIELD(FIELD_DISTANCE      , HI_RES, 0, 3),
    EVENT_FIELD(FIELD_SEQUENCE      , HI_RES, 0, 1),
};
static CustomDef_t FixedPacket_31 = {
    PKT_CLIENT_FIXED_FMT_HIGH,
    (sizeof(FixedFields_31)/sizeof(FixedFields_31[0])),
    FixedFields_31
};

/* table of predefined formats */
static CustomDef_t *FixedEventTable[] = {
    &FixedPacket_30,        // standard resolution
    &FixedPacket_31         // high resolution
};

// ----------------------------------------------------------------------------

/* table of custom formats */
static CustomDef_t *CustomEventTable[] = {
    (CustomDef_t*)0,
    (CustomDef_t*)0,
    (CustomDef_t*)0,
    (CustomDef_t*)0,
    (CustomDef_t*)0
};

// ----------------------------------------------------------------------------

/* add a custom format field definition */
// This should be done at startup initialization
utBool evAddCustomDefinition(CustomDef_t *cd)
{
    int i, maxSize;
    maxSize = sizeof(CustomEventTable)/sizeof(CustomEventTable[0]);
    for (i = 0; i < maxSize; i++) {
        if (!CustomEventTable[i]) {
            CustomEventTable[i] = cd;
            return utTrue;
        }
    }
    return utFalse;
}

// ----------------------------------------------------------------------------

static CustomDef_t *_evGetCustomDefinitionForType(ClientPacketType_t hdrType)
{
    int i, maxSize;
    
    /* first check fixed formats */
    maxSize = sizeof(FixedEventTable)/sizeof(FixedEventTable[0]);
    for (i = 0; (i < maxSize) && FixedEventTable[i]; i++) {
        if (hdrType == FixedEventTable[i]->hdrType) {
            return FixedEventTable[i];
        }
    }
    
    /* check custom formats */
    maxSize = sizeof(CustomEventTable)/sizeof(CustomEventTable[0]);
    for (i = 0; (i < maxSize) && CustomEventTable[i]; i++) {
        if (hdrType == CustomEventTable[i]->hdrType) {
            return CustomEventTable[i];
        }
    }
    
    /* still not found */
    return (CustomDef_t*)0;
}

/* return a 'template' packet for the specified custom type */
utBool evGetCustomFormatPacket(Packet_t *pkt, ClientPacketType_t cstPktType)
{
    CustomDef_t *custDef = _evGetCustomDefinitionForType(cstPktType);
    if (custDef) {
        int i;
        
        /* custom format packet type */
        ClientPacketType_t fmtType = PKT_CLIENT_FORMAT_DEF_24;
        
        /* assemble packet */
        int rtn = pktInit(pkt, fmtType, (char*)0);
        if (rtn >= 0) {
            FmtBuffer_t bb, *bf = pktFmtBuffer(&bb, pkt);
            binFmtPrintf(bf, "%1x%1u", (UInt32)CLIENT_PACKET_TYPE(custDef->hdrType), (UInt32)custDef->fldLen);
            for (i = 0; i < custDef->fldLen; i++) {
                FieldDef_t *f = &(custDef->fld[i]);
                UInt32 fldDef = FIELD_DEF24(f);
                binFmtPrintf(bf, "%3x", fldDef);
            }
            pkt->dataLen = (UInt8)BUFFER_DATA_LENGTH(bf);
            return utTrue;
        } else {
            logERROR(LOGSRC,"Packet buffer overflow");
            return utFalse;
        }
        
    }
    return utFalse;
}

// ----------------------------------------------------------------------------

static Int32 _round(double D) { return (D>=0.0)?(Int32)(D+0.5):(Int32)(D-0.5); }
#define ROUND(D) _round(D)   // ((UInt32)((D) + 0.5))

#define LIMIT_INDEX(N,L)    (((N) >= (L))? ((L) - 1) : (N))

/* create Packet from Event structure */
static Packet_t *_evCreateEventPacket(Packet_t *pkt, ClientPacketType_t pktType, CustomDef_t *custDef, UInt32 *evtSeq, Event_t *er)
{
    UInt8 fldLen  = (UInt8)custDef->fldLen;
    FieldDef_t *fld = custDef->fld;
    
    /* init packet */
    pktInit(pkt, pktType, (char*)0); // payload filled-in below

    /* binPrintf format buffer */
    FmtBuffer_t bb, *bf = pktFmtBuffer(&bb, pkt);

    /* cache sequence number */
    UInt32 sequence = SEQUENCE_ALL;
    UInt32 seq_org = SEQUENCE_ALL;
    UInt8 seqPos = 0, seqLen = 0; // unspecified
    
    /* fill in custom fields */
    int i;
    for (i = 0; i < fldLen; i++) {
        int   len        = (int)fld[i].length;
        UInt8 ndx        = (UInt8)fld[i].index;
        utBool isHiRes   = fld[i].hiRes;
        Int32  iVal32    = 0L;
        UInt32 uVal32    = 0L;
        char *fmt        = "";
        switch ((EventFieldType_t)fld[i].type) {
            
            case FIELD_STATUS_CODE      : // hex
                // 'len' had better be '2' (however, no checking is made at this point)
                binFmtPrintf(bf, "%*x", len, (UInt32)er->statusCode);
                break;
            case FIELD_TIMESTAMP        :
                // 'len' had better be '4' (however, no checking is made at this point)
                ndx = LIMIT_INDEX(ndx, sizeof(er->timestamp)/sizeof(er->timestamp[0]));
                binFmtPrintf(bf, "%*u", len, (UInt32)er->timestamp[ndx]);
                break;
            case FIELD_INDEX            :
                binFmtPrintf(bf, "%*u", len, (UInt32)er->index);
                break;
                
            case FIELD_GPS_POINT        :
                // 'len' had better be '6' or '8' (however, no checking is made at this point)
                ndx = LIMIT_INDEX(ndx, sizeof(er->gpsPoint)/sizeof(er->gpsPoint[0]));
                binFmtPrintf(bf, "%*g", len, &(er->gpsPoint[ndx]));
                break;
            case FIELD_GPS_AGE          :
                if ((len == 1) && (er->gpsAge > 0xFF)) {
                    binFmtPrintf(bf, "%*u", len, (UInt32)0xFF);
                } else 
                if ((len == 2) && (er->gpsAge > 0xFFFF)) {
                    binFmtPrintf(bf, "%*u", len, (UInt32)0xFFFF);
                } else {
                    binFmtPrintf(bf, "%*u", len, (UInt32)er->gpsAge);
                }
                break;
            case FIELD_SPEED            : // double
                uVal32 = isHiRes? (UInt32)ROUND(er->speedKPH * 10.0) : (UInt32)ROUND(er->speedKPH);
                binFmtPrintf(bf, "%*u", len, uVal32);
                break;
            case FIELD_HEADING          : // double
                uVal32 = isHiRes? (UInt32)ROUND(er->heading * 100.0) : (UInt32)ROUND(er->heading * 255.0/360.0);
                fmt   = isHiRes? "%*u" : "%*x";
                binFmtPrintf(bf, fmt, len, uVal32);
                break;
            case FIELD_ALTITUDE         : // double +/-
                iVal32 = isHiRes? (Int32)ROUND(er->altitude * 10.0) : (Int32)ROUND(er->altitude);
                binFmtPrintf(bf, "%*i", len, iVal32);
                break;
            case FIELD_DISTANCE         : // double
                uVal32 = isHiRes? (UInt32)ROUND(er->distanceKM * 10.0) : (UInt32)ROUND(er->distanceKM);
                binFmtPrintf(bf, "%*u", len, uVal32);
                break;
            case FIELD_ODOMETER         : // double
                uVal32 = isHiRes? (UInt32)ROUND(er->odometerKM * 10.0) : (UInt32)ROUND(er->odometerKM);
                binFmtPrintf(bf, "%*u", len, uVal32);
                break;

            case FIELD_SEQUENCE         : // hex (UInt32)
                seqPos   = (UInt8)BUFFER_DATA_INDEX(bf);
                seqLen   = (UInt8)len;
                seq_org = *evtSeq;
                sequence = seq_org & SEQUENCE_MASK(len);
                binFmtPrintf(bf, "%*x", len, (UInt32)sequence);
                break;
                
            case FIELD_GEOFENCE_ID      : // hex (UInt32)
                ndx = LIMIT_INDEX(ndx, sizeof(er->geofenceID)/sizeof(er->geofenceID[0]));
                binFmtPrintf(bf, "%*x", len, (UInt32)er->geofenceID[ndx]);
                break;
            case FIELD_TOP_SPEED        : // double
                uVal32 = isHiRes? (UInt32)ROUND(er->topSpeedKPH * 10.0) : (UInt32)ROUND(er->topSpeedKPH);
                binFmtPrintf(bf, "%*u", len, uVal32);
                break;

#ifdef EVENT_INCL_STRING
            case FIELD_STRING           :
                ndx = LIMIT_INDEX(ndx, sizeof(er->string)/sizeof(er->string[0]));
                binFmtPrintf(bf, "%*s", len, er->string[ndx]);
                break;
            case FIELD_STRING_PAD       :
                ndx = LIMIT_INDEX(ndx, sizeof(er->string)/sizeof(er->string[0]));
                binFmtPrintf(bf, "%*p", len, er->string[ndx]);
                break;
#endif

#ifdef EVENT_INCL_ENTITY
            case FIELD_ENTITY           :
                ndx = LIMIT_INDEX(ndx, sizeof(er->entity)/sizeof(er->entity[0]));
                binFmtPrintf(bf, "%*s", len, er->entity[ndx]);
                break;
            case FIELD_ENTITY_PAD       :
                ndx = LIMIT_INDEX(ndx, sizeof(er->entity)/sizeof(er->entity[0]));
                binFmtPrintf(bf, "%*p", len, er->entity[ndx]);
                break;
#endif

#ifdef EVENT_INCL_BINARY
            case FIELD_BINARY           : 
                if (er->binary) {
                    if (len <= er->binaryLen) {
                        binFmtPrintf(bf, "%*b", len, er->binary);
                    } else {
                        binFmtPrintf(bf, "%*b%*z", er->binaryLen, er->binary, len - er->binaryLen);
                    }
                } else {
                    binFmtPrintf(bf, "%*z", len);
                }
                break;
#endif

#ifdef EVENT_INCL_DIGITAL_INPUT
            case FIELD_INPUT_ID         : // hex
                binFmtPrintf(bf, "%*x", len, (UInt32)er->inputID);
                break;
            case FIELD_INPUT_STATE      : // hex
                binFmtPrintf(bf, "%*x", len, (UInt32)er->inputState);
                break;
            case FIELD_OUTPUT_ID        : // hex
                binFmtPrintf(bf, "%*x", len, (UInt32)er->outputID);
                break;
            case FIELD_OUTPUT_STATE     : // hex
                binFmtPrintf(bf, "%*x", len, (UInt32)er->outputState);
                break;
            case FIELD_ELAPSED_TIME     :
                ndx = LIMIT_INDEX(ndx, sizeof(er->elapsedTimeSec)/sizeof(er->elapsedTimeSec[0]));
                binFmtPrintf(bf, "%*u", len, (UInt32)er->elapsedTimeSec[ndx]);
                break;
            case FIELD_COUNTER          :
                ndx = LIMIT_INDEX(ndx, sizeof(er->counter)/sizeof(er->counter[0]));
                binFmtPrintf(bf, "%*u", len, (UInt32)er->counter[ndx]);
                break;
#endif

#ifdef EVENT_INCL_ANALOG_INPUT
            case FIELD_SENSOR32_LOW     :
                ndx = LIMIT_INDEX(ndx, sizeof(er->sensor32LO)/sizeof(er->sensor32LO[0]));
                binFmtPrintf(bf, "%*u", len, (UInt32)er->sensor32LO[ndx]);
                break;
            case FIELD_SENSOR32_HIGH    :
                ndx = LIMIT_INDEX(ndx, sizeof(er->sensor32HI)/sizeof(er->sensor32HI[0]));
                binFmtPrintf(bf, "%*u", len, (UInt32)er->sensor32HI[ndx]);
                break;
            case FIELD_SENSOR32_AVER    :
                ndx = LIMIT_INDEX(ndx, sizeof(er->sensor32AV)/sizeof(er->sensor32AV[0]));
                binFmtPrintf(bf, "%*u", len, (UInt32)er->sensor32AV[ndx]);
                break;
#endif
#ifdef EVENT_INCL_TEMPERATURE
            case FIELD_TEMP_LOW         : // double +/-
                ndx = LIMIT_INDEX(ndx, sizeof(er->tempLO)/sizeof(er->tempLO[0]));
                iVal32 = isHiRes? (Int32)ROUND(er->tempLO[ndx] * 10.0) : (Int32)ROUND(er->tempLO[ndx]);
                if (len <= 1) {
                    if (iVal32 < -TEMPERATURE_LO_RES_INVALID) { iVal32 = -TEMPERATURE_LO_RES_INVALID; } else
                    if (iVal32 >  TEMPERATURE_LO_RES_INVALID) { iVal32 =  TEMPERATURE_LO_RES_INVALID; }
                } else {
                    if (iVal32 < -TEMPERATURE_HI_RES_INVALID) { iVal32 = -TEMPERATURE_HI_RES_INVALID; } else
                    if (iVal32 >  TEMPERATURE_HI_RES_INVALID) { iVal32 =  TEMPERATURE_HI_RES_INVALID; }
                }
                binFmtPrintf(bf, "%*i", len, iVal32);
                break;
            case FIELD_TEMP_HIGH        : // double +/-
                ndx = LIMIT_INDEX(ndx, sizeof(er->tempHI)/sizeof(er->tempHI[0]));
                iVal32 = isHiRes? (Int32)ROUND(er->tempHI[ndx] * 10.0) : (Int32)ROUND(er->tempHI[ndx]);
                if (len <= 1) {
                    if (iVal32 < -TEMPERATURE_LO_RES_INVALID) { iVal32 = -TEMPERATURE_LO_RES_INVALID; } else
                    if (iVal32 >  TEMPERATURE_LO_RES_INVALID) { iVal32 =  TEMPERATURE_LO_RES_INVALID; }
                } else {
                    if (iVal32 < -TEMPERATURE_HI_RES_INVALID) { iVal32 = -TEMPERATURE_HI_RES_INVALID; } else
                    if (iVal32 >  TEMPERATURE_HI_RES_INVALID) { iVal32 =  TEMPERATURE_HI_RES_INVALID; }
                }
                binFmtPrintf(bf, "%*i", len, iVal32);
                break;
            case FIELD_TEMP_AVER        : // double +/-
                ndx = LIMIT_INDEX(ndx, sizeof(er->tempAV)/sizeof(er->tempAV[0]));
                iVal32 = isHiRes? (Int32)ROUND(er->tempAV[ndx] * 10.0) : (Int32)ROUND(er->tempAV[ndx]);
                if (len <= 1) {
                    if (iVal32 < -TEMPERATURE_LO_RES_INVALID) { iVal32 = -TEMPERATURE_LO_RES_INVALID; } else
                    if (iVal32 >  TEMPERATURE_LO_RES_INVALID) { iVal32 =  TEMPERATURE_LO_RES_INVALID; }
                } else {
                    if (iVal32 < -TEMPERATURE_HI_RES_INVALID) { iVal32 = -TEMPERATURE_HI_RES_INVALID; } else
                    if (iVal32 >  TEMPERATURE_HI_RES_INVALID) { iVal32 =  TEMPERATURE_HI_RES_INVALID; }
                }
                binFmtPrintf(bf, "%*i", len, iVal32);
                break;
#endif

#ifdef EVENT_INCL_GPS_STATS
            case FIELD_GPS_DGPS_UPDATE  :
                binFmtPrintf(bf, "%*u", len, (UInt32)er->gpsDgpsUpdate);
                break;
            case FIELD_GPS_HORZ_ACCURACY: // double
                uVal32 = isHiRes? (UInt32)ROUND(er->gpsHorzAccuracy * 10.0) : (UInt32)ROUND(er->gpsHorzAccuracy);
                binFmtPrintf(bf, "%*u", len, uVal32);
                break;
            case FIELD_GPS_VERT_ACCURACY: // double
                uVal32 = isHiRes? (UInt32)ROUND(er->gpsVertAccuracy * 10.0) : (UInt32)ROUND(er->gpsVertAccuracy);
                binFmtPrintf(bf, "%*u", len, uVal32);
                break;
            case FIELD_GPS_SATELLITES   :
                binFmtPrintf(bf, "%*u", len, (UInt32)er->gpsSatellites);
                break;
            case FIELD_GPS_MAG_VARIATION: // double +/-
                iVal32 = (Int32)ROUND(er->gpsMagVariation * 100.0);
                binFmtPrintf(bf, "%*i", len, iVal32);
                break;
            case FIELD_GPS_QUALITY      :
                binFmtPrintf(bf, "%*u", len, (UInt32)er->gpsQuality);
                break;
            case FIELD_GPS_TYPE         :
                binFmtPrintf(bf, "%*u", len, (UInt32)er->gps2D3D);
                break;
            case FIELD_GPS_GEOID_HEIGHT : // double +/-
                iVal32 = isHiRes? (Int32)ROUND(er->gpsGeoidHeight * 10.0) : (Int32)ROUND(er->gpsGeoidHeight);
                binFmtPrintf(bf, "%*i", len, iVal32);
                break;
            case FIELD_GPS_PDOP         : // double (values above 20.0 are considered poor)
                uVal32 = ((len == 1) && (er->gpsPDOP >= 25.5))? 255L : (UInt32)ROUND(er->gpsPDOP * 10.0);
                binFmtPrintf(bf, "%*u", len, uVal32);
                break;
            case FIELD_GPS_HDOP         : // double (values above 20.0 are considered poor)
                uVal32 = ((len == 1) && (er->gpsHDOP >= 25.5))? 255L : (UInt32)ROUND(er->gpsHDOP * 10.0);
                binFmtPrintf(bf, "%*u", len, uVal32);
                break;
            case FIELD_GPS_VDOP         : // double (values above 20.0 are considered poor)
                uVal32 = ((len == 1) && (er->gpsVDOP >= 25.5))? 255L : (UInt32)ROUND(er->gpsVDOP * 10.0);
                binFmtPrintf(bf, "%*u", len, uVal32);
                break;
#endif

#ifdef EVENT_INCL_OBC
            case FIELD_OBC_VALUE: // EvOBCValue_t
                ndx = LIMIT_INDEX(ndx, sizeof(er->obcValue)/sizeof(er->obcValue[0]));
                if (len >= 4) {
                    EvOBCValue_t *J = &(er->obcValue[ndx]);
                    binFmtPrintf(bf, "%2u%2u", (UInt32)J->mid, (UInt32)J->pid);
                    len -= 4;
                    if (len <= J->dataLen) {
                        binFmtPrintf(bf, "%*b", len, J->data);
                    } else {
                        binFmtPrintf(bf, "%*b%*z", J->dataLen, J->data, len - J->dataLen);
                    }
                } else {
                    binFmtPrintf(bf, "%*z", len);
                }
                break;
            case FIELD_OBC_GENERIC: // UInt32
                ndx = LIMIT_INDEX(ndx, sizeof(er->obcGeneric)/sizeof(er->obcGeneric[0]));
                uVal32 = er->obcGeneric[ndx];
                binFmtPrintf(bf, "%*u", len, uVal32);
                break;
            case FIELD_OBC_J1708_FAULT: // UInt32
                ndx = LIMIT_INDEX(ndx, sizeof(er->obcJ1708Fault)/sizeof(er->obcJ1708Fault[0]));
                uVal32 = er->obcJ1708Fault[ndx];
                binFmtPrintf(bf, "%*x", len, uVal32);
                break;
            case FIELD_OBC_DISTANCE   : // double
                uVal32 = isHiRes? (UInt32)ROUND(er->obcDistanceKM * 10.0) : (UInt32)ROUND(er->obcDistanceKM);
                binFmtPrintf(bf, "%*u", len, uVal32);
                break;
            case FIELD_OBC_ENGINE_HOURS: // double
                uVal32 = (UInt32)ROUND(er->obcEngineHours * 10.0);
                binFmtPrintf(bf, "%*u", len, uVal32);
                break;
            case FIELD_OBC_ENGINE_RPM: // UInt32
                uVal32 = er->obcEngineRPM;
                binFmtPrintf(bf, "%*u", len, uVal32);
                break;
            case FIELD_OBC_COOLANT_TEMP: // double
                iVal32 = isHiRes? (UInt32)ROUND(er->obcCoolantTemp * 10.0) : (UInt32)ROUND(er->obcCoolantTemp);
                binFmtPrintf(bf, "%*i", len, iVal32);
                break;
            case FIELD_OBC_COOLANT_LEVEL: // double
                uVal32 = isHiRes? (UInt32)ROUND(er->obcCoolantLevel * 1000.0) : (UInt32)ROUND(er->obcCoolantLevel * 100.0);
                binFmtPrintf(bf, "%*u", len, uVal32);
                break;
            case FIELD_OBC_OIL_LEVEL: // double
                uVal32 = isHiRes? (UInt32)ROUND(er->obcOilLevel * 1000.0) : (UInt32)ROUND(er->obcOilLevel * 100.0);
                binFmtPrintf(bf, "%*u", len, uVal32);
                break;
            case FIELD_OBC_OIL_PRESSURE: // double
                uVal32 = isHiRes? (UInt32)ROUND(er->obcOilPressure * 10.0) : (UInt32)ROUND(er->obcOilPressure);
                binFmtPrintf(bf, "%*u", len, uVal32);
                break;
            case FIELD_OBC_FUEL_LEVEL: // double
                uVal32 = isHiRes? (UInt32)ROUND(er->obcFuelLevel * 1000.0) : (UInt32)ROUND(er->obcFuelLevel * 100.0);
                binFmtPrintf(bf, "%*i", len, uVal32);
                break;
            case FIELD_OBC_FUEL_ECONOMY: // double
                uVal32 = (UInt32)ROUND(er->obcAvgFuelEcon * 10.0); // try average first
                if (uVal32 == 0L) {
                    uVal32 = (UInt32)ROUND(er->obcFuelEconomy * 10.0); // fallback to 'instant'
                }
                binFmtPrintf(bf, "%*u", len, uVal32);
                break;
            case FIELD_OBC_FUEL_USED: // double
                uVal32 = isHiRes? (UInt32)ROUND(er->obcFuelUsed * 10.0) : (UInt32)ROUND(er->obcFuelUsed);
                binFmtPrintf(bf, "%*u", len, uVal32);
                break;
#endif

        }
    }

    /* set packet sequence */
    pkt->sequence = seq_org; // will be 'SEQUENCE_ALL', if not specified as a field
    pkt->seqLen   = seqLen;   // # bytes (will be '0' if not specified as a field)
    pkt->seqPos   = seqPos;   // position of sequence field

    /* return full length */
    pkt->dataLen = (UInt8)BUFFER_DATA_LENGTH(bf);
    return pkt;

}

// ----------------------------------------------------------------------------

/* return the event queue */
PacketQueue_t *evGetEventQueue()
{
    return &eventQueue;
}

// ----------------------------------------------------------------------------

/* encode packet from event */
utBool evEncodePacket(Packet_t *pkt, PacketPriority_t pri, ClientPacketType_t pktType, UInt32 *evSeq, Event_t *er)
{
    if (pkt && er) {
        CustomDef_t *custDef = _evGetCustomDefinitionForType(pktType);
        if (custDef) {
            _evCreateEventPacket(pkt, pktType, custDef, evSeq, er);
            pkt->priority = (pri <= PRIORITY_NONE)? PRIORITY_NORMAL : pri;
            return utTrue;
        } else {
            logERROR(LOGSRC,"Custom format not found: 0x%04X", pktType);
        }
    }
    return utFalse;
}

/* add the specified event to the queue */
utBool evAddEventPacket(Packet_t *pkt, PacketPriority_t pri, ClientPacketType_t pktType, Event_t *er)
{
	UInt32 seq = eventSequence;

    if (pkt && er) {
        if (evEncodePacket(pkt, pri, pktType, &seq, er)) {
            return evAddEncodedPacket(pkt);
        }
    } else {
        logERROR(LOGSRC,"NULL packet/event pointer!");
    }
    return utFalse;
}

/* add the specified event to the queue */
utBool evAddEncodedPacket(Packet_t *pkt)
{
	UInt32 seq;
	pthread_mutex_lock(&event_mutex);
	seq = eventSequence++;
	totalPacketCount++;
	pthread_mutex_unlock(&event_mutex);
	pkt->sequence = seq;
	pkt->data[pkt->seqPos] = (UInt8)(seq & 0xFF);
	return pqueAddPacket(evGetEventQueue(), pkt);
}

// ----------------------------------------------------------------------------

/* return the number of generated events */
Int32 evGetTotalPacketCount()
{
    // 'primary' transport events only
    return (Int32)totalPacketCount;
}

/* return the number of events found in the queue */
Int32 evGetPacketCount()
{
    // 'primary' transport events only
    return pqueGetPacketCount(evGetEventQueue());
}

/* return the number of generated events */
void evSetSequence(UInt32 new_seq)
{
	eventSequence = new_seq;
	totalPacketCount = new_seq;
}

/* return true if there are events in the queue (ie. non-empty) */
//utBool evHasPackets()
//{
//    return pqueHasPackets(evGetEventQueue());
//}

/* return an iterator on the event queue */
//PacketQueueIterator_t *evGetEventIterator(PacketQueueIterator_t *i)
//{
//    return pqueGetIterator(evGetEventQueue(), i);
//}

// ----------------------------------------------------------------------------

/* return the highest priority event in the queue */
//PacketPriority_t evGetHighestPriority()
//{
//    // 'primary' transport events only
//    return pqueGetHighestPriority(evGetEventQueue());
//}

// ----------------------------------------------------------------------------

/* delete the first event in the queue */
//void evFreeFirstEvent()
//{
//    pqueDeleteFirstEntry(evGetEventQueue());
//}

// ----------------------------------------------------------------------------

/* enable overwriting of the oldest event if the event queue fills up */
//void evEnableOverwrite(utBool overwrite)
//{
//    pqueEnableOverwrite(evGetEventQueue(), overwrite);
//}

// ----------------------------------------------------------------------------

/* acknowledge first sent event */
//utBool evAcknowledgeFirst()
//{
//    // 'primary' transport events only
//    UInt32 seq = pqueGetFirstSentSequence(evGetEventQueue()); // return 'SEQUENCE_ALL' if no first event
//    if (seq != SEQUENCE_ALL) {
//        return evAcknowledgeToSequence(seq);
//    } else {
//        return utFalse;
//    }
//}

/* acknowledge events up to and including the specified sequence */
//utBool evAcknowledgeToSequence(UInt32 sequence)
//{
//    // 'primary' transport events only
//    utBool didAck = utFalse;
//    utBool ackAll = (sequence == SEQUENCE_ALL)? utTrue : utFalse;
//    PacketQueue_t *eventQueue = evGetEventQueue();
//    if (ackAll || pqueHasSentPacketWithSequence(eventQueue,sequence)) {
//        PacketQueueIterator_t evi;
//        pqueGetIterator(eventQueue, &evi);
//        for (;;) {
//            // As we iterate through the event packets, we can assume the following:
//            // - If we get to a null packet, we are finished with the list
//            // - If we find a packet that hasn't been sent, then all following packets have 
//            //   also not been sent.
//            // - Once we find the 'first' matching sequence, we delete it then stop. This is
//            //   safer that deleting the last matching sequence.  (Note: multiple possible
//            //   matching sequence numbers can occur if the byte length of the sequence 
//            //   number is 1 (ie. 0 to 255), and more than 255 events are currently in the 
//            //   event packet queue.  Granted, an unlikely situation, but it can occur.)
//            // - No packet->sequence will ever match SEQUENCE_ALL (see 'ackAll').
//            Packet_t *pkt = pqueGetNextPacket((Packet_t*)0, &evi);
//            if (!pkt || !pkt->sent) {
//                //logWARNING(LOGSRC,"Stop at first non-sent packet");
//                break;  // stop at first null or non-sent packet
//            }
//            pqueDeleteFirstEntry(eventQueue);
//            didAck = utTrue;
//            if (ackAll) {
//                // ackowledge all sent packets
//                continue;
//            } else
//            if (pkt->sequence == SEQUENCE_ALL) {
//                // This condition can not (should not) occur.
//                // We don't know what the real sequence of the packet is.
//                // it's safer to stop here.
//                break;
//            } else
//            if (pkt->sequence != (sequence & SEQUENCE_MASK(pkt->seqLen))) {
//                // no match yet
//                continue;
//            }
//            break; // stop when sequence matches
//        }
//    } else {
//        logERROR(LOGSRC,"No packet with sequence: 0x%04lX", (UInt32)sequence);
//    }
//    return didAck;
//}

// ----------------------------------------------------------------------------

static utBool _evDidInit = utFalse;
void evInitialize()
{

    /* already initialized? */
    if (_evDidInit) {
        return;
    }
    _evDidInit = utTrue;

    /* init queue */
    PacketQueue_INIT(eventQueue,EVENT_QUEUE_SIZE);
    
    /* enable overwrite */
    pqueEnableOverwrite(&eventQueue, EVENT_QUEUE_OVERWRITE);

}

// ----------------------------------------------------------------------------
