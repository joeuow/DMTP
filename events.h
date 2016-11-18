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

#ifndef _EVENTS_H
#define _EVENTS_H
#ifdef __cplusplus
extern "C" {
#endif

#include "defaults.h"

#include "stdtypes.h"
#include "gpstools.h"

#include "packet.h"
#include "pqueue.h"
#include "props.h"
#include "statcode.h"
#include "event.h"

// ----------------------------------------------------------------------------
// Notes on field data types:
// - Numeric fields may consume between 1 and 4 bytes, depending on the granularity
//   needed to represent the data in the field.
// - All numeric fields must be presented in network-byte-order, or big-endian
//   format.
// - 'String' data may be less than, or equal to, the field size.  If the String 
//   data is less than the field size, then the String must be terminated with
//   a single null byte (hex 0x00).  The reset of the packet can then continue
//   immediately following the null byte.
// - 'Binary' data must always fill the full fixed length of the field.

// ----------------------------------------------------------------------------

#define HI_RES                  utTrue
#define LO_RES                  utFalse

enum EventFieldType_enum {
    
 // Most common fields                  // Low                          High
    FIELD_STATUS_CODE           = 0x01, // %2u                          -
    FIELD_TIMESTAMP             = 0x02, // %4u                          -
    FIELD_INDEX                 = 0x03, // %4u 0 to 4294967295          -
    
 // Sequence number field               // Low                          High
    FIELD_SEQUENCE              = 0x04, // %1u 0 to 255                 %2u 0 to 65535

 // GPS fields                          // Low                          High
    FIELD_GPS_POINT             = 0x06, // %6g                          %8g
    FIELD_GPS_AGE               = 0x07, // %2u 0 to 65535 sec           -
    FIELD_SPEED                 = 0x08, // %1u 0 to 255 kph             %2u 0.0 to 655.3 kph
    FIELD_HEADING               = 0x09, // %1u 1.412 deg un.            %2u 0.00 to 360.00 deg
    FIELD_ALTITUDE              = 0x0A, // %2i -32767 to +32767 m       %3i -838860.7 to +838860.7 m
    FIELD_DISTANCE              = 0x0B, // %3u 0 to 16777216 km         %4u 0.0 to 429496729.5 km
    FIELD_ODOMETER              = 0x0C, // %3u 0 to 16777216 km         %4u 0.0 to 429496729.5 km

 // Misc fields                         // Low                          High
    FIELD_GEOFENCE_ID           = 0x0E, // %4u 0x00000000 to 0xFFFFFFFF
    FIELD_TOP_SPEED             = 0x0F, // %1u 0 to 255 kph             %2u 0.0 to 655.3 kph

 // Generic String/ID field             // Low                          High
    FIELD_STRING                = 0x11, // %*s may contain only 'A'..'Z', 'a'..'z, '0'..'9', '-', '.'
    FIELD_STRING_PAD            = 0x12, // %*s may contain only 'A'..'Z', 'a'..'z, '0'..'9', '-', '.'

 // Entity String field                 // Low                          High
    FIELD_ENTITY                = 0x15, // %*s may contain only 'A'..'Z', 'a'..'z, '0'..'9', '-', '.'
    FIELD_ENTITY_PAD            = 0x16, // %*s may contain only 'A'..'Z', 'a'..'z, '0'..'9', '-', '.'

 // Generic binary field                // Low                          High
    FIELD_BINARY                = 0x1A, // %*b  

 // Digital I/O fields                  // Low                          High
    FIELD_INPUT_ID              = 0x21, // %4u 0x00000000 to 0xFFFFFFFF
    FIELD_INPUT_STATE           = 0x22, // %4u 0x00000000 to 0xFFFFFFFF
    FIELD_OUTPUT_ID             = 0x24, // %4u 0x00000000 to 0xFFFFFFFF
    FIELD_OUTPUT_STATE          = 0x25, // %4u 0x00000000 to 0xFFFFFFFF
    FIELD_ELAPSED_TIME          = 0x27, // %4u 0 to 4294967295 sec
    FIELD_COUNTER               = 0x28, // %4u 0 to 4294967295

 // Analog I/O fields                   // Low                          High
    FIELD_SENSOR32_LOW          = 0x31, // %4u 0x00000000 to 0xFFFFFFFF
    FIELD_SENSOR32_HIGH         = 0x32, // %4u 0x00000000 to 0xFFFFFFFF
    FIELD_SENSOR32_AVER         = 0x33, // %4u 0x00000000 to 0xFFFFFFFF

 // Temperature fields
    FIELD_TEMP_LOW              = 0x3A, // %1i -126 to +126 C           %2i -3276.6 to +3276.6 C
    FIELD_TEMP_HIGH             = 0x3B, // %1i -126 to +126 C           %2i -3276.6 to +3276.6 C
    FIELD_TEMP_AVER             = 0x3C, // %1i -126 to +126 C           %2i -3276.6 to +3276.6 C

 // GPS quality fields                  // Low                          High
    FIELD_GPS_DGPS_UPDATE       = 0x41, // %2u 0 to 65535 sec
    FIELD_GPS_HORZ_ACCURACY     = 0x42, // %1u 0 to 255 m               %2u 0.0 to 6553.5 m
    FIELD_GPS_VERT_ACCURACY     = 0x43, // %1u 0 to 255 m               %2u 0.0 to 6553.5 m
    FIELD_GPS_SATELLITES        = 0x44, // %1u 0 to 12
    FIELD_GPS_MAG_VARIATION     = 0x45, // %2i -180.00 to 180.00 deg
    FIELD_GPS_QUALITY           = 0x46, // %1u (0=None, 1=GPS, 2=DGPS, ...)
    FIELD_GPS_TYPE              = 0x47, // %1u (0/1=None, 2=2D, 3=3D, ...)
    FIELD_GPS_GEOID_HEIGHT      = 0x48, // %1i -128 to +127 m           %2i -3276.7 to +3276.7 m
    FIELD_GPS_PDOP              = 0x49, // %1u 0.0 to 25.5              %2u 0.0 to 99.9
    FIELD_GPS_HDOP              = 0x4A, // %1u 0.0 to 25.5              %2u 0.0 to 99.9
    FIELD_GPS_VDOP              = 0x4B, // %1u 0.0 to 25.5              %2u 0.0 to 99.9

 // OBC/J1708 fields (believed to be the most common requested OBC data)
    FIELD_OBC_VALUE             = 0x50, // %*b (at least 4 bytes, includes mid/pid)
    FIELD_OBC_GENERIC           = 0x51, // %4u
    FIELD_OBC_J1708_FAULT       = 0x52, // %4u
    FIELD_OBC_DISTANCE          = 0x54, // %3u 0 to 16777216 km         %4u 0.0 to 429496729.5 km
    FIELD_OBC_ENGINE_HOURS      = 0x57, // %3u 0 to 1677721.6 hours
    FIELD_OBC_ENGINE_RPM        = 0x58, // %2u 0 to 65535 rpm
    FIELD_OBC_COOLANT_TEMP      = 0x59, // %1i -126 to 126 C            %2i -3276.7 to +3276.7 C
    FIELD_OBC_COOLANT_LEVEL     = 0x5A, // %1u 0% to 100% percent       %2u 0.0% to 100.0% percent
    FIELD_OBC_OIL_LEVEL         = 0x5B, // %1u 0% to 100% percent       %2u 0.0% to 100.0% percent
    FIELD_OBC_OIL_PRESSURE      = 0x5C, // %1u 0 to 255 kPa             %2u 0.0 to 6553.5 kPa
    FIELD_OBC_FUEL_LEVEL        = 0x5D, // %1u 0% to 100% percent       %2u 0.0% to 100.0% percent
    FIELD_OBC_FUEL_ECONOMY      = 0x5E, // %1u 0 to 255 kpl             %2u 0.0 to 6553.5 kpl
    FIELD_OBC_FUEL_USED         = 0x5F, // %3u 0 to 16777216 liters     %4u 0.0 to 429496729.5 liters

};
typedef enum EventFieldType_enum EventFieldType_t;

// ----------------------------------------------------------------------------

/* field definition */
typedef struct {
    EventFieldType_t    type;       // field type 
    utBool              hiRes;      // field flags (eg. HI_RES, LO_RES)
    UInt8               index;      // type index
    UInt8               length;     // field byte size
} FieldDef_t;

#define EVENT_FIELD(T,H,I,L)    { (T), (H), (UInt8)(I), (UInt8)(L) }

// Mask Layout:
//   23:1  HiRes    (1 bit)     0x800000
//   16:7  Type     (0..127)    0x7F0000
//    8:8  Index    (0..255)    0x00FF00
//    0:8  Length   (0..255)    0x0000FF
#define _FLDMSK_REZ_24          0x01    // 1 bits
#define _FLDMSK_TYP_24          0x7F    // 7 bits
#define _FLDMSK_NDX_24          0xFF    // 8 bits
#define _FLDMSK_LEN_24          0xFF    // 8 bits
#define _FLD_DEF24(T,F,I,L)     (((UInt32)((F)&_FLDMSK_REZ_24)<<23)|((UInt32)((T)&_FLDMSK_TYP_24)<<16)|((UInt32)((I)&_FLDMSK_NDX_24)<<8)|(UInt32)((L)&_FLDMSK_LEN_24))
#define FIELD_DEF24(F)          _FLD_DEF24((F)->type,(F)->hiRes,(F)->index,(F)->length)

/* custom packet definition */
typedef struct {
    ClientPacketType_t          hdrType;
    UInt16                      fldLen;
    FieldDef_t                  *fld;
} CustomDef_t;

// ----------------------------------------------------------------------------

/* typdef for 'evAddEventPacket' function */
typedef utBool (*eventAddFtn_t)(PacketPriority_t priority, ClientPacketType_t pktType, Event_t *er);

// ----------------------------------------------------------------------------
void evInitialize(void);
utBool evAddCustomDefinition(CustomDef_t *cd);
utBool evGetCustomFormatPacket(Packet_t *pkt, ClientPacketType_t cstPktType);

// ----------------------------------------------------------------------------

PacketQueue_t *evGetEventQueue(void);

// ----------------------------------------------------------------------------

utBool evEncodePacket(Packet_t *pkt, PacketPriority_t pri, ClientPacketType_t pktType, UInt32 *evtSeq, Event_t *er);
utBool evAddEventPacket(Packet_t *pkt, PacketPriority_t pri, ClientPacketType_t pktType, Event_t *er);
utBool evAddEncodedPacket(Packet_t *pkt);
Int32 evGetTotalPacketCount(void);
Int32 evGetPacketCount(void);
void evSetSequence(UInt32 new_seq);

// ----------------------------------------------------------------------------

#ifdef __cplusplus
}
#endif
#endif
