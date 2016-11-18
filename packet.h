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

#ifndef _PACKET_H
#define _PACKET_H
#ifdef __cplusplus
extern "C" {
#endif

//#include "defaults.h" 
//   PACKET_MAX_FIELD_COUNT
//   PACKET_MAX_PAYLOAD_LENGTH

#include "stdtypes.h"
#include "bintools.h"
#include "gpstools.h"

// ----------------------------------------------------------------------------
//
// General Binary packet format:
//   0:1 - Packet header (currently 0xE0 in this implementation)
//   1:1 - Packet type (see Client/Server packet types below)
//   2:1 - Payload length (0 to 255 bytes)
//   3:X - Payload data
//
// General ASCII packet format:
//   0:1 - Start of ASCII packet character (ASCII "$")
//   1:2 - ASCII Packet header (ASCII "E0")
//   3:2 - ASCII Packet type (see Client/Server packet types below)
//   5:1 - ASCII Payload encoding type
//          "=" for Base64 encoded payload (preferred)
//          ":" for HEX encoded payload
//          "," for CSV encoded payload
//   6:X - ASCII payload according to encoding type
//   X:3 - Optional ASCII XOR checksum (in the format "*HH")
//   X:1 - ASCII "\r" terminator
//
// ----------------------------------------------------------------------------

typedef UInt8 PacketType_t;

// ----------------------------------------------------------------------------

#define PACKET_ASCII_ENCODING_CHAR      '$'     // 0x24
#define PACKET_ASCII_ENCODING_EOL       '\r'    // '\n'    // '\r'

#define PACKET_HEADER_BASIC             ((UInt8)0xE0)   // header
#define PACKET_HEADER_CLIENT_BASIC      ((UInt8)0xE0)   // header
#define PACKET_HEADER_SERVER_BASIC      ((UInt8)0xE0)   // header
#define PACKET_HEADER_LENGTH            3

#define CLIENT_HEADER_TYPE(H,T)         (ClientPacketType_t)(((H) << 8) | ((T) & 0xFF))
#define CLIENT_PACKET_HEADER(HT)        (UInt8)(((HT) >> 8) & 0xFF) // PACKET_HEADER_CLIENT_BASIC
#define CLIENT_PACKET_TYPE(HT)          (UInt8)((HT) & 0xFF)

#define SERVER_HEADER_TYPE(H,T)         (((H) << 8) | ((T) & 0xFF))
#define SERVER_PACKET_HEADER(HT)        (UInt8)(((HT) >> 8) & 0xFF) // PACKET_HEADER_SERVER_BASIC
#define SERVER_PACKET_TYPE(HT)          (UInt8)((HT) & 0xFF)

// ----------------------------------------------------------------------------
// Client originated packets

#define PKT_CLIENT_HEADER               ((UInt16)PACKET_HEADER_CLIENT_BASIC << 8)
enum ClientPacketType_enum {
    
    /* dialog packets */
    PKT_CLIENT_EOB_DONE                 = PKT_CLIENT_HEADER|0x00,    // End of block/transmission, "no more to say"
        // Payload:
        //   0:2 - Fletcher checksum (optional)
    PKT_CLIENT_EOB_MORE                 = PKT_CLIENT_HEADER|0x01,    // End of block/transmission, "I have more to say"
        // Payload:
        //   0:2 - Fletcher checksum (optional)
    
    /* identification packets */
    PKT_CLIENT_UNIQUE_ID                = PKT_CLIENT_HEADER|0x11,    // Unique identifier
        // Payload:
        //   0:X - Unique ID [4 to 20 bytes]
    PKT_CLIENT_ACCOUNT_ID               = PKT_CLIENT_HEADER|0x12,    // Account identifier
        // Payload:
        //   0:X - Case insensitive ASCII account ID
    PKT_CLIENT_DEVICE_ID                = PKT_CLIENT_HEADER|0x13,    // Device identifier
        // Payload:
        //   0:X - Case insensitive ASCII device ID

    /* standard fixed format event packets */
    PKT_CLIENT_FIXED_FMT_STD            = PKT_CLIENT_HEADER|0x30,    // Standard GPS
        // Payload:
        //   0:2 - Status code
        //   2:4 - Timestamp (POSIX 'epoch' time)
        //   6:3 - Latitude  (see 'gpstools.c' for encoding/decoding algorithm)
        //   9:3 - Longitude (see 'gpstools.c' for encoding/decoding algorithm)
        //  12:1 - Speed (kph)
        //  13:1 - Heading (Degrees * 255/360)
        //  14:2 - Altitude (meters)
        //  16:3 - Distance (kilometers)
        //  19:1 - Sequence (0..255)
    PKT_CLIENT_FIXED_FMT_HIGH           = PKT_CLIENT_HEADER|0x31,    // High Resolution GPS
        // Payload:
        //   0:2 - Status code
        //   2:4 - Timestamp (POSIX 'epoch' time)
        //   6:4 - Latitude  (see 'gpstools.c' for encoding/decoding algorithm)
        //  10:4 - Longitude (see 'gpstools.c' for encoding/decoding algorithm)
        //  14:2 - Speed (kph * 10)
        //  16:2 - Heading (Degrees * 10)
        //  18:3 - Altitude (meters * 10)
        //  21:3 - Distance (kilometers * 10)
        //  24:1 - Sequence (0..255)
    PKT_CLIENT_FIXED_FORMAT_F           = PKT_CLIENT_HEADER|0x3F,    // (last fixed format)
        // Payload:
        //  (Not defined)

    /* DMT service provider formats */
    PKT_CLIENT_DMTSP_FORMAT_0           = PKT_CLIENT_HEADER|0x50,    // Service provider format #0
    PKT_CLIENT_DMTSP_FORMAT_1           = PKT_CLIENT_HEADER|0x51,    // Service provider format #1
    PKT_CLIENT_DMTSP_FORMAT_2           = PKT_CLIENT_HEADER|0x52,    // Service provider format #2
    PKT_CLIENT_DMTSP_FORMAT_3           = PKT_CLIENT_HEADER|0x53,    // Service provider format #3
    PKT_CLIENT_DMTSP_FORMAT_4           = PKT_CLIENT_HEADER|0x54,    // Service provider format #4
    PKT_CLIENT_DMTSP_FORMAT_5           = PKT_CLIENT_HEADER|0x55,    // Service provider format #5
    PKT_CLIENT_DMTSP_FORMAT_6           = PKT_CLIENT_HEADER|0x56,    // Service provider format #6
    PKT_CLIENT_DMTSP_FORMAT_7           = PKT_CLIENT_HEADER|0x57,    // Service provider format #7
    PKT_CLIENT_DMTSP_FORMAT_8           = PKT_CLIENT_HEADER|0x58,    // Service provider format #8
    PKT_CLIENT_DMTSP_FORMAT_9           = PKT_CLIENT_HEADER|0x59,    // Service provider format #9
    PKT_CLIENT_DMTSP_FORMAT_A           = PKT_CLIENT_HEADER|0x5A,    // Service provider format #A
    PKT_CLIENT_DMTSP_FORMAT_B           = PKT_CLIENT_HEADER|0x5B,    // Service provider format #B
    PKT_CLIENT_DMTSP_FORMAT_C           = PKT_CLIENT_HEADER|0x5C,    // Service provider format #C
    PKT_CLIENT_DMTSP_FORMAT_D           = PKT_CLIENT_HEADER|0x5D,    // Service provider format #D
    PKT_CLIENT_DMTSP_FORMAT_E           = PKT_CLIENT_HEADER|0x5E,    // Service provider format #E
    PKT_CLIENT_DMTSP_FORMAT_F           = PKT_CLIENT_HEADER|0x5F,    // Service provider format #F
        // Payload:
        //  (Not defined)

    /* custom format event packets */
    PKT_CLIENT_CUSTOM_FORMAT_0          = PKT_CLIENT_HEADER|0x70,    // Custom format #0
    PKT_CLIENT_CUSTOM_FORMAT_1          = PKT_CLIENT_HEADER|0x71,    // Custom format #1
    PKT_CLIENT_CUSTOM_FORMAT_2          = PKT_CLIENT_HEADER|0x72,    // Custom format #2
    PKT_CLIENT_CUSTOM_FORMAT_3          = PKT_CLIENT_HEADER|0x73,    // Custom format #3
    PKT_CLIENT_CUSTOM_FORMAT_4          = PKT_CLIENT_HEADER|0x74,    // Custom format #4
    PKT_CLIENT_CUSTOM_FORMAT_5          = PKT_CLIENT_HEADER|0x75,    // Custom format #5
    PKT_CLIENT_CUSTOM_FORMAT_6          = PKT_CLIENT_HEADER|0x76,    // Custom format #6
    PKT_CLIENT_CUSTOM_FORMAT_7          = PKT_CLIENT_HEADER|0x77,    // Custom format #7
    PKT_CLIENT_CUSTOM_FORMAT_8          = PKT_CLIENT_HEADER|0x78,    // Custom format #8
    PKT_CLIENT_CUSTOM_FORMAT_9          = PKT_CLIENT_HEADER|0x79,    // Custom format #9
    PKT_CLIENT_CUSTOM_FORMAT_A          = PKT_CLIENT_HEADER|0x7A,    // Custom format #A
    PKT_CLIENT_CUSTOM_FORMAT_B          = PKT_CLIENT_HEADER|0x7B,    // Custom format #B
    PKT_CLIENT_CUSTOM_FORMAT_C          = PKT_CLIENT_HEADER|0x7C,    // Custom format #C
    PKT_CLIENT_CUSTOM_FORMAT_D          = PKT_CLIENT_HEADER|0x7D,    // Custom format #D
    PKT_CLIENT_CUSTOM_FORMAT_E          = PKT_CLIENT_HEADER|0x7E,    // Custom format #E
    PKT_CLIENT_CUSTOM_FORMAT_F          = PKT_CLIENT_HEADER|0x7F,    // Custom format #F
        // Payload:
        //  (Not defined)

    /* Property packet */
    PKT_CLIENT_PROPERTY_VALUE           = PKT_CLIENT_HEADER|0xB0,    // Property value
        // Payload:
        //   0:2 - Property ID
        //   2:X - Property value
        
    /* Custom format packet */
    PKT_CLIENT_FORMAT_DEF_24            = PKT_CLIENT_HEADER|0xCF,    // Custom format definition (24 bit field def)
        // Payload:
        //   0:1 - Custom packet type (0x70..0x7F)
        //   1:1 - Number of fields
        //   2:3 - Field definition (bitmask)
        //          23:1  HiRes           [0x800000]
        //          16:7  Type            [0x7F0000]
        //           8:8  Index           [0x00FF00]
        //           0:8  Length          [0x0000FF]

    /* Diagnostic packets */
    PKT_CLIENT_DIAGNOSTIC               = PKT_CLIENT_HEADER|0xD0,    // Diagnostic codes
        // Payload:
        //   0:2 - Diagnostic code
        //   2:X - Diagnostic data

    /* Error packets */
    PKT_CLIENT_ERROR                    = PKT_CLIENT_HEADER|0xE0,    // Error codes
        // Payload:
        //   0:2 - Error code
        //   2:X - Error data

    /* File download packets? */
    
};
typedef enum ClientPacketType_enum ClientPacketType_t;

// ----------------------------------------------------------------------------
// Server originated packets

#define PKT_SERVER_HEADER               ((UInt16)PACKET_HEADER_SERVER_BASIC << 8)
enum ServerPacketType_enum {
    
    /* End-Of-Block packets */
    PKT_SERVER_EOB_DONE                 = PKT_SERVER_HEADER|0x00,    // ""       : End of transmission, query response
        // Payload:
        //   0:1 - Optional maximum number of events to send per block
        //   1:X - Optional server key
    PKT_SERVER_EOB_SPEAK_FREELY         = PKT_SERVER_HEADER|0x01,    // ""       : End of transmission, speak freely
        // Payload:
        //   0:1 - Optional maximum number of events to send per block
        //   1:X - Optional server key
        // The client may treat this packet the same as PKT_SERVER_EOB_DONE

    /* Acknowledge packet */
    PKT_SERVER_ACK                      = PKT_SERVER_HEADER|0xA0,    // "%*u"    : Acknowledge
        // Payload:
        //   0:X - Acknowledged sequence
    /* Acknowledge packet */
    PKT_SERVER_AUTH                     = PKT_SERVER_HEADER|0xA1,    // "%*u": Authentication Success
        // Payload:
        //   0:4 - time stamp

    /* Property packets */
    PKT_SERVER_GET_PROPERTY             = PKT_SERVER_HEADER|0xB0,    // "%2u"    : Get property
        // Payload:
        //   0:2 - Property ID
    PKT_SERVER_SET_PROPERTY             = PKT_SERVER_HEADER|0xB1,    // "%2u%*b" : Set property
        // Payload:
        //   0:2 - Property ID
        //   2:X - Property value
    /* File upload packets */
    PKT_SERVER_FILE_UPLOAD              = PKT_SERVER_HEADER|0xC0,    // "%1x%3u%*b" : File upload
        // Payload:
        //   0:1 - Start/Data/End indicator
        //   1:3 - File length/offset
        //   4:X - Filename/Data/Checksum
        // The client may choose to ignore this packet

    /* Error packets */
    PKT_SERVER_ERROR                    = PKT_SERVER_HEADER|0xE0,    // "%2u"    : NAK/Error codes
        // Payload:
        //   0:2 - Error code
        //   2:2 - Offending packet header
        //   4:2 - Ofending packet type
        //   6:X - Optional error data

    /* End-Of-Transmission */
    PKT_SERVER_EOT                      = PKT_SERVER_HEADER|0xFF,    // ""       : End transmission (socket will be closed)
        // Payload:
        //  (None)

};
typedef enum ServerPacketType_enum ServerPacketType_t;

// ----------------------------------------------------------------------------

typedef struct {
    ServerPacketType_t      pktType;
    char                    *format;
    UInt16                  flags;
} ServerPacketEntry_t;

// ----------------------------------------------------------------------------

#define PACKET_MAX_ENCODED_LENGTH       256// (with excess) largest possible ASCII encoded packet

// ----------------------------------------------------------------------------

enum __attribute__ ((__packed__)) PacketPriority_enum { 
    PRIORITY_NONE       = 0, 
    PRIORITY_LOW        = 1,    // generally sent via Simplex only (GPRS)
    PRIORITY_NORMAL     = 2,    // generally sent via Duplex  only (GPRS)
    PRIORITY_HIGH       = 3     // generally sent via Duplex  only (GPRS, then Satellite)
};
typedef enum PacketPriority_enum PacketPriority_t;

// ----------------------------------------------------------------------------

#define ENCODING_BASE64_CHAR        '='
#define ENCODING_HEX_CHAR           ':'
#define ENCODING_CSV_CHAR           ','

#define ENCODING_ASCII_CKSUM_       (0x8000)
#define ENCODING_IS_CHECKSUM(E)     (((E) & ENCODING_ASCII_CKSUM_) != 0)
#define ENCODING_IS_ASCII(E)        (ENCODING_VALUE(E) != ENCODING_BINARY)
#define ENCODING_VALUE(E)           ((E) & 0x0F)
#define ENCODING_CHECKSUM(E)        (ENCODING_VALUE(E) | ENCODING_ASCII_CKSUM_)

#define ENCODING_MASK(E)            (1 << ENCODING_VALUE(E))

#define ENCODING_BINARY             0    // server must support
#define ENCODING_BINARY_MASK        ENCODING_MASK(ENCODING_BINARY)  // 0x01

#define ENCODING_BASE64             1    // server must support
#define ENCODING_BASE64_CKSUM       ENCODING_CHECKSUM(ENCODING_BASE64)
#define ENCODING_BASE64_MASK        ENCODING_MASK(ENCODING_BASE64)  // 0x02

#define ENCODING_HEX                2    // server must support
#define ENCODING_HEX_CKSUM          ENCODING_CHECKSUM(ENCODING_HEX)
#define ENCODING_HEX_MASK           ENCODING_MASK(ENCODING_HEX)     // 0x04

#define ENCODING_CSV                3    // server support optional
#define ENCODING_CSV_CKSUM          ENCODING_CHECKSUM(ENCODING_CSV)
#define ENCODING_CSV_MASK           ENCODING_MASK(ENCODING_CSV)

#define ENCODING_UNDEFINED          (0xFFFF)

#define ENCODING_REQUIRED_MASK      (ENCODING_BINARY_MASK | ENCODING_BASE64_MASK | ENCODING_HEX_MASK)
#define ENCODING_ALL_MASK           (ENCODING_REQUIRED_MASK | ENCODING_CSV_MASK)

typedef UInt16 PacketEncoding_t;

// ----------------------------------------------------------------------------
// Internal packet function error codes

#define PKTERR_NULL_PACKET          -301    // internal
#define PKTERR_BIN_PRINTF           -302    // internal: buffer overflow or invalid format
#define PKTERR_ENCODING             -303    // internal: packet encoding error
#define PKTERR_OVERFLOW             -304    // internal: buffer overflow
#define PKTERR_BIN_FORMAT_DIGIT     -331    // internal: invalid format digit
#define PKTERR_BIN_FORMAT_CHAR      -332    // internal: invalid format char
// ----------------------------------------------------------------------------

// this value is used to indicate that 'all' sent event packets are to be acknowledged
#define SEQUENCE_ALL            0xFFFFFFFFL

#define SEQUENCE_MASK(N)        ((1L << ((N) * 8)) - 1L)

// ----------------------------------------------------------------------------

typedef struct {
	UInt32		sequence;   // sequence number (MUST MATCH PACKET ENCODED SEQUENCE!!!)
	ClientPacketType_t		hdrType;    // packet type (ie. EVENT_FIXED_FORMAT_TYPE)
	PacketPriority_t		priority;   // low/normal/high priority
	UInt8		status;		// status of the packet
	UInt8		seqPos;     // sequence position
	UInt8		seqLen;     // sequence length
	UInt8		dataLen;    // length of data in payload
	char		dataFmt[(PACKET_MAX_FIELD_COUNT * 3) + 3]; // null terminated
	UInt8		data[PACKET_FULL_LENGTH - (PACKET_MAX_FIELD_COUNT + 1) * 3 - 13];
} Packet_t; // sizeof(Packet_t) == 128 bytes

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------

utBool pktIsEventPacket(ClientPacketType_t pht);

FmtBuffer_t *pktFmtBuffer(FmtBuffer_t *bf, Packet_t *pkt);

int pktInit(Packet_t *pkt, ClientPacketType_t pktType, const char *fmt, ...);
int pktVInit(Packet_t *pkt, ClientPacketType_t pktType, const char *fmt, va_list ap);

int pktEncodePacket(Buffer_t *dest, Packet_t *pkt, PacketEncoding_t encoding);

void pktPrintPacket(Packet_t *pkt, const char *msg, PacketEncoding_t encoding);


// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------

#ifdef __cplusplus
}
#endif
#endif
