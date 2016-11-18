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

#ifndef _TRANSPORT_H
#define _TRANSPORT_H
#ifdef __cplusplus
extern "C" {
#endif

#include "defaults.h"
#include "stdtypes.h"

// ----------------------------------------------------------------------------
// Transport Media

// There are no external dependencies on these enum values, thus they may change
// at any time as new media support is added.
enum TransportMedia_enum {
    MEDIA_UNKNOWN               = 0,
    MEDIA_FILE                  = 1,
    MEDIA_SOCKET                = 2,
    MEDIA_SERIAL                = 3,
    MEDIA_GPRS                  = 4,
};

typedef enum TransportMedia_enum TransportMedia_t;

// ----------------------------------------------------------------------------
// Transport type

enum TransportType_enum {
    TRANSPORT_NONE              = 0,
    TRANSPORT_SIMPLEX           = 1,
    TRANSPORT_DUPLEX            = 2
};

typedef enum TransportType_enum TransportType_t;

// ----------------------------------------------------------------------------
// Transport attributes

typedef struct {
	TransportMedia_t media;
	int transport_error;
	void (*Init)(size_t buf_size);
	utBool (*IsOpen)(void);
	utBool (*Open)(void);
	utBool (*Close)(void);
	int (*Read)(UInt8 *buf, int bufLen);
	void (*ReadFlush)(void);
	int (*Write)(const UInt8 *buf, int bufLen);
	void (*ResetAddr)(int url_id);
} TransportFtns_t;

// ----------------------------------------------------------------------------
// Primary transport initializer
#define TRANSPORT_TCP	6
#define TRANSPORT_UDP	17
/* primary transport initialization */

// ----------------------------------------------------------------------------
// Socket transport
#if defined(TRANSPORT_MEDIA_SOCKET)
#  include "socket.h"
#endif

// ----------------------------------------------------------------------------
// File transport
#if defined(TRANSPORT_MEDIA_FILE)
#  include "transport/file.h"
#endif

// ----------------------------------------------------------------------------
// GPRS modem transport
#if defined(TRANSPORT_MEDIA_GPRS)
#  include "transport/gprs.h"
#endif

// ----------------------------------------------------------------------------
// Serial/Bluetooth transport
#if defined(TRANSPORT_MEDIA_SERIAL) || defined(SECONDARY_SERIAL_TRANSPORT)
#  include "transport/serial.h"
#endif

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------

#ifdef __cplusplus
}
#endif
#endif
