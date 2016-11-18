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
//  Transport source compile manager
// Notes:
//  - This module includes the proper transport media source code
//  - 1, and only 1 "TRANSPORT_MEDIA_xxxxx" should be defined.  This should
//    already be guaranteed by 'defaults.h'.
// ---
// Change History:
//  2007/07/13  Martin D. Flynn
//     -Initial release
//  2007/01/28  Martin D. Flynn
//     -WindowsCE port
// ----------------------------------------------------------------------------

#include "defaults.h"
#include <stdlib.h>
#include "transport.h"

// ----------------------------------------------------------------------------
// set source file include type
#ifdef __cplusplus
#  define _TRANSPORT_SOURCE_CPP
#endif

// ----------------------------------------------------------------------------
// File transport
#if defined(TRANSPORT_MEDIA_FILE)
#  ifdef _TRANSPORT_SOURCE_CPP
#    include "transport/file.cpp"
#  else
#    include "transport/file.c"
#  endif
#endif

// ----------------------------------------------------------------------------
// Socket transport
#if defined(TRANSPORT_MEDIA_SOCKET)
#endif

// ----------------------------------------------------------------------------
// Serial/Bluetooth transport
#if defined(TRANSPORT_MEDIA_SERIAL) || defined(SECONDARY_SERIAL_TRANSPORT)
#  ifdef _TRANSPORT_SOURCE_CPP
#    include "transport/serial.cpp"
#  else
#    include "transport/serial.c"
#  endif
#endif

// ----------------------------------------------------------------------------
// GPRS modem transport
#if defined(TRANSPORT_MEDIA_GPRS)
#  ifdef _TRANSPORT_SOURCE_CPP
#    include "transport/gprs.cpp"
#  else
#    include "transport/gprs.c"
#  endif
#endif

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// Undefine _TRANSPORT_SOURCE_CPP
#ifdef _TRANSPORT_SOURCE_CPP
#  undef _TRANSPORT_SOURCE_CPP
#endif

// ----------------------------------------------------------------------------
