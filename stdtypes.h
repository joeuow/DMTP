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

#ifndef _STD_TYPES_H
#define _STD_TYPES_H
#ifdef __cplusplus
extern "C" {
#endif

// ----------------------------------------------------------------------------

// comment the following if the platform does not support 64-bit integers
// TARGET_WINCE platforms may not support 64-bit integers
#if defined(TARGET_LINUX) || defined(TARGET_CYGWIN) || defined(TARGET_GUMSTIX)
#  define SUPPORT_UInt64
#include <stdint.h>
#include <stdbool.h>
#endif

// ----------------------------------------------------------------------------

/*typedef unsigned int        UInt;

typedef signed char         Int8;
typedef unsigned char       UInt8;
typedef signed short        Int16;
typedef unsigned short      UInt16;
typedef signed long         Int32;
typedef unsigned long       UInt32;

typedef unsigned short      UInt16Pack;   // packed bit fields */
typedef unsigned int	UInt;
typedef int8_t		Int8;
typedef uint8_t		UInt8;
typedef int16_t		Int16;
typedef uint16_t	UInt16;
typedef int32_t		Int32;
typedef uint32_t	UInt32;
typedef uint16_t	UInt16Pack;   // packed bit fields

#if defined(SUPPORT_UInt64)
#if defined(TARGET_WINCE)
typedef __int64             Int64;
typedef unsigned __int64    UInt64;
#else
/*typedef signed long long    Int64;
typedef unsigned long long  UInt64; */
typedef int64_t		Int64;
typedef uint64_t	UInt64;
#endif
#endif

// ----------------------------------------------------------------------------

enum utBool_enum { 
    utFalse = 0, 
    utTrue  = 1 
};
typedef enum utBool_enum utBool;

// ----------------------------------------------------------------------------
#ifdef __cplusplus
}
#endif
#endif
