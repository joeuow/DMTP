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

#ifndef _BINTOOLS_H
#define _BINTOOLS_H
#ifdef __cplusplus
extern "C" {
#endif

#include <stdarg.h>

#include "stdtypes.h"

// ----------------------------------------------------------------------------

enum BufferType_enum { BUFFER_SOURCE, BUFFER_DESTINATION };
typedef enum BufferType_enum BufferType_t;

typedef struct {
    BufferType_t    _type;          // BUFFER_SOURCE, BUFFER_DESTINATION
    UInt32          _dataPtrSize;   // original data size
    UInt8           *_dataPtr;      // original data pointer
    UInt16          _dataSize;      // #bytes dest:remaining, src:total
    UInt16          _dataLen;       // #bytes dest:filled, src:left
    UInt8           *_data;         // moving data pointer
} Buffer_t;

typedef struct {
    BufferType_t    _type;          // should always be BUFFER_DESTINATION
    UInt32          _dataPtrSize;   // original data size
    UInt8           *_dataPtr;      // original data pointer
    UInt16          _dataSize;      // #bytes dest:remaining, src:total
    UInt16          _dataLen;       // #bytes dest:filled, src:left
    UInt8           *_data;         // moving data pointer
    // The above MUST match Buffer_t
    UInt16          fmtSize;
    UInt16          fmtLen;
    UInt8           *fmtPtr;
    char            *fmt;
} FmtBuffer_t;

#define BUFFER_TYPE(B)          ((B)->_type)         // BUFFER_SOURCE, BUFFER_DESTINATION
#define BUFFER_PTR(B)           ((B)->_dataPtr)      // original buffer
#define BUFFER_PTR_SIZE(B)      ((B)->_dataPtrSize)  // original size
#define BUFFER_DATA(B)          ((B)->_data)         // moving pointer
#define BUFFER_DATA_LENGTH(B)   ((B)->_dataLen)      // #bytes dest:filled, src:remaining
#define BUFFER_DATA_SIZE(B)     ((B)->_dataSize)     // #bytes dest:remaining, src:total
#define BUFFER_DATA_INDEX(B)    ((B)->_data - (B)->_dataPtr) // next byte index
#define BUFFER_INIT(T,P,S)      { (T), (S), (P), (S), (((T)==BUFFER_SOURCE)?(S):0), (P) }

Buffer_t *binBuffer(Buffer_t *bf, UInt8 *data, UInt16 dataLen, BufferType_t bufType);
void binAdvanceBuffer(Buffer_t *bf, int len);
void binResetBuffer(Buffer_t *bf);

FmtBuffer_t *binFmtBuffer(FmtBuffer_t *bf, UInt8 *data, UInt16 dataLen, char *fmt, UInt16 fmtLen);
void binAppendFmtField(FmtBuffer_t *bf, UInt16 len, char ch);
void binAppendFmt(FmtBuffer_t *bf, const char *fmt);
void binAdvanceFmtBuffer(FmtBuffer_t *bf, int len);

// ----------------------------------------------------------------------------

int binGetMinimumInt32Size(UInt32 val, utBool isSigned);

// ----------------------------------------------------------------------------

UInt8 *binEncodeInt32(UInt8 *buf, int cnt, UInt32 val, utBool signExtend);
UInt32 binDecodeInt32(const UInt8 *buf, int cnt, utBool signExtend);

// ----------------------------------------------------------------------------

int binPrintf(UInt8 *buf, int bufLen, const char *fmt, ...);
int binBufPrintf(Buffer_t *bf, const char *fmt, ...);
int binFmtPrintf(FmtBuffer_t *bf, const char *fmt, ...);

int binVPrintf(UInt8 *buf, UInt16 bufLen, const char *fmt, va_list ap);
int binFmtVPrintf(FmtBuffer_t *bf, const char *fmt, va_list ap);

int binScanf(const UInt8 *buf, UInt16 bufLen, const char *fmt, ...);
int binVScanf(const UInt8 *buf, UInt16 bufLen, const char *fmt, va_list ap);

int binBufScanf(Buffer_t *src, const char *fmt, ...);
int binBufVScanf(Buffer_t *src, const char *fmt, va_list ap);

// ----------------------------------------------------------------------------

#ifdef __cplusplus
}
#endif
#endif
