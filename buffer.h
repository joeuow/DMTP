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

#ifndef _BUFFER_H
#define _BUFFER_H
#ifdef __cplusplus
extern "C" {
#endif

#include "stdtypes.h"

// ----------------------------------------------------------------------------

typedef struct
{
    long                head;
    long                tail;
    long                size;
    UInt8               buff[1]; // Len='0' is preferred, but some compiler don't allow this
} CircleBuffer_t;

// ----------------------------------------------------------------------------

CircleBuffer_t *bufferCreate(long size);
long bufferGetSize(CircleBuffer_t *cb);
long bufferGetLength(CircleBuffer_t *cb);
void bufferClear(CircleBuffer_t *cb);

utBool bufferPutChar(CircleBuffer_t *cb, UInt8 c);
int bufferGetChar(CircleBuffer_t *cb);

int bufferPutData(CircleBuffer_t *cb, const void *data, int dataLen);
int bufferGetData(CircleBuffer_t *cb, void *data, int dataLen);

utBool bufferPutString(CircleBuffer_t *cb, const char *s);
int bufferGetString(CircleBuffer_t *cb, char *d, int dlen);
char *bufferCopyString(CircleBuffer_t *cb, char *d, int dlen);
int bufferGetStringCount(CircleBuffer_t *cb);

// ----------------------------------------------------------------------------

#ifdef __cplusplus
}
#endif
#endif
