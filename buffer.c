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
//  Circular buffer utility.
// Notes:
//  This implementation uses 'malloc' to allocate space for the buffer.
// ---
// Change History:
//  2006/01/04  Martin D. Flynn
//     -Initial release
//  2007/01/28  Martin D. Flynn
//     -WindowsCE port
//     -Added 'bufferClear'
//     -'bufferGetSize' and 'bufferGetLength' now return 'long'
//     -Fixed byte addressing problem in 'bufferPutData'
// ----------------------------------------------------------------------------

#define SKIP_TRANSPORT_MEDIA_CHECK // only if TRANSPORT_MEDIA not used in this file 
#include "defaults.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

#include "log.h"

#include "stdtypes.h"
#include "strtools.h"
#include "buffer.h"

// ----------------------------------------------------------------------------

/* create buffer */
CircleBuffer_t *bufferCreate(long size)
{
    CircleBuffer_t *cb = (CircleBuffer_t*)malloc(sizeof(CircleBuffer_t) + size + 1); // [MALLOC]
    if (cb) {
        cb->head = 0L; // next available char
        cb->tail = 0L; // fist valid char
        cb->size = size + 1L; // 1 byte is used as the marker byte at the head
    } else {
        logCRITICAL(LOGSRC,"OUT OF MEMORY!!");
    }
    return cb;
}

/* destroy buffer */
void bufferDestroy(CircleBuffer_t *cb)
{
    if (cb) {
        free((void*)cb);
    }
}

/* clear all data in buffer */
void bufferClear(CircleBuffer_t *cb)
{
    if (cb) {
        cb->head = 0L; // next available char
        cb->tail = 0L; // fist valid char
    }
}

// ----------------------------------------------------------------------------

/* return total size of buffer */
long bufferGetSize(CircleBuffer_t *cb)
{
    // return size of buffer
    return cb? cb->size - 1L : 0L;
}

/* return amount of data in buffer */
long bufferGetLength(CircleBuffer_t *cb)
{
    // return length of data currently in buffer
    if (!cb) {
        return 0;
    } else
    if (cb->head >= cb->tail) {
        return cb->head - cb->tail;
    } else {
        return cb->size - (cb->tail - cb->head);
    }
}

// ----------------------------------------------------------------------------

/* put character into buffer */
utBool bufferPutChar(CircleBuffer_t *cb, UInt8 c)
{
    
    /* save pointer to current head */
    UInt8 *p = &cb->buff[cb->head];
    
    /* advance head */
    int newHead = cb->head + 1;
    if (newHead >= cb->size) { 
        //logDEBUG(LOGSRC,"bufferPutChar", "Buffer wrap-around");
        newHead = 0; 
    }
    if (newHead == cb->tail) {
        return utFalse; // buffer overflow
    }
    
    /* save char */
    *p = c;
    cb->head = newHead;
    return utTrue;
    
}

/* get character from buffer */
int bufferGetChar(CircleBuffer_t *cb)
{
    if (cb->head == cb->tail) {
        return -1;
    } else {
        UInt8 c = cb->buff[cb->tail++];
        if (cb->tail >= cb->size) { cb->tail = 0; }
        return c;
    }
}

// ----------------------------------------------------------------------------

/* put data block into buffer */
int bufferPutData(CircleBuffer_t *cb, const void *data, int dataLen)
{
    
    /* precheck availability in buffer */
    if (dataLen >= (bufferGetSize(cb) - bufferGetLength(cb))) {
        return 0;
    }
    
    /* add data */
    int i;
    UInt8 *s = (UInt8*)data;
    for (i = 0; i < dataLen; i++) {
        bufferPutChar(cb, s[i]);
    }
    
    return dataLen;
    
}

/* get data block from buffer */
int bufferGetData(CircleBuffer_t *cb, void *data, int dataLen)
{
    if (!cb) {
        return -1;
    } else
    if (bufferGetLength(cb) <= 0) {
        return 0;
    } else {
        UInt8 *d = (UInt8*)data;
        int i = 0;
        for (;i < dataLen; i++) {
            int c = bufferGetChar(cb);
            if (c < 0) {
                // we've run out of buffer before we got to the end of the data
                memset(&d[i], 0, dataLen - i); // clear remainder of buffer
                break;
            } else {
                d[i] = (UInt8)c;
            }
        }
        return i;
    }
}

// ----------------------------------------------------------------------------

/* put null-terminated string into buffer */
utBool bufferPutString(CircleBuffer_t *cb, const char *s)
{
    
    /* precheck availability in buffer */
    int slen = strlen(s) + 1; // include terminator in length
    if (slen >= (bufferGetSize(cb) - bufferGetLength(cb))) {
        return utFalse;
    }
    
    /* add string */
    // could be optimized (most of the time this strng will not wrap around)
    for (; (*s); s++) {
        bufferPutChar(cb, *s);
    }
    
    /* add string terminator */
    bufferPutChar(cb, 0);
    
    return utTrue;
    
}

/* get null-terminated string from buffer */
int bufferGetString(CircleBuffer_t *cb, char *d, int dlen)
{
    if (bufferGetLength(cb) <= 0) {
        
        /* buffer is empty */
        return 0;
        
    } else {
        
        /* if destination string is NULL, then just throw away the next string */
        if (!d) { dlen = bufferGetSize(cb); }
        
        /* get */
        int i = 0;
        for (; i < (dlen - 1); i++) { // <-- save room for terminator
            int c = bufferGetChar(cb);
            if (c < 0) {
                // we've run out of buffer before we got to the terminator
                if (d) { d[i] = 0; } // terminate
                break;
            } else
            if (c == 0) {
                // here's the terminator
                if (d) { d[i] = 0; } // terminate
                break;
            } else {
                if (d) { d[i] = (UInt8)c; }
            }
        }
        
        /* check final termination */
        if (i >= (dlen - 1)) {
            // we've run out of space in the destination string
            if (d) { d[dlen - 1] = 0; } // make sure string is terminated
        }
        
        return strlen(d);
        
    }
}

// ----------------------------------------------------------------------------

/* copy the next string into the buffer, do not advance buffer pointers */
char *bufferCopyString(CircleBuffer_t *cb, char *d, int dlen)
{
    if (bufferGetLength(cb) <= 0) {
        
        /* buffer is empty */
        return (char*)0;
        
    } else {
        
        /* save current head/tail values */
        int oldHead = cb->head;
        int oldTail = cb->tail;
        
        /* if destination string is NULL, then just throw away the next string */
        if (!d) { dlen = bufferGetSize(cb); }
        
        /* copy */
        int i = 0;
        for (; i < (dlen - 1); i++) { // <-- save room for terminator
            int c = bufferGetChar(cb);
            if (c < 0) {
                // we've run out of buffer before we got to the terminator
                if (d) { d[i] = 0; } // terminate
                break;
            } else
            if (c == 0) {
                // here's the terminator
                if (d) { d[i] = 0; } // terminate
                break;
            } else {
                if (d) { d[i] = (UInt8)c; }
            }
        }
        
        /* check final termination */
        if (i >= (dlen - 1)) {
            // we've run out of space in the destination string
            if (d) { d[dlen - 1] = 0; } // make sure string is terminated
        }
        
        /* restore prior values */
        cb->head = oldHead;
        cb->tail = oldTail;
        
        return d;
    }
}

// ----------------------------------------------------------------------------

/* count the number of strings in the buffer */
int bufferGetStringCount(CircleBuffer_t *cb)
{
    int count = 0;
    int t = cb->tail;
    for (;t != cb->head;) {
        if (!cb->buff[t]) { count++; }
        t++;
        if (t > cb->size) { t = 0; }
    }
    return count;
}

// ----------------------------------------------------------------------------
