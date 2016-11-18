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
//  Binary encoding/decoding. 
//  Tools to encode/decode binary data.
// ---
// Change History:
//  2006/01/04  Martin D. Flynn
//     -Initial release
//  2007/01/28  Martin D. Flynn
//     -WindowsCE port.
//     -Added 'p' format to support "padded" strings.
// ----------------------------------------------------------------------------

#define SKIP_TRANSPORT_MEDIA_CHECK // only if TRANSPORT_MEDIA not used in this file 
#include "defaults.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <ctype.h>

#include "log.h"

#include "stdtypes.h"
#include "strtools.h"
#include "gpstools.h"
#include "bintools.h"

// ----------------------------------------------------------------------------

/* initialize buffer */
Buffer_t *binBuffer(Buffer_t *bf, UInt8 *data, UInt16 dataSize, BufferType_t type)
{
    if (bf) {
        BUFFER_TYPE(bf)         = type;
        BUFFER_PTR(bf)          = data;
        BUFFER_PTR_SIZE(bf)     = dataSize;
        BUFFER_DATA_SIZE(bf)    = dataSize;
        BUFFER_DATA_LENGTH(bf)  = (type == BUFFER_SOURCE)? dataSize : 0;
        BUFFER_DATA(bf)         = data;
    }
    return bf;
}

/* reset buffer to initial state */
void binResetBuffer(Buffer_t *bf)
{
    if (bf) {
        BUFFER_DATA(bf)= BUFFER_PTR(bf);
        if (BUFFER_TYPE(bf) == BUFFER_SOURCE) {
            BUFFER_DATA_LENGTH(bf) = (UInt16)BUFFER_PTR_SIZE(bf);
            BUFFER_DATA_SIZE(bf)   = (UInt16)BUFFER_PTR_SIZE(bf);
        } else
        if (BUFFER_TYPE(bf) == BUFFER_DESTINATION) {
            BUFFER_DATA_LENGTH(bf) = (UInt16)0;
            BUFFER_DATA_SIZE(bf)   = (UInt16)BUFFER_PTR_SIZE(bf);
        }
    }
}

/* advance buffer pointer by 'len' bytes */
void binAdvanceBuffer(Buffer_t *bf, int len)
{
    // data is being removed from this buffer
    if (bf && (len > 0)) {
        BUFFER_DATA(bf) += len; // move pointer to next field
        if (BUFFER_TYPE(bf) == BUFFER_SOURCE) {
            // data is being removed from this buffer
            // 'dataLen' is the number of bytes still available in buffer
            if (len > BUFFER_DATA_LENGTH(bf)) { len = BUFFER_DATA_LENGTH(bf); }
            BUFFER_DATA_LENGTH(bf) -= len; // remaining length is being decreased
        } else
        if (BUFFER_TYPE(bf) == BUFFER_DESTINATION) {
            // data is being added to this buffer
            // 'dataLen' is the number of bytes we've placed in the buffer
            // 'dataSize' is the number of bytes we have left to place data
            if (len > BUFFER_DATA_SIZE(bf)) { len = BUFFER_DATA_SIZE(bf); }
            BUFFER_DATA_LENGTH(bf) += len; // length is being increased
            BUFFER_DATA_SIZE(bf)   -= len; // remaining size is being decreased
        }
    }
}

// ----------------------------------------------------------------------------

/* initialize buffer */
FmtBuffer_t *binFmtBuffer(FmtBuffer_t *fb, UInt8 *data, UInt16 dataSize, char *fmt, UInt16 fmtSize)
{
    if (fb) {
        binBuffer((Buffer_t*)fb, data, dataSize, BUFFER_DESTINATION); // always destination
        fb->fmtSize  = fmtSize;
        fb->fmtLen   = 0;
        fb->fmtPtr   = (UInt8*)fmt;
        fb->fmt      = fmt;
    }
    return fb;
}

// ----------------------------------------------------------------------------

void binAppendFmtField(FmtBuffer_t *fb, UInt16 len, char ch)
{
    if (fb && fb->fmt && (fb->fmtSize >= 6)) {
        sprintf(fb->fmt, "%%%d%c", len, ch);
        int slen     = strlen(fb->fmt);
        fb->fmt     += slen;
        fb->fmtLen  += slen;
        fb->fmtSize -= slen;
    }
}

void binAppendFmt(FmtBuffer_t *fb, const char *fmt)
{
    int fmtLen = strlen(fmt);
    if (fb && fb->fmt && (fb->fmtSize >= fmtLen)) {
        strcpy(fb->fmt, fmt);
        fb->fmt     += fmtLen;
        fb->fmtLen  += fmtLen;
        fb->fmtSize -= fmtLen;
    }
}

// ----------------------------------------------------------------------------

/* advance buffer pointer by 'len' bytes */
void binAdvanceFmtBuffer(FmtBuffer_t *fb, int len)
{
    binAdvanceBuffer((Buffer_t*)fb, len);
}

// ----------------------------------------------------------------------------

// return the minimum number of bytes that will accuately represent this value
// The return value will be at-least 1, and at most 4
int binGetMinimumInt32Size(UInt32 val, utBool isSigned)
{
    UInt8 mask = (isSigned && (val & 0x80000000L))? 0xFF : 0x00;
    int n;
    for (n = 3; n >= 1; n--) { // <-- only look at the 3 most-significant bytes
        UInt8 x = (UInt8)((val >> (n * 8)) & 0xFF);
        if (x != mask) { break; }
    }
    // 'n' will be '0' if no 'mismatch' was found, thus returning '1'
    return n + 1;
}

// ----------------------------------------------------------------------------

/* encode 32-bit value into byte array (Big-Endian) */
UInt8 *binEncodeInt32(UInt8 *buf, int cnt, UInt32 val, utBool signExtend)
{
    // - This function places the least-significant bytes of the value 'val' into the
    // byte array 'buf'. It is left to the developer to ensure that the size of the
    // buffer is large enough to accomodate the value stored in 'val'.
    // - 'signExtend' is needed only if 'cnt' is > 4.
    if (buf && (cnt > 0)) {
        
        /* fill excess space */
        if (cnt > 4) {
            UInt8 fill = (signExtend && (val & 0x80000000L))? 0xFF : 0x00;
            memset(buf, cnt - 4, fill);
            buf += cnt - 4;
            cnt = 4;
        }

        /* copy in value */
        int n;
        for (n = cnt - 1; n >= 0; n--) {
            buf[n] = (UInt8)(val & 0xFF);
            val >>= 8;
        }
        
    }
    
    return buf;
}

/* decode byte array into 32-bit value (Native-Endian) */
UInt32 binDecodeInt32(const UInt8 *buf, int cnt, utBool signExtend)
{
    // This function decodes the numeric value in 'buf' and returns the resulting
    // value as a 32-bit unsigned integer.  If 'signExtend' is true, then sign
    // extension will be performed and the returned value can be cast to a signed
    // integer to obtain a signed value.
    if (buf && (cnt > 0)) {
        UInt32 val = (signExtend && (buf[0] & 0x80))? -1L : 0L;
        int n;
        for (n = 0; n < cnt; n++) {
            val = (val << 8) | buf[n];
        }
        return val;
    } else {
        return 0L;
    }
}

// ----------------------------------------------------------------------------
// Binary format:
//      %<length><type>
// Length:
//      0..X - fixed field length specifier
//      *    - variable field length specifier (argument MUST be of type 'int')
// Valid types:
//      i - signed integer (argument MUST be of type UInt32 or Int32)
//      u - unsigned integer (argument MUST be of type UInt32 or Int32)
//      x - unsigned integer (argument MUST be of type UInt32 or Int32)
//      s - null terminated string (argument must be a pointer to a null-terminated string)
//      b - fixed length binary (argument must be a pointer to a byte array)
//      g - gps point (argument MUST be a pointer to a GPSShort_t structure)
//      z - zero-filled field (no argument)
// Example:
//      This will place the 2 LSBs of 'a', followed by at-most the first 5 bytes of 's'
//      into the specified buffer:
//          binPrintf(buf, sizeof(buf), "%2u%*s", (UInt32)a, 5, s);
// Notes:
//      - The compiler won't validate the format against the actual argument types as it
//      does with the 'printf' and 'scanf' functions, so special care must be taken to
//      insure that all numeric arguments are always specified as 32-bit variables, and 
//      lengths are specified as 16-bit variables.

int binPrintf(UInt8 *buf, int bufSize, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    FmtBuffer_t bb,*fb=binFmtBuffer(&bb,buf,bufSize,(char*)0,0); // BUFFER_DESTINATION
    int len = binFmtVPrintf(fb, fmt, ap);
    va_end(ap);
    return len;
}

int binBufPrintf(Buffer_t *buf, const char *fmt, ...)
{
    if (buf && (BUFFER_TYPE(buf) == BUFFER_DESTINATION)) {
        va_list ap;
        va_start(ap, fmt);
        FmtBuffer_t bb,*fb=binFmtBuffer(&bb,BUFFER_DATA(buf),BUFFER_DATA_SIZE(buf),(char*)0,0);
        int len = binFmtVPrintf(fb, fmt, ap);
        if (len >= 0) {
            binAdvanceBuffer(buf, len);
        }
        va_end(ap);
        return len;
    } else {
        logERROR(LOGSRC,"Buffer is not designated for BUFFER_DESTINATION\n");
        return -1;
    }
}

int binFmtPrintf(FmtBuffer_t *fb, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    int len = binFmtVPrintf(fb, fmt, ap);
    va_end(ap);
    return len;
}

int binVPrintf(UInt8 *buf, UInt16 bufSize, const char *fmt, va_list ap)
{
    FmtBuffer_t bb, *fb = binFmtBuffer(&bb, buf, bufSize, (char*)0, 0);
    return binFmtVPrintf(fb, fmt, ap);
}

int binFmtVPrintf(FmtBuffer_t *dest, const char *fmt, va_list ap)
{
    UInt16 startingDataLen = BUFFER_DATA_LENGTH(dest);
    const char *v = fmt? fmt : ""; // 'v' must not be null
    
    /* valid data pointer? */
    if (!dest || !BUFFER_DATA(dest)) {
        logERROR(LOGSRC,"Buffer is null\n");
        return -1;
    }
    
    /* loop through formats */
    int f = 0;
    for (f = 0; ;f++) {
        
        /* find start of next format */
        while (*v && (*v != '%')) { v++; }
        if (!*v) {
            break; 
        }
        v++;
        
        /* 1+ digits must follow */
        int len = 0;
        if (*v == '*') {
            // pull the field length off the stack
            len = va_arg(ap, int);
            v++;
        } else
        if (isdigit(*v)) {
            // the field length is explicitly specified in the format
            const char *d = v;
            while (*d && isdigit(*d)) { d++; }
            len = atoi(v);
            v = d;
        } else {
            // invalid format (not a digit)
            logERROR(LOGSRC,"Invalid format at #%d (not a digit)\n", f);
            return -1;
        }

        /* check length */
        if (len > BUFFER_DATA_SIZE(dest)) {
            // overflow
            logERROR(LOGSRC,"Overflow at %d [%d > %d]\n", f, len, BUFFER_DATA_SIZE(dest));
            return -1;
        }

        /* make sure the field is cleared */
        memset(BUFFER_DATA(dest), 0, len);

        /* next char must be one of 'iuxsbgz' */
        UInt32 i;
        const char *s;
        utBool padString = utFalse;
        const UInt8 *b;
        const GPSPoint_t *gp;
        switch (*v) {
            case 'x': case 'X': // unsigned
            case 'u': case 'U': // unsigned
            case 'i': case 'I': // signed
                i = va_arg(ap, UInt32);
                if (len < sizeof(UInt32)) {
                    i &= (1L << (len*8)) - 1L;
                }
                binEncodeInt32(BUFFER_DATA(dest), len, i, (((*v == 'i')||(*v == 'I'))? utTrue : utFalse));
                binAppendFmtField(dest, len, *v);
                binAdvanceFmtBuffer(dest, len);
                break;
            case 'p': case 'P': // padded strings
            case 's': case 'S': // strings
                s = va_arg(ap, const char*);
                padString = ((*v == 'p') || (*v == 'P'))? utTrue : utFalse;
                if (s) {
                    char *d = (char*)BUFFER_DATA(dest);
                    int actualLen = strLength(s, len); // MIN(strlen(s),len)
                    memcpy(d, s, actualLen);
                    if (actualLen < len) {
                        if (padString) {
                            // pad short strings with spaces
                            for (;actualLen < len; actualLen++) {
                                d[actualLen] = ' '; // pad with space
                            }
                        } else {
                            // include a string terminator
                            d[actualLen++] = 0;
                            len = actualLen;
                        }
                    }
                } else {
                    if (padString) {
                        // fill the field with spaces
                        char *d = (char*)BUFFER_DATA(dest);
                        int actualLen = 0;
                        for (;actualLen < len; actualLen++) {
                            d[actualLen] = ' '; // pad with space
                        }
                    } else {
                        // no string
                        len = 0;
                    }
                }
                binAppendFmtField(dest, len, *v);
                binAdvanceFmtBuffer(dest, len);
                break;
            case 'b': case 'B': // binary
                b = va_arg(ap, const UInt8*);
                if (b) {
                    memcpy(BUFFER_DATA(dest), b, len); // copy
                } else {
                    memset(BUFFER_DATA(dest), 0, len); // clear
                }
                binAppendFmtField(dest, len, *v);
                binAdvanceFmtBuffer(dest, len);
                break;
            case 'g': case 'G':
                gp = va_arg(ap, const GPSPoint_t*);
                if (gp) {
                    if ((len >= 6) && (len < 8)) {
                        gpsPointEncode6(BUFFER_DATA(dest), gp);
                    } else
                    if (len >= 8) {
                        gpsPointEncode8(BUFFER_DATA(dest), gp);
                    } else {
                        // invalid format
                    }
                }
                binAppendFmtField(dest, len, *v);
                binAdvanceFmtBuffer(dest, len);
                break;
            case 'z': case 'Z': // zero filler
                binAppendFmtField(dest, len, *v);
                binAdvanceFmtBuffer(dest, len);
                break;
            default:
                // invalid format (unrecognized type)
                logERROR(LOGSRC,"Invalid format (unrecognized type)\n");
                return -1;
        }
        
    }
    
    /* return number of bytes written to buffer in this function */
    return BUFFER_DATA_LENGTH(dest) - startingDataLen;
    
}

// ----------------------------------------------------------------------------

int binScanf(const UInt8 *buf, UInt16 bufLen, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    int rtn = binVScanf(buf, bufLen, fmt, ap);
    va_end(ap);
    return rtn;
}

int binVScanf(const UInt8 *buf, UInt16 bufLen, const char *fmt, va_list ap)
{
    Buffer_t bb, *src = binBuffer(&bb, (UInt8*)buf, bufLen, BUFFER_SOURCE);
    return binBufVScanf(src, fmt, ap);
}

int binBufScanf(Buffer_t *src, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    int rtn = binBufVScanf(src, fmt, ap);
    va_end(ap);
    return rtn;
}

int binBufVScanf(Buffer_t *src, const char *fmt, va_list ap)
{
    const char *v = fmt;
    int fldCnt;
    for (fldCnt = 0; (BUFFER_DATA_LENGTH(src) > 0); fldCnt++) {
        
        /* find start of next format */
        while (*v && (*v != '%')) { v++; }
        if (!*v) {
            // no more fields
            break; 
        }
        v++;
        
        /* 1+ digits must follow */
        int len = 0;
        if (*v == '*') {
            // pull the field length off the stack
            len = va_arg(ap, int);
            v++;
        } else
        if (isdigit(*v)) {
            // the field length is explicitly specified in the format
            const char *d = v;
            while (*d && isdigit(*d)) { d++; }
            len = atoi(v);
            v = d;
        } else {
            // invalid format (not a digit)
            logERROR(LOGSRC,"Invalid format at #%d (not a digit)\n", fldCnt);
            return -1;
        }

        /* check length */
        if (len > BUFFER_DATA_LENGTH(src)) {
            // overflow
            //return -1;
            len = BUFFER_DATA_LENGTH(src); // use whatever we have left
        }
        
        /* next char must be one of 'iuxsbgz' */
        UInt32 *i;
        char *s;
        UInt8 *b;
        GPSPoint_t *gp;
        int actualLen = 0;
        switch (*v) {
            case 'x': case 'X': // unsigned
            case 'u': case 'U': // unsigned
            case 'i': case 'I': // signed
                i = va_arg(ap, UInt32*);
                if (i) {
                    *i = binDecodeInt32(BUFFER_DATA(src), len, (((*v == 'i')||(*v == 'I'))? utTrue : utFalse));
                }
                binAdvanceBuffer(src, len);
                break;
            case 's': case 'S': // strings
            case 'p': case 'P': // padded strings
                s = va_arg(ap, char*);
                actualLen = strLength((char*)BUFFER_DATA(src), len); // may be less than 'len'
                if (s) {
                    sprintf(s, "%.*s", actualLen, BUFFER_DATA(src)); // copy and terminate
                    // ??? trim trailing space?
                }
                binAdvanceBuffer(src, ((actualLen<len)?actualLen+1:len)); // incl terminator
                break;
            case 'b': case 'B':
                b = va_arg(ap, UInt8*);
                if (b) { 
                    memcpy(b, BUFFER_DATA(src), len); 
                }
                binAdvanceBuffer(src, len);
                break;
            case 'g': case 'G':
                gp = va_arg(ap, GPSPoint_t*);
                if (gp) {
                    if ((len >= 6) && (len < 8)) {
                        gpsPointDecode6(gp, BUFFER_DATA(src));
                    } else
                    if (len >= 8) {
                        gpsPointDecode8(gp, BUFFER_DATA(src));
                    } else {
                        // invalid format
                    }
                }
                binAdvanceBuffer(src, len);
                break;
            case 'z': case 'Z': // zero filled
                binAdvanceBuffer(src, len);
                break;
            default:
                // invalid format (unrecognized type)
                logERROR(LOGSRC,"Invalid format at #%d (unrecognized type)\n", fldCnt);
                return -1;
        }

    }
    return fldCnt;
}

// ----------------------------------------------------------------------------
