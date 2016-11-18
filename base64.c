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
//  Base64 encoding/decoding. 
// ---
// Change History:
//  2006/01/04  Martin D. Flynn
//     -Initial release
//  2007/01/28  Martin D. Flynn
//     -WindowsCE port
// ----------------------------------------------------------------------------

#define SKIP_TRANSPORT_MEDIA_CHECK // only if TRANSPORT_MEDIA not used in this file 
#include "defaults.h"

#include <stdio.h>
#include <stdlib.h>

#include "base64.h"

// ----------------------------------------------------------------------------
    
static UInt8 Base64Map[] = {
    'A','B','C','D','E','F','G','H','I','J','K','L','M',
    'N','O','P','Q','R','S','T','U','V','W','X','Y','Z',
    'a','b','c','d','e','f','g','h','i','j','k','l','m',
    'n','o','p','q','r','s','t','u','v','w','x','y','z',
    '0','1','2','3','4','5','6','7','8','9','+','/'
};

static UInt8 *_b64GetMap()
{
    return Base64Map;
}

static int _b64IndexOf(UInt8 ch) 
{
    if ((ch >= 'A') && (ch <= 'Z')) { return  0 + (ch - 'A'); }
    if ((ch >= 'a') && (ch <= 'z')) { return 26 + (ch - 'a'); }
    if ((ch >= '0') && (ch <= '9')) { return 52 + (ch - '0'); }
    if (ch == '+')                  { return 62;              }
    if (ch == '/')                  { return 63;              }
    return 0; // invalid character found (map it to '0')
}

// ----------------------------------------------------------------------------

long base64Encode(char *b64Out, long b64OutLen, const UInt8 *dataIn, long dataInLen)
{
    
    /* calc length of output */
    // 'minOutlen' is the required output length
    long minOutlen = ((dataInLen + 2L) / 3L) * 4L;
    if (b64OutLen < minOutlen) {
        // not enough space to encode data
        return -1L; 
    }

    /* encode */
    long i, d = 0L;
    UInt8 *b64Map = _b64GetMap();
    for (i = 0L; (i < dataInLen) /*&& ((d + 4L) <= b64OutLen)*/; i += 3L) {

        /* place next 3 bytes into register */
        UInt32                  reg24  = ((UInt32)dataIn[i+0L] << 16) & 0xFF0000;
        if ((i+1L)<dataInLen) { reg24 |= ((UInt32)dataIn[i+1L] <<  8) & 0x00FF00; }
        if ((i+2L)<dataInLen) { reg24 |= ((UInt32)dataIn[i+2L]      ) & 0x0000FF; }
        
        /* encode data 6 bits at a time */
        b64Out[d++] =                     b64Map[(reg24 >> 18) & 0x3F];
        b64Out[d++] =                     b64Map[(reg24 >> 12) & 0x3F];
        b64Out[d++] = ((i+1L)<dataInLen)? b64Map[(reg24 >>  6) & 0x3F] : BASE64_PAD;
        b64Out[d++] = ((i+2L)<dataInLen)? b64Map[(reg24      ) & 0x3F] : BASE64_PAD;
        
    }
    
    /* terminate an return length */
    b64Out[d] = 0;
    return d;
    
}

// ----------------------------------------------------------------------------
    
/* decode string */
long base64Decode(const char *b64In, long b64InLen, UInt8 *dataOut, long dataOutlen)
{
    
    /* remove padding */
    while ((b64InLen > 0) && (b64In[b64InLen - 1] == BASE64_PAD)) { b64InLen--; }
    
    /* calc minimum length of output */
    // (1)XX==, (2)XXX=, (3)XXXX, (4)XXXXXX==
    long minOutlen = (((b64InLen - 1L) / 4L) * 3L) + ((b64InLen - 1L) % 4L);
    if (((b64InLen - 1L) % 4L) == 0L) {
        // the encoded Base64 String has an invalid length
        minOutlen++;
    }
    // 'minOutlen' is the required output length
    // 1=?0, 2=1, 3=2, 4=3, 5=?3, 6=4, 7=5, 8=6, 9=?6, 10=7
    if (dataOutlen < minOutlen) {
        // not enough space to decode stream
        return -1L;
    }
    
    /* decode */
    long i, d = 0L;
    for (i = 0L; (i < b64InLen) /*&& ((d + 3) <= dataOutlen)*/; i += 4L) {
        
        /* place next 3 bytes into register */
        UInt32                 reg24  = (_b64IndexOf(b64In[i+0L]) << 18) & 0xFC0000;
        if ((i+1L)<b64InLen) { reg24 |= (_b64IndexOf(b64In[i+1L]) << 12) & 0x03F000; }
        if ((i+2L)<b64InLen) { reg24 |= (_b64IndexOf(b64In[i+2L]) <<  6) & 0x000FC0; }
        if ((i+3L)<b64InLen) { reg24 |= (_b64IndexOf(b64In[i+3L])      ) & 0x00003F; }

                               dataOut[d++] = (UInt8)((reg24 >> 16) & 0xFF);
        if ((i+2L)<b64InLen) { dataOut[d++] = (UInt8)((reg24 >>  8) & 0xFF); }
        if ((i+3L)<b64InLen) { dataOut[d++] = (UInt8)((reg24      ) & 0xFF); }

    }

    return d;
    
}

// ----------------------------------------------------------------------------
