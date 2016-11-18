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
//  Pseudo random number generator 
// ---
// Change History:
//  2007/01/28  Martin D. Flynn
//      WindowsCE port
// ----------------------------------------------------------------------------

#define SKIP_TRANSPORT_MEDIA_CHECK // only if TRANSPORT_MEDIA not used in this file 
#include "defaults.h"
#if defined(SUPPORT_UInt64) // currently 'random' only supports 64-bit architectures

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <ctype.h>
//#include <unistd.h>

#include "log.h"

#include "stdtypes.h"
#include "utctools.h"
#include "random.h"

// ----------------------------------------------------------------------------
// For some reasone Windows eVC++ compiler will not recognize 'LL' as a valid 
// suffix to define 64-bit integer constants!  Because of this 64-bit integers
// are defined a 32-bit integer constants (ie. "3L" instead of "3LL") and we
// let the compiler cast it into a 64-bit quantity where possible.
// ----------------------------------------------------------------------------

typedef struct 
{
    UInt64  IA;     // multiplier
    UInt64  IC;     // addend
    UInt64  IM;     // modulo
} RandomGenerator_t;

// Taken from Numerical Recipies p211.
static RandomGenerator_t ranGen[] = {
  //    IA        IC          IM
    {  2416L,   374441L,   1771875L },
    { 84589L,    45989L,    217728L }, 
    { 17221L,   107839L,    510300L },
    {  7141L,    54773L,    259200L },
};

#define RAND_BITS           16
#define RAND_MASK           ((((UInt64)1L) << RAND_BITS) - ((UInt64)1L))

static UInt64   nextRandom[4] = { 1L, 1L, 1L, 1L };
static UInt64   nextRandomCount = 0L;

static UInt64 _randomBits(int ndx)
{
    if (ndx < 0) { ndx = (int)(nextRandomCount++ % 4); }
    RandomGenerator_t *rg = &ranGen[ndx];
    nextRandom[ndx] = (((nextRandom[ndx] * rg->IA) + rg->IC) % rg->IM) & RAND_MASK;
    return nextRandom[ndx];
}

void randomSeed(UInt64 seed)
{
    int i;
    for (i = 0; i < 4; i++) {
        nextRandom[i] = ((seed >> (i * 16)) & ((UInt64)0xFFFFL)) & RAND_MASK;
    }
    nextRandomCount = 0L;
}

UInt64 randomBits(int bits)
{
    UInt64 r = 0L;
    int b = 0;
    
    /* limit bit size */
    if (bits > 64) { 
        bits = 64; 
    }
    
    /* fill random accumulator */
    while (b < bits) {
        r = (r << RAND_BITS) | _randomBits(-1);
        b += RAND_BITS;
    }
    
    /* extract requested bits */
    if (b > bits) { 
        r = (r >> (b - bits)) & ((((UInt64)1L) << bits) - ((UInt64)1L)); 
    }
    
    return r;
}

UInt16 randomNext16(UInt16 low, UInt16 high)
{
    int bits = 16;
    UInt16 delta = high - low;
    return low + (UInt16)((double)delta * (double)randomBits(bits) / (double)(1 << bits));
}

UInt32 randomNext32(UInt32 low, UInt32 high)
{
    int bits = 32;
    UInt32 delta = high - low;
    return low + (UInt32)((double)delta * (double)randomBits(bits) / (double)(1 << bits));
}

// ----------------------------------------------------------------------------
#endif // defined(SUPPORT_UInt64)
