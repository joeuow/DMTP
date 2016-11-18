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
//  Checksum calculations. 
//  Tools to create/verify checksums.
// ---
// Change History:
//  2006/01/04  Martin D. Flynn
//     -Initial release
//  2007/01/28  Martin D. Flynn
//     -WindowsCE port
// ----------------------------------------------------------------------------

#define SKIP_TRANSPORT_MEDIA_CHECK // only if TRANSPORT_MEDIA not used in this file 
#include "defaults.h"

#include "stdtypes.h"
#include "strtools.h"
#include "checksum.h"

// ----------------------------------------------------------------------------

#define ASCII_ENCODING_CHAR     '$'
#define CHECKSUM_SEPARATOR      '*'

// ----------------------------------------------------------------------------

/* return true if the checksum is valid */
// the locaton of the '*' is placed in *len
utBool cksumIsValidCharXOR(const char *d, int *len)
{
    UInt8 cksum = 0x00;
    int _len = 0;
    if (!len) { len = &_len; }
    
    /* calculate checksum */
    if (*d == ASCII_ENCODING_CHAR) {
        *len = cksumCalcCharXOR(d + 1, &cksum) + 1;
    } else {
        *len = cksumCalcCharXOR(d, &cksum);
    }
    
    /* test checksum */
    if (!d[*len]) {
        // checksum is automatically valid if it is not present
        return utTrue;
    } else { // (d[*len] == '*') is assumed
        // test checksum for match
        UInt8 found = 0x00;
        int hlen = strParseHex(&d[*len + 1], 2, &found, 1);
        return ((hlen == 1) && (found == cksum))? utTrue : utFalse;
    }
}

/* calculate the checksum for the specified string */
// the checksum is returned in *cksum
int cksumCalcCharXOR(const char *d, ChecksumXOR_t *cksum) 
{
    // 'd' points to the first character after the prefix
    if (d && *d) {
        const char *cs = d;
        ChecksumXOR_t ck = *cs++;
        for (; *cs && (*cs != CHECKSUM_SEPARATOR); cs++) { 
            ck ^= *cs; 
        }
        *cksum = ck & 0xFF;
        return cs - d;
    } else {
        *cksum = 0x00;
        return 0;
    }
}

// ----------------------------------------------------------------------------

static ChecksumFletcher_t    fletcherCalc = { { 0, 0 } };

void _cksumResetFletcher(ChecksumFletcher_t *fcsv)
{
    fcsv->C[0] = 0;
    fcsv->C[1] = 0;
}

void cksumResetFletcher()
{
    _cksumResetFletcher(&fletcherCalc);
}

ChecksumFletcher_t *cksumGetFletcherValues(ChecksumFletcher_t *fcs)
{
    fcs->C[0] = fletcherCalc.C[0];
    fcs->C[1] = fletcherCalc.C[1];
    return fcs;
}

// ----------------------------------------------------------------------------

ChecksumFletcher_t *_cksumGetFletcherChecksum(ChecksumFletcher_t *fcsv, ChecksumFletcher_t *fcs)
{
    fcs->C[0] = fcsv->C[0] - fcsv->C[1];
    fcs->C[1] = fcsv->C[1] - (fcsv->C[0] * 2);
    return fcs;
}

ChecksumFletcher_t *cksumGetFletcherChecksum(ChecksumFletcher_t *fcs)
{
    fcs->C[0] = fletcherCalc.C[0] - fletcherCalc.C[1];
    fcs->C[1] = fletcherCalc.C[1] - (fletcherCalc.C[0]*2);
    return fcs;
}

// ----------------------------------------------------------------------------

void _cksumCalcFletcher(ChecksumFletcher_t *fcsv, const UInt8 *buf, int bufLen)
{
    int i;
    for (i = 0; i < bufLen; i++) {
        fcsv->C[0] += buf[i];
        fcsv->C[1] += fcsv->C[0];
    }
}

void cksumCalcFletcher(const UInt8 *buf, int bufLen)
{
    _cksumCalcFletcher(&fletcherCalc, buf, bufLen);
}

// ----------------------------------------------------------------------------

utBool _cksumEqualsFletcher(ChecksumFletcher_t *fcsCalc, ChecksumFletcher_t *fcsTest)
{
    ChecksumFletcher_t fcsa;
    _cksumGetFletcherChecksum(&fcsa, fcsCalc);
    return ((fcsa.C[0] == fcsTest->C[0]) && (fcsa.C[1] == fcsTest->C[1]))? utTrue : utFalse;
}

utBool cksumEqualsFletcher(ChecksumFletcher_t *fcst)
{
    return _cksumEqualsFletcher(&fletcherCalc, fcst);
}

// ----------------------------------------------------------------------------
