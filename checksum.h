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

#ifndef _CHECKSUM_H
#define _CHECKSUM_H
#ifdef __cplusplus
extern "C" {
#endif

#include "stdtypes.h"

// ----------------------------------------------------------------------------

#define FLETCHER_CHECKSUM_LENGTH 2 // fixed length [NOT "sizeof(ChecksumFletcher_t)"]

typedef UInt8   ChecksumXOR_t;

typedef struct {
    UInt8       C[2];
} ChecksumFletcher_t;

// ----------------------------------------------------------------------------

int cksumCalcCharXOR(const char *d, ChecksumXOR_t *cksum);
utBool cksumIsValidCharXOR(const char *d, int *len);

void _cksumResetFletcher(ChecksumFletcher_t *fcsv);
void cksumResetFletcher();

ChecksumFletcher_t *cksumGetFletcherValues(ChecksumFletcher_t *fcs);

ChecksumFletcher_t *_cksumGetFletcherChecksum(ChecksumFletcher_t *fcsv, ChecksumFletcher_t *fcs);
ChecksumFletcher_t *cksumGetFletcherChecksum(ChecksumFletcher_t *fcs);

void _cksumCalcFletcher(ChecksumFletcher_t *fcsv, const UInt8 *buf, int bufLen);
void cksumCalcFletcher(const UInt8 *buf, int bufLen);

utBool _cksumEqualsFletcher(ChecksumFletcher_t *fcsv, ChecksumFletcher_t *fcst);
utBool cksumEqualsFletcher(ChecksumFletcher_t *fcst);

// ----------------------------------------------------------------------------

#ifdef __cplusplus
}
#endif
#endif
