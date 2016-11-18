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

#ifndef _ACCTING_H
#define _ACCTING_H
#ifdef __cplusplus
extern "C" {
#endif

#include "defaults.h"

#include "stdtypes.h"

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------

#if !defined(TRANSPORT_MEDIA_FILE) && !defined(TRANSPORT_MEDIA_SERIAL)
#define MAX_MASK_SIZE               8 // 8 * 30 minutes == 4 hours
typedef struct {
    TimerSec_t      shiftTime;
    TimerSec_t      lastConnTime;
    UInt32          mask[MAX_MASK_SIZE];    // 4 hour max
} ConnectionMask_t;
#endif

// ----------------------------------------------------------------------------

utBool acctSetDuplexConnection();
utBool acctSetSimplexConnection();

utBool acctAbsoluteDelayExpired();
utBool acctMinIntervalExpired();
utBool acctMaxIntervalExpired();

utBool acctHasQuota();

utBool acctUnderTotalQuota();
utBool acctUnderDuplexQuota();

utBool acctSupportsDuplex();
utBool acctSupportsSimplex();

void acctInitialize();

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------

#ifdef __cplusplus
}
#endif
#endif
