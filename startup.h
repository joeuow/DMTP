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

#ifndef _STARTUP_H
#define _STARTUP_H
#ifdef __cplusplus
extern "C" {
#endif

#include "defaults.h"

#include "stdtypes.h"

#include "cmderrs.h"   // for "CommandError_t"
#include "packet.h"    // for "PacketPriority_t"
#include "statcode.h"  // for "StatusCode_t"

// ----------------------------------------------------------------------------

utBool startupIsSuspended();

void startupSetSuspended(utBool suspend);

// ----------------------------------------------------------------------------
CommandError_t startupPingStatus(PacketPriority_t priority, StatusCode_t code, int ndx);
// ----------------------------------------------------------------------------

void startupMainLoopCallback();
void property_maintenance(void);
void schedule_property_save(UInt32 save_type);

#if defined(TARGET_WINCE)
int startupMain(int argc, char *argv[]);
#endif

// ----------------------------------------------------------------------------

#ifdef __cplusplus
}
#endif
#endif
