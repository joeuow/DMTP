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

#ifndef _OS_H
#define _OS_H
#ifdef __cplusplus
extern "C" {
#endif

#include "defaults.h"

// ----------------------------------------------------------------------------

#define DEFAULT_HOSTNAME        "hostdmtp"

// ----------------------------------------------------------------------------

const char *osGetHostname(char *host, int hostLen);
utBool osSetHostname(const char *s);

utBool osExec(char *cmd);
void osStartConsole(int port);

utBool osReboot();

utBool osHasBluetooth();
Int64 osGetBluetoothMac();
const char *osGetBluetoothMacID(char *ser);
utBool osSetBluetoothName(const char *name);

UInt64 osGetSerialNumber();
const char *osGetSerialNumberID();

const char *osGetUniqueID();
const char *_osGetUniqueID(utBool useBluetooth);

Int32 osGetDiskUsage(const char *mount, UInt32 *total, UInt32 *avail);
char *osGetOSInfo();

// ----------------------------------------------------------------------------

#ifdef __cplusplus
}
#endif
#endif
