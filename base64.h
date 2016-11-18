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

#ifndef _BASE64_H
#define _BASE64_H
#ifdef __cplusplus
extern "C" {
#endif

#include "stdtypes.h"

// ----------------------------------------------------------------------------

#define BASE64_PAD      '='

// ----------------------------------------------------------------------------

long base64Encode(char *b64Out, long b64OutLen, const UInt8 *dataIn, long dataInLen);
long base64Decode(const char *b64In, long b64InLen, UInt8 *dataOut, long dataOutlen);

// ----------------------------------------------------------------------------

#ifdef __cplusplus
}
#endif
#endif
