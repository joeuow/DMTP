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

#ifndef _RANDOM_H
#define _RANDOM_H
#ifdef __cplusplus
extern "C" {
#endif
#if defined(SUPPORT_UInt64) // currently 'random' only supports 64-bit architectures

// ----------------------------------------------------------------------------

#if defined(SUPPORT_UInt64)
void randomSeed(UInt64 seed);
UInt64 randomBits(int bits);
#endif

UInt16 randomNext16(UInt16 low, UInt16 high);
UInt32 randomNext32(UInt32 low, UInt32 high);

// ----------------------------------------------------------------------------
#endif // defined(SUPPORT_UInt64)

#ifdef __cplusplus
}
#endif
#endif
