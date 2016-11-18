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

#ifndef _STRTOOLS_H
#define _STRTOOLS_H
#ifdef __cplusplus
extern "C" {
#endif

#include "stdtypes.h"

#if defined(TARGET_WINCE)
#include <Windows.h>
#endif

// ----------------------------------------------------------------------------

int strLength(const char *s, int maxLen);
char *strTrimTrailing(char *s);
char *strTrim(char *s);
char *strCopy(char *d, int dlen, const char *s, int maxLen);
char *strCopyID(char *d, int dlen, const char *s, int maxLen);

#if defined(TARGET_WINCE)
TCHAR *strWideCopy(TCHAR *d, int dlen, const char *s, int maxLen);
#endif

utBool strEquals(const char *s, const char *v);
utBool strEqualsIgnoreCase(const char *s, const char *v);
utBool strStartsWith(const char *s, const char *v);
utBool strStartsWithIgnoreCase(const char *s, const char *v);
utBool strEndsWith(const char *s, const char *v);
utBool strEndsWithIgnoreCase(const char *s, const char *v);

const char *strIndexOf(const char *s, const char *v);

const char *strLastIndexOfChar(const char *s, char v);

char *strToUpperCase(char *s);
char *strToLowerCase(char *s);

// ----------------------------------------------------------------------------

utBool strParseBoolean(const char *s, utBool dft);

double strParseDouble(const char *s, double dft);

Int32 strParseInt32(const char *s, Int32 dft);
UInt32 strParseUInt32(const char *s, UInt32 dft);
UInt32 strParseHex32(const char *s, UInt32 dft);

#if defined(SUPPORT_UInt64)
Int64 strParseInt64(const char *s, Int64 dft);
UInt64 strParseUInt64(const char *s, UInt64 dft);
UInt64 strParseHex64(const char *s, UInt64 dft);
#endif

// ----------------------------------------------------------------------------

utBool strIsHexDigit(char ch);
int strParseHex(const char *hex, int hexLen, UInt8 *data, int dataLen);
char *strEncodeHex(char *hex, int hexLen, const UInt8 *data, int dataLen);

// ----------------------------------------------------------------------------

int strParseArray(char *s, char *arry[], int maxSize);
int strParseArray_sep(char *s, char *arry[], int maxSize, char separator);

// ----------------------------------------------------------------------------

#ifdef __cplusplus
}
#endif
#endif
