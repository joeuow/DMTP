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
//  String tools
//  Various tools to providing string based parsing and encoding/decoding.
// ---
// Change History:
//  2006/01/04  Martin D. Flynn
//     -Initial release
//  2007/01/28  Martin D. Flynn
//     -WindowsCE port
//     -Added functions:
//      strTrimTrailing/strTrim/strCopy/strCopyID/strEndsWith
//  2007/03/11  Martin D. Flynn
//     -Removed "on"/"yes" options in 'strParseBoolean'
//     -Added ability to parse "true"/"false" in 'strParseInt32'
// ----------------------------------------------------------------------------

#define SKIP_TRANSPORT_MEDIA_CHECK // only if TRANSPORT_MEDIA not used in this file 
#include "defaults.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <ctype.h>

#if defined(TARGET_WINCE)
#include <Windows.h>
#endif

#include "strtools.h"

// ----------------------------------------------------------------------------

/* replacement for 'strnlen' (not all platforms support 'strnlen') */
int strLength(const char *s, int maxLen)
{
    if (maxLen >= 0) {
        // return strnlen(s, maxLen);
        int len = 0;
        for (len = 0; (len < maxLen) && s[len]; len++);
        return len;
    } else {
        return s? strlen(s) : 0;
    }
}

// ----------------------------------------------------------------------------

/* trim trailing spaces from specified null-terminated string */
char *strTrimTrailing(char *s)
{
    if (s) {
        int len = strlen(s);
        if (len > 0) {
            // trim trailing spaces
            char *n = &s[len - 1];
            for (; (n >= s) && isspace(*n); n--) {
                *n = 0;
            }
        }
    }
    return s;
}

/* return string starting at first non-space, and clobber all trailing spaces, in-place */
char *strTrim(char *s)
{
    if (s) {
        // trim trailing space
        strTrimTrailing(s);
        // advance start of string to first non-space
        while (*s && isspace(*s)) { s++; }
    }
    return s;
}

// ----------------------------------------------------------------------------

/* copy 'maxLen' characters from 's' to 'd', and make sure 'd' is terminated */
// (similar to 'strncpy', but does best attempt to terminate 'd')
char *strCopy(char *d, int dlen, const char *s, int maxLen)
{
    
    /* validate arguments */
    if (!d || (dlen <= 0)) {
        // no destination
        return d;
    } else
    if (!s || (maxLen == 0) || !*s) {
        // no source
        d[0] = 0;
        return d;
    }

    /* default length */
    if (maxLen < 0) {
        maxLen = strlen(s) + 1; // include null terminator
    }

    /* copy */
    int sndx = 0, dndx = 0;
    for (; (sndx < maxLen) && (dndx < dlen) && s[sndx]; sndx++) {
        d[dndx++] = s[sndx];
    }
    
    /* terminate destination */
    if (dndx < dlen) {
        d[dndx] = 0;
    }

    /* return destination buffer */
    return d;

}

/* filter copy 'maxLen' characters from 's' to 'd', and make sure 'd' is terminated */
// (similar to 'strCopy', but filters out invalid ID characters)
// Filter: destination may contain only 'A'..'Z', 'a'..'z, '0'..'9', '-', '.'
char *strCopyID(char *d, int dlen, const char *s, int maxLen)
{
    
    /* validate arguments */
    if (!d || (dlen <= 0)) {
        // no destination
        return d;
    } else
    if (!s || (maxLen == 0) || !*s) {
        // no source
        d[0] = 0;
        return d;
    }

    /* default length */
    if (maxLen < 0) {
        maxLen = strlen(s) + 1; // include null terminator
    }
    
    /* copy (with filter) */
    int sndx = 0, dndx = 0;
    for (;(sndx < maxLen) && (dndx < dlen) && s[sndx]; sndx++) {
        if (isalnum(s[sndx]) || (s[sndx] == '.') || (s[sndx] == '-')) {
            d[dndx++] = islower(s[sndx])? toupper(s[sndx]) : s[sndx];
        }
    }
    
    /* terminate destination */
    if (dndx < dlen) {
        d[dndx] = 0;
    }

    /* return destination buffer */
    return d;

}

#if defined(TARGET_WINCE)
/* copy 'maxLen' characters from 's' to wide char array 'd', and make sure 'd' is terminated */
TCHAR *strWideCopy(TCHAR *d, int dlen, const char *s, int maxLen)
{

    /* validate arguments */
    if (!d || (dlen <= 0)) {
        // no destination
        return d;
    } else
    if (!s || (maxLen == 0) || !*s) {
        // no source
        d[0] = 0;
        return d;
    }

    /* default length */
    if (maxLen < 0) {
        maxLen = strlen(s) + 1; // include null terminator
    }
    
    /* reset maxLen if necessary */
    // 'maxLen' cannot be larger than 'dlen'
    if (dlen < maxLen) {
        maxLen = dlen;
    }

    /* multibyte copy */
    int n = MultiByteToWideChar(CP_UTF8, 0, s, maxLen, d, dlen);
    
    /* terminate destination */
    if (n < dlen) {
        d[n] = 0;
    }

    /* return destination buffer */
    return d;
    
}
#endif

// ----------------------------------------------------------------------------

/* return true is strings are equal */
utBool strEquals(const char *s, const char *v)
{
    return (s && v && (strcmp(s,v) == 0))? utTrue : utFalse;
}

/* return true is strings are equal, without regard to case */
utBool strEqualsIgnoreCase(const char *s, const char *v)
{
    // (not all platforms support 'strcasecmp')
    //return (s && v && (strcasecmp(s,v) == 0))? utTrue : utFalse;
    if (s && v) {
        while (*s && *v) {
            char _s = *s, sc = isupper(_s)? tolower(_s) : _s;
            char _v = *v, vc = isupper(_v)? tolower(_v) : _v;
            if (vc != sc) {
                return utFalse;
            }
            s++;
            v++;
        }
        return (!*s && !*v)? utTrue : utFalse; // true if at end-of-string for both strings
    } else {
        return utFalse;
    }
}

// ----------------------------------------------------------------------------

/* return true if the target string starts with the value string */
utBool strStartsWith(const char *s, const char *v)
{
    return (s && v && (strncmp(s,v,strlen(v)) == 0))? utTrue : utFalse;
}

/* return true if the target string starts with the value string, without regard to case */
utBool strStartsWithIgnoreCase(const char *s, const char *v)
{
    // (not all platforms support 'strcasecmp')
    //return (s && v && (strncasecmp(s,v,strlen(v)) == 0))? utTrue : utFalse;
    if (s && v) {
        while (*s && *v) {
            char _s = *s, sc = isupper(_s)? tolower(_s) : _s;
            char _v = *v, vc = isupper(_v)? tolower(_v) : _v;
            if (vc != sc) {
                return utFalse;
            }
            s++;
            v++;
        }
        return (!*v)? utTrue : utFalse; // true if 'v' at end-of-string
    } else {
        return utFalse;
    }
}

// ----------------------------------------------------------------------------

/* return true if the target string ends with the value string */
utBool strEndsWith(const char *s, const char *v)
{
    if (!s || !v) {
        return utFalse;
    } else {
        Int32 sLen = strlen(s);
        Int32 vLen = strlen(v);
        if (vLen > sLen) {
            return utFalse;
        } else {
            s += (sLen - vLen);
            return strEquals(s,v);
        }
    }
}

/* return true if the target string ends with the value string */
utBool strEndsWithIgnoreCase(const char *s, const char *v)
{
    if (!s || !v) {
        return utFalse;
    } else {
        Int32 sLen = strlen(s);
        Int32 vLen = strlen(v);
        if (vLen > sLen) {
            return utFalse;
        } else {
            s += (sLen - vLen);
            return strEqualsIgnoreCase(s,v);
        }
    }
}

// ----------------------------------------------------------------------------
// See also "Boyer and Moore searching algorithm": 
//   http://www.csr.uvic.ca/~nigelh/Publications/stringsearch.pdf
// "Simplified" BM algorithm:
//   const char *strIndexOf(const char *s, const char *p) 
//   {
//       // 'sLen' must be >= 'pLen', and 'pLen' must be <= 256
//       Int32 sLen = strlen(s);
//       Int32 pLen = strlen(p);
//       if (sLen >= pLen) {
//           // initialize for search
//           UInt8 delta[256];
//           memset(delta12, pLen, 256);
//           for (Int32 j = 0; j < pLen - 1; j++) { delta12[(UInt8)p[j]] = pLen - j - 1; }
//           UInt8 lastch = (UInt8)p[patlen - 1];
//           // search
//           for (Int32 i = patlen - 1; i < sLen;) {
//               UInt8 ch = (UInt8)s[i];
//               if (ch == lastch) {
//                   Int32 ndx = i - pLen + 1;
//                   if (!strcmp(s + ndx, p, pLen)) {
//                       return s + ndx;
//                   }
//               }
//               i += delta12[ch]; // <-- may advance beyond sLen
//               // for (Int32 j = i + delta12[ch]; i <= j; i++) { 
//               //     if (!*(s+i)) { return (char*)0; }
//               // }
//           }
//       }
//       return (char*)0;
//   }

/* return index of value string in target string */
const char *strIndexOf(const char *s, const char *p)
{
    if (!s || !p) {
        // null string specified
        return (char*)0;
    } else {
        Int32 pLen = strlen(p);
        for (;*s; s++) {
            if ((*s == *p) && (strncmp(s,p,pLen) == 0)) {
                return s;
            }
        }
        return (char*)0;
    }
}

// ----------------------------------------------------------------------------

/* return last index of value char in target string */
// note: not all platforms support 'rindex'
const char *strLastIndexOfChar(const char *s, char v)
{
    if (!s) {
        // null string specified
        return (char*)0;
    } else {
        Int32 sLen = (Int32)strlen(s); // must be signed
        if (!v) {
            return &s[sLen]; // point to terminator
        } else {
            for (; sLen >= 0; sLen--) {
                if (s[sLen] == v) {
                    return &s[sLen];
                }
            }
            return (char*)0;
        }
    }
}

// ----------------------------------------------------------------------------

/* upshift in-place */
char *strToUpperCase(char *s)
{
    if (s) {
        char *d = s;
        for (; *d; d++) {
            if (islower(*d)) {
                *d = toupper(*d);
            }
        }
    }
    return s;
}

/* downshift in-place */
char *strToLowerCase(char *s)
{
    if (s) {
        char *d = s;
        for (; *d; d++) {
            if (isupper(*d)) {
                *d = tolower(*d);
            }
        }
    }
    return s;
}

// ----------------------------------------------------------------------------

/* return boolean value of string */
utBool strParseBoolean(const char *s, utBool dft)
{
    
    /* return default if string is null */
    if (!s || !*s) {
        return dft;
    }

    /* skip prefixing spaces and return default if string is empty */
    while (*s && isspace(*s)) { s++; }
    if (!*s) {
        return dft;
    }

    /* parse */
    // Warning: This will also attempt to parse 'Hex' values as well.
    //   "0x01" - will return 'true'
    //   "0x00" - will return 'false'
    //   "0xAA" - will return 'true'
    //   "0xZZ" - will return the default ('0xZZ' is an invalid hex value)
    if (dft) {
        // default is true, check for false
        if (isdigit(*s)) {
            return strParseUInt32(s,1L)? utTrue : utFalse;
        } else {
            return strStartsWithIgnoreCase(s,"false")? utFalse : utTrue;
        }
    } else {
        // default is false, check for true
        if (isdigit(*s)) {
            return strParseUInt32(s,0L)? utTrue : utFalse;
        } else {
            return strStartsWithIgnoreCase(s,"true")? utTrue : utFalse;
        }
    }

}

// ----------------------------------------------------------------------------

/* return signed 32-bit value of string */
Int32 strParseInt32(const char *s, Int32 dft)
{
    
    /* return default if string is null */
    if (!s || !*s) {
        return dft;
    }

    /* skip prefixing spaces and return default if string is empty */
    while (*s && isspace(*s)) { s++; }
    if (!*s) {
        return dft;
    }

    /* Hex? */
    if ((s[0] == '0') && ((s[1] == 'x') || (s[1] == 'X'))) {
        return (Int32)strParseHex32(s+2, (UInt32)dft);
    }
    
    /* Boolean? (test for 'T'/'F' character first to exit quickly) */
    if (((*s == 't') || (*s == 'T')) && strStartsWithIgnoreCase(s,"true" )) {
        // Thus, "config.value=true" will set the property value to '1'
        return 1L;
    } else
    if (((*s == 'f') || (*s == 'F')) && strStartsWithIgnoreCase(s,"false")) {
        // Thus, "config.value=false" will set the property value to '0'
        return 0L;
    }
    
    /* Integer */
    if (*s == '+') { s++; }
    if (isdigit(*s) || ((*s == '-') && isdigit(*(s+1)))) {
        return (Int32)atol(s);
    }
    
    /* unparsable default */
    return dft;

}

/* return unsigned 32-bit value of string */
UInt32 strParseUInt32(const char *s, UInt32 dft)
{
    return (UInt32)strParseInt32(s, (Int32)dft);
}

/* return unsigned 32-bit value of hex-encoded string */
UInt32 strParseHex32(const char *s, UInt32 dft)
{
    
    /* return default if string is null */
    if (!s || !*s) {
        return dft;
    }
    
    /* skip prefixing spaces and return default if string is empty */
    while (*s && isspace(*s)) { s++; }
    if ((s[0] == '0') && ((s[1] == 'x') || (s[1] == 'X'))) { s += 2; } // skip "0x"
    if (!*s) {
        return dft;
    }
    
    /* parse hex string */
    UInt32 accum = 0L;
    int i, maxNybbles = 8; // max 8 nybbles
    for (i = 0; *s && (i < maxNybbles); i++, s++) {
        UInt8 C = toupper(*s), N = 0;
        if ((C >= '0') && (C <= '9')) {
            N = C - '0';            // N == 0..9
        } else
        if ((C >= 'A') && (C <= 'F')) {
            N = 10 + (C - 'A');     // N == 10..15
        } else {
            // non-hex character, return default if nothing was parsed
            return (i > 0)? accum : dft;
        }
        accum = (accum << 4) | N; // 4 bits per nybble [N == 0..15]
    }
    return accum;

}

// ----------------------------------------------------------------------------

#if defined(SUPPORT_UInt64)

/* return signed 64-bit value of string */
Int64 strParseInt64(const char *s, Int64 dft)
{
    
    /* return default if string is null */
    if (!s || !*s) {
        return dft;
    }

    /* skip prefixing spaces and return default if string is empty */
    while (*s && isspace(*s)) { s++; }
    if (!*s) {
        return dft;
    }

    /* Hex? */
    if ((s[0] == '0') && ((s[1] == 'x') || (s[1] == 'X'))) {
        return (Int64)strParseHex64(s+2, (UInt64)dft);
    }

    /* Decimal */
    if (*s == '+') { s++; }
    if (isdigit(*s) || ((*s == '-') && isdigit(*(s+1)))) {
#if defined(TARGET_WINCE)
        return (Int64)_atoi64(s);
#else
        return (Int64)atoll(s);
#endif
    }

    /* unparsable default */
    return dft;
    
}

/* return unsigned 64-bit value of string */
UInt64 strParseUInt64(const char *s, UInt64 dft)
{
    return (UInt32)strParseInt64(s, (Int64)dft);
}

/* return unsigned 64-bit value of hex-encoded string */
UInt64 strParseHex64(const char *s, UInt64 dft)
{
    
    /* return default if string is null */
    if (!s) {
        return dft;
    }
    
    /* skip prefixing spaces and return default if string is empty */
    while (*s && isspace(*s)) { s++; }
    if ((s[0] == '0') && ((s[1] == 'x') || (s[1] == 'X'))) { s += 2; } // skip "0x"
    if (!*s) {
        return dft;
    }
    
    /* parse hex string */
    UInt64 accum = (UInt64)0;
    int i, maxNybbles = 16; // max 16 nybbles
    for (i = 0; *s && (i < maxNybbles); i++, s++) {
        UInt8 C = toupper(*s), N = 0;
        if ((C >= '0') && (C <= '9')) {
            N = C - '0';            // N == 0..9
        } else
        if ((C >= 'A') && (C <= 'F')) {
            N = 10 + (C - 'A');     // N == 10..15
        } else {
            // non-hex character, return default if nothing was parsed
            return (i > 0)? accum : dft;
        }
        accum = (accum << 4) | N; // 4 bits per nybble [N == 0..15]
    }
    return accum;

}

#endif

// ----------------------------------------------------------------------------

/* return double value of string */ 
double strParseDouble(const char *s, double dft)
{
    if (s && *s) {
        while (*s && isspace(*s)) { s++; }
        if (*s == '+') { s++; }
        if (isdigit(*s) || ((*s == '-') && isdigit(*(s+1)))) {
            return (double)atof(s);
        }
    }
    return dft;
}

// ----------------------------------------------------------------------------

static int _hexNybble(UInt8 ch)
{
    if ((ch >= '0') && (ch <= '9')) {
        return (int)(ch - '0') & 0xF;
    } else
    if ((ch >= 'a') && (ch <= 'f')) {
        return ((int)(ch - 'a') & 0xF) + 10;
    } else
    if ((ch >= 'A') && (ch <= 'F')) {
        return ((int)(ch - 'A') & 0xF) + 10;
    } else {
        return -1;
    }
}

/* return true if character is a hex digit */
utBool strIsHexDigit(char ch)
{
    return (_hexNybble(ch) >= 0)? utTrue : utFalse;
}

/* decode hex-encoded string into byte array */
// returns the length of the parse hex value (or '0' if unparsable)
int strParseHex(const char *hex, int hexLen, UInt8 *data, int dataLen)
{
    // The number of valid nybbles is assumed to be even.  
    // If not even, the last byte will not be parsed.
    if (!hex) {
        return 0;
    } else {
        int len = (hexLen >= 0)? strLength(hex, hexLen) : strlen(hex);
        const char *h = hex;
        if ((len >= 2) && (*h == '0') && ((*(h+1) == 'x') || (*(h+1) == 'X'))) {
            h += 2;
            len -= 2;
        }
        if (len & 1) { len--; } // make sure length is even
        if (len >= 2) {
            int i;
            for (i = 0; (i < len) && ((i/2) < dataLen); i += 2) {
                int c1 = _hexNybble(h[i]);
                if (c1 < 0) {
                    // Invalid Hex char (the first of the pair)
                    break; // stop parsing
                }
                int c2 = _hexNybble(h[i+1]);
                if (c2 < 0) { 
                    // Invalid Hex char (the second of the pair)
                    break; // stop parsing (last byte ignored
                }
                data[i/2] = (UInt8)(((c1 << 4) & 0xF0) | (c2 & 0x0F));
            }
            return i/2;
        } else {
            return 0;
        }
    }
}

/* encode byte array into hex-encoded string */
char *strEncodeHex(char *hex, int hexLen, const UInt8 *data, int dataLen)
{
    if (!hex || (hexLen == 0)) {
        return (char*)0;
    }
    *hex = 0;
    if ((hexLen < 0) || (hexLen >= ((dataLen * 2) + 1))) {
        if (data && (dataLen > 0)) {
            int i;
            char *h = hex;
            for (i = 0; i < dataLen; i++) {
                sprintf(h, "%02X", (unsigned int)data[i] & 0xFF);
                h += strlen(h);
            }
        }
        return hex;
    } else {
        return (char*)0;
    }
}

// ----------------------------------------------------------------------------

/* parse (1,2,"hello","hi") in place (',' separator assummed) */
// return the number of parsed fields
int strParseArray(char *s, char *arry[], int maxSize)
{
    return strParseArray_sep(s, arry, maxSize, ',');
}

/* parse (1,2,"hello","hi") in place (separator specified) */
// return the number of parsed fields
int strParseArray_sep(char *s, char *arry[], int maxSize, char separator)
{
    
    /* null string specified? */
    if (!s) {
        return 0;
    }
    
    /* skip leading spaces */
    while (*s && isspace(*s)) { s++; }
    if (*s == '(') { s++; } // skip start of array (if specified)
    
    /* read until end of string */
    int size = 0;
    for (;(size < maxSize) && (*s != 0);) {
        
        /* skip white space */
        while (*s && isspace(*s)) { s++; }
        
        /* find data start */
        utBool quoted = utFalse;
        if (*s == '\"') {
            quoted = utTrue;
            s++;
        }
    
        /* look for end of string [",)] */
        char *start = s, *end = s;
        while (utTrue) {
            if (*s == 0) {
                // end of string
                end = s; // already terminated (premature end of array)
                break;
            } else
            if (*s == ')') {
                // end of array ('quoted' should be false)
                end = s;
                *s = 0; // terminate, end of array
                break;
            } else
            if ((*s == '\"') && quoted) {
                // end of quote
                end = s;
                *s++ = 0; // terminate and skip quote
                while (*s && isspace(*s)) { s++; } // skip spaces
                if (*s == separator) {
                    // ready for next element
                    s++; // skip separator
                } else 
                if (*s == ')') {
                    // end of array
                    *s = 0; // terminate
                }
                break;
            } else
            if ((*s == separator) && !quoted) {
                // end of element
                end = s;
                *s++ = 0; // terminate and skip separator for next element
                break;
            } else {
                // advance to next character in element
                s++; 
            }
        }
        
        /* remove ending space (only if not quoted) */
        if (!quoted) {
            while ((end > start) && isspace(*(end-1))) { *(--end) = 0; }
        }
        
        /* set array element */
        arry[size++] = start;
        
    }
    
    /* return size */
    return size;
    
}

// ----------------------------------------------------------------------------
