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

#ifndef _IO_H
#define _IO_H
#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include "stdtypes.h"

// ----------------------------------------------------------------------------

// File 'ioOpenStream' modes
#define IO_OPEN_READ            "r"
#define IO_OPEN_WRITE           "w+"
#define IO_OPEN_APPEND          "a+"

// ----------------------------------------------------------------------------

#if defined(TARGET_WINCE)
// not supported
#else
void ioResetSTDIN();
void ioResetSTDOUT();
#endif

// ----------------------------------------------------------------------------

void ioInitialize();

// ----------------------------------------------------------------------------

utBool ioExists(const char *fn);
utBool ioIsFile(const char *fn);
utBool ioIsDirectory(const char *fn);

// ----------------------------------------------------------------------------

utBool ioDeleteFile(const char *fn);
long ioGetFileSize(const char *fn, int fd);

// ----------------------------------------------------------------------------

FILE *ioOpenStream(const char *fileName, const char *mode);
void ioCloseStream(FILE *file);

long ioReadStream(FILE *file, void *data, long maxLen);
long ioReadFile(const char *fileName, void *data, long maxLen);
long ioReadLine(FILE *file, char *data, long dataLen);

long ioWriteStream(FILE *file, const void *data, long dataLen);
void ioFlushStream(FILE *file);
long ioWriteFile(const char *fileName, const void *data, long dataLen);
long ioAppendFile(const char *fileName, const void *data, long dataLen);
long ioCreateFile(const char *fileName, long fileSize);

// ----------------------------------------------------------------------------

utBool ioMakeDirs(const char *dirs, utBool omitLast);
int ioReadDir(const char *dirName, char *buf, int bufLen, char *file[], int fileLen);

// ----------------------------------------------------------------------------

#ifdef __cplusplus
}
#endif
#endif
