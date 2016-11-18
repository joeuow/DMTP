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
//  File I/O utilities 
//  Utilities for reading/writing files.
// ---
// Change History:
//  2006/01/04  Martin D. Flynn
//     -Initial release
//  2006/01/18  Martin D. Flynn
//     -Added 'ioAppendFile'
//  2007/01/28  Martin D. Flynn
//     -WindowsCE port
//     -Added 'ioCreateFile'
//     -Added 'ioOpenStream', 'ioCloseStream', 'ioReadStream', 'ioWriteStream'
//     -Added option for locking file i/o
// ----------------------------------------------------------------------------

#define SKIP_TRANSPORT_MEDIA_CHECK // only if TRANSPORT_MEDIA not used in this file 
#include "defaults.h"

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>

#if defined(TARGET_WINCE)
// already included
#else
#  include <unistd.h>
#  include <fcntl.h>
#  include <dirent.h>
#  include <errno.h>
#  include <termios.h>
#  include <sys/stat.h>
#  include <sys/types.h>
#  include <sys/select.h>
#  include <sys/errno.h>
#endif

#include "log.h"

#include "stdtypes.h"
#include "strtools.h"
#include "io.h"

// ----------------------------------------------------------------------------

#if defined(TARGET_WINCE)
// 'mkdir' not yet implemented on WinCE
#else
#  ifndef IO_mkdir
//   define 'mkdir' if not defined elsewhere
#    define IO_mkdir(DIR, PERM)     mkdir((DIR),(PERM))
#  endif
#endif

// ----------------------------------------------------------------------------

// uncomment to enable file access thread locking 
// this is only needed if the platform 'fread'/'fwrite' functions
// are not thread-safe. (not fully implemented)
//#define IO_THREAD_LOCK

#ifdef IO_THREAD_LOCK
#  include "threads.h"
static threadMutex_t                ioMutex;
#  define IO_LOCK                   MUTEX_LOCK(&ioMutex);
#  define IO_UNLOCK                 MUTEX_UNLOCK(&ioMutex);
#else
#  define IO_LOCK
#  define IO_UNLOCK
#endif

// ----------------------------------------------------------------------------

/* initialization */
void ioInitialize()
{
#if defined(IO_THREAD_LOCK)
    /* create mutex */
    threadMutexInit(&ioMutex);
#endif
}

// ----------------------------------------------------------------------------

#if defined(TARGET_WINCE)
// not supported
#else
/* turn off buffering on stdin */
void ioResetSTDIN()
{
    // turn off buffering
    struct termios options;
    //setvbuf(stdin, 0/*(char*)malloc(200)*/, _IONBF, 0); // [MALLOC]
    fcntl(STDIN_FILENO, F_SETFL, 0);
    tcgetattr(STDIN_FILENO, &options);
    options.c_lflag &= ~ECHO;
    options.c_lflag &= ~ICANON;
    tcsetattr(STDIN_FILENO, TCSANOW, &options);
}
#endif

// ----------------------------------------------------------------------------

#if defined(TARGET_WINCE)
// not supported
#else
/* turn off buffering on stdout */
void ioResetSTDOUT()
{
    // turn off buffering
    struct termios options;
    //setvbuf(stdout, (char*)malloc(1), _IOFBF, 1); // [MALLOC]
    setvbuf(stdout, 0, _IONBF, 0);
    fcntl(STDOUT_FILENO, F_SETFL, 0);
    tcgetattr(STDOUT_FILENO, &options);
    options.c_lflag &= ~ICANON;
    tcsetattr(STDOUT_FILENO, TCSANOW, &options);
}
#endif

// ----------------------------------------------------------------------------

/* return true if specified name exists */
utBool ioExists(const char *fn)
{
    if (fn && *fn) {
#if defined(TARGET_WINCE)
        wchar_t wfn[512];
        strWideCopy(wfn, sizeof(wfn)/sizeof(wfn[0]), fn, -1);
        //MultiByteToWideChar(CP_UTF8, 0, fn, -1, wfn, sizeof(wfn)/sizeof(wchar_t));
        DWORD attr = GetFileAttributes(wfn);
        return (attr != 0xFFFFFFFFL)? utTrue : utFalse;
#else
        struct stat buf;
        int rtn = stat(fn, &buf);
        return (rtn == 0)? utTrue : utFalse;
#endif
    } else {
        return utFalse;
    }
}

/* return true if specified name is a file */
utBool ioIsFile(const char *fn)
{
    if (fn && *fn) {
#if defined(TARGET_WINCE)
        wchar_t wfn[512];
        strWideCopy(wfn, sizeof(wfn)/sizeof(wfn[0]), fn, -1);
        //MultiByteToWideChar(CP_UTF8, 0, fn, -1, wfn, sizeof(wfn)/sizeof(wchar_t));
        DWORD attr = GetFileAttributes(wfn);
        return ((attr != 0xFFFFFFFFL) && ((attr & FILE_ATTRIBUTE_DIRECTORY) == 0x0))? utTrue : utFalse;
#else
        struct stat buf;
        int rtn = stat(fn, &buf);
        return (rtn == 0)? S_ISREG(buf.st_mode) : utFalse;
#endif
    } else {
        return utFalse;
    }
}

/* return true if specified name is a file */
utBool ioIsDirectory(const char *fn)
{
    if (fn && *fn) {
#if defined(TARGET_WINCE)
        wchar_t wfn[512];
        strWideCopy(wfn, sizeof(wfn)/sizeof(wfn[0]), fn, -1);
        DWORD attr = GetFileAttributes(wfn);
        return ((attr != 0xFFFFFFFFL) && ((attr & FILE_ATTRIBUTE_DIRECTORY) != 0x0))? utTrue : utFalse;
#else
        struct stat buf;
        int rtn = stat(fn, &buf);
        return (rtn == 0)? S_ISDIR(buf.st_mode) : utFalse;
#endif
    } else {
        return utFalse;
    }
}

// ----------------------------------------------------------------------------

/* read file list from directory */
int ioReadDir(const char *dirName, char *buf, int bufLen, char *file[], int fileLen)
{
    
    /* no place to store the list of files? */
    if (!buf || (bufLen < 5) || !file || (fileLen < 1)) {
        logERROR(LOGSRC,"Invalid parameters ...");
        return 0;
    }
    
    /* file count */
    int fileNdx = 0, bufNdx = 0;

#if defined(TARGET_LINUX) || defined(TARGET_CYGWIN)

    // open directory 
    DIR *dir = opendir(dirName);
    if (!dir) {
        logERROR(LOGSRC,"Unable to open dir %s", dirName);
        return -1;
    }
    
    // read directory contents
    while (utTrue) {
        struct dirent *de = readdir(dir);
        if (!de) {
            // no more directory entries?
            break; 
        }
        char *name = de->d_name;
        if (!strEquals(name,".") && !strEquals(name,"..")) {
            //logINFO(LOGSRC,"Found: %s", de->d_name);
            int nameLen = strlen(name);
            if ((bufNdx + nameLen + 1) >= bufLen) {
                break; // buffer overflow
            }
            strcpy(&buf[bufNdx], name);
            file[fileNdx++] = &buf[bufNdx];
            bufNdx += (nameLen + 1);
            if (fileNdx >= fileLen) {
                break; // file list overflow
            }
        }
    }
    
    // close directory
    if (closedir(dir)) {
        logERROR(LOGSRC,"I/O closedir");
    }
    
#elif defined(TARGET_WINCE)

    // open directory 
    WIN32_FIND_DATA FileData;
    TCHAR wszDir[MAX_PATH + 1];
    strWideCopy(wszDir, sizeof(wszDir)-3, dirName, -1);
    wcscat(wszDir, L"\\*");
    HANDLE hList = FindFirstFile(wszDir, &FileData);
    if (hList == INVALID_HANDLE_VALUE) {
        logERROR(LOGSRC,"Unable to open dir %s", dirName);
        return -1;
    }
    
    // read directory contents
    while (utTrue) {
        TCHAR *name = FileData.cFileName;
        if ((wcscmp(name,L".")==0) && (wcscmp(name,L"..")==0)) {
            logINFO(LOGSRC,"Found: %ls", name);
            int nameLen = wcslen(name);
            if ((bufNdx + nameLen + 1) >= bufLen) {
                break; // buffer overflow
            }
            sprintf(&buf[bufNdx], "%ls", name); // wide to narrow copy
            file[fileNdx++] = &buf[bufNdx];
            bufNdx += (nameLen + 1);
            if (fileNdx >= fileLen) {
                break; // file list overflow
            }
        }
        if (!FindNextFile(hList, &FileData)) {
            break;
        }
    }
    
    // close directory
    FindClose(hList); // ignore any returned errors

#else
#   warning 'ioReadDir' not supported on this platform
#endif

    /* return count */
    return fileNdx;

}

// ----------------------------------------------------------------------------

/* return size of file name */
long ioGetFileSize(const char *fn, int fd)
{
#if defined(TARGET_WINCE)
    if (fn && *fn) {
        wchar_t wfn[512];
        strWideCopy(wfn, sizeof(wfn)/sizeof(wfn[0]), fn, -1);
        HANDLE fh = CreateFile(wfn, GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE, 
            NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
        if (fh != INVALID_HANDLE_VALUE) {
            DWORD fileSize = GetFileSize(fh, NULL);
            CloseHandle(fh);
            if (fileSize != INVALID_FILE_SIZE) {
                return (long)fileSize;
            } else {
                DWORD err = GetLastError();
                logWARNING(LOGSRC,"Unable to get file size: %ls [%lu]", wfn, err);
                return -1L;
            }
        } else {
            return -1L;
        }
    } else
    if (fd >= 0) {
        // not yet supported
        logERROR(LOGSRC,"'ioGetFileSize' (with fd) not yet implemented");
        return -1L;
    } else {
        return -1L;
    }
#else
    struct stat buf;
    int rtn = -1; // ERROR
    if (fn && *fn) {
        rtn = stat(fn, &buf);
    } else
    if (fd >= 0) {
        rtn = fstat(fd, &buf);
    }
    return (rtn == 0)? buf.st_size : -1L;
#endif
}

// ----------------------------------------------------------------------------

utBool ioDeleteFile(const char *fn)
{
    if (fn && *fn) {
#if defined(TARGET_WINCE)
        wchar_t wfn[512];
        strWideCopy(wfn, sizeof(wfn)/sizeof(wfn[0]), fn, -1);
        BOOL ok = DeleteFile(wfn);
        return ok? utTrue : utFalse;
#else
        return (unlink(fn) == 0)? utTrue : utFalse;
#endif
    } else {
        return utFalse;
    }
}

// ----------------------------------------------------------------------------

/* open stream */
FILE *ioOpenStream(const char *fileName, const char *mode)
{
    // 'mode' should be one of the following:
    //  IO_OPEN_READ   ["rb"]
    //  IO_OPEN_WRITE  ["wb"]
    //  IO_OPEN_APPEND ["ab"]

    /* fileName & mode specified? */
    if (!fileName || !*fileName || !mode || !*mode) {
        return (FILE*)0;
    }
    
    /* mode length */
    //int modeLen = strlen(mode);
    //if (modeLen > 3) {
    //    return (FILE*)0;
    //}
    
    /* binary mode? */
    //if (mode[modeLen - 1] != 'b') {
    //    logWARNING(LOGSRC,"Opening file in ASCII text mode: %s", fileName);
    //}

    /* open file */
    FILE *file = (FILE*)0;
    IO_LOCK {
        file = fopen(fileName, mode);
    } IO_UNLOCK
    return file;
    
}

/* close stream */
void ioCloseStream(FILE *file)
{
    if (file && (file != stderr) && (file != stdout)) {
        ioFlushStream(file);
        if (fclose(file)) {
            logERROR(LOGSRC,"I/O fclose");
        }
    }
}

// ----------------------------------------------------------------------------

/* read file contents */
long ioReadStream(FILE *file, void *vData, long maxLen)
{
    
    /* invalid file handle? */
    if (!file || !vData || (maxLen < 0L)) {
        return -1L;
    }
    
    /* read data */
    UInt8 *uData = (UInt8*)vData;
    long readLen = 0L;
    IO_LOCK {
        while (readLen < maxLen) {
            long len = fread(&uData[readLen], 1, maxLen - readLen, file);
            if (len <= 0L) {
                break; // Error/EOF
            }
            readLen += len;
        }
    } IO_UNLOCK
    return readLen;

}

/* read file contents */
long ioReadFile(const char *fileName, void *data, long maxLen)
{
    
    /* invalid config file name? */
    if (!fileName || !*fileName) {
        return -1L;
    }
    
    /* open file */
    FILE *file = ioOpenStream(fileName, IO_OPEN_READ);
    if (!file) {
        // error openning
        return -1L;
    }

    /* read data */
    long len = ioReadStream(file, data, maxLen);
    ioCloseStream(file);
    if (len < 0L) {
        // error reading
        return -1L;
    }

    return len;
}

/* read line */
// Warning: this function currently does not look ahead to see if a '\n' follows a '\r'
long ioReadLine(FILE *file, char *data, long dataLen)
{
    
    /* no data buffer? */
    if (!data || (dataLen <= 0L)) {
        return -1L;
    }
    data[0] = 0; // pre-terminate
    
    /* invalid file? */
    if (!file) {
        return -1L;
    }
    
    /* read until '\r' or '\n' */
    long d = 0;
    for (;;) {
        
        /* read character */
        char ch;
        long len = ioReadStream(file, &ch, 1);
        if (len < 0L) {
            // error reading
            data[d] = 0; // terminate what we've already read
            return -1L;
        } else 
        if (len == 0L) {
            // EOF
            break;
        }
        
        /* check EOL */
        if (ch == '\n') {
            // checking for existing characters in the buffer avoids sending blank lines 
            // should the records be terminated with CR pairs
            if (d > 0) {
                break; // data is present, return
            }
        } else
        if (ch == '\r') {
            break;
        } else {
            if (d < dataLen - 1L) {
                // at most 'dataLen' characters are saved in the buffer
                data[d++] = ch;
            }
        }
        
    }
    
    /* terminate data and return length */
    data[d] = 0;
    return d;

}

// ----------------------------------------------------------------------------

/* write file contents */
long ioWriteStream(FILE *file, const void *data, long dataLen)
{
    
    /* invalid file handle? */
    if (!file || (dataLen < 0L)) {
        return -1L;
    }
    
    /* write to stream */
    long len = 0L;
    IO_LOCK {
        len = fwrite(data, 1, dataLen, file);
    } IO_UNLOCK
    if (len < 0) {
        // error writing
        return -1L;
    }
    return len;

}

/* flush stream */
void ioFlushStream(FILE *file)
{
    if (file) {
        if (fflush(file)) { 
            logERROR(LOGSRC,"I/O fflush");
        }
    }
}

/* write file contents */
long ioWriteFile(const char *fileName, const void *data, long dataLen)
{
    
    /* invalid config file name? */
    if (!fileName || !*fileName) {
        return -1L;
    }

    /* open file */
    FILE *file = ioOpenStream(fileName, IO_OPEN_WRITE);
    if (!file) {
        // error openning
        return -1L;
    }
    
    /* write to file */
    long len = ioWriteStream(file, data, dataLen);
    ioCloseStream(file);
    if (len < 0) {
        // error writing
        return -1L;
    }
    
    /* return number of bytes written */
    return len;

}

/* append to file contents */
long ioAppendFile(const char *fileName, const void *data, long dataLen)
{
    
    /* invalid config file name? */
    if (!fileName || !*fileName) {
        return -1L;
    }
    
    /* open file */
    FILE *file = ioOpenStream(fileName, IO_OPEN_APPEND);
    if (!file) {
        // error openning
        return -1L;
    }
    
    /* write to file */
    long len = ioWriteStream(file, data, dataLen);
    ioCloseStream(file);
    if (len < 0) {
        // error writing
        return -1L;
    }
    
    /* return number of bytes written */
    return len;

}

// ----------------------------------------------------------------------------

/* create file with specified pre-allocated size */
long ioCreateFile(const char *fileName, long fileSize)
{
    
    /* invalid config file name? */
    if (!fileName || !*fileName || (fileSize < 0L)) {
        return -1L;
    }

    /* open file */
    FILE *file = ioOpenStream(fileName, IO_OPEN_WRITE);
    if (!file) {
        // error openning
        return -1L;
    }
    
    /* write to file */
    UInt8 buf[1024];
    long size = 0L;
    while (size < fileSize) {
        long wrtLen = ((fileSize - size) > sizeof(buf))? sizeof(buf) : (fileSize - size);
        long len = ioWriteStream(file, buf, wrtLen);
        if (len < 0) {
            break;
        }
        size += len;
    }
    ioCloseStream(file);

    /* return size */
    if (size < fileSize) {
        // error writing
        ioDeleteFile(fileName);
        return -1L;
    } else {
        // return number of bytes written
        return size;
    }

}

// ----------------------------------------------------------------------------

#if defined(TARGET_WINCE)
utBool ioMakeDirs(const char *dirs, utBool omitLast)
{
    // not yet supported
    logERROR(LOGSRC,"'ioMakeDirs' not yet supported on WindowsCE platform");
    return utFalse;
}
#else
#define IS_DIR_SEP(C)   (((C) == '/') || ((C) == '\\'))
utBool ioMakeDirs(const char *dirs, utBool omitLast)
{
    char path[256], *p = path;
    const char *d = dirs;
    
    /* first character must be a '/' */
    if (!IS_DIR_SEP(*d)) {
        return utFalse;
    }
    
    /* scan through directories to attempt creating each one */
    *p++ = *d++; // copy first '/'
    for (;;d++) {
        utBool mkd = (IS_DIR_SEP(*d) || ((*d == 0) && !omitLast))? utTrue : utFalse;
        if (mkd) {
            // end of directory name reached in 'dirs'
            if (IS_DIR_SEP(*(p - 1))) {
                // found an empty directory [ie. "//"]
            } else {
                *p = 0; // terminate
                //logDEBUG(LOGSRC,"Making dir: %s\n", path);
                int status = IO_mkdir(path, 0777);
                if ((status != 0) && (errno != EEXIST)) {
                    return utFalse;
                }
                if (*d != 0) {
                    *p++ = *d; // copy '/'
                }
            }
        } else {
            if (*d != 0) {
                *p++ = *d; // copy dir character
            }
        }
        if (*d == 0) { 
            break;
        }
    }
    
    return utTrue;
}
#endif

// ----------------------------------------------------------------------------
