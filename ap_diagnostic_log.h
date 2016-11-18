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

#ifndef _LOG_H
#define _LOG_H
#ifdef __cplusplus
extern "C" {
#endif

#include <stdarg.h>

#include "stdtypes.h"

// ----------------------------------------------------------------------------
//   LOG_EMERG       0      // not used
//   LOG_ALERT       1      // not used
//   LOG_CRIT        2
//   LOG_ERR         3
//   LOG_WARNING     4
//   LOG_NOTICE      5      // not used
//   LOG_INFO        6
//   LOG_DEBUG       7
/*
// NOTE: These MUST mirror the "LOG_XXXX" definitions
#define SYSLOG_NONE         101 // LOG_EMERG   [not used]
#define SYSLOG_ALERT        1 // LOG_ALERT   [not used]
#define SYSLOG_CRITICAL     2 // LOG_CRIT
#define SYSLOG_ERROR        3 // LOG_ERR
#define SYSLOG_WARNING      4 // LOG_WARNING
#define SYSLOG_NOTICE       5 // LOG_NOTICE  [not used]
#define SYSLOG_INFO         6 // LOG_INFO
#define SYSLOG_DEBUG        7 // LOG_DEBUG

// ----------------------------------------------------------------------------
#define SYSLOG_GENERIC	1
#define SYSLOG_TAG		2
#define SYSLOG_WIFI		4
#define SYSLOG_DEFAULT_SIZE 256
// ----------------------------------------------------------------------------

#define LOGSRC          __FILE__,__LINE__
#define logDEBUG        logDebug_
#define logINFO         logInfo_
#define logWARNING      logWarning_
#define logERROR        logError_
#define logCRITICAL     logCritical_
//#define logPRINTF       logPrintf_ 
#define isDebugMode()	(_isDebugMode)
// Macro elipsis not supported by Windows compilers

void logDebug_(const char *ftn, int line, const char *fmt, ...);
void logInfo_(const char *ftn, int line, const char *fmt, ...);
void logWarning_(const char *ftn, int line, const char *fmt, ...);
void logError_(const char *ftn, int line, const char *fmt, ...);
void logCritical_(const char *ftn, int line, const char *fmt, ...);
//void logPrintf_(const char *ftn, int line, int level, const char *fmt, ...); 

// ----------------------------------------------------------------------------
void setDebugMode(utBool mode);
void request_log_collect(void);
bool is_wifi_logging(void);
UInt32 get_log_type(void);
// ----------------------------------------------------------------------------
const char *logSrcFile(const char *fn);
*/
// ----------------------------------------------------------------------------
utBool ap_diagnostic_logStartThread();
extern void report_mission_status(int mission_status, char *Reason);
extern int http_uploader(void *arg);
/*
// ----------------------------------------------------------------------------
#if defined(SYSLOG_THREAD)
//utBool logIsLevel(int level);
int logParseLevel(const char *level);
void logSetLevel(int level);
#endif // defined(SYSLOG_THREAD)
int enableSyslog(UInt32 type, int log_size_kilo);
// ----------------------------------------------------------------------------
int print_debug(const char *format, ...);
int print_critical(const char *format, ...);
void print_tag_raw(unsigned char *raw);
int print_hex(const char *header, const UInt8 *hex, int len);
#ifdef __cplusplus
}
#endif
*/
#endif

