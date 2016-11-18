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
//  Debug/Info logging.
//  - Since each platform may provide its own type of logging facilities, this
//  module has been placed in the 'custom/' folder.
//  - The logging facility provided here now includes support for Linux 'syslog'
//  output (which may be overkill for some embedded systems).
// ---
// Change History:
//  2006/01/04  Martin D. Flynn
//     -Initial release
//  2006/01/12  Martin D. Flynn
//     -Upgraded with syslog support
//  2007/01/28  Martin D. Flynn
//     -WindowsCE port
//     -Added Aux storage logging
//     -Added threaded logging (to prevent logging from blocking other threads)
// ----------------------------------------------------------------------------

#include "defaults.h"

// For auxiliary message logging, the following must be defined:
//   "LOGGING_MESSAGE_FILE"
//   "LOGGING_AUX_DIRECTORY"

// ----------------------------------------------------------------------------

// uncomment to include syslog support
#if defined(TARGET_LINUX) || defined(TARGET_GUMSTIX)
//#  define INCLUDE_SYSLOG_SUPPORT
#endif

// uncomment to display log messages in a separate thread
//#define SYSLOG_THREAD

// ----------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <ctype.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

#if defined(INCLUDE_SYSLOG_SUPPORT)
#  include <syslog.h>
#endif

#include "log.h"
#include "stdtypes.h"
#include "strtools.h"
#include "utctools.h"
#include "threads.h"
#include "buffer.h"
#include "propman.h"
#include "startup.h"
#include "io.h"
#include "libwlcomm.h"

// ----------------------------------------------------------------------------

#if defined(TARGET_WINCE)
// (Windows return '-1' if lengths exceeds 'size' which is different than Linux)
#  define VSNPRINTF             _vsnprintf
#else
// (Linux return the number of bytes that would have been written if the buffer were long enough)
#  define VSNPRINTF             vsnprintf
#endif

// ----------------------------------------------------------------------------

// This value may be overridden in "defaults.h" by defining "LOGGING_DEFAULT_LEVEL"
#define LOG_BUFFER_SIZE		4096
#define MAX_MESSAGE_LENGTH	256
#define LOG_COLLECT_SIZE	1024
#define CONSOLE_OUTPUT		stderr
#define FLUSH_MODULO		5L
#define LOG_BUF_ALERT	LOG_BUFFER_SIZE - MAX_MESSAGE_LENGTH
#define LOG_BUF_FULL	((LOG_BUFFER_SIZE >> 1) + (LOG_BUFFER_SIZE >> 2))
#define LOG_MAIN_PERIOD	30
#define SYSLOG_NORMAL	(SYSLOG_GENERIC | SYSLOG_WIFI)
#define SYSLOG_TYPE_MASK	7
// ----------------------------------------------------------------------------
static char			*log_buffer = NULL;
static char 		*collect_buffer = NULL;
static char 		*hex_disp = NULL;
static char 		*log_format;
static char			*log_ptr_tail = NULL;
static int			syslogLevel = LOGGING_DEFAULT_LEVEL;
static time_t		rt_time;
static bool			log_collect_request = false;
static bool			log_upload_request = false;
static bool			log_close_request = false;
static bool			property_saving_request = false;
static utBool		syslogRunThread = utFalse;
static UInt32		syslogSwitch = 0;
static UInt32		current_log_index = 0;
static UInt32		log_control_props[3] = {0, (SYSLOG_DEFAULT_SIZE << 12), 0};
static int			log_maximum_size = (SYSLOG_DEFAULT_SIZE << 12);
static char			sys_log_name[2][64] = {SYSLOG_NAME0, SYSLOG_NAME1};
static char			cell_log_name[64];
char pppd_log_name[64] = "/mnt/flash/titan-data/pppd_act.log";

#if defined(INCLUDE_SYSLOG_SUPPORT)
static utBool               syslogLibrary = utFalse;
#endif

static threadMutex_t        syslogMutex;
static threadCond_t         syslogCond;
#define LOG_LOCK            MUTEX_LOCK(&syslogMutex);
#define LOG_UNLOCK          MUTEX_UNLOCK(&syslogMutex);
#define LOG_WAIT(T)         CONDITION_TIMED_WAIT(&syslogCond, &syslogMutex, T);
#define LOG_NOTIFY          CONDITION_NOTIFY(&syslogCond);

#if defined(SYSLOG_THREAD)
#  define LOG_BUFFER_SIZE   3000 // should be sufficient for most cases
static threadThread_t       syslogThread;
static CircleBuffer_t       *syslogBuffer = (CircleBuffer_t*)0;
static struct timespec      syslogCondWaitTime;
#  define LOG_WAIT(T)       CONDITION_TIMED_WAIT(&syslogCond,&syslogMutex,utcGetAbsoluteTimespec(&syslogCondWaitTime,(T)));
#  define LOG_NOTIFY        CONDITION_NOTIFY(&syslogCond);
#endif

#if defined(LOGGING_MESSAGE_FILE) && defined(LOGGING_AUX_DIRECTORY)
static FILE                 *syslogAuxFile = (FILE*)0;
#endif


bool				_isDebugMode = true;
threadThread_t       logThread;
// ----------------------------------------------------------------------------
extern int upload_log(UInt32 l, char *s);
extern utBool startupSaveProperties(UInt32 save_type);
// ----------------------------------------------------------------------------
int logInitialize(void);
static void * log_thread_main(void * arg);
static void closeLog(FILE * lfs); 
static int consolidate_stream(FILE *fcollector, const char *source_path);
static void cleanup_aux_log(const char *source_path);
static utBool save_log_property(void);
// ----------------------------------------------------------------------------

/* maintain debug mode */
/* set debug mode */
void setDebugMode(utBool mode)
{ 
	_isDebugMode = mode;
}

void request_log_collect(void)
{ 
	if (syslogSwitch & SYSLOG_WIFI) {
		log_collect_request = true;
		LOG_NOTIFY
	}
}

bool is_wifi_logging(void)
{ 
	return ((syslogSwitch & SYSLOG_WIFI) != 0);
}
// ----------------------------------------------------------------------------
/* extract the source file name from the full path (ie. from '__FILE__') */
const char *logSrcFile(const char *fn)
{
    if (fn && *fn) {
        int fnLen = strlen(fn), fi = fnLen - 1, fLen = 0;
        const char *f = (char*)fn, *fp = (char*)0;
        for (; fi >= 0; fi--) {
            if (fn[fi] == '.') { fp = &fn[fi]; }
            if ((fn[fi] == '/') || (fn[fi] == '\\')) { 
                f = &fn[fi + 1];
                fLen = fp? (fp - f) : (fnLen - (fi + 1));
                break;
            }
        }
        // 'f'    - points to the beginning of the source file name
        // 'fp'   - points to the (first) '.' before the extension
        // 'fLen' - is the length of the source file name without the extension
        return f; // just return the source file pointer
    } else {
        return fn;
    }
}

// ----------------------------------------------------------------------------

/* syslog output */
#if defined(INCLUDE_SYSLOG_SUPPORT)
static void _logSyslogPrint(int level, const char *trace, const char *fmt, va_list ap)
{
    char buf[MAX_MESSAGE_LENGTH];
    if (!(syslogSwitch & SYSLOG_NORMAL)) {logInitialize();} // <-- if not yet initialized
    int maxLen = 1 + strlen(trace) + 2 + strlen(fmt) + 1;
    if (trace && *trace && (maxLen < sizeof(buf))) {
        // combine 'trace' with 'fmt'
        sprintf(buf, "[%s] %s", trace, fmt);
        fmt = buf;
    }
    vsyslog(level, fmt, ap);
}
#endif

// ----------------------------------------------------------------------------

/* auxilliary output */
#if defined(LOGGING_MESSAGE_FILE) && defined(LOGGING_AUX_DIRECTORY)
static void _logAuxlogPrint(int level, const char *msg, int msgLen)
{
    // Log to aux storage.
    //  - Log to aux storage, but minimize the burden on the CPU.
    //  - Allow the aux storage media to be removed at any time.
        int ret = -1;
        time_t now;
        char fmt_buf[128];
        struct tm tm1, *ptm;
        va_list ap;
        va_start(ap, format);
        if (!cell_log_ctrl.active)
                ret = vprintf(format, ap);
        else {
                now = time(NULL);
                ptm = localtime_r(&now, &tm1);
                snprintf(fmt_buf, sizeof(fmt_buf),  "%02d/%02d/%4d %02d:%02d:%02d %s",
                        ptm->tm_mon + 1, ptm->tm_mday, ptm->tm_year + 1900,
                        ptm->tm_hour,ptm->tm_min, ptm->tm_sec, format);
                ret = cell_log_ctrl.print_func(fmt_buf, ap);
        }
        va_end(ap);
        return ret;
    if (syslogAuxFile || ioIsDirectory(LOGGING_AUX_DIRECTORY)) {
        // aux-storage dir exists (SD/MMC card is available)
        // Note: opening/closing the log file multiple times does impact
        // performance, so this feature should only be used for debugging purposes.
        if (!syslogAuxFile) {
            // open aux file
            syslogAuxFile = ioOpenStream(LOGGING_MESSAGE_FILE, IO_OPEN_APPEND);
        }
        if (syslogAuxFile) {
            // aux file is open, continue ...
            if (msgLen < 0) { msgLen = strlen(msg); } // includes '\n'
            if (fwrite(msg,1,msgLen,syslogAuxFile) < 0) {
                // write error occurred, close file
                ioCloseStream(syslogAuxFile);
                syslogAuxFile = (FILE*)0;
            }
        }
    }
}
#endif

#if defined(SYSLOG_THREAD)
/* console output */
#if defined(LOGGING_CONSOLE)
static void _logConsolePrint(int level, const char *msg, int msgLen)
{
    if ((level <= SYSLOG_INFO) || isDebugMode()) {
        fwrite(msg, 1, msgLen, CONSOLE_OUTPUT);
    }
}
#endif

/* lock and log */
static void _logMessagePrint(const char *msg, int msgLen)
{

    /* nothing to print? */
    if (!msg || !*msg || (msgLen <= 0)) {
        return;
    }

    /* skip first message character */
    int level = *msg - '0';
    const char *m = msg + 1;
    int mLen = msgLen - 1;

    /* aux file logging */
#if defined(LOGGING_MESSAGE_FILE) && defined(LOGGING_AUX_DIRECTORY)
    _logAuxlogPrint(level, m, mLen);
#endif

    /* console logging */
#if defined(LOGGING_CONSOLE)
    _logConsolePrint(level, m, mLen);
#endif

}

/* flush output */
static void _logMessageFlush()
{
#if defined(LOGGING_MESSAGE_FILE) && defined(LOGGING_AUX_DIRECTORY)
    if (syslogAuxFile) {
        //if (isDebugMode()) { fprintf(CONSOLE_OUTPUT, "Flush ...\n"); } // ignore returned error code
        ioCloseStream(syslogAuxFile); // flush & close
        syslogAuxFile = (FILE*)0;
    }
#endif
#if defined(LOGGING_CONSOLE)
    ioFlushStream(CONSOLE_OUTPUT);
#endif
}
#endif
// ----------------------------------------------------------------------------
int memorizeLog(const char *format, va_list ap) 
{
	int alen = 0;
	bool overflow_alert = false;
	if (syslogSwitch && log_ptr_tail != NULL) {
		if (log_ptr_tail - log_buffer >  LOG_BUF_ALERT)
			return 0;
		LOG_LOCK
		alen = vsnprintf(log_ptr_tail, MAX_MESSAGE_LENGTH, format, ap);
		log_ptr_tail += alen;
		if (log_ptr_tail - log_buffer > LOG_BUF_ALERT) {
			if (syslogRunThread) {
				LOG_NOTIFY
				overflow_alert = true;
			}
		}
		LOG_UNLOCK 
		if (overflow_alert)
			usleep(1000); //give log thread a chance to flush buffer
	}
	return alen;
}
int memorizeLogWrap(const char *format, ...)
{
	int ret;
	va_list ap;
	va_start(ap, format);
	ret = memorizeLog(format, ap);
	va_end(ap);
	return ret;
}
	
// ----------------------------------------------------------------------------
static char debug_format[MAX_MESSAGE_LENGTH];
static char warnings[6][12] = {"CRITICAL! ", "ERROR! ", "WARNING! ", ":", ":", ":"};
    // Levels:
    //   LOG_EMERG      0   [not used]
    //   LOG_ALERT      1   [not used]
    //   LOG_CRIT       2   
    //   LOG_ERR        3   
    //   LOG_WARNING    4   
    //   LOG_NOTICE     5   [not used]
    //   LOG_INFO       6   
    //   LOG_DEBUG      7
/* base logging print function */
static void _logPrint(utBool force, int level, const char *fn, int line, const char *fmt, va_list ap)
{
	char *lvlName;
    /*write to display*/
	if (_isDebugMode) {
		/* format the message */
		if (level < 2)
			lvlName = warnings[0];
		else
			lvlName = warnings[level - 2];
		// pre-pend function information
		snprintf(debug_format, MAX_MESSAGE_LENGTH, "[%s:%d]%s%s\n", fn, line, lvlName, fmt);
		vprintf(debug_format, ap);
	}
    /*write to log */
	if ((syslogSwitch & SYSLOG_NORMAL) && level <= syslogLevel) {
		snprintf(log_format, MAX_MESSAGE_LENGTH, "%s\n", fmt);
		memorizeLog(log_format, ap);
	}
}
// ----------------------------------------------------------------------------
#if defined(SYSLOG_THREAD)
/* return true if syslog is in effect */
utBool logIsSyslog()
{
#if defined(INCLUDE_SYSLOG_SUPPORT)
    return syslogLibrary;
#else
    return utFalse;
#endif
}

/* parse logging level name */
int logParseLevel(const char *level)
{
    if (level && *level) {
        // a prefixing '+'/'-' character means send log messages to 'stderr'
        // ('+' was included to allow for easier command-line parsing)
        int tty = (*level == '+') || (*level == '-')? -1 : 1;
        const char *lvl = (tty < 0)? (level + 1) : level;
        if (isdigit(*lvl)) {
            return tty * (int)strParseInt32(lvl, SYSLOG_ERROR);
        } else
        if (strStartsWithIgnoreCase(lvl,"cri")) {
            return tty * SYSLOG_CRITICAL;
        } else
        if (strStartsWithIgnoreCase(lvl,"err")) {
            return tty * SYSLOG_ERROR;
        } else
        if (strStartsWithIgnoreCase(lvl,"war")) {
            return tty * SYSLOG_WARNING;
        } else
        if (strStartsWithIgnoreCase(lvl,"inf")) {
            return tty * SYSLOG_INFO;
        } else
        if (strStartsWithIgnoreCase(lvl,"deb") || strStartsWithIgnoreCase(lvl,"dbg")) {
            return tty * SYSLOG_DEBUG;
        } else {
            return tty * SYSLOG_ERROR; // the default
        }
    } else {
        return SYSLOG_NONE;
    }
}

/* set logging level */
void logSetLevel(int level)
        int ret = -1;
        time_t now;
        char fmt_buf[128];
        struct tm tm1, *ptm;
        va_list ap;
        va_start(ap, format);
        if (!cell_log_ctrl.active)
                ret = vprintf(format, ap);
        else {
                now = time(NULL);
                ptm = localtime_r(&now, &tm1);
                snprintf(fmt_buf, sizeof(fmt_buf),  "%02d/%02d/%4d %02d:%02d:%02d %s",
                        ptm->tm_mon + 1, ptm->tm_mday, ptm->tm_year + 1900,
                        ptm->tm_hour,ptm->tm_min, ptm->tm_sec, format);
                ret = cell_log_ctrl.print_func(fmt_buf, ap);
        }
        va_end(ap);
        return ret;
{
	if (level <= 0)
		return;
	syslogLevel =  level;
}
#endif // defined(SYSLOG_THREAD)

int enableSyslog(UInt32 type, int log_size_4k)
{
	int ret = 0;
	UInt32 old_switch;

	if (type > 7)
		return -1;
	if (type > 0 && log_size_4k > 0) {
		if (syslogSwitch == 0) {
			if (logInitialize() != 0) {
				syslogSwitch = 0;
				return -1;
			}
		}
		old_switch = syslogSwitch;
		syslogSwitch = type & 7;
		if (old_switch != syslogSwitch) {
			if (syslogSwitch == SYSLOG_WIFI)
				syslogLevel = SYSLOG_CRITICAL;
			else
				syslogLevel = SYSLOG_DEBUG;
		}
		if (log_size_4k > 256)
			log_size_4k = 256;
		log_maximum_size = (log_size_4k << 12);
		if (syslogSwitch != log_control_props[0] || log_maximum_size != log_control_props[1]) {
			property_saving_request = true;
		}
	} 
	if (type == 0) {
		syslogSwitch = 0;
//		log_close_request = true;
		log_upload_request = false;
		}
	if (syslogSwitch > 0) {
		log_close_request = true;
		log_upload_request = true;
	}
//	printf("!!%s %d syslogSwitch %d\n", __FUNCTION__, __LINE__, syslogSwitch);
//	printf("!!%s %d log_upload_request %d\n", __FUNCTION__, __LINE__, log_upload_request);
	return ret;
}
// ----------------------------------------------------------------------------

/* log messages */
void logPrintf_(const char *ftn, int line, int level, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    _logPrint(utTrue, level, ftn, line, fmt, ap);
    va_end(ap);
}

/* log debug messages */
void logDebug_(const char *ftn, int line, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    _logPrint(utFalse, SYSLOG_DEBUG, ftn, line, fmt, ap);
    va_end(ap);
}

/* log info messages */
void logInfo_(const char *ftn, int line, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    _logPrint(utFalse, SYSLOG_INFO, ftn, line, fmt, ap);
    va_end(ap);
}

/* log warning messages */
void logWarning_(const char *ftn, int line, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    _logPrint(utFalse, SYSLOG_WARNING, ftn, line, fmt, ap);
    va_end(ap);
}

/* log error messages */
void logError_(const char *ftn, int line, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    _logPrint(utFalse, SYSLOG_ERROR, ftn, line, fmt, ap);
    va_end(ap);
}

/* log critical messages */
void logCritical_(const char *ftn, int line, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    _logPrint(utFalse, SYSLOG_CRITICAL, ftn, line, fmt, ap);
    va_end(ap);
}

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------

#if defined(SYSLOG_THREAD)

/* thread main */
void * _syslogThreadRunnable(void *arg)
{
    char data[MAX_MESSAGE_LENGTH + 2];
    int maxDataLen = sizeof(data);
    while (syslogRunThread) {
        
        /* wait for data */
        int len = 0;
        while (syslogRunThread && (len <= 0)) {
            LOG_LOCK {
                // get a single string
                len = bufferGetString(syslogBuffer, data, maxDataLen);
            } LOG_UNLOCK
            if (len <= 0L) {
                // queue is empty, flush output now (outside of the lock!)
                _logMessageFlush();
                // wait until we have something in the queue
                LOG_LOCK {
                    if (bufferGetLength(syslogBuffer) <= 0L) {
                        // still nothing in the queue
                        // wait a few seconds, or until we get notified
                        LOG_WAIT(5000L);
                    }
                } LOG_UNLOCK
            }
        }
        if (!syslogRunThread) {
            break;
        }

        /* print string */
        //fprintf(CONSOLE_OUTPUT, "{Thread} ");  // ignore returned error code
        _logMessagePrint(data, len);

    }
    // once this thread stops, it isn't starting again
    logERROR(LOGSRC,"Stopping thread");
    threadExit();
    return NULL;
}


/* start serial communication thread */
static utBool _syslogStartThread()
{
    
    /* thread already running? */
    if (syslogRunThread) {
        return utTrue;
    }
    
    /* init */
    threadConditionInit(&syslogCond);
    syslogBuffer = bufferCreate(LOG_BUFFER_SIZE);

    /* create thread */
    syslogRunThread = utTrue;
    if (threadCreate(&syslogThread,&_syslogThreadRunnable,0,"Syslog") == 0) {
        // thread started successfully
        threadAddThreadStopFtn(&_syslogStopThread,0);
        return utTrue;
    } else {
        syslogRunThread = utFalse;
        return utFalse;
    }
    
}

#endif // defined(SYSLOG_THREAD)

/* call-back to stop thread */
static void logStopThread(void *arg)
{
	syslogRunThread = utFalse;
	LOG_LOCK
	LOG_NOTIFY 
	LOG_UNLOCK
}

void * log_thread_main(void * arg)  
{
	int rt; 
	ssize_t wlen, len1, upload_size = 0, log_size = 0;
	struct timespec later;
	bool closing_consolidated = false;
	UInt32 upload_index = 0, saving_tick = 0;
	char *main_log_path;
	struct stat stat1, stat2; 
	FILE *fup = NULL, *log_fs = NULL;

	if (syslogSwitch == 0) {
		syslogSwitch = propGetUInt32AtIndex(PROP_STATE_DIAGNOSTIC, 0, 0);
		if (syslogSwitch != 0) {
			if (logInitialize() != 0) {
				printf("Error in initializing log buffer\n");
				syslogSwitch = 0;
			} else {
				if (syslogSwitch == SYSLOG_WIFI)
					syslogLevel = SYSLOG_CRITICAL;
				else
					syslogLevel = SYSLOG_DEBUG;
				log_maximum_size = propGetUInt32AtIndex(PROP_STATE_DIAGNOSTIC, 1, (SYSLOG_DEFAULT_SIZE << 12));
			}
		}
	}
	if (syslogSwitch != 0) {
		int r1 = 0, r2 = 0;
		current_log_index = propGetUInt32AtIndex(PROP_STATE_DIAGNOSTIC, 2, 0) & 1;
		upload_index = current_log_index ^ 1;
		log_control_props[2] = current_log_index;
		r1 = stat(sys_log_name[upload_index], &stat1);
		r2 = stat(sys_log_name[current_log_index], &stat2);
		if (r1 == 0 && r2 == 0) {
			if ((fup = fopen(sys_log_name[upload_index], "a+")) == NULL)
				goto main_loop;
			consolidate_stream(fup, sys_log_name[current_log_index]);
		} else {
			if (r1 < 0 && r2 == 0) {
				upload_index = current_log_index;
				current_log_index ^= 1;
			}
			if ((fup = fopen(sys_log_name[upload_index], "a+")) == NULL)
				goto main_loop;
		}
		if (syslogSwitch & SYSLOG_WIFI) {
			len1 = consolidate_stream(fup, cell_log_name);
			if (len1 < 0 && (r1 < 0 && r2 < 0)) {
				fclose(fup);
				goto main_loop;
			}
		}
		if (fseek(fup, 0, SEEK_END) == 0) {
			if ((upload_size = ftell(fup)) > 0) {
//				printf("!! %s ftell %d\n", __FUNCTION__, upload_size);
				log_upload_request = true;
				}
		}
		fclose(fup);
		log_control_props[0] = syslogSwitch;
		log_control_props[1] = log_maximum_size;
		if (current_log_index != log_control_props[2])
			property_saving_request = true;
	}
main_loop:
	syslogRunThread = utTrue;
	while (syslogRunThread) { 
//	printf("!! %s %d syslogSwitch %d log_upload_request %d\n", __FUNCTION__, __LINE__, syslogSwitch, log_upload_request);
		if (syslogSwitch != 0 && log_buffer != NULL) {
			main_log_path = sys_log_name[current_log_index];
			if ((stat(main_log_path, &stat1) < 0) || (log_fs == NULL )) {
				if ((log_fs = fopen(main_log_path, "w+")) == NULL) {
					perror("Create Log");
					sleep(LOG_MAIN_PERIOD);
					closeLog(log_fs);
					continue;
				}
				log_size = 0;
			}
			saving_tick = 0;
			LOG_LOCK 
			while (((saving_tick++ < 4) || 
				(log_ptr_tail - log_buffer == 0)) && (syslogRunThread)) {
				if (clock_gettime(CLOCK_REALTIME, &later) != 0) {
                   			perror("Realtime Clock:");
                   			break;
                		}
				rt_time = later.tv_sec;
				later.tv_sec += LOG_MAIN_PERIOD;
				rt = LOG_WAIT(&later)
				if ((rt != ETIMEDOUT) || (log_ptr_tail - log_buffer > LOG_BUF_FULL))
					break;
//printf("!!%s %d  saving_tick = %d\n", __FUNCTION__, __LINE__, saving_tick);
			}
			wlen = log_ptr_tail - log_buffer;
			LOG_UNLOCK
			if (wlen > 0) {
				if ((wlen = fwrite(log_buffer, 1, wlen, log_fs)) <= 0)
					perror("Writing log");
				else {
					log_size += wlen;
					fflush(log_fs);
				}
				LOG_LOCK 
				if ((log_ptr_tail - log_buffer) <= wlen)
					log_ptr_tail = log_buffer;
				else {
					memmove(log_buffer, log_buffer + wlen,  log_ptr_tail - log_buffer - wlen);
					log_ptr_tail -= wlen;
				}
				LOG_UNLOCK
			}
			if (log_collect_request) {
				len1 = consolidate_stream(log_fs, cell_log_name); 
				if (len1 > 0)
					log_size += len1;
				log_collect_request = false;
				cleanup_aux_log(cell_log_name);
			}
			if (!syslogRunThread) {
				printf("LOG thread stops\n");
				break;
			}
//printf("!! %s %d log_upload_request = %d\n", __FUNCTION__, __LINE__, log_upload_request);
			if (log_upload_request) {
				if (log_close_request && !closing_consolidated) {
					upload_index = current_log_index ^ 1;
					if (stat(sys_log_name[upload_index], &stat1) == 0) {
						if ((fup = fopen(sys_log_name[upload_index], "a")) != NULL) {
							fclose(log_fs);
							consolidate_stream(fup, sys_log_name[current_log_index]);
							if (syslogSwitch & SYSLOG_WIFI)
								consolidate_stream(fup, cell_log_name);
							fseek(fup, 0, SEEK_END);
							upload_size = ftell(fup);
							fclose(fup);
						}
					} else {
						if (syslogSwitch & SYSLOG_WIFI)
							consolidate_stream(log_fs, cell_log_name);
						fseek(log_fs, 0, SEEK_END);
						upload_size = ftell(log_fs);
						fclose(log_fs);
						upload_index = current_log_index;
					}
					log_fs = NULL;
					closing_consolidated = true;
					syslogLevel = SYSLOG_NONE;
				}
				if (upload_log(upload_size, sys_log_name[upload_index]) == 0) {
					log_upload_request = false;
					if (log_close_request) {
						closeLog(NULL);
						closing_consolidated = false;
						property_saving_request = true;
						continue;
					}
				}
			}
			if (log_size >= log_maximum_size && !log_upload_request) {
				fclose(log_fs);
				log_fs = NULL;
				current_log_index ^= 1;
				property_saving_request = true;
			}
		}
		else 
			sleep(LOG_MAIN_PERIOD);

		if (property_saving_request) {
			save_log_property();
			property_saving_request = false;
		}
	}
	closeLog(log_fs);
	return NULL;
}

/* start serial communication thread */
utBool logStartThread()
{
	/* init log mutex */
	threadMutexInit(&syslogMutex);
	threadConditionInit(&syslogCond);
	if (threadCreate(&logThread, &log_thread_main, &current_log_index, "Syslog") == 0) {
		// thread started successfully
		threadAddThreadStopFtn(&logStopThread, 0);
		return utTrue;
	} else {
		syslogRunThread = utFalse;
		return utFalse;
	}
}

// ----nt ret = -1;
//         time_t now;
//                 char fmt_buf[128];
//                         struct tm tm1, *ptm;
//                                 va_list ap;
//                                         va_start(ap, format);
//                                                 if (!cell_log_ctrl.active)
//                                                                 ret = vprintf(format, ap);
//                                                                         else {
//                                                                                         now = time(NULL);
//                                                                                                         ptm = localtime_r(&now, &tm1);
//                                                                                                                         snprintf(fmt_buf, sizeof(fmt_buf),  "%02d/%02d/%4d %02d:%02d:%02d %s",
//                                                                                                                                                 ptm->tm_mon + 1, ptm->tm_mday, ptm->tm_year + 1900,
//                                                                                                                                                                         ptm->tm_hour,ptm->tm_min, ptm->tm_sec, format);
//                                                                                                                                                                                         ret = cell_log_ctrl.print_func(fmt_buf, ap);
//                                                                                                                                                                                                 }
//                                                                                                                                                                                                         va_end(ap);
//                                                                                                                                                                                                                 return ret;
//                                                                                                                                                                                                                 ------------------------------------------------------------------------
/* logging initialization */
extern int PPP_DEBUG;
int logInitialize(void)
{
        /* save name */
	if (log_buffer == NULL) {
		if ((log_buffer = malloc(LOG_BUFFER_SIZE)) == NULL)
			return -1;
		if ((hex_disp = malloc(MAX_MESSAGE_LENGTH * 3)) == NULL)
			return -1;
		if ((collect_buffer = malloc(LOG_COLLECT_SIZE)) == NULL)
			return -1;
	}
	log_ptr_tail = log_buffer;
	memset(log_buffer, 0, LOG_BUFFER_SIZE);
	log_format = hex_disp + MAX_MESSAGE_LENGTH * 2;
	memset(hex_disp, 0, MAX_MESSAGE_LENGTH * 3);
	if (PPP_DEBUG == 1) {
		ppp_debug_func(memorizeLog);
		strncpy(cell_log_name, pppd_log_name, sizeof(cell_log_name));
	}
	return 0;
}
// ----------------------------------------------------------------------------
/* logging close*/
void closeLog(FILE * lfs) 
{
	syslogSwitch = 0;
	if (log_buffer != NULL) {
		free(log_buffer);
		free(hex_disp);
		log_buffer = NULL;
	}
	if (lfs != NULL)
		fclose(lfs);
}	
// ----------------------------------------------------------------------------
int print_debug(const char *format, ...)
{
	int len = 0;
	va_list ap;
	va_start(ap, format);
	if (_isDebugMode) {
		len = vprintf(format, ap);
	}
	if ((syslogSwitch & SYSLOG_NORMAL) && syslogLevel == SYSLOG_DEBUG) {
		len = memorizeLog(format, ap);
	}
	va_end(ap);
	return len;
}
void print_tag_raw(unsigned char *raw)
{
	unsigned char *r;
	if (syslogSwitch & SYSLOG_TAG) { 
		r = raw;
		memorizeLogWrap("^%08X:%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X\n",
				rt_time, r[0],r[1],r[2],r[3],r[4],r[5],r[6],r[7],r[8],r[9],r[10],r[11],
				r[12],r[13],r[14],r[15],r[16],r[17],r[18]); 
	}
}
int print_hex(const char *header, const UInt8 *hex, int len)
{
	const UInt8 *hp;
	char *dp;
	int i;
	if (_isDebugMode) {
		printf("%s:", header);
		hp = hex;
		for (i = 0; i < (len / 4); i++) {
			printf("%02X%02X%02X%02X", hp[0], hp[1], hp[2], hp[3]);
			hp += 4;
		}
		if ((len % 4) == 1)
			printf("%02X\n", hp[0]);
		else if ((len % 4) == 2)
			printf("%02X%02X\n", hp[0], hp[1]);
		else if ((len % 4) == 3)
			printf("%02X%02X%02X\n", hp[0], hp[1], hp[2]);
		else
			printf("\n");
	}
	if ((syslogSwitch & SYSLOG_NORMAL) && syslogLevel == SYSLOG_DEBUG) {
		dp = hex_disp;
		sprintf(dp, "%s:", header);
		dp += strnlen(hex_disp, MAX_MESSAGE_LENGTH);
		hp = hex;
		for (i = 0; i < (len / 4); i++) {
			sprintf(dp, "%02X%02X%02X%02X", hp[0], hp[1], hp[2], hp[3]);
			hp += 4;
			dp += 8;
		}
		if ((len % 4) == 1)
			sprintf(dp, "%02X\n", hp[0]);
		else if ((len % 4) == 2)
			sprintf(dp, "%02X%02X\n", hp[0], hp[1]);
		else if ((len % 4) == 3)
			sprintf(dp, "%02X%02X%02X\n", hp[0], hp[1], hp[2]);
		else
			sprintf(dp, "\n");
		len = memorizeLogWrap("%s", hex_disp);
	}
	return len;
}
int print_critical(const char *format, ...)
{
	int len = 0;
	va_list ap;
	char time_stamp[MAX_MESSAGE_LENGTH];
	va_start(ap, format);
	if (_isDebugMode) {
		len = vprintf(format, ap);
	}
	if (syslogSwitch & SYSLOG_NORMAL) {
		time_t now;
		struct tm tm1, *ptm; 
		now = time(NULL);
		ptm = localtime_r(&now, &tm1);
		sprintf(time_stamp, "%02d/%02d/%4d %02d:%02d:%02d ",
				ptm->tm_mon+1, ptm->tm_mday, ptm->tm_year+1900, 
				ptm->tm_hour, ptm->tm_min, ptm->tm_sec); 
		strncat(time_stamp, format, MAX_MESSAGE_LENGTH);
		len = memorizeLog(time_stamp, ap);
	}
	va_end(ap);
	return len;
}
UInt32 get_log_type(void)
{
	return (syslogSwitch & SYSLOG_TYPE_MASK);
}
int consolidate_stream(FILE *fcollector, const char *source_path)
{
	FILE *fsrc;
	size_t rlen = 0, alen = 0;
	
	if ((fsrc = fopen(source_path, "r")) == NULL)
		return -1;
	while (!feof(fsrc)) {
		rlen = fread(collect_buffer, 1, LOG_COLLECT_SIZE, fsrc);
		if (rlen <= 0)
			break;
		alen += fwrite(collect_buffer, 1, rlen, fcollector);
	}
	return alen;
}
void cleanup_aux_log(const char *source_path)
{
	usleep(100000);
	unlink(source_path);
}
utBool save_log_property(void)
{
	log_control_props[0] = syslogSwitch;
	log_control_props[1] = log_maximum_size;
	log_control_props[2] = current_log_index;
	propSetUInt32_a(PROP_STATE_DIAGNOSTIC, log_control_props, 3);
	return (startupSaveProperties(1));
}
