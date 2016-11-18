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
//  Timer tools
//  Various timer based tools.
// ---
// Change History:
//  2006/01/04  Martin D. Flynn
//      -Initial release
//  2007/01/28  Martin D. Flynn
//      -WindowsCE port
//      -Added 'utcGetTimestampDelta'
//      -If ENABLE_SET_TIME is not defined, 'utcSetTimeSec' will instead save the
//      time offset between the local system time and UTC so that 'utcGetTimeSec'
//      will now return the corrected UTC time.
// ----------------------------------------------------------------------------

#define SKIP_TRANSPORT_MEDIA_CHECK // only if TRANSPORT_MEDIA not used in this file 
#include "defaults.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <time.h>

#include "log.h"

#include "threads.h"
#include "utctools.h"

// ----------------------------------------------------------------------------
// 1 second == 1000 milliseconds == 1000,000 microseconds = 1000,000,000 nanoseconds
// ----------------------------------------------------------------------------

/* offset from system clock time and UTC time */
#if !defined(ENABLE_SET_TIME)
// utcSec = now + systemTimeOffet;
static Int32 systemTimeOffet = 0L;
#endif

/* system startup time */
static UInt32 startupTime = 0L;
static char tz_utc[8] = "UTC0";
void utcMarkStartupTime()
{
	setenv("TZ", tz_utc, 1);
    // this could set time startup time to '0' (or near zero) if the system
    // clock hasn't been updated by GPS yet.
    startupTime = utcGetTimeSec();
    // We back up one second so that all timer initialized right at the startup of the
    // program will not be zero.
    if (startupTime > 0L) { startupTime--; }
}

UInt32 utcGetStartupTimeSec()
{
    return startupTime;
}

void utcSetStartupTimeSec(UInt32 newStartupTime)
{
    startupTime = newStartupTime;
}
// ----------------------------------------------------------------------------

/* convert YMDHMS to UTC seconds since Epoch */
UInt32 utcYmdHmsToSeconds(YMDHMS_t *yh)
{
    long TOD = (yh->wHour * 3600L) + (yh->wMinute * 60L) + yh->wSecond;
    long yr  = ((long)yh->wYear * 1000L) + (long)(((yh->wMonth - 3) * 1000) / 12);
    long dd  = yh->wDay;
    long DAY = ((367L * yr + 625L) / 1000L) - (2L * (yr / 1000L))
             + (yr / 4000L) - (yr / 100000L) + (yr / 400000L)
             + (long)dd - 719469L; // offset to 'Epoch'
    return (UInt32)((DAY * 24L * 60L * 60L) + TOD);
}

/* convert UTC EPoch seconds to YMDHMS */
YMDHMS_t *utcSecondsToYmdHms(YMDHMS_t *yh, UInt32 utcSec)
{
    memset(yh, 0, sizeof(YMDHMS_t));
    Int32 TOD    = utcSec % (24L * 60L * 60L); // in seconds
    yh->wHour    = (int)(TOD / (60L * 60L));
    yh->wMinute  = (int)((TOD % (60L * 60L)) / 60L);
    yh->wSecond  = (int)((TOD % (60L * 60L)) % 60L);
    Int32 N      = (utcSec / (24L * 60L * 60L)) + 719469L;
    Int32 C      = ((N * 1000L) - 200L) / 36524250L;
    Int32 N1     = N + C - (C / 4L);
    Int32 Y1     = ((N1 * 1000L) - 200L) / 365250L;
    Int32 N2     = N1 - ((365250L * Y1) / 1000L);
    Int32 M1     = ((N2 * 1000L) - 500L) / 30600L;
    yh->wDay     = (int)(((N2 * 1000L) - (30600L * M1) + 500L) / 1000L);
    yh->wMonth   = (int)((M1 <= 9L)? (M1 + 3L) : (M1 - 9L));
    yh->wYear    = (int)((M1 <= 9L)? Y1 : (Y1 + 1));
    return yh;
}

/* format date/time */
// The destination buffer is assumed to be at least 20 bytes in length
// the output format will be: "YYYY/MM/DD hh:mm:ss" (with a terminating null)
const char *utcFormatDateTime(char *dt, UInt32 utcSec)
{
    if (dt) {
        YMDHMS_t yh;
        utcSecondsToYmdHms(&yh, utcSec);
        sprintf(dt, "%04d/%02d/%02d %02d:%02d:%02d", 
            yh.wYear, yh.wMonth , yh.wDay, yh.wHour, yh.wMinute, yh.wSecond);
    }
    return dt;
}

// ----------------------------------------------------------------------------

/* get a timestamp with millisecond resolution (based on actual current system clock) */
// This may not provide an accurate UTC time!
static struct timeval *_utcGetTimestamp(struct timeval *tv)
{
    if (tv) {
        // struct timeval:
        //   tv_sec     - seconds
        //   tv_usec    - microseconds
#if defined(TARGET_WINCE)
        SYSTEMTIME st;
        GetSystemTime(&st); // UTC
        long TOD = (st.wHour * 3600L) + (st.wMinute * 60L) + st.wSecond;
        long yr  = ((long)st.wYear * 1000L) + (long)(((st.wMonth - 3) * 1000) / 12);
        long dd  = st.wDay;
        long DAY = ((367L * yr + 625L) / 1000L) - (2L * (yr / 1000L))
                 + (yr / 4000L) - (yr / 100000L) + (yr / 400000L)
                 + (long)dd - 719469L; // offset to 'Epoch'
        long sec = (DAY * 24L * 60L * 60L) + TOD;
        tv->tv_usec = (UInt32)st.wMilliseconds * 1000L;
        tv->tv_sec  = (UInt32)sec;
#else
        gettimeofday(tv, 0);
#endif
    }
    return tv;
}

/* get a timestamp with millisecond resolution (adjusted for UTC time) */
struct timeval *utcGetTimestamp(struct timeval *tv)
{
    if (tv) {
        _utcGetTimestamp(tv);
#if !defined(ENABLE_SET_TIME)
        // apply offset to UTC time
        tv->tv_sec = tv->tv_sec + systemTimeOffet;
#endif
    }
    return tv;
}

/* get a timestamp with the specified number of millisecond in the future */
struct timeval *utcGetTimestampDelta(struct timeval *tv, Int32 deltaMS)
{
    if (tv) {
        utcGetTimestamp(tv);
        Int32 sec    = (Int32)tv->tv_sec  + deltaMS / 1000L;
        Int32 usec   = (Int32)tv->tv_usec + (deltaMS % 1000L) * 1000L;
        if (usec > 1000000L) {
            usec -= 1000000L;
            sec  += 1L;
        } else
        if (usec < 0L) {
            usec += 1000000L;
            sec  -= 1L;
        }
        tv->tv_sec  = sec;
        tv->tv_usec = usec;
    }
    return tv;
}

/* return the difference between two timestamps in milliseconds */
UInt32 utcGetDeltaMillis(struct timeval *ts1, struct timeval *ts2)
{
    if (ts1 == ts2) {
        // if they point to the same structure, or are both null
        return 0L;
    } else {
        struct timeval now;
        if (!ts1) {
            utcGetTimestamp(&now);
            ts1 = &now;
        }
        if (!ts2) {
            utcGetTimestamp(&now);
            ts2 = &now;
        }
        Int32 delta = ((ts1->tv_sec - ts2->tv_sec) * 1000L) + ((ts1->tv_usec - ts2->tv_usec) / 1000L);
        return (UInt32)((delta >= 0L)? delta : -delta); // absolute value
    }
}

// ----------------------------------------------------------------------------

/* return current time in seconds */
static UInt32 _utcGetTimeSec()
{
#if defined(TARGET_WINCE)
    struct timeval tv;
    _utcGetTimestamp(&tv);
    return (UInt32)tv.tv_sec;
#else
    return (UInt32)time((time_t*)0);
#endif
}

/* return current time in seconds (adjusted for UTC) */
UInt32 utcGetTimeSec()
{
    UInt32 ts = _utcGetTimeSec();
#if !defined(ENABLE_SET_TIME)
    ts = ts + systemTimeOffet;
    if (ts < MIN_CLOCK_TIME) {
        // the time has not yet been initialized by the GPS!
    }
#endif
    return ts;
}

/* set current time in seconds */
utBool utcSetTimeSec(UInt32 utcSec)
{
    // Notes:
    // - This implementation is platform specific.  On Linux systems, this can be
    //   accomplished with the 'settimeofday' function.
    // - IMPORTANT: When setting this time, it is very important that the value of
    //   'startupTime' also be adjusted accordingly so that time deltas will still
    //   be calculated properly!
    // - This time can be set to the latest time aquired from the GPS fix.  However,
    //   It shouldn't be set on every new GPS fix, but rather only when the current
    //   system clock has drifted beyond some acceptable tolarance (eg. 5 seconds).

    /* get the elapsed time since startup */
    UInt32 preAdjustedTime = utcGetTimeSec();
    UInt32 startupTimeDeltaSec = preAdjustedTime - startupTime;

#if defined(ENABLE_SET_TIME)

    /* set current clock time */
#if defined(TARGET_WINCE)
    // this may not actually work on all WinCE platforms!  Test it!
    SYSTEMTIME st;
    utcSecondsToYmdHms((YMDHMS_t*)&st, utcSec);
    //logINFO(LOGSRC,"UTC: %d/%d/%d %d:%d:%d", st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond);
    SetSystemTime(&st); // UTC
#else
    //int settimeofday(const struct timeval *tv , const struct timezone *tz);
    struct timeval tv;
    tv.tv_sec         = utcSec;
    tv.tv_usec        = 0;
    struct timezone tz;
    tz.tz_minuteswest = 0;
    tz.tz_dsttime     = 0;
    settimeofday(&tv, &tz);
#endif // defined(TARGET_WINCE)
    
#else

    /* reset system time offset */
    UInt32 now = _utcGetTimeSec(); // unadjusted
    systemTimeOffet = (Int32)(utcSec - now);

#endif // defined(ENABLE_SET_TIME)

    /* print new time */
    char ts[32];
    utcFormatDateTime(ts, utcGetTimeSec());
    logINFO(LOGSRC,"Synchronized clock: %s UTC\n", ts);

    /* adjust startup time */
    UInt32 postAdjustedTime = utcGetTimeSec();
    if (postAdjustedTime > startupTimeDeltaSec) {
        // This should ALWAYS be true
        startupTime = postAdjustedTime - startupTimeDeltaSec;
        // 'startupTime' now still reflects that startup occured 'startupTimeDeltaSec' 
        // seconds ago.
    } else {
        // This means that the 'startupTime' was previously initialized to some 
        // terribly erroneous value.  Reset the startup time to the current time.
        // All initialized timers will be effected.
        startupTime = postAdjustedTime;
    }

    /* return successful */
    return utTrue;

}

// ----------------------------------------------------------------------------
// Timers:
// These functions exist to eliminate the inaccuracy on the system clock
// (that is, when the current system clock does not, or cannot, provide an 
// accurate current UTC time).

/* get a timer base on the current time */
TimerSec_t utcGetTimer()
{
    UInt32 time = utcGetTimeSec();
    TimerSec_t upTime = UTC_TO_TIMER(time);
    return upTime;
}

/* return the age of the timer in seconds */
Int32 utcGetTimerAgeSec(TimerSec_t timerSec)
{
    TimerSec_t baseTimeSec = utcGetTimer();
    Int32 delta = (Int32)baseTimeSec - (Int32)timerSec;
    return delta;
}

/* return true if the timer has elapsed beyond the specified interval */
utBool utcIsTimerExpired(TimerSec_t timerSec, Int32 intervalSec)
{

    /* return true if the timer has not yet been initialized */
    if (timerSec <= 0L) {
        return utTrue;
    }

    /* return true if there is no timeout interval */
    if (intervalSec <= 0L) {
        return utTrue;
    }

    /* return true if 'intervalSec' has passed */
    if (utcGetTimerAgeSec(timerSec) > intervalSec) {
        return utTrue;
    } else {
        return utFalse;
    }

}

// ----------------------------------------------------------------------------

/* get an absolute time 'offsetMS' milliseconds into the future */
struct timespec *utcGetAbsoluteTimespec(struct timespec *ts, UInt32 offsetMS)
{
    if (ts) {
        struct timeval tv;
        utcGetTimestamp(&tv);
        ts->tv_sec  = tv.tv_sec;
        ts->tv_nsec = tv.tv_usec * 1000L;               // micro -> nano
        if (offsetMS >= 1000L) {                        // at least 1 second?
            ts->tv_sec += offsetMS / 1000L;             // milli -> sec
            offsetMS %= 1000L;                          // strip seconds
        }
        ts->tv_nsec += offsetMS * 1000000L;             // milli -> nano
        if (ts->tv_nsec >= 1000000000L) {               // at least 1 second?
            ts->tv_sec += ts->tv_nsec / 1000000000L;    // nano-> sec
            ts->tv_nsec %= 1000000000L;                 // strip seconds
        }
    }
    return ts;
}

// ----------------------------------------------------------------------------
