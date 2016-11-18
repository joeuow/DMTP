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
//  Connection accounting module.
//  Keeps track of DMT service provider connection policy.
// ---
// Change History:
//  2006/01/04  Martin D. Flynn
//     -Initial release
//  2007/01/28  Martin D. Flynn
//     -WindowsCE port
//     -Default values are always returned for file & serial transport.  
// ----------------------------------------------------------------------------
#if !defined(PROTOCOL_THREAD)
#include "defaults.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <time.h>

#include "log.h"

#include "stdtypes.h"
#include "utctools.h"

#include "propman.h"
#include "accting.h"

// ----------------------------------------------------------------------------

#if !defined(TRANSPORT_MEDIA_FILE) && !defined(TRANSPORT_MEDIA_SERIAL)
static ConnectionMask_t     duplexConnectMask;
static ConnectionMask_t     simplexConnectMask;
#endif

// ----------------------------------------------------------------------------

/* return the length of the connection time accounting mask in 30 minute
** intervals.  ie. a return value of 6 indicates 6-30 minute intervals, or
** a total of 3 hours */
#if !defined(TRANSPORT_MEDIA_FILE) && !defined(TRANSPORT_MEDIA_SERIAL)
static UInt16 _acctGetMaskLen()
{
    UInt16 newMaskLen = 2;
    UInt16 maxMinutes = (UInt16)propGetUInt32AtIndex(PROP_COMM_MAX_CONNECTIONS, 2, 60L);
    UInt16 len = (maxMinutes + 15) / 30; // round to nearest half-hour
    if (len < 1) {
        newMaskLen = 1;
    } else
    if (len > MAX_MASK_SIZE) {
        newMaskLen = MAX_MASK_SIZE;
    } else {
        newMaskLen = len;
    }
    return newMaskLen;
}
#endif

/* shift connection accounting mask by the specified number of minutes. */
#if !defined(TRANSPORT_MEDIA_FILE) && !defined(TRANSPORT_MEDIA_SERIAL)
static int _acctShiftMinutesMask(ConnectionMask_t *conn, UInt32 minutes)
{
    
    /* has enought time elapsed? */
    if (minutes <= 0L) {
        // nothing to shift
        return 0;
    }

    /* maximum possible minutes have elapsed */
    UInt16 maskLen = _acctGetMaskLen();
    if (minutes >= ((UInt32)maskLen * 30L)) {
        // enough time has passed to reset everything
        memset(conn->mask, 0, sizeof(conn->mask)); // clear entire buffer
        return maskLen;
    }
    
    /* shift */
    while (minutes > 0L) {
        UInt32 maxMin = (minutes <= 30L)? minutes : 30L; // shift a maximum of 30 minutes at a time
        UInt32 carry = 0L;
        UInt16 i;
        for (i = 0; i < maskLen; i++) {
            UInt32 c = (conn->mask[i] >> (30 - maxMin)) & ((1L << maxMin) - 1L);
            conn->mask[i] = ((conn->mask[i] << maxMin) | carry) & 0x3FFFFFFFL;
            carry = c;
        }
        minutes -= maxMin;
    }
    return maskLen; // did shift

}
#endif

/* shift connection accounting mask by the specified number of minutes */
#if !defined(TRANSPORT_MEDIA_FILE) && !defined(TRANSPORT_MEDIA_SERIAL)
static UInt16 _acctShiftMinutes(ConnectionMask_t *conn)
{
    
    /* check elapsed time */
    TimerSec_t nowTimer  = utcGetTimer();
    UInt32 deltaTime = nowTimer - conn->shiftTime;
    UInt32 minutes   = deltaTime / 60L;
    if (minutes <= 0L) {
        // not enough time has passed since last shift
        return 0;
    }
    conn->shiftTime = nowTimer - (deltaTime % 60L);
    
    /* shift */
    return _acctShiftMinutesMask(conn, minutes);
    
}
#endif

// ----------------------------------------------------------------------------

/* return the number of connections made in the masked time interval */
#if !defined(TRANSPORT_MEDIA_FILE) && !defined(TRANSPORT_MEDIA_SERIAL)
static UInt16 _acctCountConnections(ConnectionMask_t *conn)
{
    UInt16 i, count = 0;
    UInt16 maskLen = _acctShiftMinutes(conn);
    for (i = 0; i < maskLen; i++) {
        UInt32 bc = conn->mask[i];
        bc -= (bc & 0xAAAAAAAAL) >> 1; // <-- the counting ocurs here, the rest is addition
        bc  = (bc & 0x33333333L) + ((bc >> 2) & 0x33333333L); // add with carry
        bc  = (bc + (bc >>  4)) & 0x0F0F0F0FL; // add, no carry
        bc  = (bc + (bc >>  8)) & 0x00FF00FFL; // */ bc += bc >>  8;  // add, no carry
        bc  = (bc + (bc >> 16)) & 0x000000FFL; // */ bc += bc >> 16;  // add, no carry
        count += (UInt16)(bc & 0xFF);
    }
    return count;
}
#endif

// ----------------------------------------------------------------------------

/* mark that a connection has occured in the mask */
#if !defined(TRANSPORT_MEDIA_FILE) && !defined(TRANSPORT_MEDIA_SERIAL)
static utBool _acctSetConnection(ConnectionMask_t *conn)
{
    
    /* save last connection time */
    conn->lastConnTime = utcGetTimer();
    
    /* mark this connection in the mask */
    _acctShiftMinutes(conn);
    // we assume here that 'maskLen' is at-least '1'
    if (conn->mask[0] & 1L) {
        // Already set.
        // This indicates that more than one transmission has occurred in the same minute.
        // The absolute minimum time delay is apparently not working.
        return utFalse;
    } else {
        conn->mask[0] |= 1L;
        return utTrue;
    }
    
}
#endif

/* mark that a duplex connection has been made */
utBool acctSetDuplexConnection()
{
#if defined(TRANSPORT_MEDIA_FILE)
    return utFalse; // this never occurs for file transport
#elif defined(TRANSPORT_MEDIA_SERIAL)
    return utTrue; // always successful for serial transport
#else
    return _acctSetConnection(&duplexConnectMask);
#endif
}

/* mark that a simplex connection has been made */
utBool acctSetSimplexConnection()
{
#if defined(TRANSPORT_MEDIA_FILE)
    return utTrue; // always successful for file transport
#elif defined(TRANSPORT_MEDIA_SERIAL)
    return utFalse; // this never occurs for serial transport
#else
    return _acctSetConnection(&simplexConnectMask);
#endif
}

// ----------------------------------------------------------------------------

/* return the time of the last connection */
#if !defined(TRANSPORT_MEDIA_FILE) && !defined(TRANSPORT_MEDIA_SERIAL)
static TimerSec_t _acctGetLastConnectionTime()
{
    return (duplexConnectMask.lastConnTime > simplexConnectMask.lastConnTime)?
        duplexConnectMask.lastConnTime : simplexConnectMask.lastConnTime;
}
#endif

// ----------------------------------------------------------------------------

/* return true if quotas are in effect */
utBool acctHasQuota()
{
#if defined(TRANSPORT_MEDIA_FILE)
    return utFalse; // no quotas for file transport
#elif defined(TRANSPORT_MEDIA_SERIAL)
    return utFalse; // no quotas for serial transport
#else
    UInt32 maxMinutes = propGetUInt32AtIndex(PROP_COMM_MAX_CONNECTIONS, 2, 60L);
    return (maxMinutes > 0L)? utTrue : utFalse;
#endif
}

// ----------------------------------------------------------------------------

/* return true if we are currently under the total allowed connection count */
utBool acctUnderTotalQuota()
{
#if defined(TRANSPORT_MEDIA_FILE)
    return utTrue; // always under quota for file transport
#elif defined(TRANSPORT_MEDIA_SERIAL)
    return utTrue; // always under quota for serial transport
#else
    if (acctHasQuota()) {

        /* check total connection limit */
        UInt32 maxTotConn = propGetUInt32AtIndex(PROP_COMM_MAX_CONNECTIONS, 0, 1L);
        if (maxTotConn == 0) { return utFalse; } // NO connections allowed
    
        /* count actual connections and compare to limit */
        UInt32 simplexConnCount = _acctCountConnections(&simplexConnectMask);
        UInt32 duplexConnCount  = _acctCountConnections(&duplexConnectMask);
        return ((simplexConnCount + duplexConnCount) < maxTotConn)? utTrue : utFalse;
        
    } else {
        
        return utTrue;
        
    }
#endif
}

/* return true if we are currently under the number of allowed duplex connections */
utBool acctUnderDuplexQuota()
{
#if defined(TRANSPORT_MEDIA_FILE)
    return utFalse; // file transport does not support duplex
#elif defined(TRANSPORT_MEDIA_SERIAL)
    return utTrue; // always under duplex quota for serial transport
#else
    if (!acctSupportsDuplex()) {
        
        /* never under duplex quota if duplex connections aren't supported */
        return utFalse;
        
    } else
    if (acctHasQuota()) {
        
        /* check total connection limit */
        UInt32 maxTotConn = propGetUInt32AtIndex(PROP_COMM_MAX_CONNECTIONS, 0, 1L); // Simplex
        if (maxTotConn == 0L) { return utFalse; } // NO connections allowed
    
        /* check Duplex connection limit */
        UInt32 maxDuplexConn = propGetUInt32AtIndex(PROP_COMM_MAX_CONNECTIONS, 1, 1L); // Duplex
        if (maxDuplexConn == 0L) { return utFalse; } // NO Duplex connections allowed
        if (maxDuplexConn > maxTotConn) { maxDuplexConn = maxTotConn; } // limit Duplex conn to total
    
        /* count actual connections and compare to limit */
        UInt32 duplexConnCount = _acctCountConnections(&duplexConnectMask);
        return (duplexConnCount < maxDuplexConn)? utTrue : utFalse;
    
    } else {
        
        return utTrue;
        
    }
#endif
}

/* return true if the device is allowed to make duplex connections to the server */
utBool acctSupportsDuplex()
{
#if defined(TRANSPORT_MEDIA_FILE)
    return utFalse; // file transport does not support duplex
#elif defined(TRANSPORT_MEDIA_SERIAL)
    return utTrue; // serial transport only supports duplex
#else
    UInt32 maxEvents = propGetUInt32(PROP_COMM_MAX_DUP_EVENTS, 1L);
    if (maxEvents > 0L) {
        UInt32 maxDuplexConn = propGetUInt32AtIndex(PROP_COMM_MAX_CONNECTIONS, 1, 1L); // Duplex
        return (maxDuplexConn > 0L)? utTrue : utFalse;
    } else {
        return utFalse;
    }
#endif
}

/* return true if the device is allowed to make simplex connections to the server */
utBool acctSupportsSimplex()
{
#if defined(TRANSPORT_MEDIA_FILE)
    return utTrue; // serial transport only supports simplex
#elif defined(TRANSPORT_MEDIA_SERIAL)
    return utFalse; // serial transport does not support simplex
#else
    UInt32 maxEvents = propGetUInt32(PROP_COMM_MAX_SIM_EVENTS, 1L);
    if (maxEvents > 0L) {
        UInt32 maxTotalConn  = propGetUInt32AtIndex(PROP_COMM_MAX_CONNECTIONS, 0, 1L); // Total
        UInt32 maxDuplexConn = propGetUInt32AtIndex(PROP_COMM_MAX_CONNECTIONS, 1, 1L); // Duplex
        return (maxTotalConn > maxDuplexConn)? utTrue : utFalse;
    } else {
        return utFalse;
    }
#endif
}

// ----------------------------------------------------------------------------

/* return true if minimum time between connections has ecpired */
utBool acctAbsoluteDelayExpired()
{
#if defined(TRANSPORT_MEDIA_FILE)
    return utTrue; // no absolute minimum time
#elif defined(TRANSPORT_MEDIA_SERIAL)
    return utTrue; // no absolute minimum time
#else
    TimerSec_t lastConnTime = _acctGetLastConnectionTime();
    UInt32 minXmitDelay = propGetUInt32(PROP_COMM_MIN_XMIT_DELAY, MINUTE_SECONDS(30));
    if ((minXmitDelay < MIN_XMIT_DELAY) && !isDebugMode()) { minXmitDelay = MIN_XMIT_DELAY; }
    utBool timerExp = utcIsTimerExpired(lastConnTime,minXmitDelay);
    //if (timerExp) { logINFO(LOGSRC,"AbsoluteDelay expired %lu/%lu", lastConnTime, minXmitDelay); }
    return timerExp;
#endif
}

/* return true if the minimum time between connections has expired */
utBool acctMinIntervalExpired()
{
#if defined(TRANSPORT_MEDIA_FILE)
    return utTrue; // no minimum interval for file transport
#elif defined(TRANSPORT_MEDIA_SERIAL)
    return utTrue; // no minimum interval for serial transport
#else
    TimerSec_t lastConnTime = _acctGetLastConnectionTime();
    UInt32 minXmitInterval = propGetUInt32(PROP_COMM_MIN_XMIT_RATE, HOUR_SECONDS(2));
    if ((minXmitInterval < MIN_XMIT_RATE) && !isDebugMode()) { minXmitInterval = MIN_XMIT_RATE; }
    return utcIsTimerExpired(lastConnTime,minXmitInterval);
#endif
}

/* return true if the maximum connection interval has expired. (ie the maximum amount
of time elapsing without having made a connection to the server) */
utBool acctMaxIntervalExpired()
{
#if defined(TRANSPORT_MEDIA_FILE)
    return utFalse; // no max interval for file transport
#elif defined(TRANSPORT_MEDIA_SERIAL)
    return utFalse; // no max interval for serial transport
#else
    // This function specifically checks the last time we've made a Duplex connection
    TimerSec_t lastConnTime = duplexConnectMask.lastConnTime;
    UInt32 maxXmitInterval = propGetUInt32(PROP_COMM_MAX_XMIT_RATE, HOUR_SECONDS(24));
    return utcIsTimerExpired(lastConnTime,maxXmitInterval);
#endif
}

// ----------------------------------------------------------------------------

/* initialize new connection mask */
#if !defined(TRANSPORT_MEDIA_FILE) && !defined(TRANSPORT_MEDIA_SERIAL)
static void _acctInitConnectionMask(ConnectionMask_t *conn)
{
    memset(conn, 0, sizeof(ConnectionMask_t));
    conn->shiftTime    = (TimerSec_t)0L;
    conn->lastConnTime = (TimerSec_t)0L;
}
#endif

/* initialize accounting */
void acctInitialize()
{
#if !defined(TRANSPORT_MEDIA_FILE) && !defined(TRANSPORT_MEDIA_SERIAL)
    _acctInitConnectionMask(&duplexConnectMask);
    _acctInitConnectionMask(&simplexConnectMask);
#endif
}

// ----------------------------------------------------------------------------
#endif
