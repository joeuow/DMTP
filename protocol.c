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
//  DMTP communication protocol manager
//  Manages the dialog between the client and server during connections.
// Notes:
//  - Thread support would be a useful addition to this module.  This would allow
//  an immediate return to the main GPS checking loop without blocking.  This
//  reference implementation will block until data communication has been made
//  and data has been transferred.
// ---
// Change History:
//  2006/01/04  Martin D. Flynn
//     -Initial release
//  2006/02/09  Martin D. Flynn
//     -Send ERROR_GPS_EXPIRED/ERROR_GPS_FAILURE is GPS fix has expired
//  2006/04/02  Martin D. Flynn
//     -Clear 'sendIdentification' after sending UniqueID
//  2006/05/07  Martin D. Flynn
//     -Add support for uploading GeoZones
//  2006/05/26  Martin D. Flynn
//     -Allow UniqueID to be >= 4
//  2007/01/28  Martin D. Flynn
//     -WindowsCE port
//     -Fix 'tight loop' issue when 'serial/bluetooth' transport goes down
//      (propagate error up from 'protocolReadServerPacket').
//     -Return ERROR_COMMAND_ERROR for PROP_ERROR_COMMAND_ERROR on 'set' property.
//     -Switched to generic thread access methods in 'tools/threads.h'
//     -Flush server input buffers on received EOB-done (serial transport only)
//     -Separated all protocol variables into a separate structure to allow
//      multiple simultaneous connecting protocols.
//     -Changed PKT_SERVER_GET_PROPERTY to retrieve a single property value.
//      The remaining bytes behind the property key can now be used as arguments
//      on the property 'get' (via 'PROP_REFRESH_GET').
//     -Made some significant changes to this module to allow multiple virtual
//      protocol "instances".  (While this would have been easier in 'C++', this
//      has been implemented in 'C' to allow compiling on any 'C' capable platform.)
//  2007/02/05  Martin D. Flynn
//     -Fixed Fletcher checksum packet generation that inappropriately used
//      "sizeof(ChecksumFletcher_t)" to determine the length of the checksum value.
//      This created a compiler dependency that could return a length of 4, instead
//      of the corrent value '2'. (Thanks to Tomasz Rostanski for catching this!).
//  2007/04/28  Martin D. Flynn
//     -Moved the following functions from 'events.c' to this modules to support 
//      dual transport: evGetEventQueue, evGetHighestPriority,
//      evEnableOverwrite, evAcknowledgeFirst, evAcknowledgeToSequence
// ----------------------------------------------------------------------------

#include "defaults.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdarg.h>
#include <string.h>
#include <ctype.h>
#include <signal.h>
#include <time.h>
#include <errno.h>
#include <sys/syslog.h>

#include "log.h"
#include "transport.h"
#include "gps.h"

#include "stdtypes.h"
#include "strtools.h"
#include "utctools.h"
#include "base64.h"
#include "checksum.h"

#include "propman.h"
#include "events.h"
#include "cerrors.h"
#include "serrors.h"
#include "pqueue.h"
#if !defined(PROTOCOL_THREAD)
#include "accting.h"
#endif
#include "packet.h"
#include "sockets.h"
#include "protocol.h"
#if defined(ENABLE_UPLOAD)
#  include "upload.h"
#endif

#include "motion.h"
#include "rfid.h"
#include "watchdog_f.h"
#include "mainloop.h"
#include "diagnostic.h"
#include "network_diagnostic.h"
// ----------------------------------------------------------------------------
// thread control

#ifdef PROTOCOL_THREAD
//#warning Protocol thread support enabled
#define PROTOCOL_LOCK(PV)       MUTEX_LOCK(&((PV)->protocolMutex));
#define PROTOCOL_UNLOCK(PV)     MUTEX_UNLOCK(&((PV)->protocolMutex));
//#define PROTOCOL_WAIT(PV)       CONDITION_WAIT(&((PV)->protocolCond), &((PV)->protocolMutex));
#define PROTOCOL_WAIT(PV, T)	CONDITION_TIMED_WAIT(&((pv)->protocolCond), \
				&((pv)->protocolMutex), (T));
#define PROTOCOL_NOTIFY(PV)     CONDITION_NOTIFY(&((PV)->protocolCond));
#else
//#warning Protocol thread support disabled
#define PROTOCOL_LOCK(PV)
#define PROTOCOL_UNLOCK(PV)
#define PROTOCOL_WAIT(PV)
#define PROTOCOL_NOTIFY(PV)
#endif

// ----------------------------------------------------------------------------

/* severe error counts */
#define MAX_SEVERE_ERRORS			3
#define EXCESSIVE_SEVERE_ERRORS		10
#define PROTOCOL_READ_BUF_SIZE		PACKET_MAX_ENCODED_LENGTH * 7
#define DEFAULT_SESSION_PERIOD		79

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// '_protoGetVars' allows separating the protocol handler into separate virtual
// instances. Ideally, this should be a C++ class, but this has been implemented
// in 'C' to allow this module to compile on any 'C' compatible platform.
UInt32 transport_protocol = 0;
threadThread_t	protocolThread;
pthread_cond_t	network_up_sema = PTHREAD_COND_INITIALIZER;
pthread_cond_t	network_down_sema = PTHREAD_COND_INITIALIZER;
pthread_mutex_t	network_status_mutex = PTHREAD_MUTEX_INITIALIZER;
uint32_t network_link_status = NETWORK_STATUS_ERROR;

extern bool link_down_occurred;
struct sigevent sev7;
struct itimerspec it7;
timer_t timer7;
//static int diagSequence = 0;
static struct sigevent sev1;
static struct itimerspec it1;
static timer_t timer1;
static Packet_t serverPacket;
static Packet_t clientPacket;
static Packet_t messagePacket;
static uint32_t url_id = 0; 
static uint32_t url_swap_count = 0; 
static uint32_t protocol_down_count = 0;
static uint32_t protocol_reboot_timer;
static time_t my_time;
static time_t server_time;
static time_t old_time;
static bool clock_need_adjust = false;
static bool clock_synchronized_with_server = true;
static bool url_reset = true; 
static int Buffer_safety_block = 128;
static UInt32 protocol_version;
static UInt8 encode_buf[PACKET_MAX_ENCODED_LENGTH];
static ProtocolVars_t   protoVars[MAX_SIMULTANEOUS_PROTOCOLS];

static bool watchdog_protocol_monitor(void);

static ProtocolVars_t *_protoGetVars(const char *fn, int line, int protoNdx)
{
    if (protoNdx <= 0) {
        // primary protocol
        return &protoVars[0];
    } else
    if (protoNdx < MAX_SIMULTANEOUS_PROTOCOLS) {
        // secondary protocol, etc.
        //logINFO(LOGSRC,"Secondary protocol #%d [%s:%d] ...", protoNdx, fn, line);
        return &protoVars[protoNdx];
    } else {
        // invalid index, return last protocol
        //logWARNING(LOGSRC,"Secondary protocol out-of-bounds #%d [%s:%d] ...", protoNdx, fn, line);
        return &protoVars[MAX_SIMULTANEOUS_PROTOCOLS - 1];
    }
}

// ----------------------------------------------------------------------------
static void  _protocolInitVars(ProtocolVars_t *pv);
// ----------------------------------------------------------------------------

#if defined(SECONDARY_SERIAL_TRANSPORT)
PacketQueue_DEFINE(secondaryQueue,SECONDARY_EVENT_QUEUE_SIZE);
#endif

/* get event queue (evGetEventQueue) */
static PacketQueue_t *_protocolGetEventQueue(ProtocolVars_t *pv)
{
#if defined(SECONDARY_SERIAL_TRANSPORT)
    return pv->isPrimary? evGetEventQueue() : &secondaryQueue;
#else
    return evGetEventQueue();
#endif
}

PacketQueue_t *protocolGetEventQueue(int protoNdx)
{
    ProtocolVars_t *pv = _protoGetVars(LOGSRC,protoNdx);
    return _protocolGetEventQueue(pv);
}

#if !defined(PROTOCOL_THREAD)
/* return the highest priority event in the queue (evGetHighestPriority) */
static PacketPriority_t _protocolGetHighestPriority(ProtocolVars_t *pv)
{
    PacketQueue_t *evQue = _protocolGetEventQueue(pv);
    if (evQue) {
        return pqueGetHighestPriority(evQue);
    } else {
        return PRIORITY_NONE;
    }
}
#endif
/* enable overwriting of the oldest event if the event queue fills up (evEnableOverwrite) */
static void _protocolEnableOverwrite(ProtocolVars_t *pv, utBool overwrite)
{
    PacketQueue_t *evQue = _protocolGetEventQueue(pv);
    if (evQue) {
        pqueEnableOverwrite(evQue, overwrite);
    }
}

/* acknowledge events up to and including the specified sequence (evAcknowledgeToSequence) */
static utBool _protocolAcknowledge(ProtocolVars_t *pv, int num_ack)
{
    // 'primary' transport events only
	utBool didAck = utFalse;
	if (num_ack == 0)
		return didAck;
	if (pv->payload_type == PAYLOAD_EVENT) {
		PacketQueue_t *eventQueue = _protocolGetEventQueue(pv);
		didAck = pqueDeleteSentPackets(eventQueue, pv->sequence_first, num_ack);
	} else {
		didAck = pqueDeleteSentPackets(&pv->pendingQueue, pv->sequence_first, num_ack);
	}	
	pv->num_sent = 0;
	return didAck;
}

// ----------------------------------------------------------------------------
#if !defined(PROTOCOL_THREAD)
// ----------------------------------------------------------------------------

/* set current session packet encoding */
static void _protocolSetSessionEncoding(ProtocolVars_t *pv, TransportType_t xportType, PacketEncoding_t enc)
{
    // Since this reference implementation does not support received CSV packet encoding
    // from the server, we must make sure that our first packet to the server is NOT
    // CSV encoded if we are transmitting via TRANSPORT_DUPLEX
    pv->sessionEncoding         = enc;
    pv->sessionEncodingChanged  = utFalse;
    pv->sessionFirstEncoding    = pv->sessionEncoding;
    if (xportType == TRANSPORT_DUPLEX) {
        if (ENCODING_VALUE(pv->sessionEncoding) == ENCODING_CSV) {
            PacketEncoding_t dft = ENCODING_BASE64;
            pv->sessionFirstEncoding = ENCODING_IS_CHECKSUM(enc)? ENCODING_CHECKSUM(dft) : dft;
        }
    }
}
/* return true if specified encoding is supported */
static PacketEncoding_t _protocolGetSupportedEncoding(ProtocolVars_t *pv, PacketEncoding_t dftEncoding)
{
    UInt32 enc = ENCODING_VALUE(dftEncoding); // <-- max 15
    if (enc == ENCODING_BINARY) {
        return (PacketEncoding_t)enc;
    } else {
        UInt32 propEncodings = pv->isPrimary? propGetUInt32(PROP_COMM_ENCODINGS, 0L) : 0L; // protocol dependent
        UInt32 encodingMask = propEncodings | ENCODING_REQUIRED_MASK; // <-- must include binary
        while ((ENCODING_MASK(enc) & encodingMask) == 0L) {
            // ENCODING_CSV     3 
            // ENCODING_HEX     2 
            // ENCODING_BASE64  1 
            // ENCODING_BINARY  0 
            enc--;
        }
        //if (enc != ENCODING_VALUE(dftEncoding)) { logDEBUG(LOGSRC,"Encoding down-graded to %ld\n", enc); }
        return ENCODING_IS_CHECKSUM(dftEncoding)? 
            (PacketEncoding_t)ENCODING_CHECKSUM(enc) : 
            (PacketEncoding_t)enc;
    }
}

// ----------------------------------------------------------------------------
// The following are wrappers to the 'transport.c' module function calls

/* return transport 'open' state */
utBool protocolIsSpeakFreely(int protoNdx)
{
    ProtocolVars_t *pv = _protoGetVars(LOGSRC,protoNdx);
    return pv->xFtns->IsOpen()? pv->speakFreely : utFalse;
}

#endif
/* return transport 'open' state */
utBool protocolIsOpen(int protoNdx)
{
    ProtocolVars_t *pv = _protoGetVars(LOGSRC,protoNdx);
    return pv->xFtns->IsOpen();
}

/* open connection to server */
static utBool _udpOpen(ProtocolVars_t *pv)
{
	utBool didOpen;
	if (url_reset) {
		pv->xFtns->ResetAddr(url_id);
		url_reset = false;
	}
	didOpen = pv->xFtns->Open();
	if (didOpen) {
        // opened, reset session
		_protocolEnableOverwrite(pv, utFalse); // disable overwrites while connected
		if (pv->isPrimary) { // data stats
		    // data stats only recorded for 'primary' transport
			pv->totalReadBytes      = propGetUInt32(PROP_COMM_BYTES_READ   , 0L); // primary only
			pv->totalWriteBytes     = propGetUInt32(PROP_COMM_BYTES_WRITTEN, 0L); // primary only
		} else {
		    // data stats not recorded for 'secondary' transport
			pv->totalReadBytes      = 0L;
			pv->totalWriteBytes     = 0L;
		}
		pv->sessionReadBytes        = 0L;

#if defined(TRANSPORT_MEDIA_SERIAL)
		// send account/device for serial transport
		pv->sendIdentification      = SEND_ID_ACCOUNT;
#else
		// try unique-id first for everything else
		pv->sendIdentification      = SEND_ID_UNIQUE;
#endif
		pv->severeErrorCount        = 0;
		pv->checkSumErrorCount      = 0;
		pv->invalidAcctErrorCount   = 0;
		pv->sessionWrittenBytes = 0;
		pv->IdentificationBytes = 0;
		pv->sequence_first = 0;
		pv->num_sent = 0;
		pv->pending = false;
		pv->session_continue = true;
		memset(&serverPacket, 0, sizeof(Packet_t));

#if defined(TRANSPORT_MEDIA_SERIAL)
		if (pv->isSerial) {
			motionResetMovingMessageTimer();
            // TODO: generate a connection message?
		}
#endif
	}
	return didOpen;
}

static utBool _tcpOpen(ProtocolVars_t *pv)
{
	utBool didOpen;
	if (url_reset) {
		pv->xFtns->ResetAddr(url_id);
		url_reset = false;
	}
	/*start timer1 */
	it1.it_value.tv_sec = NETWORK_OPEN_TIMEOUT;
	timer_settime(timer1, 0, &it1, NULL);
	didOpen = pv->xFtns->Open();
	/*stop timer1 */
	it1.it_value.tv_sec = 0;
	it1.it_value.tv_nsec = 0;
	timer_settime(timer1, 0, &it1, NULL);
	if (didOpen) {
        // opened, reset session
		_protocolEnableOverwrite(pv, utFalse); // disable overwrites while connected
    	cksumResetFletcher();
		if (pv->isPrimary) { // data stats
		    // data stats only recorded for 'primary' transport
			pv->totalReadBytes      = propGetUInt32(PROP_COMM_BYTES_READ   , 0L); // primary only
			pv->totalWriteBytes     = propGetUInt32(PROP_COMM_BYTES_WRITTEN, 0L); // primary only
		} else {
		    // data stats not recorded for 'secondary' transport
			pv->totalReadBytes      = 0L;
			pv->totalWriteBytes     = 0L;
		}
		pv->sessionReadBytes        = 0L;

#if defined(TRANSPORT_MEDIA_SERIAL)
		// send account/device for serial transport
		pv->sendIdentification      = SEND_ID_ACCOUNT;
#else
		// try unique-id first for everything else
		pv->sendIdentification      = SEND_ID_UNIQUE;
#endif
		pv->severeErrorCount        = 0;
		pv->checkSumErrorCount      = 0;
		pv->invalidAcctErrorCount   = 0;
		pv->sessionWrittenBytes = 0;
		pv->IdentificationBytes = 0;
		pv->sequence_first = 0;
		pv->num_sent = 0;
		pv->pending = true;
		pv->session_continue = true;
		memset(&serverPacket, 0, sizeof(Packet_t));

#if defined(TRANSPORT_MEDIA_SERIAL)
		if (pv->isSerial) {
			motionResetMovingMessageTimer();
            // TODO: generate a connection message?
		}
#endif
	}
	return didOpen;
}

/* close connection to server */
static utBool _protocolClose(ProtocolVars_t *pv)
{
    /* close transport */
    // If the connection is via Simplex, the data will be sent now.
	utBool didClose = pv->xFtns->Close();
		
	if (didClose && pv->isPrimary) { // data stats
		// save read/write byte counts if 'close' was successful (primary only)
		propSetUInt32(PROP_COMM_BYTES_READ   , pv->totalReadBytes );
		propSetUInt32(PROP_COMM_BYTES_WRITTEN, pv->totalWriteBytes);
	}
	
	/* re-enable event queue overwrites while not connected */
	_protocolEnableOverwrite(pv, EVENT_QUEUE_OVERWRITE); // enabled only while not connected
	
	if (pv->severeErrorCount > 0) {
		// this helps prevent runnaway clients from abusing the server
		pv->totalSevereErrorCount += pv->severeErrorCount;
		logWARNING(LOGSRC,"Severe errors encountered %d", pv->totalSevereErrorCount);
		if (pv->totalSevereErrorCount >= EXCESSIVE_SEVERE_ERRORS) {
			// Turn off periodic messaging
			logERROR(LOGSRC,"Excessive severe errors!!");
			pv->totalSevereErrorCount = 0;
		}
	} else  {
		// a session without any severe errors clears this count
		pv->totalSevereErrorCount = 0;
	}
	/*restore the unsent packets as freshly added */
	if (pqueGetPacketCount(&pv->pendingQueue) > 0) {
		pqueRestoreSentPacket(&pv->pendingQueue);
	} else {
		pqueResetQueue(&pv->pendingQueue);
	}
	PacketQueue_t *eventQueue = _protocolGetEventQueue(pv);
	if (pqueGetPacketCount(eventQueue) > 0) {
		pqueRestoreSentPacket(eventQueue);
	}
	else {
		pqueResetQueue(eventQueue);
		pqueResetPreserve();
	}
	return didClose;
}

/* read packet to server */
static int _tcpReadPacket(ProtocolVars_t *pv, bool blocking)
{
    int len;
	/*start timer1 */
	if (blocking)
		it1.it_value.tv_sec = NETWORK_RECEIVE_TIMEOUT;
	else
		it1.it_value.tv_sec = BLOCK_RECEIVING_TIMEOUT;
	it1.it_value.tv_nsec = 0;
	timer_settime(timer1, 0, &it1, NULL);
	if ((len = pv->xFtns->Read(pv->readBuf + pv->sessionReadBytes, 
						PROTOCOL_READ_BUF_SIZE - pv->sessionReadBytes)) > 0) {
		pv->totalReadBytes   += len;
		pv->sessionReadBytes += len;
	}
	/*stop timer1 */
	it1.it_value.tv_sec = 0;
	it1.it_value.tv_nsec = 0;
	timer_settime(timer1, 0, &it1, NULL);
    return len;
}

/* decide end of reading*/
static bool _tcpIsEndOfReceiving(ProtocolVars_t *pv)
{
    int rlen;
	UInt8 *pptr = NULL, *sptr = NULL; 
	UInt32 pkt_type;
	bool ret = false;

	pptr = memchr(pv->readBuf, PACKET_HEADER_BASIC, pv->sessionReadBytes); 
	if (pptr == NULL)
		return ret;
	rlen = pv->sessionReadBytes -  (pptr - pv->readBuf);
	while (rlen >= 2 && pptr[0] == PACKET_HEADER_BASIC) {
		rlen -= (pptr[2] + 3);
		sptr = pptr;
		pptr += (pptr[2] + 3);
	}
	if (rlen > 0) {
		if (pptr[0] == PACKET_HEADER_BASIC)
			return false;
		else
			return true;
	}
	pkt_type = (sptr[0] << 8) | sptr[1];
	if (pkt_type == PKT_SERVER_EOT || pkt_type == PKT_SERVER_EOB_DONE ||
								pkt_type == PKT_SERVER_EOB_SPEAK_FREELY)
		ret = true;
    return ret;
}

/* write packet to server */
static int _protocolWritePacket(ProtocolVars_t *pv, Packet_t *pkt)
{
    int oldWrittenBytes;
	pv->sendBuf[pv->sessionWrittenBytes] = (pkt->hdrType >> 8) & 0xFF;
	pv->sendBuf[pv->sessionWrittenBytes + 1] = pkt->hdrType & 0xFF;
	pv->sendBuf[pv->sessionWrittenBytes + 2] = pkt->dataLen & 0xFF;
	memcpy(pv->sendBuf + pv->sessionWrittenBytes + 3, pkt->data, pkt->dataLen);
	oldWrittenBytes = pv->sessionWrittenBytes;
	pv->sessionWrittenBytes += (pkt->dataLen + 3);
	print_hex("[Tx]", pv->sendBuf + oldWrittenBytes, pkt->dataLen + 3);
    return pkt->dataLen + 3;
}

static utBool flush_sending(ProtocolVars_t *pv)
{
	/*char disp[PACKET_MAX_ENCODED_LENGTH]; */
	int len = 0;
	int wlen = 0;
	while (wlen < pv->sessionWrittenBytes) {
		if ((len = pv->xFtns->Write(pv->sendBuf + wlen, pv->sessionWrittenBytes - wlen)) < 0)
			break;
		wlen += len;
	}
	if (len < 0)
		return utFalse;
	else
		return utTrue;
}
static bool send_buffer_overflow(ProtocolVars_t *pv, Packet_t *pkt)
{
	return ((pv->sessionWrittenBytes + pkt->dataLen > pv->SEND_BUF_SIZE - Buffer_safety_block)? true : false);
}
// ----------------------------------------------------------------------------
/* queue specified packet for transmission */
static utBool _protocolQueuePacket(ProtocolVars_t *pv, Packet_t *pkt)
{
    // Notes:
    // - By default packets are created with 'PRIORITY_NORMAL', and will be placed
    // into the volatile queue which is reset at the start of each session.  Thus,
    // the volatile queue should only be used within a server connected session. 
    // - If a packet is important enough to be retained until it is transmitted to
    // the server, then its priority should be set to 'PRIORITY_HIGH'. It will then
    // be added to the pending queue and will be retained until it is successfully 
    // transmitted to the server.  This is also true for important packets that are
    // queued while NOT within a server connected session.
    // This queue persists across sessions
	pkt->sequence = pv->pending_sequence++;
	return (pqueAddPacket(&(pv->pendingQueue), pkt));
}

// ----------------------------------------------------------------------------
/* queue specified error for transmission */
static utBool _protocolQueueError(ProtocolVars_t *pv, const char *fmt, ...)
{
	Packet_t *pkt = &messagePacket;
	va_list ap;
	va_start(ap, fmt);
	if (pktVInit(pkt, PKT_CLIENT_ERROR, fmt, ap) < 0) {
		logERROR(LOGSRC,"Wrong Packet Format");
		va_end(ap);
		return utFalse;
	}
	va_end(ap);
	_protocolWritePacket(pv, pkt);
	pv->pending = true;
    return utTrue;
}
#if defined(ENABLE_UPLOAD)
utBool protocolQueueError(int protoNdx, const char *fmt, ...)
{
    ProtocolVars_t *pv = _protoGetVars(LOGSRC,protoNdx);
    Packet_t *pkt = &messagePacket;
    va_list ap;
    va_start(ap, fmt);
    if (pktVInit(pkt, PKT_CLIENT_ERROR, fmt, ap) < 0) {
	logERROR(LOGSRC,"Wrong Packet Format");
	return utFalse;
	}
    va_end(ap);
    return _protocolQueuePacket(pv,pkt);
}
#endif
/* parse packet received from server */
static Packet_t *_protocolParseServerPacket(ProtocolVars_t *pv, Packet_t *pkt, const UInt8 *pktBuf)
{
    /* clear packet */
    memset(pkt, 0, sizeof(Packet_t));
    /* parse header/data */
	if (*pktBuf == PACKET_HEADER_BASIC) {
		/* parse into packet */
		pkt->hdrType = CLIENT_HEADER_TYPE(pktBuf[0], pktBuf[1]);
		pkt->dataLen = pktBuf[2];
		if (pkt->dataLen > 0) {
			if (pkt->hdrType == PKT_SERVER_SET_PROPERTY) {
				memcpy(pv->extBuf, pktBuf + 3, (int)pkt->dataLen);
			} else
				memcpy(pkt->data, pktBuf + 3, (int)pkt->dataLen);
		}
		print_hex("[Rx]", pktBuf, (int)pkt->dataLen + 3);
	} else {
		// invalid header: ERROR_PACKET_HEADER
		ClientPacketType_t hdrType = CLIENT_HEADER_TYPE(pktBuf[0],pktBuf[1]);
		_protocolQueueError(pv,"%2x%2x", (UInt32)ERROR_PACKET_HEADER, (UInt32)hdrType);
		return (Packet_t*)0;
	}
    /* return packet */
    return pkt;
}

// ----------------------------------------------------------------------------
#if !defined(PROTOCOL_THREAD)
/* read packet from server */
static int _protocolReadServerPacket(ProtocolVars_t *pv, Packet_t *pkt)
{
    UInt8 buf[PACKET_MAX_ENCODED_LENGTH];
    int bufLen = pv->xFtns->Read(buf, sizeof(buf), utTrue);
    
    /* parse and validate packet */
    if (bufLen < 0) {
        // read error (transport not open?)
        return -1; // Fix: MDF 2006/07/28
    } else
    if (bufLen == 0) {
        // timeoout (or no data)
        return 0;
    } else {
        // this count won't be accurate if an error occurred during a packet read
        pv->totalReadBytes   += bufLen;
        pv->sessionReadBytes += bufLen;
        // parse packet
        _protocolParseServerPacket(pv, pkt, buf); // , bufLen);
        return 1;
    }
    
}
#endif
// ----------------------------------------------------------------------------
#ifdef TRANSPORT_MEDIA_SERIAL
/* flush input buffer (from server) */
static void _protocolFlushInput(ProtocolVars_t *pv)
{
    // This typically only has meaning in speakFreely mode (eq. serial transport)
    // This prevents reading old/obsolete messages that may have been queue by
    // the server some time ago that we weren't able to pick up quickly enough.
    if (pv->isSerial) {
        pv->xFtns->ReadFlush();
    }
}
#endif

#if !defined(PROTOCOL_THREAD)
/* return true if we have pending data to send */
static utBool _protocolHasDataToSend(ProtocolVars_t *pv)
{
    if (pv->sendIdentification != SEND_ID_NONE) {
        // identification has been requested
        return utTrue;
    } else
    if (pqueHasPackets(&(pv->pendingQueue))) {
        // has pending important packets
        return utTrue;
    } else
    if (pqueHasPackets(&(pv->volatileQueue))) {
        // has miscellaneous volatile packets
        return utTrue;
    } else
    if (pqueHasPackets(_protocolGetEventQueue(pv))) {
        // has event packets
        return utTrue;
    }
    return utFalse;
}
#endif //!defined(PROTOCOL_THREAD)

/* send identification packets */
static utBool _udpSendIdentification(ProtocolVars_t *pv)
{
	Packet_t idPkt;
	const char *acctId, *devId;
	int idLen;
	
	if (pv->IdentificationBytes > 0)
		return utTrue;
	pktInit(&idPkt, PKT_CLIENT_UNIQUE_ID, "%1U", protocol_version);
	_protocolWritePacket(pv, &idPkt);
	/* account/device */
	acctId = propGetAccountID(); // propGetString(PROP_STATE_ACCOUNT_ID,"");
	idLen = strnlen(acctId, MAX_ID_SIZE);
	pktInit(&idPkt, PKT_CLIENT_ACCOUNT_ID, "%*s", idLen, acctId);
	_protocolWritePacket(pv, &idPkt);
	devId = propGetDeviceID(pv->protoNdx);  // propGetString(PROP_STATE_DEVICE_ID)
	idLen = strnlen(devId, MAX_ID_SIZE);
	pktInit(&idPkt, PKT_CLIENT_DEVICE_ID, "%*s", idLen, devId);
	_protocolWritePacket(pv, &idPkt);
	pv->IdentificationBytes = pv->sessionWrittenBytes;
	return utTrue;
}
/* send identification packets */
static utBool _tcpSendIdentification(ProtocolVars_t *pv)
{
	Packet_t idPkt;
	const char *acctId, *devId;
	int idLen;
	utBool ret = utFalse;
	
	pktInit(&idPkt, PKT_CLIENT_UNIQUE_ID, "%1U", protocol_version);
	_protocolWritePacket(pv, &idPkt);
	/* account/device */
	acctId = propGetAccountID(); // propGetString(PROP_STATE_ACCOUNT_ID,"");
	idLen = strnlen(acctId, MAX_ID_SIZE);
	pktInit(&idPkt, PKT_CLIENT_ACCOUNT_ID, "%*s", idLen, acctId);
	_protocolWritePacket(pv, &idPkt);
	devId = propGetDeviceID(pv->protoNdx);  // propGetString(PROP_STATE_DEVICE_ID)
	idLen = strnlen(devId, MAX_ID_SIZE);
	pktInit(&idPkt, PKT_CLIENT_DEVICE_ID, "%*s", idLen, devId);
	_protocolWritePacket(pv, &idPkt);
	pv->IdentificationBytes = pv->sessionWrittenBytes;
	ret = flush_sending(pv);
	if (ret) {
		cksumCalcFletcher(pv->sendBuf, pv->sessionWrittenBytes);
		pv->totalWriteBytes += pv->sessionWrittenBytes;
		pv->sessionWrittenBytes = 0;
	}
	return ret;
}

/* send contents of specified queue */
static int _protocolSendQueue(ProtocolVars_t *pv, PacketQueue_t *pq)
{
	int cnt = 0; 
	Packet_t *quePkt = &clientPacket;
	PacketQueueIterator_t queIter;
	/* adjust arguments */
    /* iterate through queue */
    // This loop stops as soon as one of the following has occured:
    //  - We've sent the specified 'maxEvents'.
    //  - All events in the queue have been sent.
    //  - We've run into a packet that exceeds our maximum allowable priority.
	pqueGetIterator(pq, &queIter);
	quePkt = pqueGetNextPacket(&clientPacket, &queIter);
	if (quePkt != (Packet_t*)0)
		pv->sequence_first = quePkt->sequence;
	else
		return cnt;
	while ((quePkt != (Packet_t*)0) && !send_buffer_overflow(pv, quePkt)) {
		if (pqueIsPacketSent(quePkt))
			goto next_packet;
		_protocolWritePacket(pv, &clientPacket);
		pqueMarkPacketSent(quePkt); /*mark it as sent*/
		cnt++;
next_packet:
		quePkt = pqueGetNextPacket(&clientPacket, &queIter);
	}
	pv->num_sent = cnt;
	return cnt;
}

/* end End-Of-Block */
static utBool _tcpSendEOB(ProtocolVars_t *pv, bool more_events)
{
	utBool ret;
	ChecksumFletcher_t fcs;
	UInt16 pkt_type = (more_events? PKT_CLIENT_EOB_MORE : PKT_CLIENT_EOB_DONE);
	UInt8 *eobp = pv->sendBuf + pv->sessionWrittenBytes;
	eobp[0] = PACKET_HEADER_BASIC;
	eobp[1] = pkt_type & 0xFF;
	eobp[2] = FLETCHER_CHECKSUM_LENGTH;
	eobp[3] = 0;
	eobp[4] = 0;
	/* encode packet with a placeholder for the checksum */
	/*_protocolWrite(pv, buf, FLETCHER_CHECKSUM_LENGTH + 3); */
	pv->sessionWrittenBytes += 5;
	cksumCalcFletcher(pv->sendBuf, pv->sessionWrittenBytes);
	cksumGetFletcherChecksum(&fcs); // encode
	eobp[3] = fcs.C[0];
	eobp[4] = fcs.C[1];
	print_hex("[Tx]", eobp, 5);
	ret = flush_sending(pv);
	if (ret) {
		pv->totalWriteBytes += pv->sessionWrittenBytes;
		pv->sessionWrittenBytes = 0;
		cksumResetFletcher();
	}
	return ret;
}
/* end End-Of-Block */
static utBool _udpSendEOB(ProtocolVars_t *pv)
{
	utBool ret;
	UInt16 pkt_type = PKT_CLIENT_EOB_DONE;
	UInt8 *eobp = pv->sendBuf + pv->sessionWrittenBytes;
	eobp[0] = PACKET_HEADER_BASIC;
	eobp[1] = pkt_type & 0xFF;
	eobp[2] = 0;
	pv->sessionWrittenBytes += 3;
	print_hex("[Tx]", eobp, 3);
	ret = flush_sending(pv);
	if (ret) {
		pv->totalWriteBytes += pv->sessionWrittenBytes;
		pv->sessionWrittenBytes = pv->IdentificationBytes;
	}
	return ret;
}

#if !defined(PROTOCOL_THREAD)
/* send packets to server */
static utBool _protocolSendAllPackets(ProtocolVars_t *pv, TransportType_t xportType, utBool brief, int dftMaxEvents)
{

    /* reset checksum before we start transmitting */
    cksumResetFletcher();

    /* transmit identification packets */
    if (!_udpSendIdentification(pv)) {
        return utFalse; // write error
    }

    /* 'brief' means send only the identification and EOB packets */
    // If the ID packet aren't sent (don't need to be sent), and 'speakFreekly' is true,
    // then its possible that nothing will be sent.
    utBool hasMoreEvents = utFalse;
    if (brief) {

        hasMoreEvents = _protocolHasDataToSend(pv);

    } else {

        /* transmit pending packets (sent first) */
        if (!_protocolSendQueue(pv, &(pv->pendingQueue), PRIORITY_HIGH, -1)) {
            return utFalse; // write error: close socket
        }

        /* transmit volatile packets (sent second) */
        if (!_protocolSendQueue(pv, &(pv->volatileQueue), PRIORITY_HIGH, -1)) {
            return utFalse; // write error: close socket
        }

        /* reset queues */
        // wait until all queues have successfully been sent before clearing
        pqueResetQueue(&(pv->volatileQueue));
        pqueResetQueue(&(pv->pendingQueue));

        /* send events flag */
        // default to sending events if the specified default maximum is not explicitly '0'
        PacketQueue_t *eventQueue = _protocolGetEventQueue(pv);
        utBool sendEvents = (eventQueue && (dftMaxEvents != 0))? utTrue : utFalse;
        // other criteria may also set this to false below

        /* determine if we should force-relinquish speak-freely */
        if (pv->speakFreely) {
            
            // Always relinquish speak-freely after sending a block of events
            if (sendEvents && pqueHasPackets(eventQueue)) {
                // If we have any events at all, relinquish speak-freely
                // This will allow the server to acknowledge these events and let the client
                // know that the server is listening.
                pv->speakFreely = utFalse;
                pv->speakFreelyMaxEvents = -1;
            }
        }

        /* send events */
        if (sendEvents) { // && (dftMaxEvents != 0)

            /* max events to send during this block */
            int maxEvents = 8;
            switch ((int)xportType) {
                case TRANSPORT_SIMPLEX:
                    if (pv->isPrimary) { // max simplex events
                        maxEvents = (int)propGetUInt32(PROP_COMM_MAX_SIM_EVENTS, 4L); // primary only
                        if (maxEvents > MAX_SIMPLEX_EVENTS) { maxEvents = MAX_SIMPLEX_EVENTS; }
                    } else
                    if (pv->isSerial) {
                        maxEvents = 1; // will never occur, 'serial' doesn't send events via 'simplex' 
                    } else {
                        maxEvents = MAX_SIMPLEX_EVENTS;
                    }
                    break;
                case TRANSPORT_DUPLEX:
                    if (pv->isPrimary) { // max duplex events
                        maxEvents = (int)propGetUInt32(PROP_COMM_MAX_DUP_EVENTS, 8L); // primary only
                        if (maxEvents > MAX_DUPLEX_EVENTS) { maxEvents = MAX_DUPLEX_EVENTS; }
                    } else
                    if (pv->isSerial) {
                        maxEvents = 1;
                    } else {
                        maxEvents = MAX_DUPLEX_EVENTS;
                    }
                    break;
                default:
                    logCRITICAL(LOGSRC,"Invalid Transport-Type: %d", (int)xportType);
                    break;
            }
            if ((dftMaxEvents > 0) && (maxEvents > dftMaxEvents)) {
                // limit number of event to the specified default maximum
                // (the value of 'dftMaxEvent' is disregarded if less than '0')
                maxEvents = dftMaxEvents;
            }

            /* max priority events to send */
            // - This function is getting called because "_getTransportType()" returned a transport
            // type based on what it found in the event queue.  If it chose a Simplex connection
            // based on 'Low' priority events found in the queue, then we should make sure that
            // only low priority events will get sent via Simplex.  This prevents Normal and High
            // priority events from getting sent that may have entered the queue while we are
            // still trying to set up the connection (which could take a while).
            // - If it is desirable to go ahead and send all (including High priority) events found
            // in the queue, then this restriction should be relaxed, however it would then be 
            // necessary to insure that these event don't get purged from the queue until they were 
            // successfully later sent via Duplex.
            PacketPriority_t maxPri;
            maxPri = (pv->isSerial || (xportType == TRANSPORT_DUPLEX) || !acctSupportsDuplex())? 
                PRIORITY_HIGH :     // all priority events will be sent
                PRIORITY_LOW;       // only low priority events will be sent
    
            /* transmit unacknowledged event packets */
            if (eventQueue && !_protocolSendQueue(pv, eventQueue, maxPri, maxEvents)) {
                return utFalse; // write error: close socket
            }

        } else {

            /* We didn't send events yet.  Check to see if we have any to send */
            hasMoreEvents = _protocolHasDataToSend(pv);

        }

    }

    /* send end-of-block packet */
    if ((xportType == TRANSPORT_DUPLEX) && !pv->speakFreely) {
        if (!_protocolSendEOB(pv,hasMoreEvents)) {
            return utFalse;
        }
    }

    return utTrue;
}
#endif //!defined(PROTOCOL_THREAD)
// ----------------------------------------------------------------------------

/* handle server-originated error packet */
static utBool _protocolHandleErrorCode(ProtocolVars_t *pv, UInt16 errCode, ClientPacketType_t pktHdrType, UInt8 *valData, int valDataLen)
{
	utBool ret = utTrue;
    //Buffer_t argBuf, *argSrc = binBuffer(&argBuf, valData, valDataLen, BUFFER_SOURCE);
	switch ((ServerError_t)errCode) {
	
	case NAK_ID_INVALID             : // Invalid unique id
		// The DMT server doesn't recognize our unique-id. 
		// We should try to send our account and device id.
		pv->sendIdentification = SEND_ID_ACCOUNT;
		break;
		
	case NAK_ACCOUNT_ERROR          : // Internal server error
	case NAK_DEVICE_ERROR           : // Internal server error
	case NAK_ACCOUNT_INVALID        : // Invalid/missing account id
	case NAK_DEVICE_INVALID         : // Invalid/missing device id
	case NAK_ACCOUNT_INACTIVE       : // Account has expired, or has become inactive
	case NAK_DEVICE_INACTIVE        : // Device has expired, or has become inactive
		// The DMT server encountered an error retrieve our ID
		logWARNING(LOGSRC,"Login error: account or device ID rejected by server");
		pv->severeErrorCount++;
		pv->invalidAcctErrorCount++; 
		break;

#if defined(TRANSPORT_MEDIA_SERIAL)
	case NAK_PACKET_ENCODING: // Encoding not supported
		// DMT servers are required to support Binary, ASCII-Base64, and ASCII-Hex encoding.
		// This should only occur if we are encoding packets in ASCII CSV format and the
		// server doesn't support this encoding.  We shoud try again with HEX or Base64
		// encoding for the remainder of the session.
		// We need to handle getting several of these errors during a transmission block.
		if (!pv->sessionEncodingChanged) {
			pv->sessionEncodingChanged = utTrue; // mark changed
			UInt32 encMask = ENCODING_MASK(pv->sessionEncoding);
			if (encMask & ENCODING_REQUIRED_MASK)  {
				// We're already encoding in a server supported encoding
				// This is likely some protocol compliance issue with the client
				// (of course it can't be the server! :-)
			}
			if (pv->isPrimary) { // session encodings
				UInt32 propEncodings = propGetUInt32(PROP_COMM_ENCODINGS, 0L); // primary only
				UInt32 encodingMask = propEncodings & ~encMask;
				propSetUInt32(PROP_COMM_ENCODINGS, encodingMask | ENCODING_REQUIRED_MASK); // primary only
				pv->sessionEncoding = _protocolGetSupportedEncoding(pv, pv->sessionEncoding);
			} else {
				pv->sessionEncoding = _protocolGetSupportedEncoding(pv, ENCODING_HEX);
			}
			if ((pktHdrType == PKT_CLIENT_UNIQUE_ID)  ||
				(pktHdrType == PKT_CLIENT_ACCOUNT_ID) ||
				(pktHdrType == PKT_CLIENT_DEVICE_ID)    ) {
				// error occured on identification packets, resend id
				// send account/device for serial transport
				pv->sendIdentification = SEND_ID_ACCOUNT;
				// try unique-id first for everything else
				pv->sendIdentification = SEND_ID_UNIQUE;
			}
		}
		break;
#endif
	case NAK_PACKET_CHECKSUM: // Invalid packet checksum (ASCII encoding only)
	case NAK_BLOCK_CHECKSUM: // Invalid block checksum
		// increment checksum failure indicator, if it gets too large, quit
		if (++(pv->checkSumErrorCount) >= 3) { // fail on 3rd error (per session)
			pv->severeErrorCount++;
		} 
		break;
	
	case NAK_PROTOCOL_ERROR: // Protocol error
		// This indicates a protocol compliance issue in the client
		logWARNING(LOGSRC,"protocol error: data unrecognized by server");
		pv->severeErrorCount++;
		break;
		
	case NAK_FORMAT_DEFINITION_INVALID: // Custom format type is invalid
	case NAK_PACKET_LENGTH:   // Invalid packet length
	case NAK_PACKET_PAYLOAD: // Invalid packet payload
	case NAK_FORMAT_NOT_SUPPORTED: // Custom formats not supported
	case NAK_FORMAT_NOT_RECOGNIZED: // Custom format not recognized
		/*The DMT server does not support custom formats (at least not for our current level of service)
		The DMT does support custom formats, but it doesn't recognize the 
		format we've used in an event packet.  We should send the custom
		format template(s), then resend the events. */
		_protocolAcknowledge(pv, pv->num_sent);
		logWARNING(LOGSRC,"format error: data format unrecognized by server");
		pv->severeErrorCount++;
		if (pv->isPrimary) { // custom formats not supported
			// We should acknowledge all sent events, and set a flag indicating that
			// we should not send custom formats to this server in the future.
			propSetBoolean(PROP_COMM_CUSTOM_FORMATS, utFalse);
		} 
		break;
	default:
		logWARNING(LOGSRC,"unknown error indicated by server");
		pv->severeErrorCount++;
		ret= utFalse;
		// ignore error
	}
    /* unhandled error - ignore */
	return ret;
}

// ----------------------------------------------------------------------------
/* handle server-originated packet */
static utBool _udpHandleServerPacket(ProtocolVars_t *pv, Packet_t *srvPkt) 
{
    // client should return 'utFalse' if an error has occured and communication should not continue
    /* handle packet */
	if (!clock_synchronized_with_server && (ServerPacketType_t)srvPkt->hdrType ==PKT_SERVER_SET_PROPERTY)
			return utTrue;
	
	switch ((ServerPacketType_t)srvPkt->hdrType) {
        // End of transmission, query response
        // Payload: maxEvents[optional], serverKey[optional]
	case PKT_SERVER_EOB_DONE: 
		return utTrue;
	case PKT_SERVER_EOB_SPEAK_FREELY: {
#ifdef TRANSPORT_MEDIA_SERIAL
            // flush server input buffer (useful for 'serial' transport only)
		_protocolFlushInput(pv); 
#endif
		pv->pending = true;
		return utTrue;
	}
        // Acknowledge [optional sequence]
        // Payload: sequence[optional]
	case PKT_SERVER_ACK: {
		int num_ack, fldCnt;
		if (srvPkt->dataLen > 0) {
			fldCnt = binScanf(srvPkt->data, (int)srvPkt->dataLen, "%1x", &num_ack);
			if (fldCnt <= 0) {
				num_ack = -1;
				logWARNING(LOGSRC,"Server Acknowledge: Wrong payload format");
				return utFalse;
			}
			++num_ack;
		}
		else 
			num_ack = pv->num_sent;
		if (!_protocolAcknowledge(pv, num_ack)) {
			logWARNING(LOGSRC,"Server Acknowledge: Cannot find corresponding packets");
		}
		return utTrue;
	}
	case PKT_SERVER_EOT: {
		long tdiff;
		if (!(clock_source & CLOCK_SYNC_INTERNET))
			return utTrue;
		if (srvPkt->dataLen == 4) {
			server_time = (time_t)((srvPkt->data[0] << 24) | (srvPkt->data[1] << 16)
								 	| (srvPkt->data[2] << 8) | srvPkt->data[3]);
			old_time = time(NULL);
			tdiff = server_time - old_time;
			if (tdiff > clock_delta || tdiff < -clock_delta)
				clock_need_adjust = true;
		}
		return utTrue;
	}
        // Get property
        // Payload: propertyKey(s)
	case PKT_SERVER_GET_PROPERTY : {
		Packet_t *propPkt = &messagePacket;
		int err;

		err = propGetPropertyPacket(pv->protoNdx, propPkt, srvPkt->data, srvPkt->dataLen, my_time);
		_protocolQueuePacket(pv, propPkt);
		pv->pending = true;
		return utTrue;
	}
        // Set property
        // Payload: propertyKey, propertyValue[optional]
	case PKT_SERVER_SET_PROPERTY : { 
	// set property value
	// TODO: If property is a 'Command', it would be very handy to be able to 
	// inform the property manager where to queue the returned data (specifically 
	// the primary/secondary protocol)
		Packet_t *propPkt = &messagePacket;
		int err;
		err = propSetValueCmd(pv->protoNdx, propPkt, pv->extBuf, srvPkt->dataLen, my_time); 
		_protocolQueuePacket(pv, propPkt);
		pv->pending = true;
		return utTrue;
	}
        // File upload
	case PKT_SERVER_FILE_UPLOAD  : { 
#if defined(ENABLE_UPLOAD)
		uploadProcessRecord(pv->protoNdx, srvPkt->data, (int)srvPkt->dataLen);
		// this already sends error/ack packets
		return utTrue;
#else
		_protocolQueueError(pv,"%2x%2x", (UInt32)ERROR_PACKET_TYPE, (UInt32)srvPkt->hdrType);
		// returned errors are ignored (possible internal buffer overflow)
		return utFalse;
#endif
	}
        
        // NAK/Error codes
        // Payload: errorCode, packetHeader, packetType, extraData
	case PKT_SERVER_ERROR        : { 
		UInt32 errCode = 0L, pktHdrType = 0L;
		utBool ok = utTrue;
		int valDataLen = (srvPkt->dataLen > 2)? (int)(srvPkt->dataLen - 2) : 0;
		int fldCnt = binScanf(srvPkt->data, (int)srvPkt->dataLen, "%2x%2x%*b", &errCode, &pktHdrType, valDataLen, encode_buf);
		if (fldCnt >= 1) {
			// 'errCode' contains the Error Code
			// 'pktHdr'  contains the error packet header
			// 'pktTyp'  containt the error packet type
			// 'valData' contains any additional data
			ok = _protocolHandleErrorCode(pv, (UInt16)errCode, (ClientPacketType_t)pktHdrType, encode_buf, valDataLen);
		} 
		return ok;
	}
    // Invalid packet type
	default: {
		_protocolQueueError(pv,"%2x%2x", (UInt32)ERROR_PACKET_TYPE, (UInt32)srvPkt->hdrType);
            // returned errors are ignored (possible internal buffer overflow)
		return utFalse;
	}
	}
	return utTrue;
}
/* handle server-originated packet */
static utBool _tcpHandleServerPacket(ProtocolVars_t *pv, Packet_t *srvPkt) 
{
	if (!clock_synchronized_with_server && (ServerPacketType_t)srvPkt->hdrType ==PKT_SERVER_AUTH)
		return utTrue;
    // client should return 'utFalse' if an error has occured and communication should not continue
    /* handle packet */
	switch ((ServerPacketType_t)srvPkt->hdrType) {
        
        // End of transmission, query response
        // Payload: maxEvents[optional], serverKey[optional]
	case PKT_SERVER_EOB_DONE: 
	case PKT_SERVER_EOB_SPEAK_FREELY: {
		if (pv->payload_type == PAYLOAD_PENDING && pv->num_sent > 0) {
			if (!_protocolAcknowledge(pv, pv->num_sent))
				logWARNING(LOGSRC,"Server Acknowledge: Cannot find corresponding packets");
		}
#ifdef TRANSPORT_MEDIA_SERIAL
            // flush server input buffer (useful for 'serial' transport only)
		_protocolFlushInput(pv); 
#endif
		return utTrue;
	}
        // Acknowledge [optional sequence]
        // Payload: sequence[optional]
	case PKT_SERVER_ACK: {
		int num_ack, fldCnt;
		UInt32 last_seq = 0;
		if (srvPkt->dataLen > 0) {
			fldCnt = binScanf(srvPkt->data, (int)srvPkt->dataLen, "%1x", &last_seq);
			if (fldCnt <= 0) {
				logWARNING(LOGSRC,"Server Acknowledge: Wrong payload format");
				return utFalse;
			}
			num_ack = last_seq + 1 - (pv->sequence_first & 0xFF);
			if (num_ack <= 0)
				num_ack += 0x100;
		}
		else 
			num_ack = pv->num_sent;
		if (!_protocolAcknowledge(pv, num_ack)) {
			logWARNING(LOGSRC,"Server Acknowledge: Cannot find corresponding packets");
		}
		return utTrue;
	}
	case PKT_SERVER_AUTH: {
		if (!(clock_source & CLOCK_SYNC_INTERNET))
			return utTrue;
		if (srvPkt->dataLen == 4) {
			long tdiff;
			server_time = (srvPkt->data[0] << 24) | (srvPkt->data[1] << 16) |
								 		(srvPkt->data[2] << 8) | srvPkt->data[3];
			old_time = time(NULL);
			tdiff = server_time - old_time;
			if (tdiff > clock_delta || tdiff < -clock_delta)
				clock_need_adjust = true;
		}
		return utTrue;
	}
	// End-of-transmission
	case PKT_SERVER_EOT: {
		if (pv->payload_type == PAYLOAD_PENDING && pv->num_sent > 0) {
			if (!_protocolAcknowledge(pv, pv->num_sent))
				logWARNING(LOGSRC,"Server Acknowledge: Cannot find corresponding packets");
		}
		pv->session_continue = false;
		return utTrue;
	}
        // Get property
        // Payload: propertyKey(s)
	case PKT_SERVER_GET_PROPERTY : {
		Packet_t *propPkt = &messagePacket;
		int err;
		err = propGetPropertyPacketTCP(pv->protoNdx, propPkt, srvPkt->data, srvPkt->dataLen);
		if (err == 0) {
			_protocolQueuePacket(pv, propPkt);
		}
		else {
			UInt32 key = (srvPkt->data[0] << 8) | srvPkt->data[1];
			_protocolQueueError(pv, "%2X%2X", err, key);
		}
		pv->pending = true;
		return utTrue;
	}
        // Set property
        // Payload: propertyKey, propertyValue[optional]
	case PKT_SERVER_SET_PROPERTY : { 
	// set property value
	// TODO: If property is a 'Command', it would be very handy to be able to 
	// inform the property manager where to queue the returned data (specifically 
	// the primary/secondary protocol)
		int err;
		UInt32 cmd_err = 0;
		err = propSetValueCmdTCP(pv->protoNdx, pv->extBuf, srvPkt->dataLen, &cmd_err); 
		if (err != 0) {
			UInt32 key = (srvPkt->data[0] << 8) | srvPkt->data[1];
			if (cmd_err == 0)
				_protocolQueueError(pv, "%2X%2X", err, key);
			else
				_protocolQueueError(pv, "%2X%2X%2X", err, key, cmd_err);
		}
		pv->pending = true;
		return utTrue;
	}
        // File upload
	case PKT_SERVER_FILE_UPLOAD  : { 
#if defined(ENABLE_UPLOAD)
		uploadProcessRecord(pv->protoNdx, srvPkt->data, (int)srvPkt->dataLen);
		// this already sends error/ack packets
		return utTrue;
#else
		_protocolQueueError(pv,"%2x%2x", (UInt32)ERROR_PACKET_TYPE, (UInt32)srvPkt->hdrType);
		// returned errors are ignored (possible internal buffer overflow)
		return utFalse;
#endif
	}
        
        // NAK/Error codes
        // Payload: errorCode, packetHeader, packetType, extraData
	case PKT_SERVER_ERROR        : { 
		UInt32 errCode = 0L, pktHdrType = 0L;
		utBool ok = utTrue;
		int valDataLen = (srvPkt->dataLen > 2)? (int)(srvPkt->dataLen - 2) : 0;
		int fldCnt = binScanf(srvPkt->data, (int)srvPkt->dataLen, "%2x%2x%*b", &errCode, &pktHdrType, valDataLen, encode_buf);
		if (fldCnt >= 1) {
			// 'errCode' contains the Error Code
			// 'pktHdr'  contains the error packet header
			// 'pktTyp'  containt the error packet type
			// 'valData' contains any additional data
			ok = _protocolHandleErrorCode(pv, (UInt16)errCode, (ClientPacketType_t)pktHdrType, encode_buf, valDataLen);
		} 
		return ok;
	}
    // Invalid packet type
	default: {
		_protocolQueueError(pv,"%2x%2x", (UInt32)ERROR_PACKET_TYPE, (UInt32)srvPkt->hdrType);
            // returned errors are ignored (possible internal buffer overflow)
		return utFalse;
	}
	}
	return utTrue;
}

#if !defined(PROTOCOL_THREAD)
// ----------------------------------------------------------------------------

// This function checks to see if it is time to make a connection to the DMT service
// provider, and what type of connection to make.  The time of the last connection
// and the number of connections made in the last hour are considered.
static TransportType_t _getTransportType(ProtocolVars_t *pv)
{
    
    /* secondary protocol */
    if (!pv->isPrimary) { // duplex transport for non-primary transport
        // Assume the secondary protocol supports duplex
        // - Highest event priority is PRIORITY_NONE (no events are sent to the secondary protocol)
        // - Never over duplex quota
        return TRANSPORT_DUPLEX;
    }

    /* first check absolute minimum delay between connections */
    if (!acctAbsoluteDelayExpired()) {
        // absolute minimum delay interval has not expired
        //logINFO(LOGSRC,"Absolute minimum delay interval has not expired");
        return TRANSPORT_NONE;
    }
    
    /* check specific event priority */
    TransportType_t xportType = TRANSPORT_NONE;
    PacketPriority_t evPri = _protocolGetHighestPriority(pv);
    switch (evPri) {

        // no events, time for 'checkup'?
        case PRIORITY_NONE: 
            if (!acctUnderTotalQuota()) {
                // over Total quota
                xportType = TRANSPORT_NONE;
            } else
            if (!acctMaxIntervalExpired()) {
                // MAX interval has not expired
                xportType = TRANSPORT_NONE;
            } else
            if (acctUnderDuplexQuota()) {
                // under Total/Duplex quota and MAX interval expired, time for Duplex checkup
                //logDEBUG(LOGSRC,"_getTransportType: NONE/DUPLEX ...");
                xportType = TRANSPORT_DUPLEX;
            } else {
                // over Duplex quota
                xportType = TRANSPORT_NONE;
            }
            break;

        // low priority events
        case PRIORITY_LOW: 
            if (!acctUnderTotalQuota()) {
                // over Total quota, no sending
                xportType = TRANSPORT_NONE;
            } else
            if (!acctMinIntervalExpired()) {
                // min interval has not expired, no sending
                xportType = TRANSPORT_NONE;
            } else
            if (acctSupportsSimplex()) {
                // under Total quota, min interval expired, send Simplex
                //logDEBUG(LOGSRC,"_getTransportType: 1 LOW/SIMPLEX ...");
                xportType = TRANSPORT_SIMPLEX;
            } else
            if (acctUnderDuplexQuota()) {
                // under Total/Duplex quota and min interval expired, Simplex not supported, send Duplex
                //logDEBUG(LOGSRC,"_getTransportType: 2 LOW/DUPLEX ...");
                xportType = TRANSPORT_DUPLEX;
            } else {
                if (!acctSupportsDuplex()) {
                    logCRITICAL(LOGSRC,"Transport does not support Simplex or Duplex!!!");
                }
                // over Duplex quota (or Duplex not supported), no sending
                xportType = TRANSPORT_NONE;
            }
            break;

        // normal priority events
        case PRIORITY_NORMAL:
            if (!acctUnderTotalQuota()) {
                // over Total quota, no sending
                xportType = TRANSPORT_NONE;
            } else
            if (!acctMinIntervalExpired()) {
                // min interval has not expired, no sending
                xportType = TRANSPORT_NONE;
            } else
            if (acctUnderDuplexQuota()) {
                // under Total/Duplex quota and min interval expired, send Duplex
                //logDEBUG(LOGSRC,"_getTransportType: 1 NORMAL/DUPLEX ...");
                xportType = TRANSPORT_DUPLEX;
            } else
            if (!acctSupportsDuplex()) {
                // under Total quota, but the client doesn't support Duplex connections, send Simplex
                //logDEBUG(LOGSRC,"_getTransportType: 2 NORMAL/SIMPLEX ...");
                xportType = TRANSPORT_SIMPLEX;
            } else {
                // over Duplex quota, no sending
                xportType = TRANSPORT_NONE;
            }
            break;

        // high priority events
        case PRIORITY_HIGH:
        default: // catch-all
            if (acctUnderDuplexQuota()) { // (disregard timer interval and total quota)
                // under Duplex quota and critical event, send Duplex
                //logDEBUG(LOGSRC,"_getTransportType: 1 HIGH/DUPLEX ...");
                xportType = TRANSPORT_DUPLEX;
            } else
            if (!acctSupportsDuplex()) {
                // critical event, but the client does not support duplex connections, send Simplex
                //logDEBUG(LOGSRC,"_getTransportType: 2 HIGH/SIMPLEX ...");
                xportType = TRANSPORT_SIMPLEX;
            } else {
                // over Duplex quota, no sending
                xportType = TRANSPORT_NONE;
            }
            break;

    }
    
    //logINFO(LOGSRC,"_getTransportType: %d/%d", evPri, xportType);
    return xportType;
    
}

// ----------------------------------------------------------------------------
/* open duplex session */
static utBool _protocolDuplexTransport(ProtocolVars_t *pv)
{
    Packet_t pkt;
    
    /* open transport */
    if (!_protocolOpen(pv, TRANSPORT_DUPLEX)) {
        if (utcIsTimerExpired(pv->lastDuplexErrorTimer,300L)) {
            pv->lastDuplexErrorTimer = utcGetTimer();
            logINFO(LOGSRC,"Unable to open Duplex transport [%d]", pv->protoNdx);
        }
        return utFalse;
    }
    logINFO(LOGSRC,"Duplex start [%d] ...", pv->protoNdx);

    /* check for GPS Fix expiration ("stale") */
    if (gpsIsFixStale()) {
        // queue GPS error message
        GPSDiagnostics_t gpsDiag;
        gpsGetDiagnostics(&gpsDiag);
        if (utcGetTimeSec() > (gpsDiag.lastSampleTime + GPS_EVENT_INTERVAL)) {
            // Likely serious GPS problem.
            // We haven't received ANYTHING from the GPS reciver in the last GPS_EVENT_INTERVAL seconds
            // The GPS receiver is no longer working!
            _protocolQueueError(pv,"%2x%4u", (UInt32)ERROR_GPS_FAILURE, (UInt32)gpsDiag.lastSampleTime);
        } else {
            // The GPS receiver still appears to be working, the fix is just expired
            _protocolQueueError(pv,"%2x%4u", (UInt32)ERROR_GPS_EXPIRED, (UInt32)gpsDiag.lastValidTime);
        }
    }

    /* default speak freely permission on new connections */
    pv->speakFreely = utFalse;
    pv->speakFreelyMaxEvents = -1;
    pv->relinquishSpeakFreely = utFalse;

    /* default speak-brief on new connection */
    utBool speakFirst = utTrue;
    if (pv->isPrimary) { // first/brief primary transport
        speakFirst     = propGetBoolean(PROP_COMM_SPEAK_FIRST, utTrue); // protocol dependent
        pv->speakBrief = propGetBoolean(PROP_COMM_FIRST_BRIEF, utFalse); // protocol dependent
    } else
    if (pv->isSerial) {
        speakFirst     = utFalse;
        pv->speakBrief = utTrue;
    } else {
        speakFirst     = utTrue;
        pv->speakBrief = utFalse;
    }

    /* packet handling loop */
    utBool rtnOK       = utTrue;
    utBool keepLooping = utTrue;
    utBool firstPass   = utTrue;
    for (;keepLooping;) {
        
        /* send queued packets */
        if (firstPass) {
            firstPass = utFalse;
            if (speakFirst) {
                // client initiates conversation
                // send identification and first block of events
                // 'pv->speakFreely' is always false here
                int firstMaxEvents = -1; // <-- if 'speakBrief' is true, no events will be sent
                if (!_protocolSendAllPackets(pv, TRANSPORT_DUPLEX, pv->speakBrief, firstMaxEvents)) {
                    rtnOK = utFalse; // write error
                    break;
                }
                pv->speakBrief = utFalse;
            }
        } else
        if (pv->speakFreely) {
            // send any pending packets
            // During 'speak-freely' wait until we have something to send.
            if (_protocolHasDataToSend(pv)) {
                int dupMaxEvents = pv->speakFreelyMaxEvents;
                // The thread may decide whether, or not, to relinquish 'speakFreely' permission
                if (pv->relinquishSpeakFreely) {
                    pv->speakFreely = utFalse; // relinquish speak-freely permission
                    pv->speakFreelyMaxEvents = -1;
                }
                if (!_protocolSendAllPackets(pv, TRANSPORT_DUPLEX, utFalse, dupMaxEvents)) {
                    rtnOK = utFalse; // write error
                    break;
                }
            }
        }
        
        /* read packet */
        int err = _protocolReadServerPacket(pv, &pkt); // <-- timeout is specified by transport
        if (err < 0) {
            // read/parse error (transport no longer open?)
            rtnOK = utFalse;
            break;
        } else 
        if (err == 0) {
            // read timeout
            if (pv->speakFreely) {
                // read timeouts are allowed in 'speak-freely' mode
                continue;
            }
            if (pv->isSerial) {
                // timeouts are ignored with serial transport (only exit on transmit errors)
                continue;
            } else {
                logINFO(LOGSRC,"Duplex server read timeout [%d]", pv->protoNdx);
                // this is an error when not in 'speak-freely' mode, or not in a thread
                // otherwise we'll be blocking the mainloop for too long.
                rtnOK = utFalse;
                break;
            }
        }
        
        /* handle received packet */
        keepLooping = _protocolHandleServerPacket(pv, &pkt);

    }
    
    /* close transport */
    _protocolClose(pv, TRANSPORT_DUPLEX);
    if (pv->isPrimary) { // record duplex connection
        // 'primary' only
        acctSetDuplexConnection();
    }
    logINFO(LOGSRC,"Duplex end [%d] ...", pv->protoNdx);
    return rtnOK;

}

/* open simplex session */
static utBool _protocolSimplexTransport(ProtocolVars_t *pv)
{
    //logINFO(LOGSRC,"Simplex ...");
    
    /* must be the primary protocol */
    if (!pv->isPrimary) { // no simplex for non-primary transport
        // Simplex only allowed for 'primary'
        return utFalse;
    }

    /* open transport */
    if (!_protocolOpen(pv, TRANSPORT_SIMPLEX)) {
        return utFalse;
    }
    
    /* check for GPS Fix expiration ("stale") */
    if (gpsIsFixStale()) {
        // queue GPS error message
        GPSDiagnostics_t gpsDiag;
        gpsGetDiagnostics(&gpsDiag);
        if (utcGetTimeSec() > (gpsDiag.lastSampleTime + GPS_EVENT_INTERVAL)) {
            // Likely serious GPS problem.
            // We haven't received ANYTHING from the GPS reciver in the last GPS_EVENT_INTERVAL seconds
            // The GPS receiver is no longer working!
            _protocolQueueError(pv,"%2x%4u", (UInt32)ERROR_GPS_FAILURE, (UInt32)gpsDiag.lastSampleTime);
        } else {
            // The GPS receiver still appears to be working, the fix is just expired
            _protocolQueueError(pv,"%2x%4u", (UInt32)ERROR_GPS_EXPIRED, (UInt32)gpsDiag.lastValidTime);
        }
    }

    /* send queued packets/events */
    int simMaxEvents = -1;
    if (!_protocolSendAllPackets(pv, TRANSPORT_SIMPLEX, utFalse, simMaxEvents)) {
        _protocolClose(pv, TRANSPORT_SIMPLEX); // <-- will only occur if we have a buffer overflow
        return utFalse;
    }
    
    /* acknowledge sent events */
    if (_protocolClose(pv, TRANSPORT_SIMPLEX)) {
        // - Data doesn't get transmitted until the close for Simplex connections.
        // So events should not be auto-acknowledged until the close has occured
        // and didn't get any errors. (This still doesn't guarantee that the server
        // received the data).
        // - Since most wireless data services will be placing the device behind a
        // NAT'ed router, there is no way for the server to send back a UDP
        // acknowledgement to the device.  As such, no attempt is made to read an
        // acknowledgement from the server.
        pqueResetQueue(&(pv->pendingQueue)); // remove all pending messages
        _protocolAcknowledgeToSequence(pv,SEQUENCE_ALL);
        acctSetSimplexConnection();
        return utTrue;
    } else {
        return utFalse;
    }

}

// ----------------------------------------------------------------------------
/* start transport session */
static utBool _protocolRunSession(ProtocolVars_t *pv)
{
    // Warning: If running in a multi-threaded environment make sure this function, 
    // and all functions it calls, are thread-safe!
    //logINFO(LOGSRC,"Transport session: %s", pv->xFtns->name);
    
    /* check for transport ready */
    TransportType_t xportType = TRANSPORT_NONE;
    PROTOCOL_LOCK(pv) {
        if (pv->currentTransportType != TRANSPORT_NONE) {
            xportType = pv->currentTransportType;
            PacketEncoding_t encoding = _protocolGetSupportedEncoding(pv, pv->currentEncoding);
            _protocolSetSessionEncoding(pv, xportType, encoding);
        }
    } PROTOCOL_UNLOCK(pv)
    if (xportType == TRANSPORT_NONE) {
        // not ready for connection yet
        return utFalse;
    }
    
    /* reset speak freely */
    pv->speakFreely = utFalse;
    pv->speakFreelyMaxEvents = -1;

    /* establish connections */
    if (xportType == TRANSPORT_SIMPLEX) {
        // establish Simplex communication here
        _protocolSimplexTransport(pv);
    } else 
    if (xportType == TRANSPORT_DUPLEX) {
        // establish Duplex communication here
        _protocolDuplexTransport(pv);
    }
    // If the above connections fail, the outer protocol loop will be calling this function again
    
    /* reset for next connection */
    PROTOCOL_LOCK(pv) {
        // even though the above may have failed, reset the transport type anyway.
        // it will be set again by the main thread.
        pv->currentTransportType = TRANSPORT_NONE;
    } PROTOCOL_UNLOCK(pv)
    return utTrue;

}
#endif
// ----------------------------------------------------------------------------
/* start transport session */
bool udp_session(ProtocolVars_t *pv)
{
	int len, err;
	int remain_len;
	bool more_events = false;
	UInt8 *pkt_ptr;
	PacketQueue_t *eventQueue = _protocolGetEventQueue(pv);
	/* open transport */
	if (!_udpOpen(pv)) {
		logINFO(LOGSRC,"Failed establishing UDP connection");
		return utTrue;
	}
	do {
		if (!_udpSendIdentification(pv))
			break;

		if (clock_synchronized_with_server) {
			if (pqueHasUnsentPacket(&pv->pendingQueue)) {
				if ((err = _protocolSendQueue(pv, &pv->pendingQueue)) < 0)
					break;
				else
					pv->payload_type = PAYLOAD_PENDING;
			}
			else if (pqueHasUnsentPacket(eventQueue)) { 
				if ((err = _protocolSendQueue(pv, eventQueue)) < 0)
					break;
				else
					pv->payload_type = PAYLOAD_EVENT;
			}
		}

		if (!_udpSendEOB(pv)) { 
			break;
		}
		memset(pv->readBuf, 0, 32);
		pv->sessionReadBytes = 0;
		pv->pending = false;
		if ((len = pv->xFtns->Read(pv->readBuf, PROTOCOL_READ_BUF_SIZE)) > 0) {
			pv->totalReadBytes   += len;
			pv->sessionReadBytes += len;
		}
		else
			break;
		/*process server packets*/
		pkt_ptr = memchr(pv->readBuf + pv->overheadBytes, PACKET_HEADER_BASIC, pv->sessionReadBytes); 
		if (pkt_ptr == NULL)
			break;
		remain_len = pv->sessionReadBytes -  (pkt_ptr - pv->readBuf);
		my_time = time(NULL);
		while (remain_len >= 3) {
			if (_protocolParseServerPacket(pv, &serverPacket, pkt_ptr) == (Packet_t*)0)
				break;
			_udpHandleServerPacket(pv, &serverPacket);
				pkt_ptr += (serverPacket.dataLen + 3);
				remain_len -= (serverPacket.dataLen + 3);
				serverPacket.dataLen = 0;
		}
		if (!clock_synchronized_with_server) {
			break;
		}
		more_events = ((pqueHasUnsentPacket(eventQueue)) || (pqueHasUnsentPacket(&pv->pendingQueue)));
	}
	while ((pv->pending || more_events) && (pv->severeErrorCount == 0));
	_protocolClose(pv);
	if (pv->xFtns->transport_error < -1 || pv->severeErrorCount > 0)
		return utTrue;
	else
		return utFalse;
}

bool tcp_session(ProtocolVars_t *pv)
{
	int remain_len, err;
	bool more_events;
	UInt8 *pkt_ptr;
	PacketQueue_t *eventQueue = _protocolGetEventQueue(pv);
	/* open transport */
	if (!_tcpOpen(pv)) {
		logINFO(LOGSRC,"Failed establishing TCP connection");
		return utTrue;
	}
	/* transmit identification packets */
	if (!_tcpSendIdentification(pv))
		goto session_end;
	
	if (_tcpReadPacket(pv, true) <= 0)
		goto session_end;
	else {
		/*process server packets*/
		pkt_ptr = memchr(pv->readBuf, PACKET_HEADER_BASIC, pv->sessionReadBytes); 
		if (pkt_ptr == NULL)
			goto session_end;
		remain_len = pv->sessionReadBytes -  (pkt_ptr - pv->readBuf);
		my_time = time(NULL);
		while (remain_len >= 3) {
			if (_protocolParseServerPacket(pv, &serverPacket, pkt_ptr) == (Packet_t*)0)
				break;
			_tcpHandleServerPacket(pv, &serverPacket);
			pkt_ptr += (serverPacket.dataLen + 3);
			remain_len -= (serverPacket.dataLen + 3);
			serverPacket.dataLen = 0;
		}
		if (!clock_synchronized_with_server || (pv->severeErrorCount > 0)) {
			_tcpSendEOB(pv, false);
			goto session_end;
		}
	}
		
	more_events = pqueHasUnsentPacket(eventQueue);
	/*receive and process server packets */
	while (pv->session_continue && (pv->pending || more_events)) {
		if (pqueHasUnsentPacket(&pv->pendingQueue)) {
			if ((err = _protocolSendQueue(pv, &pv->pendingQueue)) < 0)
				break;
			else
				pv->payload_type = PAYLOAD_PENDING;
		} 
		else if (pqueHasUnsentPacket(eventQueue)) { 
			if ((err = _protocolSendQueue(pv, eventQueue)) < 0)
				break;
			else
				pv->payload_type = PAYLOAD_EVENT;
		}
		more_events = pqueHasUnsentPacket(eventQueue);
		if (!_tcpSendEOB(pv, more_events)) { 
			break;
		}
		/*read server packets*/
		memset(pv->readBuf, 0, 32);
		pv->sessionReadBytes = 0;
		if (_tcpReadPacket(pv, true) < 0)
			break;
		if (!_tcpIsEndOfReceiving(pv)) {
			usleep(300000);
			if (_tcpReadPacket(pv, false) < 0)
				break;
		}
		/*process server packets*/
		pkt_ptr = memchr(pv->readBuf, PACKET_HEADER_BASIC, pv->sessionReadBytes); 
		if (pkt_ptr == NULL)
			break;
		remain_len = pv->sessionReadBytes -  (pkt_ptr - pv->readBuf);
		my_time = time(NULL);
		pv->pending = false;
		while (remain_len >= 3) {
			if (_protocolParseServerPacket(pv, &serverPacket, pkt_ptr) == (Packet_t*)0)
				break;
			_tcpHandleServerPacket(pv, &serverPacket);
			pkt_ptr += (serverPacket.dataLen + 3);
			remain_len -= (serverPacket.dataLen + 3);
			serverPacket.dataLen = 0;
		}
	} // while (!pv->session_continue)
session_end:
	_protocolClose(pv);
	if (pv->xFtns->transport_error < -1 || pv->severeErrorCount > 0)
		return utTrue;
	else
		return utFalse;
}
// ----------------------------------------------------------------------------

#ifdef PROTOCOL_THREAD
static void report_link_down(void)
{
	pthread_mutex_lock(&network_status_mutex);
	if (!(network_link_status)) {
		network_link_status = NETWORK_STATUS_TIMEOUT;
		pthread_cond_signal(&network_down_sema);
	}
	pthread_mutex_unlock(&network_status_mutex);
}
/* where the thread spends its time */
void * _protocolThreadRunnable(void *arg)
{
	char message[64] = {};

	ProtocolVars_t *pv = (ProtocolVars_t*)arg;
	struct timespec later;
	int err, err1, beat, beat1;
	bool transmission_error = false;
	bool link_down_symptom = false;
	bool link_recover;
	time_t now;

	time_t current;
	struct tm tm1; 
	struct tm *ptm;
	unsigned int network_chck_wait_times = propGetUInt32(PROP_STATE_NETWORK_CHECK_WAIT_TIMES, 3);
//	network_chck_wait_times -= 1;
	unsigned int network_check_countdown = 0;
	
	_protocolInitVars(pv);
	add_watchdog_func(watchdog_protocol_monitor);
	pv->protoRunThread = utTrue;
	sleep(1);
	if (!pv->protoRunThread)
		goto protocol_end;
	link_recover = (network_link_status != 0);
	/* protocol handler loop */
	while (pv->protoRunThread) {
		if (link_recover)
			goto check_link_status;
		/*waiting loop: wait for events*/
		PROTOCOL_LOCK(pv) {
		for (beat = 0, beat1 = 1; beat < pv->silent_cycle; beat++, beat1++) {
			if ((err = clock_gettime(CLOCK_REALTIME, &later)) != 0) {
				perror("Realtime Clock");
				goto protocol_end;
			}
			later.tv_sec += pv->main_period;
			err1 = PROTOCOL_WAIT(pv, &later);
			if (!pv->protoRunThread) {
				PROTOCOL_UNLOCK(pv)
				goto protocol_end;
			}
			if (err1 == 0 || beat1 >= pv->session_cycle) {
				if (pqueHasPackets(_protocolGetEventQueue(pv)))
					break;
				beat1 = 0;
			} else if (err1 == ETIMEDOUT && pv->paranoid_preserving)
				pquePreserveQueue(_protocolGetEventQueue(pv));
		}
		} PROTOCOL_UNLOCK(pv)
		if (pv->power_saving)
			report_link_down();
check_link_status:
		pthread_mutex_lock(&network_status_mutex);
		while (network_link_status) {
			if ((err = clock_gettime(CLOCK_REALTIME, &later)) != 0) {
				pthread_mutex_unlock(&network_status_mutex);
				perror("Realtime Clock");
				goto protocol_end;
			}
			later.tv_sec += pv->recession_period;
			err1 = pthread_cond_timedwait(&network_up_sema, &network_status_mutex, &later);
			if (!pv->protoRunThread) {
				pthread_mutex_unlock(&network_status_mutex);
				goto protocol_end;
			}
			else if (err1 == ETIMEDOUT && pv->event_preserving)
				pquePreserveQueue(_protocolGetEventQueue(pv));
		}	
		pthread_mutex_unlock(&network_status_mutex);
		if (link_down_symptom && !link_down_occurred) {
			sleep(DEFAULT_SESSION_PERIOD);
//			sleep(pv->recession_period);
			link_down_symptom = false;
			if (++url_swap_count > 18) {
				terminateNetwork(timer7, &it7);
				report_link_down();
				url_swap_count = 0;
				link_recover = true;
				continue;
			} else {
				url_id ^= 1;
				url_reset = true;
			}
		}
		/* start transmission */
		debuglog(LOG_INFO,"TX->Server...");
		if (transport_protocol == TRANSPORT_UDP)
			transmission_error = udp_session(pv);
		else
			transmission_error = tcp_session(pv);
		/* error check*/
		if (!transmission_error) {
			debuglog(LOG_INFO,"...RX<-Server");
			if (url_id != 0) {
				url_id = 0;
				url_reset = true;
			}
			if (pv->power_saving) {
				terminateNetwork(timer7, &it7);
				current = time(NULL);
				ptm = localtime_r(&current, &tm1);
				print_debug("%02d/%02d/%4d %02d:%02d:%02d: Terminated Network\n", 
					ptm->tm_mon+1, ptm->tm_mday, ptm->tm_year+1900, 
					ptm->tm_hour, ptm->tm_min, ptm->tm_sec);
			}
			link_recover = false;
			link_down_symptom = false;
			protocol_down_count = 0;
			url_swap_count = 0;
			network_check_countdown = 0;
			if (clock_need_adjust) {
				long tdiff;
				tdiff = server_time - old_time;
				if (tdiff > clock_delta || tdiff < -clock_delta) {
					now = time(NULL);
					synchronize_system_clock((time_t)(server_time + (now - old_time)));
					logINFO(LOGSRC, "Clock synchronized with server");
					sprintf(message, "Clock synchronized from %ld to %ld", old_time, (server_time + (now - old_time)));
					printf("%s\n", message);
					update_timestamp(tdiff);
					diagnostic_report(DIAGNOSTIC_MESSAGE, 0, message);
				}
				clock_need_adjust = false;
				clock_synchronized_with_server = true;
				sleep(25);
			}
		} else {
			Server_NO_Response();
			if (pv->xFtns->transport_error < COMERR_SERVER) {
				/* error check: communication failure*/
				if (network_chck_wait_times - network_check_countdown) {
					debuglog(LOG_INFO,"retry Tx %d more times", network_chck_wait_times - network_check_countdown);
				}
				if (network_check_countdown++ < network_chck_wait_times) {
					if (url_id != 0) {
						url_id = 0;
						url_reset = true;
					}
					
					link_recover = false;
					link_down_symptom = false;
					protocol_down_count = 0;
					url_swap_count = 0;
					continue;
				}	
				if (!pv->protoRunThread)
					goto protocol_end;
				if (pv->event_preserving)
					pquePreserveQueue(_protocolGetEventQueue(pv));
			
				link_recover = true;
				link_down_symptom = true;
				url_reset = true;
				report_link_down();
			} else {
		/* error check: self error */
				link_recover = true;
				if (pv->event_preserving)
					pquePreserveQueue(_protocolGetEventQueue(pv));
				if (++url_swap_count > 18) {
					terminateNetwork(timer7, &it7);
					report_link_down();
					url_swap_count = 0;
				} else {
					url_id ^= 1;
					url_reset = true;
					sleep(DEFAULT_SESSION_PERIOD);
				}
			}
		}
	} // while (pv->protoRunThread)
    
protocol_end:
	free(pv->readBuf);
	free(pv->sendBuf);
	pqueReleaseQueue(&pv->pendingQueue);
	pqueReleaseQueue(_protocolGetEventQueue(pv));
	threadExit();
	return NULL;
}

/* indicate thread should stop */
static void _protocolStopThread(void *arg)
{
    ProtocolVars_t *pv = (ProtocolVars_t*)arg;
    pv->protoRunThread = utFalse;
    PROTOCOL_LOCK(pv) {
        // nudge thread
        PROTOCOL_NOTIFY(pv)
    } PROTOCOL_UNLOCK(pv)
}

/* start thread */
static utBool _protocolStartThread(int instance_id)
{
	ProtocolVars_t * pv = &protoVars[instance_id];
    /* create thread */
	if (threadCreate(&protocolThread, _protocolThreadRunnable, (void*)pv, "Protocol") == 0) {
        // thread started successfully
		threadAddThreadStopFtn(&_protocolStopThread, pv);
	} else {
		logCRITICAL(LOGSRC,"Unable to create protocol thread!!");
	}
	return utTrue;
}

#endif

// ----------------------------------------------------------------------------
static void protocol_init_timer(ProtocolVars_t *pv)
{
	UInt32	t0, t1, t2, t3;
	t0 = propGetUInt32(PROP_COMM_SAVE_RATE, 1) * 60;
	t1 = propGetUInt32(PROP_COMM_MIN_XMIT_RATE, 1);
	t2 = propGetUInt32(PROP_COMM_MAX_XMIT_RATE, 1);
	t3 = t0 >> 1;
	if ((t0 == 0) || ((t0 + t3) > t1)) {
		pv->main_period = t1;
		pv->session_cycle = 1;
		pv->silent_cycle = (t2 + t1 - 1) / t1;
		pv->paranoid_preserving = false;
		if (t0 == 0) {
			pv->recession_period = t2;
			pv->event_preserving = false;
		}
		else {
			pv->recession_period = t0;
			pv->event_preserving = true;
		}
	} else {
		pv->main_period = t0;
		pv->recession_period = t0;
		pv->session_cycle = (t1 + t3) / t0;
		pv->silent_cycle = (t2 + t3) / t0;
		pv->paranoid_preserving = true;
		pv->event_preserving = true;
	}
	protocol_reboot_timer = propGetUInt32(PROP_COMM_MAX_DELAY, 24) * 360; 	
//	protocol_reboot_timer = 60;
}

static void  _protocolInitVars(ProtocolVars_t *pv)
{
	int err;
	UInt32 mtu;
	// transport
	if (!transport_protocol) {
		protocol_version = propGetUInt32(PROP_STATE_PROTOCOL, 3);
		if (protocol_version == PROTO_VERSION_TCP)
			transport_protocol = TRANSPORT_TCP;
		else
			transport_protocol = TRANSPORT_UDP;
	}
	if (transport_protocol == TRANSPORT_TCP) {
		protocol_version = PROTO_VERSION_TCP;
		pv->xFtns = &socketTCP_funcs;
		pv->overheadBytes = 0;
	}
	else if (transport_protocol == TRANSPORT_UDP) {
		protocol_version = PROTO_VERSION_UDP;
		pv->xFtns = &socketUDP_funcs;
		pv->overheadBytes = 1;
	}
	/*maximum transfer unit size */
	mtu = propGetUInt32(PROP_COMM_MTU, 1500);
	if (mtu < 1000)
		pv->SEND_BUF_SIZE = 640; 
	else if (mtu < 1500)
		pv->SEND_BUF_SIZE = 1024; 
	else if (mtu < 2000)
		pv->SEND_BUF_SIZE = 1536; 
	else if (mtu < 3000)
		pv->SEND_BUF_SIZE = 2048; 
	else if (mtu < 4000)
		pv->SEND_BUF_SIZE = 3072; 
	else
		pv->SEND_BUF_SIZE = 4096; 
	if (pv->SEND_BUF_SIZE <= 1024)
		Buffer_safety_block = 80;
	else
		Buffer_safety_block = 128; 
	pv->xFtns->Init(pv->SEND_BUF_SIZE);
	// thread support
#ifdef PROTOCOL_THREAD
	pv->protoRunThread = utFalse; // see 'pv->protocolThread'
	threadMutexInit(&(pv->protocolMutex));
	threadConditionInit(&(pv->protocolCond));
#endif
	// pending/volatile queues
	if (pv->isPrimary) {  // queue init
		// primary
		pqueInitQueue(&(pv->pendingQueue) , PRIMARY_PENDING_QUEUE_SIZE);
		// uses standard event queue
	} else {
		// secondary
		pqueInitQueue(&(pv->pendingQueue) , SECONDARY_PENDING_QUEUE_SIZE);
#if defined(SECONDARY_SERIAL_TRANSPORT)
		/pqueInitQueue(&(pv->volatileQueue), SECONDARY_VOLATILE_QUEUE_SIZE);
		pqueInitQueue(&secondaryQueue     , SECONDARY_EVENT_QUEUE_SIZE);
#endif
	}
	// encoding
	if (pv->isPrimary)
		// primary
		pv->sessionEncoding = DEFAULT_ENCODING;
	else
	if (pv->xFtns->media == MEDIA_FILE)
		// secondary: file
		pv->sessionEncoding = DEFAULT_FILE_ENCODING;
	else
	if (pv->xFtns->media == MEDIA_SOCKET)
		// secondary: socket
		pv->sessionEncoding = DEFAULT_SOCKET_ENCODING;
	else
	if (pv->xFtns->media == MEDIA_SERIAL)
		// secondary: serial (currently the only secondary used)
		pv->sessionEncoding = DEFAULT_SERIAL_ENCODING;
	else
	if (pv->xFtns->media == MEDIA_GPRS)
		// secondary: GPRS
		pv->sessionEncoding = DEFAULT_GPRS_ENCODING;
	else 
		// secondary: default
	pv->sessionEncoding = ENCODING_BASE64;

	// duplex connect error timer
 /*   pv->lastDuplexErrorTimer = 0L; */
	
	// Accumulation of read/write byte counts.
	// Notes:
	// - 'transport.c' would be a better place to put this responsibility, but it's been
	// moved here to reduce the burden on the customization of the 'transport.c' module.
	// - This module currently only counts the data bytes actually read from and written
	// to the server.  It does not take into account any TCP/UDP packet overhead.  
	// To make the actual byte counts more accurate, the transport medium service provider 
	// (ie. the wireless data plan) must be consulted to determine how they charge for
	// data transfer.

	//Allocate read buffer. All server packets are saved in read buffer in the 
	//middle of transmission. The packets are processed when transmission ends.
	if ((pv->sendBuf = malloc(pv->SEND_BUF_SIZE)) == NULL) {
		perror("Buffer Allocation");
		logCRITICAL(LOGSRC,"Out Of Memory!!!");
	}
	if ((pv->readBuf = malloc(PROTOCOL_READ_BUF_SIZE + PACKET_MAX_ENCODED_LENGTH)) == NULL) {
		perror("Buffer Allocation");
		logCRITICAL(LOGSRC,"Out Of Memory!!!");
	}
	pv->extBuf = pv->readBuf + PROTOCOL_READ_BUF_SIZE;
	/*setup timer */
	sev1.sigev_notify = SIGEV_SIGNAL;
	sev1.sigev_signo = SIG_TIMEOUT;
	sev1.sigev_value.sival_int = TIMER_PROTOCOL_1;
	err = timer_create(CLOCK_REALTIME, &sev1, &timer1);
	it1.it_value.tv_sec = 0;
	it1.it_value.tv_nsec = 0;
	it1.it_interval.tv_sec = 0;
	it1.it_interval.tv_nsec = 0;
	sev7.sigev_notify = SIGEV_SIGNAL;
	sev7.sigev_signo = SIG_TIMEOUT;
	sev7.sigev_value.sival_int = TIMER_POWER_SAVING;
	err = timer_create(CLOCK_REALTIME, &sev7, &timer7);
	it7.it_value.tv_sec = 0;
	it7.it_value.tv_nsec = 0;
	it7.it_interval.tv_sec = 0;
	it7.it_interval.tv_nsec = 0;
	if (clock_source & CLOCK_SYNC_INTERNET) {
		clock_synchronized_with_server = false;
		clock_need_adjust = true;
	}
	pv->power_saving = propGetUInt32(PROP_COMM_POWER_SAVING, 0L)? true : false;
	protocol_init_timer(pv);
}

/* protocol module initialization */
// most be called once, and only once per protocol
void protocolInitialize(int protoNdx)
{
	/* clear structure */
	memset(&protoVars[protoNdx], 0, sizeof(ProtocolVars_t));
	// primary protocol?
	if (protoNdx == 0)
		protoVars[protoNdx].isPrimary = utTrue;
	/* start thread */
#ifdef PROTOCOL_THREAD
	_protocolStartThread(protoNdx);
#endif

}

#if !defined(PROTOCOL_THREAD)
/* indicate transport request */
void protocolTransport(int protoNdx, PacketEncoding_t encoding)
{
    ProtocolVars_t *pv = _protoGetVars(LOGSRC,protoNdx);
    TransportType_t xportType = TRANSPORT_NONE;
    PROTOCOL_LOCK(pv) {
        if (pv->currentTransportType == TRANSPORT_NONE) {
            xportType = _getTransportType(pv);
            if (xportType != TRANSPORT_NONE) {
                pv->currentEncoding      = encoding;
                pv->currentTransportType = xportType;
                PROTOCOL_NOTIFY(pv)
            }
        }
    } PROTOCOL_UNLOCK(pv)
    if (xportType != TRANSPORT_NONE) {
        // run protocol now, if not in a separate thread
        _protocolRunSession(pv);
    }
}
#endif
// ----------------------------------------------------------------------------
/* indicate transport request */
void protocolStartSession(void)
{
	ProtocolVars_t *pv = _protoGetVars(LOGSRC, 0);
	PROTOCOL_LOCK(pv)
	PROTOCOL_NOTIFY(pv)
	PROTOCOL_UNLOCK(pv)
}
// ----------------------------------------------------------------------------
/* reset URL parameters*/
void protocolScheduleResetURL(void)
{
	url_reset = true;
}
// ----------------------------------------------------------------------------
void update_timestamp(long tv_diff)
{
	PacketQueue_t *eventQueue = protocolGetEventQueue(0);

	if (tv_diff > DAY_SECONDS(1)) {
		pqueUpdateTimestamp(eventQueue, tv_diff);
		pqueUpdateTimestamp(&protoVars[0].pendingQueue, tv_diff);
	} else {
		pqueTuneTimestamp(eventQueue, tv_diff);
		pqueTuneTimestamp(&protoVars[0].pendingQueue, tv_diff);
	}
}
// ----------------------------------------------------------------------------
bool watchdog_protocol_monitor(void)
{
	if (++protocol_down_count > protocol_reboot_timer) {
		print_critical("Lost wireless link for too long, reboot\n");
		debuglog(LOG_INFO,"Lost wireless link for too long");
		diagnostic_report(DIAGNOSTIC_CLIENT_REBOOT, REBOOT_DOWN_TOO_LONG, NULL);
		return true;
	}
	else 
		return false;
}
