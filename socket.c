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
//  Socket based transport support..
// Notes:
//  - This implementation supports UDP/TCP communications with a DMTP server
//  using the standard library 'socket' functions.  The embedded platform on
//  which this module is install may require other initialization methods for 
//  setting up the connection with the DMTP server.
//  - 
// ---
// Change History:
//  2006/01/04  Martin D. Flynn
//     -Initial release
//  2007/01/28  Martin D. Flynn
//     -WindowsCE port
// ----------------------------------------------------------------------------

#include "defaults.h"

// ----------------------------------------------------------------------------

#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <time.h>
#include <signal.h>

#include "log.h"

#include "socket.h"

#include "stdtypes.h"
#include "strtools.h"
#include "bintools.h"
#include "utctools.h"
#include "threads.h"  // for 'threadSleepMS'
#include "comport.h"
#include "sockets.h"
#include "rfid.h"
#include "props.h"
#include "propman.h"
#include "packet.h"

// ----------------------------------------------------------------------------
int udp_recv_timeout;
/* transport structure */
typedef struct {
	TransportType_t	type;
	utBool			isOpen;
	Socket_t		sock;
	char			name[16];
} SocketTransport_t;

static SocketTransport_t	Socket_TCP = {.name = "TCP Socket"};
static SocketTransport_t	Socket_UDP = {.name = "UDP Socket"};
static unsigned char *send_buf;
static unsigned int send_len;
static unsigned int send_att;
static struct sigevent sev2;
static struct itimerspec it2;
static timer_t timer2;
static size_t Send_BUF_SIZE;
static unsigned int Send_TIMEOUT;
static unsigned int Retry_TIMES;

// ----------------------------------------------------------------------------

// The size of the datagram buffer is arbitrary, however the amount of 
// data transmitted in a single datagram should not be larger than the MTU
// to avoid possible fragmentation which could result in a higher possibility
// of data loss.
/*static UInt8                    sockDatagramData[64]; */

// ----------------------------------------------------------------------------

/* flush input buffer */
static void socket_ReadFlush()
{
    // NO-OP
}

/* return true if transport is open */
static utBool socketTCP_IsOpen()
{
    // this may be called by threads other than the protocol thread.
    return Socket_TCP.isOpen;
}

/* close transport */
static utBool socketTCP_Close(void)
{
	utBool ret = utTrue;
	if (Socket_TCP.isOpen) {
        /* close socket */
        socketCloseClient(&(Socket_TCP.sock));
        /* transport is closed */
        Socket_TCP.type   = TRANSPORT_NONE;
        Socket_TCP.isOpen = utFalse;
        logDEBUG(LOGSRC,"%s closed ......", Socket_TCP.name);
    }
	return ret;
}

/* open transport */
static utBool socketTCP_Open(void)
{
	int err = 0;
	/* close, if already open */
	if (Socket_TCP.isOpen) {
		// it shouldn't already be open
		logINFO(LOGSRC,"Transport seems to still be open!");
		return Socket_TCP.isOpen;
	}
	/* open */
	socketTCP_funcs.transport_error = 0;
	err = socketOpenTCPClient(&(Socket_TCP.sock));
	if (err != COMERR_SUCCESS) {
		socketTCP_funcs.transport_error = err;
		return utFalse;
	}
	else {
		Socket_TCP.isOpen = utTrue;
		return utTrue;
	}
}

// ----------------------------------------------------------------------------
/* read packet from transport */
static int socketTCP_ReadPacket(UInt8 *buf, int bufLen)
{
	int readLen;
    
	socketTCP_funcs.transport_error = 0;
	readLen = socketRead(&Socket_TCP.sock, buf, bufLen);
	if (readLen < 0) {
		socketTCP_funcs.transport_error = readLen;
	}
	return readLen;
}

// ----------------------------------------------------------------------------

/* write packet to transport */
static int socketTCP_WritePacket(const UInt8 *buf, int bufLen)
{
    
	int len = 0;
    /* transport not open? */
	if (!Socket_TCP.isOpen) {
		logERROR(LOGSRC,"Transport is not open");
		socketTCP_funcs.transport_error = -1;
		return -1;
	}
    /* write data per transport type */
	socketTCP_funcs.transport_error = 0;
	len = socketWrite(&(Socket_TCP.sock), buf, bufLen);
	if (len < 0)
		socketTCP_funcs.transport_error = len;
	return len;
}

// ----------------------------------------------------------------------------
void socketTCP_Reset(int url_id)
{
	const char *host =  NULL; 
	int port = 0;
	/* get host:port */
	if (url_id == 0) {
		host = propGetString(PROP_COMM_HOST, "");
		port = (int)propGetUInt32(PROP_COMM_PORT, 0L);
	}
	else {
		host = propGetString(PROP_COMM_HOST_B, "");
		port = (int)propGetUInt32(PROP_COMM_PORT_B, 0L);
	}
	if (host == NULL || strlen(host) < 3 || port <= 0) {
        // If this is true, the client will NEVER connect.
		logCRITICAL(LOGSRC,"Transport host/port not specified ...\n");
		return;
	}
	socketInitStruct(&Socket_TCP.sock, host, port, 0);
}

/* initialize transport */
void socketTCP_Initialize(size_t buf_size)
{
    /* init transport structure */
	Socket_TCP.type    = TRANSPORT_NONE;
	Socket_TCP.isOpen  = utFalse;
	/* init datagram buffer */
	socketTCP_Reset(0);
    /* other initialization */
}
// ----------------------------------------------------------------------------
TransportFtns_t socketTCP_funcs = 
{	MEDIA_SOCKET, 
	0,
	socketTCP_Initialize,
	socketTCP_IsOpen,
	socketTCP_Open,
	socketTCP_Close,
	socketTCP_ReadPacket,
	socket_ReadFlush,
	socketTCP_WritePacket, 
	socketTCP_Reset 
};

/* return true if transport is open */
static utBool socketUDP_IsOpen()
{
    // this may be called by threads other than the protocol thread.
    return Socket_UDP.isOpen;
}

/* close transport */
static utBool socketUDP_Close(void)
{
	utBool ret = utTrue;
	if (Socket_UDP.isOpen) {
        /* close socket */
        socketCloseClient(&Socket_UDP.sock);
        /* transport is closed */
        Socket_UDP.type   = TRANSPORT_NONE;
        Socket_UDP.isOpen = utFalse;
        logDEBUG(LOGSRC,"%s closed ......", Socket_UDP.name);
    }
	return ret;
}

/* open transport */
static utBool socketUDP_Open(void)
{
	int err = 0;
	/* close, if already open */
	if (Socket_UDP.isOpen) {
		// it shouldn't already be open
		logINFO(LOGSRC,"Transport seems to still be open!");
		return Socket_UDP.isOpen;
	}
	/* open */
	socketUDP_funcs.transport_error = 0;
	err = socketOpenUDPClient(&(Socket_UDP.sock));
	if (err != COMERR_SUCCESS) {
		socketUDP_funcs.transport_error = err;
		return utFalse;
	}
	else {
		Send_TIMEOUT = propGetUInt32AtIndex(PROP_COMM_UDP_TIMER, 0, 20);
		Retry_TIMES = propGetUInt32AtIndex(PROP_COMM_UDP_TIMER, 1, 3);
		send_att = 0;
		send_len = 0;
		Socket_UDP.isOpen = utTrue;
		return utTrue;
	}
}

// ----------------------------------------------------------------------------
/* read packet from transport */
static int socketUDP_ReadPacket(UInt8 *buf, int bufLen)
{
	int readLen = 0;
	int len = 0;
	int n;

	socketUDP_funcs.transport_error = 0;
	for (n = 0; n < Retry_TIMES && readLen == 0; n++) {
		it2.it_value.tv_sec = Send_TIMEOUT;
		timer_settime(timer2, 0, &it2, NULL);
		udp_recv_timeout = 0;
		readLen = socketRead(&Socket_UDP.sock, buf, bufLen);
		if (readLen == COMERR_SOCKET_TIMEOUT &&  udp_recv_timeout < 0) {
			if (n == Retry_TIMES - 1)
				break;
			send_buf[0] = (unsigned char)(++send_att);
			len = socketWrite(&Socket_UDP.sock, send_buf, send_len);
			if (len < 0) {
				socketUDP_funcs.transport_error = len;
				break;
			}
			else
				readLen = 0;
		} 
		else if (readLen > 0 && ((unsigned int)buf[0] > send_att))
			readLen = 0;
	}
	it2.it_value.tv_sec = 0;
	it2.it_value.tv_nsec = 0;
	timer_settime(timer2, 0, &it2, NULL);
	if (readLen < 0)
		socketUDP_funcs.transport_error = readLen;
	else if (readLen == 0)
		socketUDP_funcs.transport_error = COMERR_SOCKET_FILENO;
	
	return readLen;
}

// ----------------------------------------------------------------------------

/* write packet to transport */
static int socketUDP_WritePacket(const UInt8 *buf, int bufLen)
{
    
	int len = 0;
    /* transport not open? */
	if (!Socket_UDP.isOpen) {
		logERROR(LOGSRC,"Transport is not open");
		socketUDP_funcs.transport_error = -1;
		return -1;
	}
	send_len = (bufLen >= Send_BUF_SIZE)? Send_BUF_SIZE : bufLen + 1;
	memcpy(send_buf + 1, buf, send_len - 1);
	send_att = 0;
	send_buf[0] = 0;
    /* write data per transport type */
	socketUDP_funcs.transport_error = 0;
	len = socketWrite(&Socket_UDP.sock, send_buf, send_len);
	if (len < 0)
		socketUDP_funcs.transport_error = len;
	return len;
}

// ----------------------------------------------------------------------------
void socketUDP_Reset(int url_id)
{
	const char *host =  NULL; 
	int port = 0;
	/* get host:port */
	if (url_id == 0) {
		host = propGetString(PROP_COMM_HOST, "");
		port = (int)propGetUInt32(PROP_COMM_PORT, 0L);
	}
	else {
		host = propGetString(PROP_COMM_HOST_B, "");
		port = (int)propGetUInt32(PROP_COMM_PORT_B, 0L);
	}
	if (host == NULL || strlen(host) < 3 || port <= 0) {
        // If this is true, the client will NEVER connect.
		logCRITICAL(LOGSRC,"Transport host/port not specified ...\n");
		return;
	}
	socketInitStruct(&Socket_UDP.sock, host, port, 0);
}
/* initialize transport */
void socketUDP_Initialize(size_t buf_size)
{
	/* init datagram buffer */
	Send_BUF_SIZE = buf_size; 
	if ((send_buf = malloc(buf_size)) == NULL) {
		logCRITICAL(LOGSRC,"OUT OF MEMORY\n");
		return;
	}
	send_att = 0;
	send_len = 0;
	memset(send_buf, 0, Send_BUF_SIZE);
	/*setup timer */
	sev2.sigev_notify = SIGEV_SIGNAL;
	sev2.sigev_signo = SIG_TIMEOUT;
	sev2.sigev_value.sival_int = TIMER_PROTOCOL_2;
	timer_create(CLOCK_REALTIME, &sev2, &timer2);
	it2.it_value.tv_sec = 0;
	it2.it_value.tv_nsec = 0;
	it2.it_interval.tv_sec = 0;
	it2.it_interval.tv_nsec = 0;
    /* init transport structure */
	Socket_UDP.type    = TRANSPORT_NONE;
	Socket_UDP.isOpen  = utFalse;
	socketUDP_Reset(0);
    /* other initialization */
}
TransportFtns_t socketUDP_funcs = 
{	MEDIA_SOCKET, 
	0,
	socketUDP_Initialize,
	socketUDP_IsOpen,
	socketUDP_Open,
	socketUDP_Close,
	socketUDP_ReadPacket,
	socket_ReadFlush,
	socketUDP_WritePacket, 
	socketUDP_Reset 
};
