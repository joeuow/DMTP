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
//  Example UDP/TCP socket utilities for data transport use.
// ---
// Change History:
//  2006/01/04  Martin D. Flynn
//     -Initial release
//  2006/01/12  Martin D. Flynn
//     -Fixed read timeout problem on Linux.
//  2007/01/28  Martin D. Flynn
//     -WindowsCE port
//     -Enabled non-blocking mode on tcp sockets (TARGET_WINCE only)
//     -Added 'send' select check (see 'socketIsSendReady')
//     -Fixed premature timeout problem in socketReadTCP
//  2007/02/05  Martin D. Flynn
//     -Fixed size of 'heBuf' in call to 'gethostbyname_r'.  Was 256, which was
//      too small for uClibc. (Thanks to Tomasz Rostanski for catching this!).
// ----------------------------------------------------------------------------

#define SKIP_TRANSPORT_MEDIA_CHECK // only if TRANSPORT_MEDIA not used in this file 
#include "defaults.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <ctype.h>
#include <time.h>
#include <string.h>
#include <limits.h>
#include <unistd.h>
#include <errno.h>
#include <termios.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <arpa/inet.h>
#include "stdtypes.h"
#include "strtools.h"
#include "utctools.h"
#include "sockets.h"

#include "log.h"

// ----------------------------------------------------------------------------

#define ALWAYS_RESOLVE_HOST     utFalse

// ----------------------------------------------------------------------------

// Overhead:
//  TCP => 20(IP) + 20{TCP) = 40
//  UDP => 20(IP) +  8(UDP) = 28
// Minimum IP datagram size = 576 bytes (include the above overhead)
// Maximum IP datagram size = 65516
//
// Protocol overhead: http://sd.wareonearth.com/~phil/net/overhead/

// ----------------------------------------------------------------------------
// This works fine on Linux and GumStix
#  include <asm/ioctls.h>
#  define IOCTIL_REQUEST_BYTES_AVAIL    FIONREAD    // (int*)
//#  define IOCTIL_REQUEST_NON_BLOCKING FIONBIO     // (int*)
#  define CLOSE_SOCKET(F)               close(F)
#  define IOCTL_SOCKET(F,R,A)           ioctl(F,R,A)
#  define IOCTL_ARG_TYPE                int
#  define INVALID_SOCKET                (-1)
#  define SOCKET_ERROR                  (-1)

// ----------------------------------------------------------------------------

// Windows CE 'errno' redefinitions
#  define ERRNO                         errno
#  define RESET_ERRNO                   /*NO-OP*/ // errno = 0

// ----------------------------------------------------------------------------

/* client/server: clear socket structure */
void socketInitStruct(Socket_t *sock, const char *host, int port, int type)
{
	if (sock) {
		memset(sock, 0, sizeof(Socket_t));
#if defined(ENABLE_SERVER_SOCKET)
		sock->serverfd = INVALID_SOCKET;
#endif
		sock->sockfd = INVALID_SOCKET;
		if (host) {
			snprintf(sock->host, MAX_ID_SIZE, "%s", host);
		}
		sock->port = port;
		sock->hostInit = utFalse;
		sock->sock_err = 0;
	}
}

// ----------------------------------------------------------------------------
#ifndef TARGET_LINUX
static int socketResolveHost(const char *host, UInt8 *addr, utBool alwaysResolve)
{
    // Note: 'addr' is assumed to be 6 bytes in length

    /* invalid addr */
    if (!addr) {
        return COMERR_SOCKET_HOST;
    }

    /* check for already resolved */
    if (!alwaysResolve) {
        // don't resolve if address has already been resolved
        // [NOTE: may have no effect if the socket structure is initialized to nulls on each open]
        int i;
        for (i = 0; i < 6; i++) {
            if (addr[i]) {
                return COMERR_SUCCESS;
            }
        }
    }

    /* invalid host name */
    if (!host || !*host) {
        return COMERR_SOCKET_HOST;
    }
    
    /* get host entry */
    struct hostent *he = (struct hostent*)0;
    // thread safe
    struct hostent rhe;
    char heBuf[512]; // was 256 - (too small for uClibc)
    // Exact size would be '460':
    //    sizeof(struct in_addr) +
    //    sizeof(struct in_addr*)*2 + 
    //    sizeof(char*)*(ALIAS_DIM) +  // (ALIAS_DIM is (2 + 5/*MAX_ALIASES*/ + 1))
    //    384/*namebuffer*/ + 
    //    32/*margin*/;
    int heErrno = 0;
    gethostbyname_r(host, &rhe, heBuf, sizeof(heBuf), &he, &heErrno);

    /* extract address */
    if (he) {
        int len = (he->h_length < 6)? he->h_length : 6;
        memcpy(addr, he->h_addr_list[0], len);
        return COMERR_SUCCESS;
    } else {
        logWARNING(LOGSRC,"Unable to resolve host [%d]: %s", heErrno, host);
        return COMERR_SOCKET_HOST;
    }
    
}

// ----------------------------------------------------------------------------
/* enable non-blocking mode */
utBool socketEnableNonBlockingClient(Socket_t *sock, utBool enable)
{
#if defined(IOCTIL_REQUEST_NON_BLOCKING)
    if (sock && (sock->sockfd != INVALID_SOCKET)) {
        IOCTL_ARG_TYPE enableNonBlocking = enable? 1 : 0;
        RESET_ERRNO; // WSASetLastError(0);
        int status = IOCTL_SOCKET(sock->sockfd, IOCTIL_REQUEST_NON_BLOCKING, &enableNonBlocking);
        if (status >= 0) {
            sock->nonBlock = enable;
            //logINFO(LOGSRC,"Socket set to non-blocking mode");
            return utTrue;
        } else {
            int err = ERRNO; // WSAGetLastError();
            logERROR(LOGSRC,"Unable to enable non-blocking mode [errno=%d]", err);
            return utFalse;
        }
    }
#endif
    return utFalse;
}

/* return true if non-blocking mode has been set for this client socket */
utBool socketIsNonBlockingClient(Socket_t *sock)
{
    if (sock && (sock->sockfd != INVALID_SOCKET)) {
        return sock->nonBlock;
    } else {
        return utFalse;
    }
}
#endif //defined TARGET_LINUX
// ----------------------------------------------------------------------------
/* client: open a UDP socket for writing */
int socketOpenUDPClient(Socket_t *sock)
{
	int err;
	char connect_display[48];
	char err_message[96];
	/* resolve socket address */
	if (!sock->hostInit) {
		struct addrinfo hints, *aip;
		char s_port[8];
		snprintf(s_port, sizeof(s_port), "%d", sock->port); 
		memset(&hints, 0, sizeof hints);
		hints.ai_family = AF_INET;
		hints.ai_socktype = SOCK_DGRAM;
		err = getaddrinfo(sock->host, s_port, &hints, &aip);
		if (err != 0) {
			sock->sock_err = err;
			logERROR(LOGSRC,"Resolving %s: %s", sock->host, gai_strerror(err));
			return COMERR_SOCKET_BIND;
		}
		sock->host_ai = *aip;
		memcpy(&sock->host_ad, aip->ai_addr, aip->ai_addrlen);
		sock->hostInit = utTrue;
		freeaddrinfo(aip);
	}
	sock->sock_err = 0;
	/*open socket */
	sock->sockfd = socket(sock->host_ai.ai_family, sock->host_ai.ai_socktype, sock->host_ai.ai_protocol);
	if (sock->sockfd == -1) {
        // unlikely to occur
		sock->sock_err = errno;
		sock->sockfd = INVALID_SOCKET;
		return COMERR_SOCKET_OPEN;
	}
	/*connect*/
	if (connect(sock->sockfd, (struct sockaddr *)(&sock->host_ad), sock->host_ai.ai_addrlen) < 0) {
		sock->sock_err = errno;
		strerror_r(errno, err_message, sizeof(err_message));
		if (sock->sock_err == EINTR)
			err = COMERR_SOCKET_TIMEOUT;
		else
			err = COMERR_SOCKET_CONNECT;
		logERROR(LOGSRC,"Connecting to server: %s", err_message);
		CLOSE_SOCKET(sock->sockfd);
		return err;
	}
	memset(connect_display, 0, 48);
	inet_ntop(sock->host_ai.ai_family, &sock->host_ad.sin_addr, 
							connect_display, sizeof(connect_display));
	logINFO(LOGSRC, "Connected to %s:%d", connect_display, sock->port);
    /* non-blocking mode */
#if defined(TARGET_WINCE)
    // set non-blocking AFTER connection
    socketEnableNonBlockingClient(sock, utTrue);
#endif
    /* success */
    return COMERR_SUCCESS;
}

/* server: open a UDP socket for reading */
#if defined(ENABLE_SERVER_SOCKET)
int socketOpenUDPServer(Socket_t *sock, int port)
{

    /* create socket */
    socketInitStruct(sock, (char*)0, port, SOCK_DGRAM);
    sock->sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock->sockfd < 0) {
        // unlikely to occur
        return COMERR_SOCKET_OPEN;
    }
        
    /* bind to local port (RX only) */
    struct sockaddr_in my_addr;             // connector's address information
    my_addr.sin_family = AF_INET;           // host byte order
    my_addr.sin_port = htons(sock->port);   // short, network byte order
    my_addr.sin_addr.s_addr = INADDR_ANY;
    if (bind(sock->sockfd, (struct sockaddr *)&my_addr, sizeof(my_addr)) == -1) {
        // Unable to bind server to specified port
        //fprintf(stderr, "Unable to bind to port %d\n", port);
        CLOSE_SOCKET(sock->sockfd);
        sock->sockfd = INVALID_SOCKET;
        return COMERR_SOCKET_BIND;
    }

    return COMERR_SUCCESS;

}
#endif

// ----------------------------------------------------------------------------
/* client: open a TCP client socket */
int socketOpenTCPClient(Socket_t *sock)
{
	int err;
	char connect_display[48];
	char err_message[96];
	/* resolve socket address */
	if (!sock->hostInit) {
		struct addrinfo hints, *aip;
		char s_port[6];
		snprintf(s_port, 6, "%d", sock->port); 
		memset(&hints, 0, sizeof hints);
		hints.ai_family = AF_INET;
		hints.ai_socktype = SOCK_STREAM;
		err = getaddrinfo(sock->host, s_port, &hints, &aip);
		if (err != 0) {
			sock->sock_err = err;
			logERROR(LOGSRC,"Resolving %s: %s", sock->host, gai_strerror(err));
        		return COMERR_SOCKET_BIND;
		}
		sock->host_ai = *aip;
		memcpy(&sock->host_ad, aip->ai_addr, aip->ai_addrlen);
		sock->hostInit = utTrue;
		freeaddrinfo(aip);
	}
	sock->sock_err = 0;
	/*open socket */
	sock->sockfd = socket(sock->host_ai.ai_family, 
		sock->host_ai.ai_socktype, sock->host_ai.ai_protocol); 
	if (sock->sockfd == INVALID_SOCKET) {
        // unlikely to occur
		sock->sock_err = errno;
		return COMERR_SOCKET_OPEN;
	}
	/*connect*/
	if (connect(sock->sockfd, (struct sockaddr *)&sock->host_ad, sock->host_ai.ai_addrlen) < 0) {
		sock->sock_err = errno;
		strerror_r(errno, err_message, sizeof(err_message));
		memset(connect_display, 0, 48);
		inet_ntop(sock->host_ai.ai_family, &sock->host_ad.sin_addr, 
							connect_display, sizeof(connect_display));
		if (sock->sock_err == ECONNREFUSED) {
			err = COMERR_SOCKET_HOST;
			logERROR(LOGSRC,"Connecting to %s - %s, server has shut down", connect_display, err_message);
		} else {
			if (sock->sock_err == EINTR)
				err = COMERR_SOCKET_TIMEOUT;
			else 
				err = COMERR_SOCKET_CONNECT;
			logERROR(LOGSRC,"Connecting to %s - %s", connect_display, err_message);
		}
		CLOSE_SOCKET(sock->sockfd);
		return err;
	}
	memset(connect_display, 0, 48);
	inet_ntop(sock->host_ai.ai_family, &sock->host_ad.sin_addr, 
					connect_display, sizeof(connect_display));
	logINFO(LOGSRC, "Connected to %s:%d", connect_display, sock->port);
    /* non-blocking mode */
#if defined(TARGET_WINCE)
    // set non-blocking AFTER connection
    socketEnableNonBlockingClient(sock, utTrue);
#endif
    /* success */
    return COMERR_SUCCESS;
}

/* server: is client socket open */
utBool socketIsOpenClient(Socket_t *sock)
{
    return (sock && (sock->sockfd != INVALID_SOCKET))? utTrue : utFalse;
}

// ----------------------------------------------------------------------------

#if defined(ENABLE_SERVER_SOCKET)

/* server: open a TCP server socket */
int socketOpenTCPServer(Socket_t *sock, int port)
{

    /* init */
    socketInitStruct(sock, 0, port, SOCK_STREAM);

    /* socket */
    sock->serverfd = socket(AF_INET, SOCK_STREAM, 0); // SOCK_DGRAM
    if (sock->serverfd == INVALID_SOCKET) {
        return COMERR_SOCKET_OPEN;
    }
    
    /* reuse address */
    int yes = 1;
    if (setsockopt(sock->serverfd, SOL_SOCKET, SO_REUSEADDR, (char*)&yes, sizeof(int)) == -1) {
        // Unable to set server socket options
        CLOSE_SOCKET(sock->serverfd);
        sock->serverfd = INVALID_SOCKET;
        return COMERR_SOCKET_OPTION;
    }
    
    /* send timeout? (SO_SNDTIMEO) */

    /* linger on close */
    struct linger so_linger;
    so_linger.l_onoff  = 1; // linger on
    so_linger.l_linger = 2; // 2 seconds
    if (setsockopt(sock->serverfd, SOL_SOCKET, SO_LINGER, (char*)&so_linger, sizeof(struct linger)) == -1) {
        // Unable to set server socket options
        CLOSE_SOCKET(sock->serverfd);
        sock->serverfd = INVALID_SOCKET;
        return COMERR_SOCKET_OPTION;
    }

    /* bind to port */
    struct sockaddr_in my_addr;             // my address information
    my_addr.sin_family = AF_INET;           // host byte order
    my_addr.sin_port = htons(sock->port);   // short, network byte order
    my_addr.sin_addr.s_addr = INADDR_ANY;   // auto-fill with my IP
    memset(&my_addr.sin_zero, 0, 8); // zero the rest of the struct
    if (bind(sock->serverfd, (struct sockaddr *)&my_addr, sizeof(struct sockaddr)) == -1) {
        // Unable to bind server to specified port
        CLOSE_SOCKET(sock->serverfd);
        sock->serverfd = INVALID_SOCKET;
        return COMERR_SOCKET_BIND;
    }
    
    /* set listen */
    if (listen(sock->serverfd, 1) == -1) {
        // Unable to listen on specified port
        CLOSE_SOCKET(sock->serverfd);
        sock->serverfd = INVALID_SOCKET;
        return COMERR_SOCKET_OPEN;
    }
    
    return 0;

}

/* server: is server socket open */
utBool socketIsOpenServer(Socket_t *sock)
{
    return (sock && (sock->serverfd != INVALID_SOCKET))? utTrue : utFalse;
}

/* server: accept an incoming client */
int socketAcceptTCPClient(Socket_t *sock)
{
    if (sock && (sock->serverfd != INVALID_SOCKET)) {
        struct sockaddr_in clientAddr;
        socklen_t alen = sizeof(clientAddr);
        sock->sockfd = accept(sock->serverfd, (struct sockaddr *)&clientAddr, &alen);
        if (sock->sockfd == INVALID_SOCKET) {
            // client accept failed
            return COMERR_SOCKET_ACCEPT;
        }
        return 0;
    } else {
        return COMERR_SOCKET_FILENO;
    }
}

#endif

// ----------------------------------------------------------------------------

/* client: close */
int socketCloseClient(Socket_t *sock)
{
    if (sock->sockfd != INVALID_SOCKET) {
        // SO_LINGER?
        CLOSE_SOCKET(sock->sockfd);
        sock->sockfd = INVALID_SOCKET;
    }
    return 0;
}

#if defined(ENABLE_SERVER_SOCKET)
/* server: close */
int socketCloseServer(Socket_t *sock)
{
    socketCloseClient(sock);
    if (sock && (sock->serverfd != INVALID_SOCKET)) {
        CLOSE_SOCKET(sock->serverfd);
        sock->serverfd = INVALID_SOCKET;
    }
    return 0;
}
#endif

// ----------------------------------------------------------------------------

#if defined(ENABLE_SERVER_SOCKET)
/* return true if any data is available for reading */
static utBool socketIsDataAvailable(Socket_t *sock, long timeoutMS)
{
    if (sock && (sock->sockfd != INVALID_SOCKET)) {
        fd_set rfds;
        struct timeval tv;
        FD_ZERO(&rfds);
        FD_SET(sock->sockfd, &rfds);
        tv.tv_sec  = timeoutMS / 1000L;
        tv.tv_usec = (timeoutMS % 1000L) * 1000L;
        RESET_ERRNO; // WSASetLastError(0);
        select(sock->sockfd + 1, &rfds, 0, 0, &tv);
        if (FD_ISSET(sock->sockfd, &rfds)) {
            return utTrue;
        }
    }
    return utFalse;
}

/* return number of bytes available for reading */
static int socketGetAvailableCount(Socket_t *sock)
{
    if (sock && (sock->sockfd != INVALID_SOCKET)) {
        IOCTL_ARG_TYPE nBytesAvail = 0;
        int status = IOCTL_SOCKET(sock->sockfd, IOCTIL_REQUEST_BYTES_AVAIL, &nBytesAvail);
        int availBytes = (status >= 0)? (int)nBytesAvail : 0;
        return availBytes;
    } else {
        return -1;
    }
}
#endif

/* client/server: read TCP */
/*return values: 
 * 0: no data availabe now
 * negative: error occurred, or connection torn down 
 * positive: good data is read*/
int socketRead(Socket_t *sock, UInt8 *buf, int bufSize)
{
	int cnt, ret = -1;
	char err_message[96];
	cnt = recv(sock->sockfd, buf, bufSize, 0);
	if (cnt < 0) {
		strerror_r(errno, err_message, sizeof(err_message));
		sock->sock_err = errno;
		if (errno < 0) {
			if (sock->sock_err == EAGAIN || sock->sock_err == EWOULDBLOCK) {
				ret = 0;
			}
			else if (sock->sock_err == EINTR) { 
				logERROR(LOGSRC,"Socket Receiving:%s (timed out)", err_message);
				ret = COMERR_SOCKET_TIMEOUT;
			}
			else if (sock->sock_err == ECONNREFUSED) {
				logINFO(LOGSRC,"Socket Receiving:%s: server has shutdown", err_message);
				ret = COMERR_SOCKET_HOST;
			}
			else if (sock->sock_err == ECONNRESET) {
				logINFO(LOGSRC,"Socket Receiving:%s", err_message);
			}
			else {
				logERROR(LOGSRC,"Socket Receiving: %s", err_message);
				ret = COMERR_SOCKET_READ;
			}
		}
		if (errno == 0)
			ret = COMERR_SOCKET_READ;
			
	}
	else 
		ret = cnt;
	return ret;
}
// ----------------------------------------------------------------------------
//
#ifdef SOCKET_MAIN
/* return true if any data is available for reading */
static utBool socketIsSendReady(Socket_t *sock, long timeoutMS)
{
    if (sock && (sock->sockfd != INVALID_SOCKET)) {
        fd_set wfds;
        struct timeval tv;
        FD_ZERO(&wfds);
        FD_SET(sock->sockfd, &wfds);
        tv.tv_sec  = timeoutMS / 1000L;
        tv.tv_usec = (timeoutMS % 1000L) * 1000L;
        RESET_ERRNO; // WSASetLastError(0);
        select(sock->sockfd + 1, 0, &wfds, 0, &tv); // 'tv' may be updated
        if (FD_ISSET(sock->sockfd, &wfds)) {
            return utTrue;
        }
    }
    return utFalse;
}
#endif

/* client/server: write TCP */
int socketWrite(Socket_t *sock, const UInt8 *buf, int bufLen)
{
	int cnt, ret, tot = 0;
    // On one occasion it appears that 'send(...)' has locked up when GPRS 
    // coverage became unavilable in the middle of the transmission.
    // Since 'SO_SNDTIMEO' is not sidely supported, socket set to non-blocking,
    // and 'select' is used to determine 'send'ability.
    if (sock->sockfd == INVALID_SOCKET)
		return COMERR_SOCKET_FILENO;

	while (tot < bufLen) {
		/* send data */
		cnt = send(sock->sockfd, (buf + tot), bufLen - tot, 0);
		if (cnt < 0) {
			sock->sock_err = errno;
			ret = -1;
			if (ERRNO == ECONNRESET) {
				logINFO(LOGSRC,"Connection reset by peer [errno=%d]", ERRNO);
			} 
			else if ((ERRNO == EWOULDBLOCK) || (ERRNO == EAGAIN)) {
				logINFO(LOGSRC,"'send' would block [errno=%d]", ERRNO);
			} 
			else {
				logERROR(LOGSRC,"Socket 'send' error [errno=%d]", ERRNO);
				ret = COMERR_SOCKET_WRITE;
			}
			return ret;
		}
		else  {
			tot += cnt;
		}
	}
	return tot;
}

#ifdef SOCKET_MAIN
/* client: write UDP */
int socketWriteUDP(Socket_t *sock, const UInt8 *buf, int bufLen)
{
    if (sock && (sock->sockfd != INVALID_SOCKET)) {

        /* check for 'send' ready */
        long selectTimeoutMS = 10000L;
        if (!socketIsSendReady(sock,selectTimeoutMS)) {
            int err = ERRNO;
            logERROR(LOGSRC,"Timeout waiting for 'sendto' [errno=%d]", err);
            return COMERR_SOCKET_TIMEOUT;
        }

        /* send datagram */
        struct sockaddr_in their_addr;
        their_addr.sin_family = AF_INET;
        their_addr.sin_port = htons(sock->port); // network byte order
        their_addr.sin_addr = *((struct in_addr *)sock->hostAddr);
        int cnt = sendto(sock->sockfd, (char*)buf, bufLen, 0, (struct sockaddr *)&their_addr, sizeof(their_addr));
        if (cnt < 0) { return COMERR_SOCKET_WRITE; }
        return bufLen;
        
    } else {
        
        // Invalid 'socketWriteUDP' fileno
        return COMERR_SOCKET_FILENO;
        
    }
}

// ----------------------------------------------------------------------------

//#define SOCKET_MAIN

int main(int argc, char *argv[])
{
    Socket_t sock;
    int port = 15152;
    int i, x;
    for (i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "-txu")) {
            printf("UDP TX ...\n");
            char msg[64];
            socketOpenUDP(&sock, "localhost", port);
            for (x = 0; x < 5000; x++) {
                sprintf(msg, "$ %d data\r", x);
                socketWriteUDP(&sock, msg, strlen(msg));
            }
            continue;
        }
        if (!strcmp(argv[i], "-txt")) {
            printf("TCP TX ...\n");
            char msg[64];
            socketOpenTCPClient(&sock, "localhost", port);
            for (x = 0; x < 5000; x++) {
                sprintf(msg, "$ %d data\r", x);
                socketWriteTCP(&sock, msg, strlen(msg));
            }
            continue;
        }
#if defined(ENABLE_SERVER_SOCKET)
        if (!strcmp(argv[i], "-rxu")) {
            printf("UDP RX ...\n");
            char data[600];
            socketOpenUDP(&sock, "localhost", port);
            for (x = 0; x < 5000; x++) {
                int dataLen = socketReadUDP(&sock, data, sizeof(data));
                if (dataLen <= 0) {
                    printf("Read error: %d\n", dataLen);
                } else
                if (*data == '$') {
                    data[dataLen] = 0;
                    char *s = data;
                    for (;*s;s++) {
                        printf("%c", *s);
                        if (*s == '\r') { printf("\n"); }
                    }
                    printf("\n");
                } else {
                    printf("UDP read: %d bytes\n", dataLen);
                }
            }
            continue;
        } 
        if (!strcmp(argv[i], "-rxt")) {
            printf("TCP RX ...\n");
            char data[600];
            socketOpenTCPServer(&sock, port);
            socketAcceptTCPClient(&sock);
            for (x = 0; x < 5000; x++) {
                int dataLen = 0;
                while (1) {
                    int len = socketReadTCP(&sock, &data[dataLen], 1, 3000L);
                    if (len <= 0) {
                        dataLen = len;
                        break;
                    }
                    if ((data[dataLen] == '\n') || (data[dataLen] == '\r')) {
                        data[dataLen] = 0;
                        break;
                    }
                    dataLen += len;
                }
                if (dataLen <= 0) {
                    printf("Read error: %d\n", dataLen);
                    break;
                }
                printf("TCP read[%d]: %s\n", dataLen, data);
            }
            continue;
        } 
#endif
    }
    return 0;
}
#endif
