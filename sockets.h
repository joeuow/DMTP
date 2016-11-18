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

#ifndef _SOCKET_H
#define _SOCKET_H
#ifdef __cplusplus
extern "C" {
#endif

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include "stdtypes.h"

// ----------------------------------------------------------------------------

// uncomment to enable server-side sockets (UDP/TCP)
// This implementation accepts only a single client at a time
//#define ENABLE_SERVER_SOCKET

// ----------------------------------------------------------------------------

typedef struct {
	struct addrinfo host_ai;
	struct sockaddr_in host_ad;
	int sock_err;
#if defined(ENABLE_SERVER_SOCKET)
	int             serverfd;   // '-1' for client side sockets
#endif
	int             sockfd;
	utBool          nonBlock;
	int             avail;
	int             port;
	utBool          hostInit;
	char            host[MAX_ID_SIZE];
	UInt8           hostAddr[8];
} Socket_t;

// ----------------------------------------------------------------------------
// Internal socket error codes

#define COMERR_SUCCESS		0       // success
#define COMERR_SOCKET_FILENO	-301	// socket invalid fileno
#define COMERR_SOCKET_OPEN	-311	// socket open error
#define COMERR_SOCKET_OPTION	-312	// socket unable to set option
#define COMERR_SOCKET_HOST	-321	// socket invalid host
#define COMERR_SERVER		-399	// Server setup error
#define COMERR_SOCKET_BIND	-401	// socket unable to bind address
#define COMERR_SOCKET_CONNECT	-423	// socket unable to connect
#define COMERR_SOCKET_ACCEPT	-424	// socket unable to accept client
#define COMERR_SOCKET_READ	-431	// socket read error
#define COMERR_SOCKET_WRITE	-432	// socket write error
#define COMERR_SOCKET_TIMEOUT	-433	// socket read/write timeout

// ----------------------------------------------------------------------------

void socketInitStruct(Socket_t *sock, const char *host, int port, int type);

int socketOpenUDPClient(Socket_t *sock);
int socketOpenTCPClient(Socket_t *sock);
utBool socketIsOpenClient(Socket_t *sock);

int socketCloseClient(Socket_t *sock);

#ifndef TARGET_LINUX
utBool socketEnableNonBlockingClient(Socket_t *sock, utBool enabled);
utBool socketIsNonBlockingClient(Socket_t *sock);
#endif

int socketRead(Socket_t *sock, UInt8 *buf, int bufSize);
int socketWrite(Socket_t *sock, const UInt8 *buf, int bufLen);

// ----------------------------------------------------------------------------

#if defined(ENABLE_SERVER_SOCKET)
int socketOpenUDPServer(Socket_t *sock, int port);
int socketOpenTCPServer(Socket_t *sock, int port);
utBool socketIsOpenServer(Socket_t *sock);
int socketAcceptTCPClient(Socket_t *sock);
int socketCloseServer(Socket_t *sock);
#endif

// ----------------------------------------------------------------------------

#ifdef __cplusplus
}
#endif
#endif
