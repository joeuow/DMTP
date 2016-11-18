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
//  Serial port handler.
//  Driver for handling serial port (RS232) communication.
// ---
// Change History:
//  2006/01/04  Martin D. Flynn
//     -Initial release
//  2006/01/16  Martin D. Flynn
//     -Changed client/server 'pipe' device names
//  2006/01/27  Martin D. Flynn
//     -Added support for 'ttyUSB#' named devices
//  2007/01/28  Martin D. Flynn
//     -Made some changes to the way that pipes are initialized to emulate
//      ComPort.  (handling of pipes appear to be different for different
//      versions of Linux).
//     -Specifying 'CPIPE'/'SPIPE' now properly defaults to 'CPIPE0'/'SPIPE0'.
//     -WindowsCE port
//  2007/02/01  Martin D. Flynn
//     -Code cleanup
//     -Removed 'COMPORT_SUPPORT_SOCKET' code (which wasn't used anyway)
//  2007/03/11  Martin D. Flynn
//     -Put back 'COMPORT_SUPPORT_SOCKET' code (somebody found it useful :-)
//     -Changed socket support port base to '32000' (see comport.h)
//     -Added support for reading data from NMEAD - port 1155 (special thanks to 
//      Tomasz Rostanski for the information on this). 
//      See "http://home.hiwaay.net/~taylorc/gps/nmea-server/" for details on
//      support for NMEAD ("NMEA Server").
// ----------------------------------------------------------------------------

#define SKIP_TRANSPORT_MEDIA_CHECK // only if TRANSPORT_MEDIA not used in this file 
#include "defaults.h"

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <ctype.h>

#  include <unistd.h>
#  include <fcntl.h>
#  include <errno.h>
#  include <sys/ioctl.h>
#  include <sys/poll.h>
#  include <sys/types.h>
#  include <sys/stat.h>
#  include <sys/select.h>

#include "log.h"

#include "stdtypes.h"
#include "utctools.h"
#include "strtools.h"
#include "comport.h"


#if defined(COMPORT_SUPPORT_SOCKET)
#  include <netdb.h>
#  include <netinet/in.h>
#  include <sys/socket.h>
#endif


#if defined(TARGET_CYGWIN)
// This works on Cygwin
#  define BYTES_AVAIL_IOCTIL_REQUEST  TIOCINQ     // Cygwin
#  define USE_COM_NAMES       // use "COM#" names on Cygwin
//#  define VTIME_SUPPORTED   <-- Cygwin does not properly support VTIME 

#elif defined(TARGET_GUMSTIX) || defined(TARGET_LINUX)
// This works fine on Linux and GumStix
#  include <asm/ioctls.h>
#  define BYTES_AVAIL_IOCTIL_REQUEST  FIONREAD    // Linux/GumStix (undefined on Cygwin)
//#  define USE_COM_NAMES    <--- use 'ttyS#' names on Linux
#  define VTIME_SUPPORTED

#else
#  define BYTES_AVAIL_IOCTIL_REQUEST  FIONREAD
//#  define VTIME_SUPPORTED
#endif

#define BYTES_AVAIL_IOCTL

// ----------------------------------------------------------------------------
// Comm simulation pipe files

#if defined(COMPORT_SUPPORT_PIPE)
/* ComPort pipe support */
// These files must exist before this feature can be used
#  define COM_PIPE_CLIENT_FILE        "/tmp/comPipe_C."
#  define COM_PIPE_SERVER_FILE        "/tmp/comPipe_S."
#endif

// ----------------------------------------------------------------------------
// local platform comport device file name
// Note: These names may be case-sensitive and must match the exact device name.

#define ComDev_COM                  "COM"       // COM# devices
#define ComDev_VCA                  "VCA"       // VCA# devices

#if defined(TARGET_GUMSTIX) || defined(TARGET_CYGWIN) || defined(TARGET_LINUX)
#  define ComDev_DIR                "/dev/"     // device directory
#  define ComDev_ttyS               "ttyS"      // serial devices
#  define ComDev_ttyUSB             "ttyUSB"    // USB devices
#endif

#if defined(TARGET_GUMSTIX) && defined(COMPORT_SUPPORT_BLUETOOTH)
#  define ComDev_rfcomm0            "rfcomm0"   // Bluetooth device 
#endif

//##################################TARGET_LINUX
#ifndef TARGET_LINUX
// ----------------------------------------------------------------------------

/* non-WinCE data formats */
#define PARITY_8N1              (0)
#define PARITY_7E1              (1)
#define PARITY_7O1              (2)
#define DEFAULT_READ_LEN 64

// ----------------------------------------------------------------------------

/* return Bluetooth device address/id */
#if defined(COMPORT_SUPPORT_BLUETOOTH) && defined(SUPPORT_UInt64)
UInt64 comPortGetBluetoothAddress()
{
    return 0L;
}
#endif

/* return true if this is a Bluetooth port */
// TODO: may not be fully supported on this platform
#if defined(COMPORT_SUPPORT_BLUETOOTH)
utBool comPortIsBluetooth(ComPort_t *com)
{
    return (com && PORT_IsBLUETOOTH(com->port))? utTrue : utFalse;
}
#endif

/* test for active/ready BT serial connection */
#if defined(COMPORT_SUPPORT_BLUETOOTH)
utBool comPortIsBluetoothReady(ComPort_t *com)
{
    
    /* comport open */
    if (!comPortIsOpen(com)) {
        // not open, not connected
        return utFalse; 
    }
    
    /* not bluetooth? */
    if (!PORT_IsBLUETOOTH(com->port)) {
        // assume true, if this port isn't bluetooth
        return utTrue;
        //return com->hwFlow? comPortGetCTS(com) : utTrue;
    }

    // TODO: this test for serial connectivity is platform dependent
    // default to 'connected'
    return utTrue;

}
#endif

// ----------------------------------------------------------------------------

/* return the port for the specified name */
static Int32 _comPortPortForName(const char *portName)
{

    /* initial filtering */
    int portNameLen = portName? strlen(portName) : 0;
    if (portNameLen <= 0) {
        return -1L;
    }

    /* parsing vars */
    const char *p = portName;
    const char *pnum = (char*)0;
    int dft = -1, base = 0;
    Int32 type = COMTYPE_STANDARD;

#if defined(COMPORT_SUPPORT_CONSOLE)
    // Console serial emulator
    if (strEqualsIgnoreCase(p, ComName_CONSOLE)) {
        pnum = p + strlen(ComName_CONSOLE);
        dft  = 0; // no port follows "console"
        base = 0;
        type = COMTYPE_CONSOLE;
    } else
#endif

#if defined(COMPORT_SUPPORT_PIPE)
    // Client Pipe serial emulator
    if (strStartsWithIgnoreCase(p, ComName_CPIPE)) { // cpipe#
        pnum = p + strlen(ComName_CPIPE);
        dft  = 0; // default to "cpipe0"
        base = 0;
        type = COMTYPE_CPIPE;
    } else
#endif

#if defined(COMPORT_SUPPORT_PIPE)
    // Server Pipe serial emulator
    if (strStartsWithIgnoreCase(p, ComName_SPIPE)) { // spipe#
        pnum = p + strlen(ComName_SPIPE);
        dft  = 0; // default to "spipe0"
        base = 0;
        type = COMTYPE_SPIPE;
    } else
#endif

#if defined(COMPORT_SUPPORT_SOCKET)
    // Client socket serial emulator
    if (strStartsWithIgnoreCase(p, ComName_CSOCK)) { // csock#
        pnum = p + strlen(ComName_CSOCK);
        dft  = 0; // default to "csock0"
        base = 0;
        type = COMTYPE_CSOCK;
    } else
    // Client socket serial emulator
    if (strStartsWithIgnoreCase(p, ComName_NMEAD)) { // nmead#
        pnum = p + strlen(ComName_NMEAD); // this value is stored, but ignored
        dft  = 0; // default to "nmead0"
        base = 0;
        type = COMTYPE_NMEAD;
    } else
    // Server socket serial emulator
    if (strStartsWithIgnoreCase(p, ComName_SSOCK)) { // ssock#
        pnum = p + strlen(ComName_SSOCK);
        dft  = 0; // default to "ssock0"
        base = 0;
        type = COMTYPE_SSOCK;
    } else
#endif

#if defined(TARGET_GUMSTIX)
    // Gumstix FFUART
    if (strEqualsIgnoreCase(p, ComName_FFUART)) {
        pnum = p + strlen(ComName_FFUART);
        dft  = 0; // FFUART == TTYS0
        base = 0;
        type = COMTYPE_STANDARD;
    } else
#endif

#if defined(TARGET_GUMSTIX)
    // Gumstix BTUART
    if (strEqualsIgnoreCase(p, ComName_BTUART)) {
        pnum = p + strlen(ComName_BTUART);
        dft  = 1; // BTUART == TTYS1
        base = 0;
        type = COMTYPE_STANDARD;
    } else
#endif

#if defined(TARGET_GUMSTIX)
    // Gumstix STUART
    if (strEqualsIgnoreCase(p, ComName_STUART)) {
        pnum = p + strlen(ComName_STUART);
        dft  = 2; // STUART == TTYS2
        base = 0;
        type = COMTYPE_STANDARD;
    } else
#endif

#if defined(TARGET_GUMSTIX)
    // Gumstix HWUART
    if (strEqualsIgnoreCase(p, ComName_HWUART)) {
        pnum = p + strlen(ComName_HWUART);
        dft  = 3; // HWUART == TTYS3
        base = 0;
        type = COMTYPE_STANDARD;
    } else
#endif

#if defined(COMPORT_SUPPORT_BLUETOOTH)
    // Gumstix Bluetooth
    if (strEqualsIgnoreCase(p, ComName_RFCOMM0)) {
        pnum = p + strlen(ComName_RFCOMM0);
#  if defined(TARGET_GUMSTIX)
        dft  = 3; // RFCOMM0 == TTYS3
#  else
        dft  = 0; // RFCOMM0 == platform dependent here
#  endif
        base = 0; // indexes start at '0' for "rfcomm0" devices
        type = COMTYPE_BLUETOOTH;
    } else
#endif

#if defined(COMPORT_SUPPORT_BLUETOOTH)
    // Bluetooth
    if (strStartsWithIgnoreCase(p,ComName_BTH)) {
        pnum = p + strlen(ComName_BTH);
        dft  = -1; // must be a valid #
        base = 1;  // indexes start at '1' for "BTH" devices
        type = COMTYPE_BLUETOOTH;
    } else
#endif

#if defined(COMPORT_SUPPORT_BLUETOOTH)
    // Bluetooth
    if (strStartsWithIgnoreCase(p,ComName_BT)) {
        pnum = p + strlen(ComName_BT);
        dft  = -1; // must be a valid #
        base = 1;  // indexes start at '1' for "BT" devices
        type = COMTYPE_BLUETOOTH;
    } else
#endif

    // COM serial port
    if (strStartsWithIgnoreCase(p, ComDev_COM)) {
        pnum = p + strlen(ComDev_COM);
        dft  = -1; // must be a valid #
        base = 1;  // indexes start at '1' for "COM" devices
        type = COMTYPE_STANDARD;
    } else 
    
#if defined(COMPORT_SUPPORT_USB)
    // USB serial port
    if (strStartsWithIgnoreCase(p,ComName_USB)) {
        pnum = p + strlen(ComName_USB);
        dft  = -1; // must be a valid #
        base = 1;  // indexes start at '1' for "USB" devices
        type = COMTYPE_USB;
    } else
#endif
    
    // 'ttyS' serial port
    if (strStartsWithIgnoreCase(p, ComName_TTYS)) {
        pnum = p + strlen(ComName_TTYS);
        dft  = -1; // must be a valid #
        base = 0;  // indexes start at '0' for "ttyS" devices
        type = COMTYPE_STANDARD;
    } else

#if defined(COMPORT_SUPPORT_USB)
    // 'ttyUSB' serial port
    if (strStartsWithIgnoreCase(p, ComName_TTYUSB)) {
        pnum = p + strlen(ComName_TTYUSB);
        dft  = -1; // must be a valid #
        base = 0;  // indexes start at '0' for "ttyUSB" devices
        type = COMTYPE_USB;
    } else
#endif

    // Error (name not recognized)
    {
        // name not found, exit now
        return -1L;
    }

    /* if found, extract index and return port */
    int port0 = (int)strParseInt32(pnum, dft) - base;
    if ((port0 >= 0) && (port0 <= MAX_COM_PORT)) {
        return (type | PORT_Index0(port0));
    } else {
        return -1L;
    }

}

/* return the name of the specified port */
static char tempPortName[MAX_COM_NAME_SIZE + 1];
static const char *_comPortNameForPort(char *name, Int32 port)
{
    if (!name) { name = tempPortName; } // not thread safe
    // 'name' must be at least (MAX_COM_NAME_SIZE + 1) bytes long
    *name = 0;
    
    // Standard serial port
    if (PORT_IsSTANDARD(port)) {
#if defined(USE_COM_NAMES)
        sprintf(name, "%s%d", ComName_COM, PORT_Index1(port));
#else
        sprintf(name, "%s%d", ComName_TTYS, PORT_Index0(port));
#endif
    } else 
    
#if defined(COMPORT_SUPPORT_USB)
    // USB serial port
    if (PORT_IsUSB(port)) {
#  if defined(USE_COM_NAMES)
        sprintf(name, "%s%d", ComName_USB, PORT_Index1(port));
#  else
        sprintf(name, "%s%d", ComName_TTYUSB, PORT_Index0(port));
#  endif
    } else
#endif
    
#if defined(COMPORT_SUPPORT_CONSOLE)
    // Console serial emulator
    if (PORT_IsCONSOLE(port)) {
        strcpy(name, ComName_CONSOLE);
    } else
#endif

#if defined(COMPORT_SUPPORT_PIPE)
    // Server Pipe serial emulator
    if (PORT_IsSPIPE(port)) {
        sprintf(name, "%s%d", ComName_SPIPE, PORT_Index0(port));
    } else
#endif

#if defined(COMPORT_SUPPORT_PIPE)
    // Client Pipe serial emulator
    if (PORT_IsCPIPE(port)) {
        sprintf(name, "%s%d", ComName_CPIPE, PORT_Index0(port));
    } else
#endif

#if defined(COMPORT_SUPPORT_SOCKET)
    // Client socket serial emulator
    if (PORT_IsCSOCK(port)) {
        sprintf(name, "%s%d", ComName_CSOCK, PORT_Index0(port));
    } else
    // Client socket serial emulator
    if (PORT_IsNMEAD(port)) {
        sprintf(name, "%s%d", ComName_NMEAD, PORT_Index0(port));
    } else
    // Server socket serial emulator
    if (PORT_IsSSOCK(port)) {
        sprintf(name, "%s%d", ComName_SSOCK, PORT_Index0(port));
    } else
#endif

#if defined(COMPORT_SUPPORT_BLUETOOTH)
    // Bluetooth serial port
    if (PORT_IsBLUETOOTH(port)) { 
#  if defined(TARGET_GUMSTIX)
        strcpy(name, ComName_RFCOMM0);
#  else
        sprintf(name, "%s%d", ComName_BT, PORT_Index1(port));
#  endif
    } else
#endif

    // Error (port not recognized)
    {
        strcpy(name, ComName_INVALID);
    }
    
    return name;
}

/* return true if specified name is valid */
utBool comPortIsValidName(const char *portName)
{
    return (_comPortPortForName(portName) > 0L)? utTrue : utFalse;
}

// ----------------------------------------------------------------------------

/* return the device name for the specified port */
static const char *_comPortDeviceForPort(char *device, Int32 port)
{
    if (device) {
        // 'device' must be at least (MAX_COM_DEV_SIZE + 1) bytes long
        *device = 0;
        
        // Standard serial ports
        if (PORT_IsSTANDARD(port)) {
            sprintf(device, "%s%s%d", ComDev_DIR, ComDev_ttyS, PORT_Index0(port));
        } else
        
#if defined(COMPORT_SUPPORT_USB)
        // USB serial ports
        if (PORT_IsUSB(port)) {
            sprintf(device, "%s%s%d", ComDev_DIR, ComDev_ttyUSB, PORT_Index0(port));
        } else
#endif

#if defined(COMPORT_SUPPORT_BLUETOOTH)
        // Bluetooth serial ports
        if (PORT_IsBLUETOOTH(port)) {
            sprintf(device, "%s%s%d", ComDev_DIR, ComDev_ttyS, PORT_Index0(port));
        } else
#endif

        // Error (port not recognized)
        {
            sprintf(device, "%s%s?", ComDev_DIR, ComDev_ttyS);
        }

    }
    return device;
}

// ----------------------------------------------------------------------------

/* log data to console */
static void _comPortDefaultLogger(const UInt8 *data, int dataLen)
{
    // this function is used for DEBUG purposes only
    int i;
    for (i = 0; i < dataLen; i++) {
        fprintf(stdout, "%c", data[i]);  // logging: ignore returned error code
        if (data[i] == '\n') {
            fprintf(stdout, "\r");       // logging: ignore returned error code
        } else
        if (data[i] == '\r') {
            fprintf(stdout, "\n");       // logging: ignore returned error code
        }
    }
    fflush(stdout);                      // logging: ignore returned error code
}

/* set external function data logger */
void comPortSetDebugLogger(ComPort_t *com, void (*logger)(const UInt8*, int))
{
    if (com) {
        com->logger = logger? logger : &_comPortDefaultLogger;
        comPortAddOptions(com, COMOPT_LOGDEBUG);
    }
}

// ----------------------------------------------------------------------------

/* initialize ComPort_t structure */
ComPort_t *comPortInitStruct(ComPort_t *com)
{
    if (com) {
        
        /* clear common */
        memset(com, 0, sizeof(ComPort_t));
        com->open     = utFalse;
        com->port     = -1L;
        com->push     = -1;
        com->error    = COMERR_NONE;
        
        com->read_fd  = -1;
        com->write_fd = -1;

    }
    return com;
}

// ----------------------------------------------------------------------------


// ----------------------------------------------------------------------------

/* open ComPort with specified speed */
// NOTE: port number indexes start at '0'
ComPort_t *comPortOpen(ComPort_t *com, const char *portName, long bps, const char *dataFmt, utBool binary, int maxLen)
{

    /* null structure? */
    if (!com) {
        // internal error
        logWARNING(LOGSRC,"ComPort structure is 'null'");
        return (ComPort_t*)0;
    }

    /* clear structure */
    comPortInitStruct(com);
    
    /* port */
    com->port = _comPortPortForName(portName);
    if (com->port < 0L) {
        // internal error
        logWARNING(LOGSRC,"Unrecognized port name: %s", portName);
        com->error = COMERR_INIT;
        strCopy(com->name, sizeof(com->name), portName, -1);
        return (ComPort_t*)0;
    }
    _comPortNameForPort(com->name, com->port);
    _comPortDeviceForPort(com->dev, com->port);
    /*logDEBUG(LOGSRC,"ComPort name=%s, port=0x%04lX, dev=%s", com->name, com->port, com->dev); */

    /* special ports */
#if defined(COMPORT_SUPPORT_CONSOLE)
    if (PORT_IsCONSOLE(com->port)) {
        com->read_fd    = fileno(stdin);
        com->write_fd   = fileno(stdout);
        com->open       = utTrue;
        comPortResetSTDIN();     // disable echo & buffering
        comPortResetSTDOUT();    // disable buffering
        return com;
    }
#endif
#if defined(COMPORT_SUPPORT_PIPE)
    if (PORT_IsSPIPE(com->port) || PORT_IsCPIPE(com->port)) {
        char pipeFile0[64], pipeFile1[64];
        if (PORT_IsSPIPE(com->port)) {
            int pipePort = PORT_Index0(com->port);
            sprintf(pipeFile0, "%s%d", COM_PIPE_CLIENT_FILE, pipePort);
            sprintf(pipeFile1, "%s%d", COM_PIPE_SERVER_FILE, pipePort);
        } else {
            int pipePort = PORT_Index0(com->port);
            sprintf(pipeFile1, "%s%d", COM_PIPE_CLIENT_FILE, pipePort);
            sprintf(pipeFile0, "%s%d", COM_PIPE_SERVER_FILE, pipePort);
        }
        /*int err0 =*/ mkfifo(pipeFile0, 0777);
        /*int err1 =*/ mkfifo(pipeFile1, 0777);
        // Note: Opening a named pipe may block indefinitely until the far end of the
        // pipe is also openned!  Use with caution!  (O_NONBLOCK)
        if (PORT_IsSPIPE(com->port)) {
            logDEBUG(LOGSRC,"Openning ComPort server pipe: W(%s) R(%s)", pipeFile1, pipeFile0);
            com->write_fd = open(pipeFile1, O_WRONLY, 0);
            com->read_fd  = open(pipeFile0, O_RDONLY, 0);
        } else {
            logDEBUG(LOGSRC,"Openning ComPort client pipe: W(%s) R(%s)", pipeFile1, pipeFile0);
            com->read_fd  = open(pipeFile0, O_RDONLY, 0);
            com->write_fd = open(pipeFile1, O_WRONLY, 0);
        }
        if ((com->read_fd >= 0) && (com->write_fd >= 0)) {
            logDEBUG(LOGSRC,"ComPort pipe openned: W(%s) R(%s)", pipeFile1, pipeFile0);
            com->open = utTrue;
            return com;
        } else {
            logWARNING(LOGSRC,"Unable to open pipe: W(%s) R(%s)", pipeFile1, pipeFile0);
            if (com->read_fd  >= 0) { close(com->read_fd ); com->read_fd  = -1; }
            if (com->write_fd >= 0) { close(com->write_fd); com->write_fd = -1; }
            com->error = COMERR_INIT;
            return (ComPort_t*)0;
        }
    }
#endif
#if defined(COMPORT_SUPPORT_SOCKET)
    if (PORT_IsCSOCK(com->port) || PORT_IsNMEAD(com->port)) {
        const char *sockHost = CSOCK_SOCKET_HOST;
        int sockPort = PORT_IsNMEAD(com->port)? NMEAD_SOCKET_PORT : (PORT_Index0(com->port) + CSOCK_SOCKET_PORT_BASE);
        logINFO(LOGSRC,"Openning client socket [%s:%d] ...", sockHost, sockPort);
        com->read_fd = socket(AF_INET, SOCK_STREAM, 0); // SOCK_DGRAM
        if (com->read_fd <= -1) {
            // unable to open socket
            logWARNING(LOGSRC,"Unable to open Comport socket");
            com->error = COMERR_INIT;
            return (ComPort_t*)0;
        }
        struct hostent *he = gethostbyname(sockHost);
        if (he == NULL) {  // get the host info
            // unable to resolve host
            logWARNING(LOGSRC,"Unable to resolve Comport socket host");
            close(com->read_fd);
            com->read_fd = -1;
            com->error = COMERR_INIT;
            return (ComPort_t*)0;
        }
        struct sockaddr_in their_addr;      // connector's address information
        their_addr.sin_family = AF_INET;    // host byte order
        their_addr.sin_port = htons(sockPort);  // short, network byte order
        their_addr.sin_addr = *((struct in_addr *)he->h_addr);
        memset(&(their_addr.sin_zero), 0, sizeof(their_addr.sin_zero));  // (8) zero the rest of the struct
        if (connect(com->read_fd, (struct sockaddr *)&their_addr, sizeof(struct sockaddr)) < 0) {
            // Unable to connect to specified host:port
            logINFO(LOGSRC,"Unable to open connection to %s:%d", sockHost, sockPort);
            close(com->read_fd);
            com->read_fd = -1;
            com->error = COMERR_INIT;
            return (ComPort_t*)0;
        }
        com->write_fd = com->read_fd;
        com->open = utTrue;
        return com;
    } else
    if (PORT_IsSSOCK(com->port)) {
        int yes = 1;
        struct sockaddr_in my_addr;    // my address information
        int sockPort = PORT_Index0(com->port) + CSOCK_SOCKET_PORT_BASE;
        logINFO(LOGSRC,"Openning server socket [%d] ...", sockPort);
        int server_fd = socket(AF_INET, SOCK_STREAM, 0); // SOCK_DGRAM
        if (server_fd <= -1) {
            // unable to open socket
            logWARNING(LOGSRC,"Unable to open Comport server socket");
            com->error = COMERR_INIT;
            return (ComPort_t*)0;
        }
        if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int)) == -1) {
            // Unable to set server socket options
            logWARNING(LOGSRC,"Unable to set Comport server socket options");
            close(server_fd);
            com->error = COMERR_INIT;
            return (ComPort_t*)0;
        }
        my_addr.sin_family = AF_INET;         // host byte order
        my_addr.sin_port = htons(sockPort);   // short, network byte order
        my_addr.sin_addr.s_addr = INADDR_ANY; // auto-fill with my IP
        memset(&my_addr.sin_zero, 0, 8); // zero the rest of the struct
        if (bind(server_fd, (struct sockaddr *)&my_addr, sizeof(struct sockaddr)) == -1) {
            // Unable to bind server to specified port
            logWARNING(LOGSRC,"Unable to bind Comport server socket port");
            close(server_fd);
            com->error = COMERR_INIT;
            return (ComPort_t*)0;
        }
        if (listen(server_fd, 1) == -1) {
            // Unable to listen on specified port
            logWARNING(LOGSRC,"Unable to listen on Comport server socket port");
            close(server_fd);
            com->error = COMERR_INIT;
            return (ComPort_t*)0;
        }
        struct sockaddr_in clientAddr;
        int alen = sizeof(clientAddr);
        com->read_fd = accept(server_fd, (struct sockaddr *)&clientAddr, &alen);
        if (com->read_fd <= -1) {
            // client accept failed
            logWARNING(LOGSRC,"Unable to accept on Comport server socket port");
            close(server_fd);
            com->error = COMERR_INIT;
            return (ComPort_t*)0;
        }
        com->write_fd = com->read_fd;
        com->open = utTrue;
        //close(server_fd);
        return com;
    }
#endif
    
    /* speed */
    com->bps = bps;
    switch (bps) {
        case   BPS_1200: com->speed =   B1200; break;
        case   BPS_2400: com->speed =   B2400; break;
        case   BPS_4800: com->speed =   B4800; break;
        case   BPS_9600: com->speed =   B9600; break;
        case  BPS_19200: com->speed =  B19200; break;
        case  BPS_38400: com->speed =  B38400; break;
        case  BPS_57600: com->speed =  B57600; break;
#if defined(B76800)
        case  BPS_76800: com->speed =  B76800; break; // undefined on Cygwin
#endif
        case BPS_115200: com->speed = B115200; break;
#if defined(B921600)
        case BPS_921600: com->speed = B921600; break; // BlueTooth 'default' speed
#endif
        default:
            // this would be an internal error
            logWARNING(LOGSRC,"Unsupport BPS: %s %ld\n", com->dev, com->bps);
            com->error = COMERR_SPEED;
            return (ComPort_t*)0;
    }

    /* parse "dataFmt" */
    int parity = PARITY_8N1;
    if (!dataFmt || !*dataFmt || strEqualsIgnoreCase(dataFmt,DTAFMT_8N1)) {
        // 8 data bits, no parity, 1 stop bit (default)
        parity = PARITY_8N1;
    } else
    if (strEqualsIgnoreCase(dataFmt,DTAFMT_8O1)) {
        // 8 data bits, odd parity, 1 stop bit (non-standard)
        // not yet supported
        logWARNING(LOGSRC,"Format not yet supported: %s", dataFmt);
    } else
    if (strEqualsIgnoreCase(dataFmt,DTAFMT_8E1)) {
        // 8 data bits, odd parity, 1 stop bit (non-standard)
        // not yet supported
        logWARNING(LOGSRC,"Format not yet supported: %s", dataFmt);
    } else
    if (strEqualsIgnoreCase(dataFmt,DTAFMT_7N2)) {
        // 7 data bits, no parity, 2 stop bits
        // not yet supported
        logWARNING(LOGSRC,"Format not yet supported: %s", dataFmt);
    } else
    if (strEqualsIgnoreCase(dataFmt,DTAFMT_7E1)) {
        // 7 data bits, even parity, 1 stop bits
        parity = PARITY_7E1;
    } else
    if (strEqualsIgnoreCase(dataFmt,DTAFMT_7O1)) {
        // 7 data bits, even parity, 1 stop bits
        parity = PARITY_7O1;
    } else {
        logWARNING(LOGSRC,"Unrecognized data format: %s", dataFmt);
    }

    /* open */
    //logDEBUG(LOGSRC,"Openning ComPort port=0x%04lX, dev=%s", com->port, com->dev);
    com->read_fd = open(com->dev, O_RDWR | O_NOCTTY);
    if (com->read_fd < 0) {
        // device is currently unavailable (or doesn't exist)
        logWARNING(LOGSRC,"Unable to open ComPort: %s [%s]", com->dev, strerror(errno));
        com->error = COMERR_INIT;
        return (ComPort_t*)0;
    }
    com->write_fd = com->read_fd; // comport read/write file descriptor is the same
    /*logDEBUG(LOGSRC,"Openned ComPort port=0x%04lX, dev=%s", com->port, com->dev); */

    /* set up blocking/non-blocking read */
/*    fcntl(com->read_fd, F_SETFL, 0);             // blocking reads */
    //fcntl(com->read_fd, F_SETFL, FNDELAY);     // non blocking reads
    
    /* ComPort configuration */
    struct termios options;
    tcgetattr(com->read_fd, &options);
	com->termios_old = options;
    
    /* BPS */
    cfsetispeed(&options, com->speed);
    cfsetospeed(&options, com->speed);
    
    /* control options 'c_cflag' */
    if (binary)
    	com->hwFlow = SERIAL_PORT_HW_FLOW;
    else 
    	com->hwFlow = utFalse;
    options.c_cflag |= CREAD;           // enable receiver
    if (com->hwFlow) {
        options.c_cflag |= CRTSCTS;     // enable hardware flow control
    } else {
        options.c_cflag &= ~CRTSCTS;    // disable hardware flow control
    }
    if (parity == PARITY_7E1) {
        // PARITY_7E1
        options.c_cflag |= PARENB;      // enable parity
        options.c_cflag &= ~PARODD;     // even parity
        options.c_cflag &= ~CSTOPB;     // 1 stop bit
        options.c_cflag &= ~CSIZE;      // [char size mask]
        options.c_cflag |= CS7;         // 7 bit
    } else
    if (parity == PARITY_7O1) {
        // PARITY_7O1
        options.c_cflag |= PARENB;      // enable parity
        options.c_cflag |= PARODD;      // odd parity
        options.c_cflag &= ~CSTOPB;     // 1 stop bit
        options.c_cflag &= ~CSIZE;      // [char size mask]
        options.c_cflag |= CS7;         // 7 bit
    } else {
        // PARITY_8N1
        options.c_cflag &= ~PARENB;     // disable parity
        options.c_cflag &= ~CSTOPB;     // 1 stop bit
        options.c_cflag &= ~CSIZE;      // [char size mask]
        options.c_cflag |= CS8;         // 8 bit
    }
    options.c_cflag |= HUPCL;           // drop DTR when port is closed
  //options.c_cflag &= LOBLK;           // don't block job ctl output (undefined on Cygwin)
    
    /* input options 'c_iflag' */
	if (binary) {
		/* input: ignore break, no software flow */
		options.c_iflag &= ~(IGNBRK | BRKINT | IXON | IXOFF | IXANY);
		/*input: no translation of anything */
		options.c_iflag &= ~(PARMRK | INLCR | IGNCR | ICRNL | IUCLC);
		/* output: no translation of anything, no non-standard processing */
		options.c_oflag &= ~(OPOST | ONLCR | OLCUC | OCRNL | ONLRET | ONOCR | OFILL | OFDEL);
		options.c_lflag &= ~(ICANON);		// non-canonical mode 
		options.c_lflag &= ~(ECHO | ISIG | IEXTEN); // no echo
		if (options.c_cflag & PARENB) {
			options.c_iflag |= INPCK;       // enable parity checking
			options.c_iflag |= ISTRIP;      // strip parity bit
		} else {
			options.c_iflag &= ~INPCK;      // DISable parity checking
			options.c_iflag &= ~ISTRIP;     // don't strip parity bit
		}
		options.c_cc[VTIME] = 1;
		options.c_cc[VMIN] = maxLen;
	}
	else {
		options.c_iflag |= IGNCR; /*ignore CR: so CRLF goes as one line*/
		options.c_oflag &= ~(ONLCR | OLCUC | OCRNL | ONLRET | ONOCR);
		options.c_lflag |= (ISIG | ICANON); //canonical mode
		options.c_lflag &= ~(ECHO | ECHOE | ECHOK | ECHONL | ECHOKE | IEXTEN);
	}
    /* set ComPort options */
    // TCSANOW   - changes occur immediately
    // TCSADRAIN - changes occur after output is transmitted
    // TCSAFLUSH - changes occur after output is transmitted, input discarded
    if (tcsetattr(com->write_fd, TCSANOW, &options) != 0) {
        logWARNING(LOGSRC,"ComPort init failed: %s [%s]", com->dev, strerror(errno));
        // continue
    }
    // From the 'tcsetattr' man page:
    // Note that tcsetattr() returns success if any of the requested changes could be
    // successfully carried out.  Therefore, when making multiple changes it may be 
    // necessary to follow this call with a further call to tcgetattr() to check that 
    // all changes have been performed successfully.
    
    /* other default options */
#if defined(VTIME_SUPPORTED)
    // VTIME doesn't work on Cygwin
    comPortAddOptions(com, COMOPT_VTIMEOUT);
#endif
    /* return structure */
    com->open = utTrue;
    return com;

}

ComPort_t *comPortReopen(ComPort_t *com)
{
    if (!com) {
        
        logWARNING(LOGSRC,"ComPort structure is 'null'");
        return (ComPort_t*)0;
        
    } else
    if (comPortIsOpen(com)) {
        
        /* close */
        //logDEBUG(LOGSRC,"Closing port %d", com->name);
        if (com->read_fd != fileno(stdin)) {
            // don't close stdin
            close(com->read_fd);
            if ((com->write_fd >= 0) && (com->read_fd != com->write_fd) && (com->write_fd != fileno(stdout))) {
                close(com->write_fd);
            }
            com->read_fd  = -1;
            com->write_fd = -1;
        }
        
        /* re-open */
        logDEBUG(LOGSRC,"Reopenning port %d", com->name);
        ComPort_t newCom;
        if (comPortOpen(&newCom, com->name, com->bps, DTAFMT_8N1, utTrue, DEFAULT_READ_LEN)) {
            newCom.flags  = com->flags;
            newCom.logger = com->logger;
            memcpy(com, &newCom, sizeof(newCom));
            return com;
        }
        
    }
    
    /* error */
    com->error = COMERR_GENERAL;
    return (ComPort_t*)0;
    
}

/* return true if port is open */
utBool comPortIsOpen(ComPort_t *com)
{
    return (com && com->open)? utTrue : utFalse;
}

// ----------------------------------------------------------------------------

/* return name of specified port */
const char *comPortName(ComPort_t *com)
{
    if (comPortIsOpen(com)) {
        if (!(*com->name)) {
            // init port name
            _comPortNameForPort(com->name, com->port);
        }
        return com->name;
    } else {
        return ComName_INVALID;
    }
}

// ----------------------------------------------------------------------------

static utBool _comPortSetVtimeMS(ComPort_t *com, long timeoutMS)
{
    if (comPortIsOpen(com)) {
        struct termios options;
        tcgetattr(com->read_fd, &options);
        if (timeoutMS <= 0L) {
            options.c_cc[VTIME] = 0L;                   // block forever
        } else {
            long deciSec = (timeoutMS + 99L) / 100L;    // millisec => decisec (round up)
            if (deciSec <= 0L) {
                options.c_cc[VTIME] = 1;                // don't let it be zero
            } else
            if (deciSec <= 255L) {
                options.c_cc[VTIME] = deciSec;          // range 1 <= VTIME <= 255
            } else {
                // internal error
                logWARNING(LOGSRC,"Warning: VTIME cannot be > 255 [%ld]", deciSec);
                options.c_cc[VTIME] = 255;              // max 255
            }
        }
        com->lastVtimeMS = (long)options.c_cc[VTIME] * 100L; // deciSec => milliSec
        if (tcsetattr(com->read_fd, TCSANOW, &options) == 0) {
            // success
            errno = 0;
            return utTrue;
        } else {
            // error (error in 'errno')
            return utFalse;
        }
    } else {
        errno = EBADF;
        return utFalse;
    }
}

static long _comPortGetVtimeMS(ComPort_t *com)
{
    if (comPortIsOpen(com)) {
        return com->lastVtimeMS;
    } else {
        return -1L;
    }
}

// ----------------------------------------------------------------------------

static int _comPortGetFD(ComPort_t *com)
{
    if (comPortIsOpen(com)) {
        return com->read_fd;
    } else {
        errno = ENODEV;
        return -1;
    }
}

static void _comPortSetControl(ComPort_t *com, int ctlMask, utBool state)
{
    int fd = _comPortGetFD(com);
    if (fd >= 0) {
        int status;
        ioctl(fd, TIOCMGET, &status);
        if (state) {
            status |= ctlMask;
        } else {
            status &= ~ctlMask;
        }
        ioctl(fd, TIOCMSET, &status);
    }
}

static utBool _comPortGetControl(ComPort_t *com, int ctlMask)
{
    int fd = _comPortGetFD(com);
    if (fd >= 0) {
        int status;
        ioctl(fd, TIOCMGET, &status);
        return (status & ctlMask)? utTrue : utFalse;
    }
    return utFalse;
}

// ----------------------------------------------------------------------------
void comPortSetDTR(ComPort_t *com, utBool state)
{
    _comPortSetControl(com, TIOCM_DTR, state);
}

utBool comPortGetDTR(ComPort_t *com)
{
    return _comPortGetControl(com, TIOCM_DTR);
}

// ----------------------------------------------------------------------------
void comPortSetRTS(ComPort_t *com, utBool state)
{
    _comPortSetControl(com, TIOCM_RTS, state);
}

utBool comPortGetRTS(ComPort_t *com)
{
    return _comPortGetControl(com, TIOCM_RTS);
}

// ----------------------------------------------------------------------------
void comPortSetCTS(ComPort_t *com, utBool state)
{
    _comPortSetControl(com, TIOCM_CTS, state);
}

utBool comPortGetCTS(ComPort_t *com)
{
    return _comPortGetControl(com, TIOCM_CTS);
}

// ----------------------------------------------------------------------------
void comPortSetDCD(ComPort_t *com, utBool state)
{
    _comPortSetControl(com, TIOCM_CD, state);
}

utBool comPortGetDCD(ComPort_t *com)
{
    return _comPortGetControl(com, TIOCM_CD);
}

// ----------------------------------------------------------------------------
/* set options for the specified ComPort */
void comPortSetOptions(ComPort_t *com, UInt16 flags)
{
    if (com != (ComPort_t*)0) {
        com->flags = flags;
    }
}

/* add options to the specified ComPort */
void comPortAddOptions(ComPort_t *com, UInt16 flags)
{
    if (com != (ComPort_t*)0) {
        com->flags |= flags;
    }
}

/* add options to the specified ComPort */
void comPortRemoveOptions(ComPort_t *com, UInt16 flags)
{
    if (com != (ComPort_t*)0) {
        com->flags &= ~flags;
    }
}

/* return true if the specified ComPort has backspace mode set */
UInt16 comPortGetOptions(ComPort_t *com)
{
    return com->flags;
}

// ----------------------------------------------------------------------------

/* push character back into comport read buffer */
void comPortPush(ComPort_t *com, UInt8 ch)
{
    if (com != (ComPort_t*)0) {
        // ComPort_t currently only support a 1 character cache.
        if (com->push > 0) {
            logWARNING(LOGSRC,"Char already pushed for port '%s'", com->name);
        }
        com->push = ch;
    }
}

// ----------------------------------------------------------------------------
/* return number of bytes available for reading on specified ComPort */
int comPortGetAvail(ComPort_t *com, long timeoutMS)
{
    // Do not check 'com->push' here!
    // This routine is for checking to see if we have data available on the 
    // physical com port.

    /* no timeout? */
    if (timeoutMS <= 0L) {
        // if we don't care about a timeout, always return that we
        // have at least 1 byte available
        return 1;
    }
    
    /* wait until data is available */
    fd_set rfds;
    struct timeval tv;
    FD_ZERO(&rfds);
    FD_SET(com->read_fd, &rfds);
    tv.tv_sec  = timeoutMS / 1000L;
    tv.tv_usec = (timeoutMS % 1000L) * 1000L;
    select(com->read_fd + 1, &rfds, 0, 0, &tv);
    if (!FD_ISSET(com->read_fd, &rfds)) { return 0; } // timeout

    /* return number of available bytes */
    // since the above did not timeout, we will have at least 1 byte available.
#ifdef BYTES_AVAIL_IOCTL
    int nBytesAvail;
    int request = BYTES_AVAIL_IOCTIL_REQUEST;
    int status = ioctl(com->read_fd, request, &nBytesAvail);
    return (status >= 0)? nBytesAvail : 1;
#else
    return 1;
#endif
            
}

// ----------------------------------------------------------------------------
/* read specified number of bytes from ComPort */
#define MAX_VTIME_MS   25500L
int comPortRead(ComPort_t *com, UInt8 *buf, int len, long timeoutMS)
{
    // Possible (non-exclusive) 'errno' values:
    //   EIO    [ 5]  input/output error
    //   ENODEV [19]  no such device
    //   ETIME  [63]  timeout
    
    /* read nothing? */
    if (len <= 0) {
        logINFO(LOGSRC,"Empty read buffer");
        return 0;
    } 
    
    /* valid com port? */
    if (!comPortIsOpen(com)) {
        if (buf) { buf[0] = 0; }
        errno = ENODEV;
        logINFO(LOGSRC,"ComPort not open");
        return -1;
    }

    /* starting timestamp */
    struct timeval startTime;
    utcGetTimestamp(&startTime);

    /* read bytes */
    int n = 0;
    for (;n < len;) {
        
        /* timeout? */
        long tms = timeoutMS;
        if (tms >= 0L) {
            tms -= utcGetDeltaMillis(0, &startTime); // subtract elapsed time
            if ((tms <= 0L) && (com->push <= 0) && (com->avail <= 0)) { // timeout
                if (buf) { buf[n] = 0; }
                errno = ETIME; // not necessary, since we're returning a value >= 0
                com->error = COMERR_TIMEOUT;
                return n;
            }
        }

        /* poll/read */
        if (com->push > 0) {
            
            /* use previously pushed char */
            com->last = (UInt8)com->push;
            com->push = -1; // clear push
            if (buf) { buf[n] = com->last; }
            n++;
            
        } else {
            
            /* poll (check for available data) */
            //logDEBUG(LOGSRC,FTN, "avail=%d, tms=%ld", com->avail, tms);
            if ((com->avail <= 0) && (tms > 0L)) {
                if (COMOPT_IsVTimeout(com)) {
                    // Note: 'tms' will be rounded up to the next highest 100ms, and
                    // will be clipped at 25500 ms (max value for VTIME).
                    if (!_comPortSetVtimeMS(com, tms)) {
                        // ENODEV/EIO is likely here (for 'rfcomm0')
                        int sv_errno = errno;
                        logWARNING(LOGSRC,"ComPort error %s : [%d] %s", com->dev, sv_errno, strerror(sv_errno));
                        if (buf) { buf[n] = 0; }
                        errno = sv_errno;
                        return -1;
                    }
                } else {
                    // wait for available data
                    int avail = comPortGetAvail(com, tms);
                    if (avail <= 0) {
                        // timeout/error
                        if (buf) { buf[n] = 0; }
                        errno = ETIME;
                        com->error = COMERR_TIMEOUT; // may be EOF
                        return n;
                    }
                    com->avail = avail;
                }
            } else {
                if (COMOPT_IsVTimeout(com)) {
                    // Either data is available for reading (without blocking), or the
                    // caller wishes this function to block forever (ie. no timeout)
                    // However, set a minimum timeout anyway to allow us to keep track
                    // of when the port is closed (externally). We'll still not return
                    // until the number of requested bytes have been read.
                    if (!_comPortSetVtimeMS(com, MAX_VTIME_MS)) {
                        // ENODEV/EIO is likely here (for 'rfcomm0')
                        int sv_errno = errno;
                        logWARNING(LOGSRC,"ComPort error %s : [%d] %s", com->dev, sv_errno, strerror(sv_errno));
                        if (buf) { buf[n] = 0; }
                        errno = sv_errno;
                        return -1;
                    }
                } else {
                    //logDEBUG(LOGSRC,FTN, "blocking wait (avail = %d)", com->avail);
                }
            }
            
            /* read 1 byte */
            // Either ...
            // - VTIME is in effect and we'll possibly be timing out, or
            // - a character available (avail > 0) and it will be returned, or
            // - we will be blocking here until a character becomes available, or
            // - we will get an error.
            struct timeval readTS;
            utcGetTimestamp(&readTS);
            errno = 0; // clear 'errno'
            int r = read(com->read_fd, &com->last, 1); // VTIME may be in effect
            if (r < 0) {
                // error: com port closed?
                if (buf) { buf[n] = 0; } // terminate buffer
                com->avail = 0;
                com->error = COMERR_EOF;
                // error already in 'errno'
                logWARNING(LOGSRC,"ComPort error %s : [%d] %s", com->dev, com->error, strerror(com->error));
                return -1;
            } else
            if (r == 0) {
                // A read length of 0 can mean an EOF, or a TIMEOUT if VTIME is in effect.
                //int readErrno = errno;
                //logDEBUG(LOGSRC,"Read EOF/TIMEOUT? %s: [%d]%s", com->dev, readErrno, strerror(readErrno));
                com->avail = 0; // in any case nothing is available
                if (COMOPT_IsVTimeout(com)) {
                    long vtimeMS = _comPortGetVtimeMS(com);
                    long deltaMS = utcGetDeltaMillis(0, &readTS) + 50L; // allow 50ms window
                    // Allow a margin on the timeout window when checking for EOF.
                    // If the elapsed time is less than what the timeout was set for, then
                    // an EOF is likely, but lets be sure of it by setting a wide enough
                    // time window.  The worst case is that the EOF occurred close enough
                    // to the expired timeout that we may try to read of a closed file
                    // descriptor again.  Hopefully, however, the next time around we will
                    // again get an EOF error right away and return accordingly.
                    if (deltaMS < vtimeMS) {
                        // this means that we were interrupted before timeout occured, EOF likely.
                        //logDEBUG(LOGSRC,"VTIME (%ld<%ld) ... [%s]", deltaMS, vtimeMS, com->dev);
                        if (buf) { buf[n] = 0; }
                        com->error = COMERR_EOF;
                        errno = ENODEV;
                        return -1;
                    } else {
                        // assume timeout, continue loop
                    }
                } else {
                    // assume an EOF
                    if (buf) { buf[n] = 0; }
                    com->error = COMERR_EOF;
                    errno = ENODEV;
                    logWARNING(LOGSRC,"ComPort error %s (EOF?) : [%d] %s", com->dev, com->error, strerror(com->error));
                    return -1;
                }
            } else
            if (r == 1) {
                if (buf) { buf[n] = com->last; }
                n++;
                if (com->avail > 0) { com->avail--; } // decrement available
                if (COMOPT_IsEcho(com)) {
                    // perform the following iff ECHO is on
                    if (com->last == KEY_RETURN) {
                        // follow CR with LF
                        comPortWrite(com, (UInt8 *)"\r\n", 2);
                    } else
                    if (KEY_IsBackspace(com->last) && COMOPT_IsBackspace(com)) {
                        // single character backspace
                        comPortWrite(com, (UInt8 *)"\b \b", 3);
                    } else
                    if ((com->last == KEY_CONTROL_D) && COMOPT_IsBackspace(com)) {
                        // backspace over all that we've just entered
                        for (; n > 1; n--) { comPortWrite(com, (UInt8 *)"\b \b", 3); }
                        if (buf) { buf[0] = com->last; }
                        n = 1;
                    } else
                    if (com->last == KEY_ESCAPE) {
                        if (!COMOPT_IgnoreEscape(com)) {
                            // echo escape character
                            comPortWrite(com, &com->last, 1);
                        }
                    } else {
                        // echo simple character
                        comPortWrite(com, &com->last, 1);
                    }
                }
                if (COMOPT_IsLogDebug(com) && com->logger) { 
                    (*com->logger)(&com->last,1); 
                }
            }
            
        }
       
    }
    
    // 'len' bytes read
    return n;
}

/* read single character */
// Use of this function is discouraged because examination of 'errno' is
// required to differentiate between an EOF and TIMEOUT.
int comPortReadChar(ComPort_t *com, long timeoutMS)
{
    UInt8 ch;
    int len = comPortRead(com, &ch, 1, timeoutMS);
    if (len < 0) {
        // error (error in 'errno')
        return -1;
    } else
    if (len == 0) {
        // timeout
        errno = ETIME;
        return -1;
    } else {
        return (int)ch;
    }
}

/* read a single byte (return '0' if error) */
UInt8 comPortGetByte(ComPort_t *com, long timeoutMS)
{
    int ch = comPortReadChar(com, timeoutMS);
    return (ch < 0)? 0 : (UInt8)ch;
}

int comPortReadLine(ComPort_t *com, char *buf, int maxLen, long timeoutMS)
{
    /*return _comPortReadLine(com, buf, 0, maxLen, timeoutMS); */
	return (read(com->read_fd, buf, maxLen));
}

static Int16 _comPortTranslateChar(ComPort_t *com, UInt8 ch)
{
    if (!COMOPT_IsPrintable(com)) {
        return (Int16)ch & 0xFF;
    } else
    if ((ch >= ' ') && (ch < 127)) {
        return (Int16)ch & 0xFF;
    } else {
        return -1;
    }
}
int _comPortReadLine(ComPort_t *com, char *buf, int startNdx, int maxLen, long timeoutMS)
{
    
    /* read nothing? */
    if (maxLen <= 0) {
        return 0;
    } 
    
    /* adjust startNdx, if necessary */
    if (startNdx < 0) {
        startNdx = 0;
    } else
    if (startNdx >= maxLen) {
        startNdx = maxLen - 1;
    }

    /* valid com port? */
    if (!comPortIsOpen(com)) {
        if (buf) { buf[0] = 0; }
        errno = ENODEV;
        return -1;
    }

    /* forced timeout? */
    // timeout < 0, is allowed
    if (timeoutMS == 0L) {
        return startNdx;
    }

    /* starting time */
    struct timeval startTime;
    utcGetTimestamp(&startTime);

    /* read bytes */
    com->error = COMERR_NONE;
    int n = startNdx;
    for (;n < maxLen;) {
        
        /* check accumulated timeout */
        long tms = timeoutMS;
        if (tms >= 0L) {
            tms -= utcGetDeltaMillis(0, &startTime); // subtract elapsed time
            if (tms <= 0L) { 
                // accumulated timeout
                com->error = COMERR_TIMEOUT;
                if (buf) { buf[n] = 0; }
                break;
            }
        }
        
        /* read byte */
        UInt8 ch, last = com->last; // save last last
        int r = comPortRead(com, &ch, 1, tms);
        if (r < 0) {
            // error: com port closed?
            if (buf) { buf[n] = 0; }
            // error in 'errno'
            return -1;
        } else
        if (r == 0) {
            // timeout?
            if (buf) { buf[n] = 0; }
            // error in 'errno'
            return 0;
        } else
        if (ch == KEY_RETURN) {
            // normal end of record (ie. keyboard input)
            if (buf) { buf[n] = 0; }
            //int LF = comPortRead(com, &ch, 1, 1L); 
            //if ((LF == 1) && (ch != KEY_NEWLINE)) {
            //    // the next byte was not an LF, throw it back
            //    comPortPush(com, ch);
            //}
            return n; // may be '0'
        } else
        if (ch == KEY_NEWLINE) {
            if (last == KEY_RETURN) {
                // a new-line following a carriage-return is ignored
                r = 0;
            } else {
                // end of record
                if (buf) { buf[n] = 0; }
                return n; // may be '0'
            }
        } else
        if (KEY_IsBackspace(ch) && COMOPT_IsBackspace(com)) {
            r = (n > 0)? -1 : 0; // backup one space (or at least consume the backspace)
        } else
        if ((ch == KEY_CONTROL_D) && COMOPT_IsEcho(com) && COMOPT_IsBackspace(com)) {
            // ECHO-ON required
            for (; n > 0; n--) { comPortWrite(com, (UInt8 *)"\b \b", 3); }
            if (buf) { buf[0] = ch; buf[1] = 0; }
            return 1; // return just the Control-D
        } else
        if ((ch == KEY_ESCAPE) && COMOPT_IgnoreEscape(com)) {
            // ignore this char
            r = 0;
        } else {
            if (buf) {
                Int16 cch = _comPortTranslateChar(com, ch);
                if (cch >= 0) {
                    buf[n] = (UInt8)cch; 
                } else {
                    r = 0; // ignore this char
                }
            }
        }
        
        /* count byte and read some more */
        n += r; // r == 1

    }
    
    // timeout, or 'maxLen' bytes read
    if (buf) { buf[(n < maxLen)? n : (maxLen - 1)] = 0; }
    return n;

}

/* read bytes until specified null-terminated byte-sequence has been found */
int comPortReadSequence(ComPort_t *com, const UInt8 *seq, long timeoutMS)
{
    
    /* valid com port? */
    if (!comPortIsOpen(com)) {
        errno = ENODEV;
        return -1;
    }
    
    /* forced timeout? */
    // timeout < 0, is allowed
    if (timeoutMS == 0L) {
        errno = ETIME;
        return 0;
    }

    /* starting time */
    struct timeval startTime;
    utcGetTimestamp(&startTime);
    
    /* read sequence loop */
    com->error = COMERR_NONE;
    int h = 0;
    for (;seq[h] != 0;) {
        
        /* check accumulated timeout */
        long tms = timeoutMS;
        if (tms >= 0L) {
            tms -= utcGetDeltaMillis(0, &startTime); // subtract elapsed time
            if (tms <= 0L) { 
                // accumulated timeout
                com->error = COMERR_TIMEOUT;
                break;
            }
        }
        
        /* read data */
        UInt8 ch;
        int len = comPortRead(com, &ch, 1, tms);
        if (len < 0) {
            // com port closed?
            logINFO(LOGSRC,"ComPort closed?\n");
            // error in 'errno'
            return -1;
        } else
        if (len == 0) {
            // timeout (error in 'errno')
            return 0;
        } else 
        if (ch == seq[h]) {
            h++;
        } else {
            h = 0;
        }
        
    }
    return (h > 0)? h : 1;
}

int comPortFlush(ComPort_t *com, long timeoutMS)
{
    
    /* valid com port? */
    if (!comPortIsOpen(com)) {
        errno = ENODEV;
        return -1;
    }
    
    /* flush internal buffers */
    // flush data received, but not read
    //logDEBUG(LOGSRC,"tcflush ...");
    tcflush(com->read_fd, TCIFLUSH);
    com->avail =  0; // clear what we know about available bytes
    com->push  = -1; // clear any 'pushed' bytes
    
    /* forced timeout? */
    // timeout < 0, not allowed
    if (timeoutMS <= 0L) {
        com->error = COMERR_TIMEOUT;
        return 0;
    }

    /* get starting time */
    struct timeval startTime;
    utcGetTimestamp(&startTime);
    
    /* read/discard loop */
    // wait for 'timeoutMS' of quiet
    com->error = COMERR_NONE;
    while (utTrue) {
        
        /* check accumulated timeout */
        long tms = timeoutMS;
        if (tms >= 0L) {
            tms -= utcGetDeltaMillis(0, &startTime); // subtract elapsed time
            if (tms <= 0L) { 
                // accumulated timeout
                com->error = COMERR_TIMEOUT;
                return 0;
            }
        }
        
        /* read/discard byte */
        int len = comPortRead(com, (UInt8*)0, 1, tms);
        if (len < 0) {
            // com->error already set
            // error in 'errno'
            return -1;
        } else
        if (len == 0) {
            // com->error already set
            // error in 'errno'
            return 0;
        }
        
    }
}

int comPortFlushWhitespace(ComPort_t *com, long timeoutMS)
{
    
    /* valid com port? */
    if (!comPortIsOpen(com)) {
        errno = ENODEV;
        return -1;
    }
    
    /* check 'pushed' characters */
    if (com->push >= 0) {
        if (isspace((char)com->push)) {
            com->push = -1;
        } else {
            return 1;
        }
    }
    
    /* forced timeout? */
    // timeout < 0, is allowed
    if (timeoutMS == 0L) {
        return 0;
    }
    
    /* get starting time */
    struct timeval startTime;
    utcGetTimestamp(&startTime);
    
    /* read space loop */
    UInt8 ch;
    com->error = COMERR_NONE;
    while (utTrue) {
        
        /* check accumulated timeout */
        long tms = timeoutMS;
        if (tms >= 0L) {
            tms -= utcGetDeltaMillis(0, &startTime); // subtract elapsed time
            if (tms <= 0L) { 
                // accumulated timeout
                com->error = COMERR_TIMEOUT;
                return 0;
            }
        }

        /* read/check space */
        int len = comPortRead(com, &ch, 1, timeoutMS);
        if (len < 0) {
            // error (error in 'errno')
            return -1;
        } else
        if (len == 0) {
            // timeout (error in 'errno')
            return 0;
        } else
        if (!isspace(ch)) {
            // push non-whitespace character back into buffer
            comPortPush(com, ch);
            return 1;
        }
        
    }
    
}

// ----------------------------------------------------------------------------
void comPortDrain(ComPort_t *com)
{
    if (comPortIsOpen(com)) {
        // wait for all output to be transmitted
        tcdrain(com->write_fd);
    }
}

int comPortWrite(ComPort_t *com, const UInt8 *buf, int len)
{
    if (!com) {
        errno = ENODEV;
        return -1;
    } else
    if (com->write_fd > 0) {
        if (COMOPT_IsLogDebug(com) && com->logger) { 
            (*com->logger)(buf, len); 
        }
        return write(com->write_fd, buf, len);
    } else {
        errno = ENODEV;
        return -1;
    }
}

int comPortWriteChar(ComPort_t *com, UInt8 ch)
{
    return comPortWrite(com, &ch, 1);
}

int comPortWriteString(ComPort_t *com, const char *s)
{
    return comPortWrite(com, (UInt8*)s, strlen(s));
}

// ----------------------------------------------------------------------------

#if defined(COMPORT_SUPPORT_AT_WRAPPER)

/* write specified formatted 'AT' command to modem */
static void _comPortWriteATFmt(ComPort_t *com, const char *fmt, va_list ap)
{
    char cmd[512];
    vsprintf(cmd, fmt, ap);
    //logDEBUG(LOGSRC,"Writing AT%s",cmd);
    comPortWriteAT(com, cmd);
}

/* write specified formatted 'AT' command to modem */
void comPortWriteATFmt(ComPort_t *com, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    _comPortWriteATFmt(com, fmt, ap);
}

/* write specified 'AT' command to modem */
void comPortWriteAT(ComPort_t *com, const char *cmd)
{
    comPortWrite(com, (UInt8*)"AT", 2);
    comPortWrite(com, (UInt8*)cmd, strlen(cmd));
    comPortWrite(com, (UInt8*)"\r", 1); // "\r\n", 2);
}

int comPortWriteReadAT(ComPort_t *com, const char *cmd, char *resp, int respLen)
{
    if (resp && (respLen > 0)) {
        *resp = 0;
        long timeoutMS = 2500L;
        comPortFlush(com, 0L);
        comPortWriteAT(com, cmd);
        while (utTrue) {
            comPortFlushWhitespace(com, 500L);
            int len = comPortReadLine(com, resp, respLen, timeoutMS);
            if (len < 0) {
                // error
                return -1;
            } else
            if ((len > 0) && strStartsWith(resp,"AT")) {
                // we'll assume that this is an 'echo' of the command
                continue; // try again
            }
            // return read length
            return len;
        }
    } else {
        return 0;
    }
}

#endif

// ----------------------------------------------------------------------------

#if defined(COMPORT_SUPPORT_CONSOLE)
/* turn off buffering on stdin */
void comPortResetSTDIN()
{
    // turn off buffering
    struct termios com.termios_new;
    //setvbuf(stdin, 0/*(char*)malloc(200)*/, _IONBF, 0); // [MALLOC]
    fcntl(STDIN_FILENO, F_SETFL, 0);
    tcgetattr(STDIN_FILENO, &com.termios_new);
    com.termios_new.c_lflag &= ~ECHO;
    com.termios_new.c_lflag &= ~ICANON;
    tcsetattr(STDIN_FILENO, TCSANOW, &com.termios_new);
}
#endif

// ----------------------------------------------------------------------------

#if defined(COMPORT_SUPPORT_CONSOLE)
/* turn off buffering on stdout */
void comPortResetSTDOUT()
{
    // turn off buffering
    struct termios com.termios_new;
    //setvbuf(stdout, (char*)malloc(1), _IOFBF, 1); // [MALLOC]
    setvbuf(stdout, 0, _IONBF, 0);
    fcntl(STDOUT_FILENO, F_SETFL, 0);
    tcgetattr(STDOUT_FILENO, &com.termios_new);
    com.termios_new.c_lflag &= ~ICANON;
    tcsetattr(STDOUT_FILENO, TCSANOW, &com.termios_new);
}
#endif

//##################################TARGET_LINUX
#else
// ----------------------------------------------------------------------------
int open_serial_binary(ComPort_t *com)
{
	int status, speed;
	char device_name[64];
	
	strcpy(device_name, ComDev_DIR);
	strncat(device_name, com->name, 48);
	printf("open %s\n", device_name);
	com->read_fd = open(device_name, O_RDWR); 
	if (com->read_fd < 0) {
		perror("Open tty port for reader");
		return com->read_fd;
	}

	status = tcgetattr(com->read_fd, &com->termios_old);
	if(status < 0) {
		perror("tcgetattr ");  
		return status;
	}
	
	com->termios_new = com->termios_old;
	com->termios_new.c_cflag |= (CREAD | HUPCL);
	com->termios_new.c_cflag &= ~(PARENB | CSIZE | CSTOPB);
	com->termios_new.c_cflag |= CS8;
	com->termios_new.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	com->termios_new.c_iflag &= ~(IGNBRK | BRKINT | IXON | IXOFF | IXANY);
	com->termios_new.c_iflag &= ~(PARMRK | INLCR | IGNCR | ICRNL | IUCLC);
	com->termios_new.c_oflag &= ~(OPOST | ONLCR | OLCUC | OCRNL | ONLRET | ONOCR | OFILL | OFDEL);
	com->termios_new.c_cc[VTIME] = 1;
	com->termios_new.c_cc[VMIN] = com->read_len;

	switch (com->bps) {
	case 115200:
		speed = B115200;
		break;
	case 57600:
		speed = B57600;
		break;
	case 4800:
		speed = B4800;
		break;
	default:
		speed = B9600;
	}
	status = cfsetispeed(&com->termios_new, speed);
	status = cfsetospeed(&com->termios_new, speed);
	status = tcsetattr(com->read_fd, TCSANOW, &com->termios_new); 
	if (status < 0)
		perror("tcsetattr ");  
	return status;
}

int open_serial_text(ComPort_t *com)
{
	int status, speed;
	char device_name[64];
	
	strcpy(device_name, ComDev_DIR);
	strncat(device_name, com->name, 48);
	com->read_fd = open(device_name, O_RDWR); 
	if (com->read_fd < 0) {
		perror("Open tty port for RFID reader");
		return com->read_fd;
	}

	status = tcgetattr(com->read_fd, &com->termios_old);
	if(status < 0) {
		perror("tcgetattr ");  
		return status;
	}
	com->termios_new = com->termios_old;
	com->termios_new.c_cflag |= (CREAD | HUPCL);
	com->termios_new.c_cflag &= ~(PARENB | CSIZE | CSTOPB);
	com->termios_new.c_cflag |= CS8;
	com->termios_new.c_lflag &= ~(ECHO | ECHONL | ECHOK | ECHONL);
	com->termios_new.c_lflag |= (ISIG | ICANON | IEXTEN);
	com->termios_new.c_oflag &= ~(OPOST | ONLCR | OLCUC | OCRNL | ONLRET | ONOCR);
	com->termios_new.c_iflag |= IGNCR;
	switch (com->bps) {
	case 4800:
		speed = B4800;
		break;
	case 19200:
		speed = B19200;
		break;
	default:
		speed = B9600;
	}
	status = cfsetispeed(&com->termios_new, speed);
	status = cfsetospeed(&com->termios_new, speed);
	status = tcsetattr(com->read_fd, TCSANOW, &com->termios_new); 
	if (status < 0)
		perror("tcsetattr ");  
	return status;
}

int change_serial_baudrate(ComPort_t *com, int rate)
{
	int status, speed;
	switch (rate) {
	case 4800:
		speed = B4800;
		break;
	default:
		speed = B9600;
	}
	status = cfsetispeed(&com->termios_new, speed);
	status = cfsetospeed(&com->termios_new, speed);
	status = tcsetattr(com->read_fd, TCSANOW, &com->termios_new); 
	if (status < 0)
		perror("tcsetattr ");  
	return status;
}
	
void comPortClose(ComPort_t *com)
{
	tcsetattr(com->read_fd, TCSAFLUSH, &com->termios_old);
	close(com->read_fd);
}
int comPortWriteString(ComPort_t *com, const char *s)
{
	return 0;
}
int comPortFlush(ComPort_t *com, long timeoutMS)
{
	return 0;
}
//##################################TARGET_LINUX
#endif
