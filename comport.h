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

#ifndef _COMPORT_H
#define _COMPORT_H
#ifdef __cplusplus
extern "C" {
#endif

// ----------------------------------------------------------------------------

#include <stdio.h>
#include <string.h>

#  include <unistd.h>
#  include <fcntl.h>
#  include <errno.h>
#  include <termios.h>

#include "stdtypes.h"

// ----------------------------------------------------------------------------

/* uncomment to support stdin/stdout communication */
#if defined(TARGET_CYGWIN)
#  define COMPORT_SUPPORT_CONSOLE
#endif

/* uncomment to support local serial port emulation using pipes */
#if defined(TARGET_CYGWIN)
#  define COMPORT_SUPPORT_PIPE
#endif

/* uncomment to include AT command wrappers */
#define COMPORT_SUPPORT_AT_WRAPPER

/* uncomment to include support for BlueTooth */
#if defined(TARGET_GUMSTIX) || defined(TARGET_WINCE) || defined(TARGET_CYGWIN)
#  define COMPORT_SUPPORT_BLUETOOTH
#endif

/* uncomment to include support for USB ports */
#if defined(TARGET_CYGWIN) || defined(TARGET_LINUX)
#  define COMPORT_SUPPORT_USB
#endif

/* uncomment to include support socket communications */
// more work still needs to be done for this to be fully supported
//#define COMPORT_SUPPORT_SOCKET

//##################################TARGET_LINUX
#ifndef TARGET_LINUX
// ----------------------------------------------------------------------------
// Socket/NMEAD support

#if defined(COMPORT_SUPPORT_SOCKET)
#define NMEAD_SOCKET_PORT           1155
#define CSOCK_SOCKET_PORT_BASE      32000
#define CSOCK_SOCKET_HOST           "127.0.0.1"
#endif

// ----------------------------------------------------------------------------
// Device types

#define COMTYPE_TYPE_MASK           0xFF00
#define COMTYPE_INDEX_MASK          0x00FF

#define COMTYPE_STANDARD            0x1000 // WinCE
#define COMTYPE_VIRTUAL             0x1100 // WinCE
#define COMTYPE_USB                 0x1200
#define COMTYPE_BLUETOOTH           0x1300 // WinCE
#define COMTYPE_SPIPE               0x2100
#define COMTYPE_CPIPE               0x2200
#define COMTYPE_CONSOLE             0x2300

#if defined(COMPORT_SUPPORT_SOCKET)
#define COMTYPE_SSOCK               0x2400
#define COMTYPE_CSOCK               0x2500
#define COMTYPE_NMEAD               0x2600
#endif

#define PORT_IsSTANDARD(P)          (((P) & COMTYPE_TYPE_MASK) == COMTYPE_STANDARD)
#define PORT_IsVIRTUAL(P)           (((P) & COMTYPE_TYPE_MASK) == COMTYPE_VIRTUAL)
#define PORT_IsUSB(P)               (((P) & COMTYPE_TYPE_MASK) == COMTYPE_USB)
#define PORT_IsBLUETOOTH(P)         (((P) & COMTYPE_TYPE_MASK) == COMTYPE_BLUETOOTH)
#define PORT_IsCPIPE(P)             (((P) & COMTYPE_TYPE_MASK) == COMTYPE_CPIPE)
#define PORT_IsSPIPE(P)             (((P) & COMTYPE_TYPE_MASK) == COMTYPE_SPIPE)
#define PORT_IsSTDIN(P)             (((P) & COMTYPE_TYPE_MASK) == COMTYPE_CONSOLE)
#define PORT_IsCONSOLE(P)           (((P) & COMTYPE_TYPE_MASK) == COMTYPE_CONSOLE)

#if defined(COMPORT_SUPPORT_SOCKET)
#define PORT_IsSSOCK(P)             (((P) & COMTYPE_TYPE_MASK) == COMTYPE_SSOCK)
#define PORT_IsCSOCK(P)             (((P) & COMTYPE_TYPE_MASK) == COMTYPE_CSOCK)
#define PORT_IsNMEAD(P)             (((P) & COMTYPE_TYPE_MASK) == COMTYPE_NMEAD)
#endif

#define PORT_Index0(P)              ((int)((P) & COMTYPE_INDEX_MASK))
#define PORT_Index1(P)              (PORT_Index0(P) + 1)

// ----------------------------------------------------------------------------
// Device names

#define MAX_COM_NAME_SIZE           16
#define MAX_COM_DEV_SIZE            32

#define ComName_INVALID             "INVALID"

/* standard ports */
#define ComName_COM                 "COM"
#define ComName_TTYS                "ttyS"

/* USB ports */
#if defined(COMPORT_SUPPORT_USB)
#  define ComName_USB               "USB"
#  define ComName_TTYUSB            "ttyUSB"
#endif

/* Bluetooth */
#if defined(COMPORT_SUPPORT_BLUETOOTH)
#  define ComName_RFCOMM0           "RFCOMM0"
#  define ComName_BT                "BT"
#  define ComName_BTH               "BTH"
#endif

/* console serial port emulator */
#if defined(COMPORT_SUPPORT_CONSOLE)
#  define ComName_CONSOLE           "CONSOLE"
#endif

/* Serial communication emulator (piped data communication) */
#if defined(COMPORT_SUPPORT_PIPE)
#  define ComName_CPIPE             "CPIPE"
#  define ComName_SPIPE             "SPIPE"
#endif

/* Client/server socket communication emulator */
#if defined(COMPORT_SUPPORT_SOCKET)
#  define ComName_SSOCK             "SSOCK"
#  define ComName_CSOCK             "CSOCK"
#  define ComName_NMEAD             "NMEAD"
#endif

/* GumStix serial port aliases */
#if defined(TARGET_GUMSTIX)
#  define ComName_FFUART            "FFUART"
#  define ComName_BTUART            "BTUART"
#  define ComName_STUART            "STUART"
#  define ComName_HWUART            "HWUART"
#  define ComName_RFCOMM0           "RFCOMM0"
#endif

// ----------------------------------------------------------------------------

/* maximum allowed serial ports */
#if defined(TARGET_GUMSTIX)
#  define MAX_COM_PORT              (3)
#else
#  define MAX_COM_PORT              (99)
#endif

// ----------------------------------------------------------------------------

#define BPS_1200                      1200
#define BPS_2400                      2400
#define BPS_4800                      4800
#define BPS_9600                      9600
#define BPS_19200                    19200
#define BPS_38400                    38400
#define BPS_57600                    57600
#define BPS_76800                    76800  // not supported by WinCE
#define BPS_115200                  115200
#define BPS_921600                  921600  // not supported by cygwin/WinCE

// ----------------------------------------------------------------------------

/* data formats */
#define DTAFMT_8N1                  "8N1"
#define DTAFMT_8O1                  "8O1"   // non-standard
#define DTAFMT_8E1                  "8E1"   // non-standard
#define DTAFMT_7N2                  "7N2"
#define DTAFMT_7O1                  "7O1"
#define DTAFMT_7E1                  "7E1"

// ----------------------------------------------------------------------------

#define COMOPT_LOGDEBUG             (0x8000)
#define COMOPT_ECHO                 (0x0001)
#define COMOPT_BACKSPACE            (0x0004)
#define COMOPT_PRINTABLE            (0x0008)
#define COMOPT_VTIMEOUT             (0x0010)
#define COMOPT_NOESCAPE             (0x0020)

#define COMOPT_IsLogDebug(C)        ((C)? ((C)->flags & COMOPT_LOGDEBUG ) : 0)
#define COMOPT_IsEcho(C)            ((C)? ((C)->flags & COMOPT_ECHO     ) : 0)
#define COMOPT_IsBackspace(C)       ((C)? ((C)->flags & COMOPT_BACKSPACE) : 0)
#define COMOPT_IsPrintable(C)       ((C)? ((C)->flags & COMOPT_PRINTABLE) : 0)
#define COMOPT_IsVTimeout(C)        ((C)? ((C)->flags & COMOPT_VTIMEOUT ) : 0)
#define COMOPT_IgnoreEscape(C)      ((C)? ((C)->flags & COMOPT_NOESCAPE ) : 0)

// ----------------------------------------------------------------------------

#define COMERR_NONE                 0   // success
#define COMERR_TIMEOUT              1   // timeout error
#define COMERR_EOF                  2   // EOF
#define COMERR_GENERAL              3   // general (unknown error)
#define COMERR_BADDEV               4   // invalid device
#define COMERR_INUSE                5   // device in use
#define COMERR_SPEED                6   // invalid device speed
#define COMERR_INIT                 7   // initialization/open

#define COMERR_IsError(C)           ((C)? (((C)->error)? 1 : 0)          : 1)
#define COMERR_IsTimeout(C)         ((C)? ((C)->error == COMERR_TIMEOUT) : 1)
#define COMERR_IsEOF(C)             ((C)? ((C)->error == COMERR_EOF    ) : 1)

// ----------------------------------------------------------------------------

#define KEY_RETURN                  '\r'    // 0x0D
#define KEY_NEWLINE                 '\n'    // 
#define KEY_DELETE                  0x7F    //
#define KEY_CONTROL_C               0x03    //  3
#define KEY_CONTROL_D               0x04    //  4
#define KEY_CONTROL_H               '\b'    //  8
#define KEY_ESCAPE                  0x1B    // 27

#define KEY_IsBackspace(K)          (((K) == KEY_CONTROL_H) || ((K) == KEY_DELETE))

// ----------------------------------------------------------------------------

#if defined(COMPORT_SUPPORT_BLUETOOTH)
utBool comPortIsBluetooth(ComPort_t *com);
utBool comPortIsBluetoothReady(ComPort_t *com);
#  if defined(SUPPORT_UInt64)
UInt64 comPortGetBluetoothAddress();
#  endif
#endif

// ----------------------------------------------------------------------------

utBool comPortIsValidName(const char *portName);

// ----------------------------------------------------------------------------

void comPortSetDebugLogger(ComPort_t *com, void (*logger)(const UInt8*, int));

// ----------------------------------------------------------------------------

ComPort_t *comPortInitStruct(ComPort_t *com);
ComPort_t *comPortOpen(ComPort_t *com, const char *portName, long bps, const char *dataFmt, utBool binary, int maxLen);
ComPort_t *comPortReopen(ComPort_t *com);
utBool comPortIsOpen(ComPort_t *com);
const char *comPortName(ComPort_t *com);

// ----------------------------------------------------------------------------

void comPortSetOptions(ComPort_t *com, UInt16 flags);
void comPortAddOptions(ComPort_t *com, UInt16 flags);
void comPortRemoveOptions(ComPort_t *com, UInt16 flags);
UInt16 comPortGetOptions(ComPort_t *com);

// ----------------------------------------------------------------------------

void comPortSetDTR(ComPort_t *com, utBool state);
utBool comPortGetDTR(ComPort_t *com);

void comPortSetRTS(ComPort_t *com, utBool state);
utBool comPortGetRTS(ComPort_t *com);

void comPortSetCTS(ComPort_t *com, utBool state);
utBool comPortGetCTS(ComPort_t *com);

void comPortSetDCD(ComPort_t *com, utBool state);
utBool comPortGetDCD(ComPort_t *com);

// ----------------------------------------------------------------------------

void comPortPush(ComPort_t *com, UInt8 ch);
int comPortGetAvail(ComPort_t *com, long timeoutMS);

int comPortRead(ComPort_t *com, UInt8 *buf, int len, long timeoutMS);
int comPortReadChar(ComPort_t *com, long timeoutMS);
UInt8 comPortGetByte(ComPort_t *com, long timeoutMS);
int _comPortReadLine(ComPort_t *com, char *buf, int startIndex, int maxLen, long timeoutMS);
int comPortReadLine(ComPort_t *com, char *buf, int maxLen, long timeoutMS);
int comPortReadSequence(ComPort_t *com, const UInt8 *seq, long timeoutMS);

int comPortFlushWhitespace(ComPort_t *com, long timeoutMS);

// ----------------------------------------------------------------------------

void comPortDrain(ComPort_t *com);
int comPortWrite(ComPort_t *com, const UInt8 *buf, int len);
int comPortWriteChar(ComPort_t *com, UInt8 ch);

// ----------------------------------------------------------------------------
int open_serial_binary(ComPort_t *com);
// ----------------------------------------------------------------------------

#if defined(COMPORT_SUPPORT_AT_WRAPPER)
void comPortWriteATFmt(ComPort_t *com, const char *fmt, ...);
void comPortWriteAT(ComPort_t *com, const char *cmd);
int comPortWriteReadAT(ComPort_t *com, const char *cmd, char *resp, int respLen);
#endif

// ----------------------------------------------------------------------------

#if defined(COMPORT_SUPPORT_CONSOLE)
void comPortResetSTDIN();
void comPortResetSTDOUT();
#endif
// ----------------------------------------------------------------------------
//##################################TARGET_LINUX
#else
// ----------------------------------------------------------------------------
#define PORT_NAME_SIZE 48

typedef struct 
{
	int         read_fd;                // read file descriptor
	char        name[PORT_NAME_SIZE]; // device name ("COM1", "PIPE0", etc)
	int			bps;                    // speed BPS_#
	int			read_len;
	struct termios termios_old;   
	struct termios termios_new;   
} ComPort_t;

int open_serial_binary(ComPort_t *com);
int open_serial_text(ComPort_t *com);
int change_serial_baudrate(ComPort_t *com, int rate);
void comPortClose(ComPort_t *com);
int comPortWriteString(ComPort_t *com, const char *s);
int comPortFlush(ComPort_t *com, long timeoutMS);
// ----------------------------------------------------------------------------
//##################################TARGET_LINUX
#endif
#ifdef __cplusplus
}
#endif
#endif
