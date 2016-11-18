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
// This OpenDMTP reference implementation is aware of the following targets:
//   TARGET_LINUX   - Generic Linux platform
//   TARGET_GUMSTIX - Gumstix platform
//   TARGET_CYGWIN  - Windows Cygwin platform
//   TARGET_WINCE   - Windows CE
//      TARGET_WINCE_PHONE - Windows CE/Mobile phones (eg. HP hw6945)
// ----------------------------------------------------------------------------
// Change History:
//  2006/03/26  Martin D. Flynn
//     -Changed default accountId/deviceId to "opendmtp"/"mobile".
//  2007/01/28  Martin D. Flynn
//     -Include many changes facilitating port to WindowsCE
//     -Define RELEASE_VERSION, PROTOCOL_VERSION
//  2007/02/08  Martin D. Flynn
//     -Included DMTP_EXPERIMENTAL compile time var to allow supporting experimental
//      DMTP implementations.
// ----------------------------------------------------------------------------

#ifndef _DEFAULTS_H
#define _DEFAULTS_H
#ifdef __cplusplus
extern "C" {
#endif

// ----------------------------------------------------------------------------

// if a WINCE sub-target is defined, make sure TARGET_WINCE is also defined
#if defined(TARGET_WINCE_PHONE) && !defined(TARGET_WINCE)
#  define TARGET_WINCE
#endif

// ----------------------------------------------------------------------------
#define PLATFORM_VERSION 2
#define HARDWARE_VERSION 1
#define FEATURE_VERSION 0
// Application name/description
#if !defined(APPLICATION_NAME)
	#define APPLICATION_NAME			"OsitechDMTP"
#endif
#if !defined(APPLICATION_DESCRIPTION)
	#define APPLICATION_DESCRIPTION	"Ositech DMTP"
#endif
#if !defined(SYSLOG_NAME)
	#define SYSLOG_NAME0			"/mnt/flash/titan-data/ositechdmtp0.log"
	#define SYSLOG_NAME1			"/mnt/flash/titan-data/ositechdmtp1.log"
	#define EVENT_QUEUE_BACKUP_NAME	"/mnt/flash/titan-data/event_queue.bak"
#endif
#define	PROTO_VERSION_TCP	2 
#define	PROTO_VERSION_UDP	3 
#define DEFAULT_PROTO_VERSION	PROTO_VERSION_UDP
// ----------------------------------------------------------------------------
// Uncomment when compiling for debug testing purposed only
//#define DEBUG_COMPILE
// ----------------------------------------------------------------------------
// 'ENABLE' directives.
// Uncomment the following '#define's to enable these features:
#define ENABLE_SET_TIME       // setting the current processor clock time
//#define ENABLE_REBOOT         // device rebooting
//#define ENABLE_UPLOAD         // uploading files from server to device
//#define ENABLE_GEOZONE        // geofence checking
//#define ENABLE_GUI            // user interface (requires support)
// ------------------------------------
// WindowsCE directives

// All WindowsCE platforms
#if defined(TARGET_WINCE)
#  define PROTOCOL_THREAD
#  define GPS_THREAD
#endif

// WindowsCE phone platform only
#if defined(TARGET_WINCE_PHONE)
#  define TRANSPORT_MEDIA_SOCKET
#  define ENABLE_GUI
#endif

// ------------------------------------
// Gumstix directives

#if defined(TARGET_GUMSTIX)
#  define ENABLE_GEOZONE
#  define ENABLE_REBOOT
#  define ENABLE_UPLOAD
#  define ENABLE_SET_TIME
#endif

// ------------------------------------
// Windows Cygwin

#if defined(TARGET_CYGWIN)
#  define ENABLE_GEOZONE
#endif

// ------------------------------------
// Generic Linux Directives

#if defined(TARGET_LINUX)
#define _GNU_SOURCE
//#  define ENABLE_GEOZONE
#define SERIAL_PORT_HW_FLOW 0
#endif

// ----------------------------------------------------------------------------
// Identification constants

/* file transport IDs */
#if defined(TRANSPORT_MEDIA_FILE)
#  define ACCOUNT_ID                ""
#  define DEVICE_ID                 ""
#endif

/* serial/bluetooth transport IDs */
#if defined(TRANSPORT_MEDIA_SERIAL)
#  define ACCOUNT_ID                ""
#  define DEVICE_ID                 ""
#endif

/* default IDs */
#if !defined(UNIQUE_ID)
#  define UNIQUE_ID                 { 0x00 } // (invalid ID) length must be >= 4 to be valid
#endif
#if !defined(ACCOUNT_ID)
#  define ACCOUNT_ID                "opendmtp"
#endif
#if !defined(DEVICE_ID)
#  define DEVICE_ID                 "mobile"
#endif

// ----------------------------------------------------------------------------
// 1, and only 1 of the following must be defined:
// - TRANSPORT_MEDIA_SOCKET 
//    - When communicating over a socket
//    - May use other additional means to establish a socket connection
// - TRANSPORT_MEDIA_FILE
//    - When writing to a file
//    - "Simplex" type comunication only
//    - No connection accounting is used
// - TRANSPORT_MEDIA_SERIAL
//    - When communicating over a Serial port (incl Bluetooth)
//    - "Duplex" type comunication only
//    - No connection accounting is used
// - TRANSPORT_MEDIA_GPRS
//    - When communicating over a GPRS modem (AT-command based)
//    - Requires that the GPRS modem supports TCP/UDP socket connection commands.
#if   defined(TRANSPORT_MEDIA_SOCKET)
#elif defined(TRANSPORT_MEDIA_FILE)
#elif defined(TRANSPORT_MEDIA_SERIAL)
#elif defined(TRANSPORT_MEDIA_GPRS)
#else
// uncomment the appropriate transport (or define elsewhere)
//#define TRANSPORT_MEDIA_SOCKET
//#define TRANSPORT_MEDIA_FILE
//#define TRANSPORT_MEDIA_SERIAL
//#define TRANSPORT_MEDIA_GPRS
#endif

// If desired, the transport media can instead be defined in the Makefile by including
// the string -DTRANSPORT_MEDIA_SOCKET (for example) as a compile-time flag.

// On the GumStix platform, to store the file on the SD flash, change to filename
// directory reference the location of the SD flash.
// For instance "/mnt/mmc/sample.gps"
//#if defined(TRANSPORT_MEDIA_FILE)
//#  if defined(TARGET_GUMSTIX)
//#    define TRANSPORT_MEDIA_FILE_NAME       ("/mnt/mmc/" TRANSPORT_MEDIA_FILE)
//#  else
//#    define TRANSPORT_MEDIA_FILE_NAME       ("./" TRANSPORT_MEDIA_FILE)
//#  endif
//#endif

// ----------------------------------------------------------------------------

// Check to see that TRANSPORT_MEDIA_xxxxxx has been properly defined
#if   defined(TRANSPORT_MEDIA_NONE)
#  define XP_NONE_          1
#endif
#if   defined(TRANSPORT_MEDIA_SOCKET)
#  define XP_SOCK_          1
#endif
#if   defined(TRANSPORT_MEDIA_FILE)
#  define XP_FILE_          1
#endif
#if   defined(TRANSPORT_MEDIA_SERIAL)
#  define XP_SER_           1
#endif
#if   defined(TRANSPORT_MEDIA_GPRS)
#  define XP_GPRS_          1
#endif
#if !defined(SKIP_TRANSPORT_MEDIA_CHECK)
// at least 1 TRANSPORT_MEDIA_xxxxx must be defined
#  if ((XP_NONE_ + XP_SOCK_ + XP_FILE_ + XP_SER_ + XP_GPRS_ + XP_GS_ + XP_LIB_) == 0)
#    error TRANSPORT_MEDIA_xxxxxx is not defined!
#  endif
// at most 1 TRANSPORT_MEDIA_xxxxx must be defined
#  if ((XP_NONE_ + XP_SOCK_ + XP_FILE_ + XP_SER_ + XP_GPRS_ + XP_GS_ + XP_LIB_) > 1)
#    error TRANSPORT_MEDIA_xxxxxx is defined more than once (only one transport media type is allowed)!
#  endif
#endif

// ----------------------------------------------------------------------------
// Packet/PacketQueue configuration

/* set Packet payload configuration */
// set to maximum number of separate fields used in packets (should be at least 10)
#define PACKET_MAX_FIELD_COUNT		10
// set to maximum size of largest possible payload (must be at least 32, but no larger than 255)
#define PACKET_FULL_LENGTH	128 //data, format, etc	

/* maximum size of cached events */
#if !defined(EVENT_QUEUE_SIZE)
	#if defined(TARGET_WINCE_PHONE)
	#define EVENT_QUEUE_SIZE	100     // limit memory use on phone
	#elif defined(TRANSPORT_MEDIA_SERIAL)
	#define EVENT_QUEUE_SIZE	5000    // serial transport (probably can be larger)
	#else
	#define EVENT_QUEUE_SIZE_EXPONENT	13
	#define EVENT_QUEUE_SIZE	(1 << EVENT_QUEUE_SIZE_EXPONENT)
	#endif
#endif
#define EVENT_QUEUE_OVERWRITE               utTrue  // overwrite unsent events when queue is full?

/* protocol volatile & pending queue sizes */
// there's probably never more that 5 or so volatile packets
#define PRIMARY_VOLATILE_QUEUE_SIZE	32	// max volatile packets (primary)
#define PRIMARY_PENDING_QUEUE_SIZE	64 // max pending packets  (primary)
#define SECONDARY_VOLATILE_QUEUE_SIZE       10      // max volatile packets (secondary)
#define SECONDARY_PENDING_QUEUE_SIZE        10      // max pending packets  (secondary)
#define SECONDARY_EVENT_QUEUE_SIZE          10      // max event packets    (secondary)

// ----------------------------------------------------------------------------
// Default event packet format (may be custom)

#if !defined(DEFAULT_EVENT_FORMAT)
#  define DEFAULT_EVENT_FORMAT              PKT_CLIENT_FIXED_FMT_HIGH   // E031: packet.h
#endif

// ----------------------------------------------------------------------------
// Default encoding

// encoding for FILE should be ASCII (Hex or Base64) - to be viewable in an editor
#define DEFAULT_FILE_ENCODING               ENCODING_HEX
#if defined(TRANSPORT_MEDIA_FILE)
#  define DEFAULT_ENCODING                  DEFAULT_FILE_ENCODING
#endif

// straight socket communication should be binary encoded - for best efficiency
#define DEFAULT_SOCKET_ENCODING             ENCODING_BINARY
#if defined(TRANSPORT_MEDIA_SOCKET)
#  define DEFAULT_ENCODING                  DEFAULT_SOCKET_ENCODING
#endif

// encoding for Serial(RS232) should be ASCII (Hex or Base64)
#define DEFAULT_SERIAL_ENCODING             ENCODING_HEX
#if defined(TRANSPORT_MEDIA_SERIAL)
#  define DEFAULT_ENCODING                  DEFAULT_SERIAL_ENCODING
#endif

// encoding for GPRS should be ASCII (Hex or Base64) since it transmits over a modem
#define DEFAULT_GPRS_ENCODING               ENCODING_BASE64
#if defined(TRANSPORT_MEDIA_GPRS)
#  define DEFAULT_ENCODING                  DEFAULT_GPRS_ENCODING
#endif

// ----------------------------------------------------------------------------
// absolute transmission rates

// explicit no transport defined
#if defined(TRANSPORT_MEDIA_NONE)
#  define MIN_XMIT_RATE                     0L      // seconds
#  define MIN_XMIT_DELAY                    0L      // seconds
#endif

// no minimums when writing to file
#if  defined(TRANSPORT_MEDIA_FILE)
#  define MIN_XMIT_RATE                     0L      // seconds
#  define MIN_XMIT_DELAY                    0L      // seconds
#endif

// set minimums when sending to server over a socket
#if defined(TRANSPORT_MEDIA_SOCKET)
#  define MIN_XMIT_RATE                     120L    // seconds
#  define MIN_XMIT_DELAY                    90L     // seconds
#endif

// no minimums for Serial communication
#if defined(TRANSPORT_MEDIA_SERIAL)
#  define MIN_XMIT_RATE                     0L      // seconds
#  define MIN_XMIT_DELAY                    0L      // seconds
#endif

// set minimums when sending to server over GPRS
#if defined(TRANSPORT_MEDIA_GPRS)
#  define MIN_XMIT_RATE                     120L    // seconds
#  define MIN_XMIT_DELAY                    90L     // seconds
#endif

// ----------------------------------------------------------------------------
// Define "PROTOCOL_THREAD" to run the transport connections in a separate thread
//#define PROTOCOL_THREAD
#if defined(PROTOCOL_THREAD) && !defined(ENABLE_THREADS)
#  define ENABLE_THREADS
#endif

// ----------------------------------------------------------------------------
// Define "GPS_THREAD" to run the GPS acquisition in a separate thread
//#define GPS_THREAD
#if defined(GPS_THREAD) && !defined(ENABLE_THREADS)
#  define ENABLE_THREADS
#endif

// ----------------------------------------------------------------------------
// "PQUEUE_THREAD_LOCK" must be defined in a multi-threaded environment to
// enable pqueue mutex locking.
#if defined(ENABLE_THREADS)
#  define PQUEUE_THREAD_LOCK
#endif

// ----------------------------------------------------------------------------
// Define "CUSTOM_PROPERTIES" if there are additional custom properties that
// should be made available in the property manager.
// Note that "custprop.h" and "custprop.inc" must also be available.
//#define CUSTOM_PROPERTIES

// ----------------------------------------------------------------------------
// Configuration/Property files

// Gumstix platform
#if defined(TARGET_GUMSTIX)                 // "/dmtp"
#  define DIR_SEP                           "/"
#  if !defined(CONFIG_DIR_NAME)
#    define CONFIG_DIR_NAME                 "dmtp"
#  endif
#endif

// Windows CE platform
#if defined(TARGET_WINCE)                   // "\DMTP"
#  define DIR_SEP                           "\\"
#  if !defined(CONFIG_DIR_NAME)
#    define CONFIG_DIR_NAME                 "DMTP"
#  endif
#endif

// Cygwin platform
#if defined(TARGET_CYGWIN)                  // "."
#  define DIR_SEP                           "/"
#  if !defined(CONFIG_DIR_NAME)
#    define CONFIG_DIR_NAME                 "."
#  endif
#  if !defined(CONFIG_DIR)
#    define CONFIG_DIR                      "."
#  endif
#endif

// Linux platform
#if defined(TARGET_LINUX)                   // "."
#  define DIR_SEP                           "/"
#  if !defined(CONFIG_DIR_NAME)
#define CONFIG_DIR_NAME		"var/conf"
//#    define CONFIG_DIR_NAME                 "tmp"
#  endif
#endif

// config directory prefix
#if !defined(CONFIG_DIR_NAME)
#  define CONFIG_DIR_NAME                   "DMTP"
#endif
#if !defined(CONFIG_DIR)
#  define CONFIG_DIR                        DIR_SEP CONFIG_DIR_NAME
#endif
#define CONFIG_DIR_                         CONFIG_DIR DIR_SEP

// uncomment to enable loading/saving property file
//#define PROPERTY_FILE                       (CONFIG_DIR_ "props.conf")
//#define PROPERTY_CACHE                      (CONFIG_DIR_ "props.dat")
#define PROPERTY_FILE                       (CONFIG_DIR_ "ositechdmtp.conf")
#define PROPERTY_CACHE                      ("/home/default/ositechdmtp.cach")
#define MAX_PAYLOAD_SIZE	256
#define MAX_ID_SIZE			32

// ----------------------------------------------------------------------------
// message logging

// default logging-level
#if defined(TARGET_WINCE_PHONE)
#  define LOGGING_DEFAULT_LEVEL             -SYSLOG_DEBUG
#endif
#if !defined(LOGGING_DEFAULT_LEVEL)
#  define LOGGING_DEFAULT_LEVEL             SYSLOG_DEBUG
#endif

// console logging
#define LOGGING_CONSOLE

// Auxiliary logging directory
// This allows logging to an auxiliary storage device (SD card, etc.) if the
// device is available, and a specific directory on the device exists.
// Set LOGGING_AUX_DEVICE to the locacation of the auxiliary storage device.
// On the different Windows CE devices that I've worked with (3 so far) this
// directory is usually "\Storage Card", but may be different for different 
// devices, and will most certainly be different for other non-WinCE devices.

// HP hw6945 phone
#if defined(TARGET_WINCE_PHONE) && !defined(LOGGING_AUX_DEVICE)
#  define LOGGING_AUX_DIRECTORY           DIR_SEP "Storage Card" DIR_SEP CONFIG_DIR_NAME
#endif

// Windows Cygwin
#if defined(TARGET_CYGWIN)
#  define LOGGING_AUX_DIRECTORY             "."
#endif

// Auxiliary message logging
#if defined(LOGGING_AUX_DIRECTORY)
#  define LOGGING_MESSAGE_FILE              LOGGING_AUX_DIRECTORY DIR_SEP "MESSAGE.log"
#endif

// ----------------------------------------------------------------------------

// Experimental code
// uncomment to include experimental DMTP support (ie. not yet part of the DMTP protocol)
#define DMTP_EXPERIMENTAL

// ----------------------------------------------------------------------------
// GPS simulator (testing/debugging only)
// This must also be supported by the GPS receiver handler (see 'custom/gps.c')

// define to compile with GPS simulation support
#if defined(TARGET_CYGWIN)
// always on Cygwin (development/testing platform)
#  define GPS_DEVICE_SIMULATOR
#elif defined(TARGET_WINCE_PHONE)
// never on a WinCE phone (it isn't useful)
#elif defined(TARGET_WINCE)
// uncomment to enable GPS simulator on other WinCE platorms
//#  define GPS_DEVICE_SIMULATOR
#else
// other non-WinCE platforms (Linux, etc.)
//#  define GPS_DEVICE_SIMULATOR
#endif

// ----------------------------------------------------------------------------
// Feature list

#if defined(ENABLE_SET_TIME)
#  define _F_STM                "/SetTime"      // set system clock enabled
#else
#  define _F_STM                ""
#endif
#if defined(ENABLE_REBOOT)
#  define _F_RBT                "/Reboot"       // Reboot enabled
#else
#  define _F_RBT                ""
#endif
#if defined(ENABLE_UPLOAD)
#  define _F_UPL                "/Upload"       // File upload support
#else
#  define _F_UPL                ""
#endif
#if defined(ENABLE_GUI)
#  define _F_GUI                "/GUI"          // Simple GUI
#else
#  define _F_GUI                ""
#endif
#if defined(ENABLE_GEOZONE)
#  define _F_GZO                "/GZone"        // GeoZone support
#else
#  define _F_GZO                ""
#endif
#if defined(GPS_DEVICE_SIMULATOR)
#  define _F_GDS                "/GPSim"        // GPS simulator
#else
#  define _F_GDS                ""
#endif
#if !defined(APPLICATION_FEATURES)
#  define APPLICATION_FEATURES _F_STM _F_RBT _F_UPL _F_GUI _F_GZO _F_GDS
#endif

// ----------------------------------------------------------------------------

#ifdef __cplusplus
}
#endif
#endif
