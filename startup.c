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
//  Main entry point and custom startup initialization
//  Provides customized startup initialization.
// ---
// Change History:
//  2006/01/04  Martin D. Flynn
//     -Initial release
//  2006/01/12  Martin D. Flynn
//     -Integrated syslog support
//  2006/01/27  Martin D. Flynn
//     -Changed '-debug' to send output logs to console
//     -Fixed bug where "TRANSPORT_MEDIA_SOCK" should have been "TRANSPORT_MEDIA_SOCKET"
//  2006/02/12  Martin D. Flynn
//     -Changed to accomodate new return type from "gpsGetDiagnostics".
//     -Changed to save properties back to 'propertyCache', instead of 'propertyFile'.
//  2006/04/10  Martin D. Flynn
//     -Changed Socket property 'PROP_MOTION_IN_MOTION' default to 15 minutes.
//  2006/06/07  Martin D. Flynn
//     -Relaxed TRANSPORT_MEDIA_SOCKET connection settings
//  2007/01/28  Martin D. Flynn
//     -Many changes to facilitate WindowsCE port
//  2007/04/28  Martin D. FLynn
//     -Don't queue events if either PROP_COMM_HOST or PROP_COMM_PORT are undefined.
// ----------------------------------------------------------------------------

#include "defaults.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <limits.h>
#include <sys/stat.h>
#include <unistd.h>
#include <debuglog.h>


#include "log.h"
#include "ap_diagnostic_log.h"
#include "gps.h"
#include "gpsmods.h"
#include "startup.h"
#include "transport.h"
#include "diagnostic.h"
#include "motion.h"
#include "odometer.h"

#if defined(ENABLE_GEOZONE)
#  include "geozone.h"
#endif

#include "stdtypes.h"
#include "utctools.h"
#include "strtools.h"
#include "bintools.h"
#include "threads.h"
#include "io.h"
#include "comport.h"

#include "cerrors.h"
#include "propman.h"
#include "statcode.h"
#include "mainloop.h"

#define HOSTSIZE 64
#if !defined(PROTOCOL_THREAD)
#include "accting.h"
#endif
#include "events.h"
#include "protocol.h"
#include "rfid.h"
#if defined(ENBALE_UPLOAD)
#  include "upload.h"
#endif

#if defined(TARGET_WINCE)
#  include <aygshell.h>
#  include "wince/wceos.h"
#else
// TODO: should separate this into gumstix/linux/cygwin
#  include "os.h"
#endif

// ----------------------------------------------------------------------------
// Version string
// Examples:
//   OpenDMTP_FILE.1.0.3
//   OpenDMTP_SOCK.1.0.3

#define DMTP_NAME                   APPLICATION_NAME

#if   defined(TRANSPORT_MEDIA_SOCKET)
#  define DMTP_TYPE                 "SOCK"  // - Socket communication
#elif defined(TRANSPORT_MEDIA_SERIAL)
#  define DMTP_TYPE                 "SER"   // - BlueTooth communication
#elif defined(TRANSPORT_MEDIA_GPRS)
#  define DMTP_TYPE                 "GPRS"  // - GPRS communication
#elif defined(TRANSPORT_MEDIA_FILE)
#  define DMTP_TYPE                 "FILE"  // - File event capture
#endif

#if !defined(DMTP_TYPE)
#  error DMTP_TYPE not defined.
#endif

// ----------------------------------------------------------------------------

#if defined(PROPERTY_FILE)
/* property file names */
static char propertyFile[96]  = {PROPERTY_FILE};
static char propertyCache[96] = {PROPERTY_CACHE};
static UInt32	save_property_schedule = 0;
pthread_mutex_t property_save_mutex = PTHREAD_MUTEX_INITIALIZER;
#endif

/* server host:port defined */
#if defined(TRANSPORT_MEDIA_SOCKET) || defined(TRANSPORT_MEDIA_GPRS)
static utBool hasServerHostPort = utTrue;
#endif
static char firmware_version[64] = DMTP_NAME;
static char software_release_version[32];
// ----------------------------------------------------------------------------
extern int start_watchdog(void);
extern int start_dmtp_alive(void);
extern int http_uploader(void *arg);
extern void report_mission_status(int mission_status, char *Reason);
static void cat_debuglog_file();
// define CUSTOM_EVENT_PACKETS to enable custom event packet
//#define CUSTOM_EVENT_PACKETS
#if defined(CUSTOM_EVENT_PACKETS) // {
// Custom defined event packets
#endif // } defined(CUSTOM_EVENT_PACKETS)
extern UInt32 transport_protocol;
extern UInt32 network_link_status;
extern pthread_cond_t	network_down_sema;
extern bool reboot_pending;
extern char logging_server_url[];
extern struct sigevent sev7;
extern struct itimerspec it7;
extern timer_t timer7;
// ----------------------------------------------------------------------------
static CommandError_t _cmdReset(int protoNdx, Key_t key, const UInt8 *data, int dataLen);

// ----------------------------------------------------------------------------
/* add the specified event to the queue */
static utBool _customAddSingleEvent(PacketPriority_t priority, ClientPacketType_t pktType, Event_t *er)
{
    /* add event packet */
	Packet_t pkt;
	struct tm tm1, *ptm; 
	utBool didAdd = evAddEventPacket(&pkt, priority, pktType, er);
	UInt32 pktSeq = pkt.sequence & 0xFF;
	time_t tstamp = (time_t)er->timestamp[0];
	/* display event */
	ptm = localtime_r(&tstamp, &tm1);
	if (er->speedKPH > 0.0) {
		print_debug("%02d/%02d/%4d %02d:%02d:%02d [Motion]%04X,%04X,%.5lf/%.5lf,%.1lf %02X\n", 
			ptm->tm_mon+1, ptm->tm_mday, ptm->tm_year+1900, 
			ptm->tm_hour, ptm->tm_min, ptm->tm_sec, 
			pktType, er->statusCode, 
			er->gpsPoint[0].latitude, er->gpsPoint[0].longitude, er->speedKPH, pktSeq);
	} else {
		print_debug("%02d/%02d/%4d %02d:%02d:%02d [Motion]%04X,%04X,%.5lf/%.5lf, 0 %02X\n", 
			ptm->tm_mon+1, ptm->tm_mday, ptm->tm_year+1900, 
			ptm->tm_hour, ptm->tm_min, ptm->tm_sec, 
			pktType, er->statusCode, 
			er->gpsPoint[0].latitude, er->gpsPoint[0].longitude, pktSeq);
	}

	/* return success */
	return didAdd;
}

/* add the specified event to the queue */
static utBool _customAddEventFtn(PacketPriority_t priority, ClientPacketType_t pktType, Event_t *er)
{

    /* no event? */
    if (!er) {
        return utFalse;
    }
    
    /* special event handling */
    // perform any special event handling here

    /* skip out if priority is PRIORITY_NONE */
    if (priority == PRIORITY_NONE) {
        // normally this will never occur, however, the client may choose to set the
        // priority on a special event to PRIORITY_NONE in order to be able to handle
        // it above and discard it here if necessary.
        return utFalse;
    }

    /* skip if the packet type is not a standard event packet */
    if (!pktIsEventPacket(pktType)) {
        // generally, this will only occur in an HOS environment
        return utFalse; // not a vehicle telemetry event
    }
    /* don't queue events if we don't have anywhere to send the data */
#if defined(TRANSPORT_MEDIA_SOCKET) || defined(TRANSPORT_MEDIA_GPRS)
    if (!hasServerHostPort) {
        return utFalse; // host:port not defined, don't queue event
    }
#endif // defined(TRANSPORT_MEDIA_SOCKET) || defined(TRANSPORT_MEDIA_GPRS)

    /* promote remaining priorities to PRIORITY_HIGH for file and serial transport */
#if defined(TRANSPORT_MEDIA_FILE) || defined(TRANSPORT_MEDIA_SERIAL)
    priority = PRIORITY_HIGH;
#endif

    /* add single event */
    return _customAddSingleEvent(priority, pktType, er);

}

// ----------------------------------------------------------------------------

/* property refresh, prior to a 'Get' */
static void _propertyPreGET(PropertyRefresh_t mode, Key_t key, UInt8 *args, int argLen)
{
    if (mode & PROP_REFRESH_GET) {
        switch (key) {
            case PROP_STATE_TIME: {
                // update property with clock time
                propSetUInt32(PROP_STATE_TIME, utcGetTimeSec());
            } break;
            case PROP_STATE_GPS: {
                // update property with current GPS location
                GPS_t lastGPS;
                GPSOdometer_t gpsOdom;
				if (gpsGetLastGPS(&lastGPS, INT_MAX)) {
					gpsOdom.point = lastGPS.point;
                	gpsOdom.meters = (UInt32)(odomGetDeviceDistanceMeters() + 0.5); // ROUND(X?)
                	propSetGPS(PROP_STATE_GPS, &gpsOdom);
				}
            } break;
            case PROP_STATE_GPS_DIAGNOSTIC: {
                // return GPS diagnostics (ie. GPS module health)
                int i;
                UInt32 *gpsStats = (UInt32*)gpsGetDiagnostics((GPSDiagnostics_t*)0);
                for (i = 0; i < (sizeof(GPSDiagnostics_t)/sizeof(UInt32)); i++) {
                    propSetUInt32AtIndex(PROP_STATE_GPS_DIAGNOSTIC, i, gpsStats[i]);
                }
            } break;
            case PROP_GEOF_COUNT: {
                // update property with number of GeoZone entries
#if defined(ENABLE_GEOZONE)
                propSetUInt32(PROP_GEOF_COUNT, (UInt32)geozGetGeoZoneCount());
#endif
            } break;
        }
    } 
}

/* property update, after a 'Set' */
static void _propertyPostSET(PropertyRefresh_t mode, Key_t key, UInt8 *args, int argLen)
{
    if (mode & PROP_REFRESH_SET) {
        switch (key) {
            case PROP_STATE_DEVICE_ID: {
                // change host/device name
                const char *s = propGetDeviceID(0); // propGetString(PROP_STATE_DEVICE_ID,"");
                if (!s || !*s) { 
                    s = DEVICE_ID; 
                    if (!s || !*s) {
                        s = propGetString(PROP_STATE_SERIAL, "");
                    }
                }
#if !defined(SECONDARY_SERIAL_TRANSPORT)
                // set bluetooth broadcast name
                osSetHostname(s);
#endif
                // save properties to save new ID?
            } break;
#if defined(SECONDARY_SERIAL_TRANSPORT)
            case PROP_STATE_DEVICE_BT: {
                // change bluetooth broadcast name
                const char *s = propGetDeviceID(1); // propGetString(PROP_STATE_DEVICE_BT,"");
                if (!s || !*s) { 
                    s = DEVICE_ID; 
                    if (!s || !*s) {
                        s = propGetString(PROP_STATE_SERIAL, "");
                    }
                }
                // set bluetooth broadcast name
                osSetHostname(s);
                // save properties to save new ID?
            } break;
#endif
        }
    }
}

// ----------------------------------------------------------------------------

/* status "ping" */
CommandError_t startupPingStatus(PacketPriority_t priority, StatusCode_t code, int ndx)
{
    ClientPacketType_t pktType = DEFAULT_EVENT_FORMAT;
    
    /* initialize an event in anticipation that the status code is valid */
    GPS_t gps;
    Event_t evRcd;
    evSetEventDefaults(&evRcd, code, 0L, gpsGetLastGPS(&gps,-1));

    /* check specified status code */
    if ((code == STATUS_LOCATION) || (code == STATUS_WAYMARK) || (code == STATUS_QUERY)) {
        if (ndx > 0) {
            // index was specified and was greater than '0'
            return COMMAND_INDEX;
        } else {
            // send current location
            _customAddEventFtn(priority, pktType, &evRcd);
            return COMMAND_OK;
        }
    } else
    if ((code >= STATUS_ODOM_0) && (code <= STATUS_ODOM_7)) {
        int odoNdx = code - STATUS_ODOM_0; // odometer index
        if ((ndx >= 0) && (ndx != odoNdx)) {
            // index was specified, but is not equal to the odomenter index
            return COMMAND_INDEX;
        } else {
            // send current odometer value
            // 'evRcd.odometerKM' is already be set above (in 'evSetEventDefaults')
            if (odoNdx == 0) {
                // vehicle distance (may need special handling
                evRcd.distanceKM = odomGetDeviceDistanceMeters() / 1000.0;
            } else {
                // driver, etc
                evRcd.distanceKM = odomGetDistanceMetersAtIndex(odoNdx) / 1000.0;
            }
            _customAddEventFtn(priority, pktType, &evRcd);
            return COMMAND_OK;
        }
    } else {
        // Other candidated:
        //  STATUS_ELAPSED_xx
        return COMMAND_STATUS;
    }
    
}

// ----------------------------------------------------------------------------
#if defined(TRANSPORT_MEDIA_FILE)
/* status "ping" [see PROP_CMD_STATUS_EVENT] */
static CommandError_t _cmdSendStatus(int protoNdx, Key_t key, const UInt8 *data, int dataLen)
{
    // 'protoNdx' contains the handle to the protocol transport

    /* parse arguments */
    UInt32 code32 = 0L;
    UInt32 ndx32  = 0L;
    int flds = binScanf(data, dataLen, "%2u%1u", &code32, &ndx32);
    if (flds < 1) {
        return COMMAND_ARGUMENTS;
    }
    StatusCode_t code = (StatusCode_t)code32;
    int ndx = (flds >= 2)? (int)ndx32 : -1;
    
    /* status ping */
    return startupPingStatus(PRIORITY_HIGH, code, ndx);
     
}

/* set digital output [see PROP_CMD_SET_OUTPUT] */
static CommandError_t _cmdSetOutput(int protoNdx, Key_t key, const UInt8 *data, int dataLen)
{
    // 'protoNdx' contains the handle to the protocol transport

    /* parse arguments */
    UInt32 ndx    = 0L;
    UInt32 state  = 0L;
    UInt32 duraMS = 0L;
    int flds = binScanf(data, dataLen, "%1u%1u%4u", &ndx, &state, &duraMS);
    if (flds < 2) {
        return COMMAND_ARGUMENTS;
    } else
    if (ndx > 15) {
        return COMMAND_INDEX;
    }
    
    /* implement setting output state here */
    // currently not supported
    return COMMAND_FEATURE_NOT_SUPPORTED;

}
#endif
/* explicitly save properties [see PROP_CMD_SAVE_PROPS] */
static CommandError_t _cmdSaveProperties(int protoNdx, Key_t key, const UInt8 *data, int dataLen)
{
	//schedule an order of saving property
	pthread_mutex_lock(&property_save_mutex);
	save_property_schedule = utTrue;
	pthread_mutex_unlock(&property_save_mutex);
	pthread_cond_signal(&network_down_sema); 
	return COMMAND_OK;
}

static CommandError_t _cmdUpdate(int protoNdx, Key_t key, const UInt8 *data, int dataLen)
{
	CommandError_t result = COMMAND_ARGUMENTS;
	char *args = (char *)data;

	if (trigger_update(args, strnlen(args, dataLen)) == 0)
		result = COMMAND_OK;
	return result;
}

static CommandError_t _cmdUploadLog(int protoNdx, Key_t key, const UInt8 *data, int dataLen)
{
	CommandError_t result = COMMAND_ARGUMENTS;
	unsigned char *args = (unsigned char *)data;
	char *colon, *slash;
	int log_size = SYSLOG_DEFAULT_SIZE; 

	if (dataLen < 1)
		return result;
	if (!isalnum(logging_server_url[0]))
		return result;
	slash = strchr(logging_server_url, '/');
	if (slash == NULL)
		return result;
	colon = strchr(logging_server_url, ':');
	if (colon != NULL  && slash - colon < 3)
		return result;
	if (args[1] != 0) {
		if (dataLen == 2)
			log_size = args[1];
	}
	if (enableSyslog(args[0], log_size) == 0)
		return COMMAND_OK;
	else
		return result;
}

#define DMTP_EXIT_FILE "/mnt/flash/titan-data/dmtp_exit"
#define DEBUGLOG_FILE_1 "/mnt/flash/titan-data/debuglog.txt"
#define DEBUGLOG_FILE_0 "/mnt/flash/titan-data/debuglog.txt.0"
#define UPLOAD_DEBUGLOG_FILE "/tmp/debuglog.txt"
#define FILE_NUM 	2

static struct upload_request_t debuglog_req = {.request_size = 0, .status = 0};
char upload_server_url[MAX_PAYLOAD_SIZE] = "216.171.106.21/dmtplog/log_receiver.cgi";

static void cat_debuglog_file() {
	struct stat filestat;
	uint32_t file_size;
	char *debuglog_file[FILE_NUM] = {DEBUGLOG_FILE_0, DEBUGLOG_FILE_1};
	char *cpy_buf;
	int fd_src, fd_dest;
	size_t read_len= 0, write_len = 0;
	int i;

	if (access(UPLOAD_DEBUGLOG_FILE, F_OK) != -1)  unlink(UPLOAD_DEBUGLOG_FILE);
	if((fd_dest = open(UPLOAD_DEBUGLOG_FILE, O_RDWR | O_CREAT | O_APPEND)) < 0) {
		perror("Open\n");
		exit(-1);
	}
	
	for(i=0; i < FILE_NUM; i++) {
		if (access(debuglog_file[i], F_OK) != -1) {
			if((fd_src = open(debuglog_file[i], O_RDONLY) ) < 0) {
				perror("Open\n");
				exit(-1);
			}
			
			if(stat(debuglog_file[i], &filestat) < 0) return;
			file_size = filestat.st_size;

			cpy_buf = (char *)malloc(file_size*sizeof(char));
			if (!cpy_buf) {
				perror("Malloc()\n");
				exit(-1);
			}
			memset(cpy_buf, 0, file_size*sizeof(char));
			
			read_len = read(fd_src, cpy_buf, file_size);
			printf("read size %d - ", read_len);
			if (read_len != file_size) 
				debuglog(LOG_INFO, "[ositechdmtp] WARNING: %s: %d/%d is READ\n", 
						debuglog_file[i], read_len, file_size);
			if (read_len > 0) {
				write_len = write(fd_dest, cpy_buf, read_len);
				printf("write size %d\n", write_len);
				if (write_len < read_len)
					debuglog(LOG_INFO, "[ositechdmtp] WARNING: %s: %d/%d is WRITE\n", 
						debuglog_file[i], write_len, read_len);
			}
			close(fd_src);
			free(cpy_buf);
			cpy_buf = NULL;
		}
	}
	close(fd_dest);
}
static CommandError_t _cmdUploadDebuglog(int protoNdx, Key_t key, const UInt8 *data, int dataLen)
//int upload_debuglog(void)
{
	char working_str[MAX_PAYLOAD_SIZE];
	char upload_degbulogfile[] = UPLOAD_DEBUGLOG_FILE;
	const char *s1, *s2;
	char *pt;
	time_t now;
	uint32_t l1, l2;
	uint32_t debuglogfile_size;
	uint32_t file_size;
	int status = -1;
	struct stat filestat;

	cat_debuglog_file();
	
	if(stat(upload_degbulogfile, &filestat) < 0)
		return COMMAND_UNAVAILABLE;

	file_size = filestat.st_size;
	printf("%s file size: %d\n", upload_degbulogfile, file_size);

//	if (network_link_status != 0)
//		return -1;
//	while (network_link_status != 0) sleep(1);
		
	if (debuglog_req.status != 0) {
		if ((strncmp(debuglog_req.upfile, upload_degbulogfile, strlen(upload_degbulogfile)) == 0)
									&& (debuglog_req.request_size == file_size))
			goto coming_back;
	}
	
	strncpy(working_str, upload_server_url, MAX_PAYLOAD_SIZE);  
	if ((s1 = strchr(working_str, '/')) == NULL)
		goto failure;
	if ((s2 = strchr(working_str, ':')) != NULL) {
		if ((s1 - s2) < 3)
			goto failure;
		strncpy(debuglog_req.port, s2 + 1, (s1 - s2 - 1));
	}
	else {
		snprintf(debuglog_req.port, 8, "443");
		s2 = s1;
	}
	strncpy(debuglog_req.host, working_str, (s2 - working_str));
	strncpy(debuglog_req.path, s1, sizeof(debuglog_req.path));
	s1 = propGetString(PROP_LOGGING_USER, "");
	strncpy(debuglog_req.user_pass, s1, MAX_ID_SIZE);
	s2 = propGetString(PROP_LOGGING_PASS, "");
	snprintf(debuglog_req.user_pass + strnlen(s1, MAX_ID_SIZE), MAX_ID_SIZE + 2, ":%s", s2);
	sprintf(working_str, "gzip -c %s > %s.gz", upload_degbulogfile, upload_degbulogfile);
	if ((status = system(working_str)) != 0)
		goto failure;
	snprintf(debuglog_req.upfile, sizeof(debuglog_req.upfile), "%s.gz", upload_degbulogfile);
coming_back:
	now = time(&now);
	s1 = propGetAccountID();
	l1 = strnlen(s1, MAX_ID_SIZE);
	strncpy(debuglog_req.query, s1, MAX_ID_SIZE);
	s2 = propGetDeviceID(0);
	snprintf(debuglog_req.query + l1, MAX_ID_SIZE + 1, "_%s", s2);
	l2 = strnlen(debuglog_req.query, sizeof(debuglog_req.query));
	sprintf(debuglog_req.query + l2, "_%08x.%s.log.gz", (unsigned)now, "debuglog");
	status = http_uploader(&debuglog_req);
failure:
	if (status == 0) {
		strncpy(working_str, debuglog_req.query, MAX_PAYLOAD_SIZE);
		pt = strstr(working_str, ".gz");
		sprintf(pt, " size:%d", file_size);
		report_mission_status(STATUS_CLIENT_LOGGING_OK, working_str);
		usleep(100000);
		unlink(debuglog_req.upfile);
		unlink(upload_degbulogfile);
	}
	else if (status != -1) {
		report_mission_status(STATUS_CLIENT_LOGGING_FAILED, debuglog_req.err_msg);
	}
	debuglog_req.request_size = file_size;
	debuglog_req.status = status;
	return COMMAND_OK;
}
// ----------------------------------------------------------------------------

/* initialize properties */
void startupPropInitialize(utBool startThread)
{
	char build_version[32];
	/* init properties */
	propInitialize(utTrue);
	/*protocol*/
	sprintf(build_version, "%d", DEFAULT_PROTO_VERSION);
	propInitFromString(PROP_STATE_PROTOCOL, build_version); 
	/* firmware */
#if defined(BUILD_VERSION)
	int major_ver = BUILD_VERSION / 100;
	int minor_ver = BUILD_VERSION % 100;
	sprintf(build_version, "%d.%d.%d.%d", PLATFORM_VERSION, FEATURE_VERSION, major_ver, minor_ver);
	strncat(firmware_version, build_version, 32);
#endif
//	propInitFromString(PROP_STATE_FIRMWARE, firmware_version); 
	char *host = (char *)malloc(HOSTSIZE*sizeof(char));
        host=osGetHostname(host, HOSTSIZE*sizeof(char));
        propInitFromString(PROP_STATE_FIRMWARE, host);
	propInitPointerFromString(PROP_LOGGING_SERVER, logging_server_url); 
#if defined(TRANSPORT_MEDIA_SERIAL)
	/* AccountID/DeviceID may be writable via BlueTooth transport (for OTA configuration) */
	propSetReadOnly(PROP_STATE_ACCOUNT_ID, utFalse);
	propSetReadOnly(PROP_STATE_DEVICE_ID , utFalse);
#endif

	if (startThread) {
		/* property initialization */
		propSetNotifyFtn(PROP_REFRESH_GET, &_propertyPreGET);
		propSetNotifyFtn(PROP_REFRESH_SET, &_propertyPostSET);
		/* set supporting commands */
		propSetCommandFtn(PROP_CMD_SAVE_PROPS, &_cmdSaveProperties);
		propSetCommandFtn(PROP_CMD_UPDATE, &_cmdUpdate);
		propSetCommandFtn(PROP_CMD_UPLOAD_LOG, &_cmdUploadLog);
		propSetCommandFtn(PROP_CMD_UPLOAD_DEBUGLOG, &_cmdUploadDebuglog);
		propSetCommandFtn(PROP_CMD_RESET, &_cmdReset);
#if defined(TRANSPORT_MEDIA_FILE)
		propSetCommandFtn(PROP_CMD_STATUS_EVENT, &_cmdSendStatus);
		propSetCommandFtn(PROP_CMD_SET_OUTPUT, &_cmdSetOutput);
#endif
	}
	
#if !defined(TRANSPORT_MEDIA_SOCKET)
	/* copyright */
	propSetString(PROP_STATE_COPYRIGHT              , COPYRIGHT);
#endif

	/* connection properties */
#if defined(TRANSPORT_MEDIA_FILE)
	// disable Duplex connection (not wanted when just writing to a file)
	propInitFromString(PROP_COMM_MAX_CONNECTIONS    , "1,0,0");     // no duplex connections, no quota
	propInitFromString(PROP_COMM_MIN_XMIT_DELAY     , "0");
	propInitFromString(PROP_COMM_MIN_XMIT_RATE      , "0");
	propInitFromString(PROP_COMM_MAX_DUP_EVENTS     , "0");
	propInitFromString(PROP_COMM_MAX_SIM_EVENTS     , "255");

#elif defined(TRANSPORT_MEDIA_SERIAL) // comm config
	// disable Simplex connection (not wanted when communicating over serial port)
	propInitFromString(PROP_COMM_SPEAK_FIRST        , "0");
	propInitFromString(PROP_COMM_FIRST_BRIEF        , "1");         // start with identification only
	propInitFromString(PROP_COMM_MAX_CONNECTIONS    , "1,1,0");     // no simplex connections, no quota
	propInitFromString(PROP_COMM_MIN_XMIT_DELAY     , "0");
	propInitFromString(PROP_COMM_MIN_XMIT_RATE      , "0");
	propInitFromString(PROP_COMM_MAX_XMIT_RATE      , "0");
	propInitFromString(PROP_COMM_MAX_DUP_EVENTS     , "10");
	propInitFromString(PROP_COMM_MAX_SIM_EVENTS     , "0");
	propInitFromString(PROP_COMM_ENCODINGS        , "0x6");       // HEX,Base64

#elif defined(TRANSPORT_MEDIA_GPRS) // comm config
	propInitFromString(PROP_COMM_MAX_CONNECTIONS    , "6,4,60");
	propInitFromString(PROP_COMM_MIN_XMIT_DELAY     , "60");        // seconds
	propInitFromString(PROP_COMM_MIN_XMIT_RATE      , "60");        // seconds
	propInitFromString(PROP_COMM_MAX_DUP_EVENTS     , "8");
	propInitFromString(PROP_COMM_MAX_SIM_EVENTS     , "4");

#endif

	/* motion parameters */
#if defined(TRANSPORT_MEDIA_FILE)
	propInitFromString(PROP_MOTION_EXCESS_SPEED     , "0.0");       // kph
	propInitFromString(PROP_MOTION_START            , "10.0");      // kph
	propInitFromString(PROP_MOTION_IN_MOTION        , "60");        // seconds (1 minutes)
	propInitFromString(PROP_MOTION_DORMANT_INTRVL   , "900");       // seconds (15 minutes)
	propInitFromString(PROP_MOTION_DORMANT_COUNT    , "0");         // unlimited dormant messages
#endif
	 //"propLoadProperties(...)" may also be used to load properties from aux storage
#if defined(PROPERTY_FILE)
	logDEBUG(LOGSRC,"Loading property config file: %s", propertyFile);
	propLoadProperties(propertyFile, 1);
	propLoadProperties(propertyCache, 1);
#endif
#if !defined(TRANSPORT_MEDIA_SOCKET)
	/* Serial # */
	// If a default serial# has not already been set, get the actual device serial#
	// (typically, a default serial number will not already be defined)
	const char *serNum = propGetString(PROP_STATE_SERIAL, "");
	if (!serNum || !*serNum) {
		// Serial number not yet defined
		propInitFromString(PROP_STATE_SERIAL, osGetSerialNumberID());
		serNum = propGetString(PROP_STATE_SERIAL, "");
	}

	/* Unique ID */
	UInt16 uniqLen = 0;
	const UInt8 *uniqId = propGetBinary(PROP_STATE_UNIQUE_ID, (UInt8*)0, (UInt16*)0, &uniqLen);
	if (!uniqId || (uniqLen < MIN_UNIQUE_SIZE)) {
		// Unique ID not yet defined
		UInt8 _uniqueID[] = UNIQUE_ID;
		if (sizeof(_uniqueID) >= MIN_UNIQUE_SIZE) {
			propSetBinary(PROP_STATE_UNIQUE_ID, _uniqueID, sizeof(_uniqueID)); // changed!
			uniqId = propGetBinary(PROP_STATE_UNIQUE_ID, (UInt8*)0, (UInt16*)0, &uniqLen);
		} else {
			// leave unique-id undefined
			//logDEBUG(LOGSRC,"Leaving Unique-ID undefined ...");
		}
	}

	/* Account ID (primary) */
	const char *acctId = propGetAccountID(); // propGetString(PROP_STATE_ACCOUNT_ID,"");
	if (!acctId || !*acctId) {
		// Account ID not yet defined
		const char *_acctId = ACCOUNT_ID;
		if (_acctId && *_acctId) {
			// set default account id
			propInitFromString(PROP_STATE_ACCOUNT_ID, _acctId);
		} else {
			// leave account-id undefined
			// NOTE: Leaving the account ID undefined allows the server to utilize 
			// the Device-ID in a similar fashion as a Unique-ID for identifying
			// a device record.
		}
		acctId = propGetAccountID();  // propGetString(PROP_STATE_ACCOUNT_ID,"");
	}
	
	/* Device ID (primary) */
	const char *devId = propGetDeviceID(0); // propGetString(PROP_STATE_DEVICE_ID, "");
	if (!devId || !*devId) {
		// Device-ID not yet defined
		const char *_devId = DEVICE_ID;
		if (_devId && *_devId) {
			// set default device id (if specified)
			logDEBUG(LOGSRC,"Setting default Device: %s", DEVICE_ID);
			propInitFromString(PROP_STATE_DEVICE_ID, _devId);
		} else
		if (serNum && *serNum) {
			// set to serial number (if available)
			logDEBUG(LOGSRC,"Setting Serial# Device: %s", serNum);
			propInitFromString(PROP_STATE_DEVICE_ID, serNum);
		} else {
			// no default device id, and no serial#, make up a name
			//propInitFromString(PROP_STATE_DEVICE_ID, "device");
			// or leave undefined
		}
		devId = propGetDeviceID(0); // propGetString(PROP_STATE_DEVICE_ID,"");
	}
#endif
   
	/* Device ID (secondary) */
//#if defined(SECONDARY_SERIAL_TRANSPORT)
//	const char *devBt = propGetDeviceID(1); // propGetString(PROP_STATE_DEVICE_BT, "");
//	if (!devBt || !*devBt) {
//		devBt = devId;
//	} else {
//		devId = devBt;
//	}
//#endif

#if !defined(TRANSPORT_MEDIA_SOCKET)
	/* reset hostname to device id */
	// This sets the bluetooth broadcast name on serial transport
	osSetHostname(devId);
#endif

	/* make sure all 'changed' flags are reset */
	propClearChanged();
	// Note that changing properties on the command line will set those properties to 'changed'.
}

/* save properties */
utBool startupSaveProperties(UInt32 save_type)
{
	utBool ret = utFalse;
	if (protocolIsOpen(0))
		sleep(10);
	if (save_type == 0)
		ret = propSaveProperties(propertyFile, save_type);
	else
		ret = propSaveProperties(propertyCache, save_type);
	if (ret)
		logINFO(LOGSRC, "Properties have been saved");
	return ret;
}

CommandError_t _cmdReset(int protoNdx, Key_t key, const UInt8 *data, int dataLen)
{
	UInt32 arg1 = 0;	
	if (dataLen > 0)
		arg1 = data[0];
	if (arg1 == 1) {
		pthread_mutex_lock(&property_save_mutex);
		save_property_schedule = PROP_SAVE_0;
		pthread_mutex_unlock(&property_save_mutex);
	}
	it7.it_value.tv_sec = NETWORK_RECEIVE_TIMEOUT;
	it7.it_value.tv_nsec = 0;
	it7.it_interval.tv_sec = BLOCK_RECEIVING_TIMEOUT;
	it7.it_interval.tv_nsec = 0;
	timer_settime(timer7, 0, &it7, NULL);
	reboot_pending = true;
	return COMMAND_OK;
}
// ----------------------------------------------------------------------------
void schedule_property_save(UInt32 save_type)
{
	pthread_mutex_lock(&property_save_mutex);
	save_property_schedule = save_type;
	pthread_mutex_unlock(&property_save_mutex);
}
	
/* main process loop callback */
void startupMainLoopCallback()
{
    // This function gets called about once per second from the main processing loop.
    // Other monitoring functions, etc. should go here.

    /* periodic gps module call */
    gpsModulePeriodic();

    /* Expired upload */
#if defined(ENBALE_UPLOAD)
    if (uploadIsExpired()) {
        // upload did not complete in allowed time
        uploadCancel();
    }
#endif
}

// ----------------------------------------------------------------------------
void property_maintenance(void)
{
#if defined(PROPERTY_FILE)
	UInt32 save_type = 0;

   	/* reload GPS parameters if related properties changed */
	if ((property_refresh_flag & PROP_REFRESH_GPS) && !(protocolIsOpen(0))) {
		gpsReloadParameter();
		property_refresh_flag &= ~PROP_REFRESH_GPS;
	} 
	pthread_mutex_lock(&property_save_mutex);
	if (save_property_schedule == 0) {
		pthread_mutex_unlock(&property_save_mutex);
		return;
	}
	else {
		save_type = (save_property_schedule == PROP_SAVE_0)? 0 : 1;
		pthread_mutex_unlock(&property_save_mutex);
   		/* save changed properties */
		if (startupSaveProperties(save_type)) {
			pthread_mutex_lock(&property_save_mutex);
			save_property_schedule = 0;
			pthread_mutex_unlock(&property_save_mutex);
		}
	}
#endif
}
// ----------------------------------------------------------------------------
// Main entry point

#define _BUILD_TIME_    ( __TIME__" " __DATE__)
#if (TIMEZONE==1)
#define _TIME_ZONE_	"EDT"
#else
#define _TIME_ZONE_	"EST"
#endif
char *get_build_version(void)
{
#if defined(BUILD_VERSION)
	int major_ver = BUILD_VERSION / 100;
	int minor_ver = BUILD_VERSION % 100;
	sprintf(software_release_version, "%d.%d.%d.%d", PLATFORM_VERSION, FEATURE_VERSION, major_ver, minor_ver);
#endif
	return software_release_version;
}
/* print the application header banner */
static void _printBanner()
{
	const char *header    = APPLICATION_DESCRIPTION;
	const char *build     = _BUILD_TIME_;
	const char *zone     = _TIME_ZONE_;
	const char *features  = APPLICATION_FEATURES + 1;
	const char *account   = propGetAccountID(); // propGetString(PROP_STATE_ACCOUNT_ID,"");
	const char *device    = propGetDeviceID(0);  // propGetString(PROP_STATE_DEVICE_ID,"");
	const char *OSInfo   = osGetOSInfo();
	char host[64] = { 0 };
#if defined(BUILD_VERSION)
	int major_ver = BUILD_VERSION / 100;
	int minor_ver = BUILD_VERSION % 100;
	if (strncmp(firmware_version, DMTP_NAME, 64) == 0) {
		sprintf(software_release_version, "%d.%d.%d.%d", PLATFORM_VERSION, FEATURE_VERSION, major_ver, minor_ver);
		strncat(firmware_version, software_release_version, 32);
			propInitFromString(PROP_STATE_FIRMWARE, firmware_version); 
	}
#endif
	char account_device[MAX_ID_SIZE * 2 + 2];
	strncpy(account_device, account, MAX_ID_SIZE);
	snprintf(account_device + strnlen(account, MAX_ID_SIZE), MAX_ID_SIZE + 2, "/%s", device); 
	osGetHostname(host, sizeof(host));
	print_debug("----------------------------------------------------------------\n");
	print_debug("%s\n", header);
	print_debug("Version: %s\n", firmware_version);
	print_debug("Built at: %s %s\n", build, zone);
	print_debug("Machine: %s\n", host);
	print_debug("Operating System: %s\n", OSInfo);
	print_debug("Queue Size: %u\n", (UInt32)EVENT_QUEUE_SIZE);
	print_debug("Feature: %s\n", features);
	print_debug("Account/Device: %s\n", account_device);
	print_debug("----------------------------------------------------------------\n");
}

/* print usage information */
static void _usage(const char *pgm)
{
	_printBanner();
	printf("Usage: \n");
	printf("    %s -h[elp]    Display this help and exit\n", pgm);
	printf("    %s -v[ersion] Display version and exit\n", pgm);
	printf("    [-deb[ug]]  Debug mode (default)\n");
	printf("    [-silent]   Silent mode (non-interactive)\n");
	printf("    [-log] <t>  Log On type t (1:normal, 2:tag, 4:cellular)\n");
	printf("    [-pfile] <property file> property file name\n");
	printf("    [-pp]       Print all properties\n");
	printf("    [-gps <port> [<bps>]]   Set GPS port and baud rate \n");
#if defined(TRANSPORT_MEDIA_SOCKET) || defined(TRANSPORT_MEDIA_GPRS)
	printf("    [-tcp <IP> [<port>]] Set Server IP and TCP port\n");
	printf("    [-udp <IP> [<port>]] Set Server IP and UDP port\n");
#endif
}

// main entry point
int startupDMTP(int argc, char *argv[], utBool startThread)
{
	/*PacketEncoding_t dftEncoding = DEFAULT_ENCODING; */
	const char *portName = (char*)0;
	const char *pgmName = (argc > 0)? argv[0] : "?";
	int argp;

	/* initialize file/stream i/o */
	ioInitialize(); 
	
	/* check for debug mode and logging 
	 These are performed first to allow debug logging to occur _before_ properties
	 are loaded and initialized. */
	for (argp = 1; argp < argc; argp++) {
		if (strncasecmp(argv[argp], "-deb", 4) == 0) {
			// -debug mode
			setDebugMode(utTrue);
		} else
		if (strncasecmp(argv[argp], "-silent", 7) == 0) {
			setDebugMode(utFalse);
		} else
		if (strncasecmp(argv[argp], "-pf", 3) == 0) {
			argp++;
			if ((argp < argc) && (*argv[argp] != '-')) {
				memset(propertyFile , 0, sizeof(propertyFile));
				strncpy(propertyFile, argv[argp], sizeof(propertyFile) - 1);
				logINFO(LOGSRC,"Property file set to '%s'" , propertyFile);
			}
		} 
	}
	/* check for existance of 'config' directory  */
	if (!ioIsDirectory(CONFIG_DIR)) {
		if (!ioExists(CONFIG_DIR)) {
			if (ioMakeDirs(CONFIG_DIR, utFalse)) {
				logDEBUG(LOGSRC,"Created CONFIG_DIR: %s ...", CONFIG_DIR);
			} else {
				logERROR(LOGSRC,"Unable to create directory: %s", CONFIG_DIR);
				return 1; // ExitProcess(1);
			}
		} else {
			logERROR(LOGSRC,"CONFIG_DIR is NOT a directory: %s", CONFIG_DIR);
			return 1; // ExitProcess(1);
		}
	}

	/* initialize properties */
	// since we need to use the property manager before the main run loop starts
	// we need to initialize the property manager here.
	
	startupPropInitialize(startThread);
	
	/* command line arguments */
	for (argp = 1; argp < argc; argp++) {
		if (strncasecmp(argv[argp], "-log", 4) == 0) {
			UInt32 log_type = 4;
			if (((argp + 1) < argc) && (*argv[argp + 1] != '-') && (isdigit(argv[argp + 1][0]))) {
				log_type = (UInt32)argv[argp + 1][0] - 0x30;
				argp++;
			}
			enableSyslog(log_type, SYSLOG_DEFAULT_SIZE);
		} else if (strncasecmp(argv[argp], "-gps", 4) == 0) {
			// -gps <port> [<baudrate>]
			argp++;
			if ((argp < argc) && (*argv[argp] != '-')) {
				propSetString(PROP_CFG_GPS_PORT, argv[argp]);
				// parse 'model', if specified (ie. 'garmin', etc).
				if (((argp + 1) < argc) && (*argv[argp + 1] != '-')) {
					argp++;
					propSetString(PROP_CFG_GPS_BPS, argv[argp]);
				}
			}
		} else 
#if defined(TRANSPORT_MEDIA_SOCKET)
		if (strncasecmp(argv[argp], "-tcp", 4) == 0) {
			// -server <host> [<port>]
			transport_protocol = TRANSPORT_TCP;
			argp++;
			if ((argp < argc) && (*argv[argp] != '-')) {
				propInitFromString(PROP_COMM_HOST, argv[argp]);
				// parse 'port', if specified
				if (((argp + 1) < argc) && (*argv[argp + 1] != '-')) {
					argp++;
					propInitFromString(PROP_COMM_PORT, argv[argp]);
				}
			} 
		}
		else
		if (strncasecmp(argv[argp], "-udp", 4) == 0) {
			// -server <host> [<port>]
			transport_protocol = TRANSPORT_UDP;
			argp++;
			if ((argp < argc) && (*argv[argp] != '-')) {
				propInitFromString(PROP_COMM_HOST, argv[argp]);
				// parse 'port', if specified
				if (((argp + 1) < argc) && (*argv[argp + 1] != '-')) {
					argp++;
					propInitFromString(PROP_COMM_PORT, argv[argp]);
				}
			} 
		}
#endif
#if defined(ENABLE_GEOZONE) && defined(GEOZ_INCL_FILE_UPLOAD)
		else 
		if (strEqualsIgnoreCase(argv[argp], "-geoz")) {
			// -geoz <geozFile>
			argp++;
			if ((argp < argc) && ((*argv[argp] != '-') || isdigit(*(argv[argp]+1)))) {
				const char *geoZoneFile = argv[argp];
				geozUploadGeozones(geoZoneFile);
			} else {
				logERROR(LOGSRC,"Missing geozone file ...");
				_usage(pgmName);
				return 3;
			}
			return 0; // ExitProcess(0);
		}
#endif
	} // "for"
	
	/* make sure we have a specified GPS port */
	portName = propGetString(PROP_CFG_GPS_PORT, "");
	if (!portName || !*portName) {
		logCRITICAL(LOGSRC,"Missing GPS port specification ...\n");
		_usage(pgmName);
		return 1;
	}
	
	/* make sure we have a specified 'server' serial port */
#if defined(TRANSPORT_MEDIA_SERIAL)
	portName = propGetString(PROP_CFG_XPORT_PORT, "");
	if (!portName || !*portName) {
		logCRITICAL(LOGSRC,"Missing serial port specification ...\n");
		_usage(pgmName);
		return 1;
	}
#endif
#if defined(TRANSPORT_MEDIA_GPRS)
	portName = propGetString(PROP_CFG_XPORT_PORT, "");
	if (!portName || !*portName) {
		logCRITICAL(LOGSRC,"Missing GPRS port specification ...\n");
		_usage(pgmName);
		return 1;
	}
#endif
#if defined(TRANSPORT_MEDIA_FILE)
	portName = propGetString(PROP_CFG_XPORT_PORT, "");
	if (!portName || !*portName) {
		logCRITICAL(LOGSRC,"Missing output file specification ...\n");
		_usage(pgmName);
		return 1;
	}
#endif

	// ***** Startup Initialization
	if (!startThread) {
		propPrintAll();
		propExportAll();
		return 0;
	}

	/* do we have a host:port defined? */
#if defined(TRANSPORT_MEDIA_SOCKET) || defined(TRANSPORT_MEDIA_GPRS)
	{
		const char *sockHost = propGetString(PROP_COMM_HOST, "");
		int sockPort = (int)propGetUInt32(PROP_COMM_PORT, 0L);
		hasServerHostPort = (sockHost && *sockHost && (sockPort > 0))? utTrue : utFalse;
		if (!hasServerHostPort) {
			//				"--------------------------------------------------------"
			logWARNING(LOGSRC,"*** No host:port defined, no events will be queued! ***");
		}
	}
#endif

	/* save startup time */
	// this must be called before any timers are defined
	utcMarkStartupTime();

	/* make sure properties have been initialized */
	// This must be called before any access is made to the property manager
	//propInitialize(utFalse); // only initialize here if not already initialized

	/* event queue initializer */
	// this must be called before event packects are defined, or events are generated
	evInitialize();
#if !defined(PROTOCOL_THREAD)
	acctInitialize();
#endif
	/* custom event record */
#if defined(CUSTOM_EVENT_PACKETS)
	// init custom event formats here (add before events can be generated)
#endif

	/* thread initializer */
	// this must be called before threads are created/started
	threadInitialize();
	
	/* set threaded logging (if so configured) */
	// (should be called before any threads are started)
	// (will return false if logging doesn't support running in a separate thread)
#if !defined(TARGET_CYGWIN)
	logStartThread();
#endif

	/* header */
	_printBanner();
//	system("run_led_on &");

	/* main loop initialization */
	// This initializes/starts all the component threads.
	mainLoopInitialize(&_customAddEventFtn);
	protocolInitialize(0); 
	/* property save interval */
#if defined(PROPERTY_SAVE_INTERVAL)
	// first property save expiration in FIRST_PROPERTY_SAVE_INTERVAL seconds
	lastSavePropertyTimer = utcGetTimer();
	//startupSaveProperties(); <-- will be saved soon
#endif

	/* other inits here */
	// Add other custom initializations here
	sleep(1);
	//upload_debuglog();
	if(propGetUInt32AtIndex(PROP_STATE_AP_DIAGNOSTIC, 0, 0)) 	ap_diagnostic_logStartThread();
	
	
	/* check and report if an exit happened at the last time ositechdmtp running */

	if (!access(DMTP_EXIT_FILE, F_OK)) {
		int fd;
		char message[90];
		size_t nread;
		
		if ((fd = open(DMTP_EXIT_FILE, O_RDONLY))) {
			nread = read(fd, message, sizeof(message));
			if (nread) {
				message[nread] = '\0';
				diagnostic_report(DIAGNOSTIC_MESSAGE, 0, message);
			}
		}
		unlink(DMTP_EXIT_FILE);
	}
	
	start_dmtp_alive();

	int pid;
	if ((pid = fork()) < 0) {
		perror("fork");
		return -1;
	} else if (pid == 0) {
		execl("/ositech/dmtp_monitor.sh", "dmtp_monitor.sh", "60", "3", (char *)0); 
		perror("execl");
	} else {

		network_manager();
	}
	 
	return 0;
}

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// 'main' entry point
int main(int argc, char *argv[])
{
	utBool go_further = utTrue;
	int i;
	
	if (argc == 2) {
		if (strcasestr(argv[1], "-h")!=NULL)  {
			_usage(argv[0]);
			go_further = utFalse;
		}
		else if (strcasestr(argv[1], "-v")!=NULL) {
			_printBanner();
			go_further = utFalse;
		}
	}
	if (!go_further)
		return 0;
	for (i = 1; i < argc; i++) {
		if (strcasestr(argv[i], "-pp")!=NULL)  {
			go_further = utFalse;
			break;
		}
	}
	if (go_further) {
		/*start watchdog*/
		start_watchdog();
		network_link_status = NETWORK_STATUS_INIT;
	}

	return startupDMTP(argc,argv, go_further);
}
// ----------------------------------------------------------------------------
