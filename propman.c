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
//  Property manager
//  Handles typed properties needed by the protocol.
//  Note: this module has not yet been made thread-safe.
// ---
// Change History:
//  2006/01/04  Martin D. Flynn
//     -Initial release
//  2006/01/17  Martin D. Flynn
//     -Changed property keyname strings
//     -Added PROP_GEOF_ARRIVE_DELAY/PROP_GEOF_DEPART_DELAY properties
//  2006/01/29  Martin D. Flynn
//     -Force KVA_NONDEFAULT attributes on properties loaded from a file
//     -Changed property save to default saving with key-name (rather than key-id)
//     -Added "propClearChanged()" function for clearing property 'changed' state.
//  2006/02/16  Martin D. Flynn
//     -Removed 'SAVE' attribute from config properties that should remain static.
//  2006/04/06  Martin D. Flynn
//     -Changed communication property defaults
//  2006/05/07  Martin D. Flynn
//     -Added PROP_CMD_GEOF_ADMIN property.
//     -Added PROP_GEOF_COUNT property.
//     -Added PROP_GEOF_VERSION property.
//  2006/07/13  Martin D. Flynn
//     -Updated DFT_PROTOCOL_VERSION to reflect protocol document version change.
//  2007/01/28  Martin D. Flynn
//     -WindowsCE port
//     -Added several new properties (see 'props.h')
//     -Added 'propSetSave' function to override property 'save' attribute
//     -Replaced DFT_PROTOCOL_VERSION with PROTOCOL_VERSION (set in 'defaults.h')
//     -Changed the following properties to store odometer values in 1 meter units:
//      PROP_GPS_ACCURACY, PROP_GPS_DISTANCE_DELTA, 
//      PROP_ODOMETER_#_VALUE, PROP_ODOMETER_#_LIMIT
//     -Changed the following properties to store elapsed time values in seconds:
//      PROP_ELAPSED_#_VALUE, PROP_ELAPSED_#_LIMIT
//     -Property key lookups now use a binary search.
//  2007/04/28  Martin D. Flynn
//     -Change default PROP_COMM_HOST to an empty string ("").  While using "localhost"
//      as the default for debugging purposes, it doesn't make sense in an embedded
//      client.  This value should be explicitly defined/set in the 'props.conf' file.
// ----------------------------------------------------------------------------

#include "defaults.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <ctype.h>
#include <strings.h>
#include <math.h>
#include <errno.h>

#include "log.h"

#include "strtools.h"
#include "gpstools.h"
#include "utctools.h"
#include "bintools.h"
#include "io.h"

#include "propman.h"

#include "packet.h"
#include "statcode.h"
#include "float_point_handle.h"

// ----------------------------------------------------------------------------

#if defined(TARGET_WINCE)
// poor-mans 'rint' (WinCE doesn't seem to have it's own 'rint' function)
#define RINT(X)         (UInt32)((X) + 0.5)
#else
#define RINT(X)         rint(X)
#endif

// ----------------------------------------------------------------------------

#define PROP_LOCK       // implement property manager locking
#define PROP_UNLOCK     // implement property manager unlocking

// ----------------------------------------------------------------------------
// These are the default settings for specific property values that may need
// to be customized for a particular client application.  During client app
// initialization the appropriate calls to the following function should be made.
//    "propInitFromString(<PopKey>, <StringValue>);"
// should be called.

#define DFT_COMM_HOST		"173.239.129.34" 
#define DFT_COMM_HOSTB		"www.polairus.com" 
#define DFT_COMM_PORT		"31000"	
#define DFT_ACCESS_PIN		"0x3132333435363738" 

// 'true' to save with key name, 'false' to save with key code
#define PROP_SAVE_KEY_NAME      utTrue

// ----------------------------------------------------------------------------
#define HR                      (KVA_HIDDEN | KVA_READONLY)
#define HS                      (KVA_HIDDEN | KVA_SAVE)
#define WO                      (KVA_WRITEONLY)
#define RO                      (KVA_READONLY)
#define SAVE                    (KVA_SAVE)

#define UInt32_to_Double(U,T)   ((double)(U) / pow(10.0,(double)KVT_DEC(T)))
#define Double_to_UInt32(D,T)   ((UInt32)RINT((D) * pow(10.0,(double)KVT_DEC(T))))

// ----------------------------------------------------------------------------
// Property definition table
// Notes:
//  - The values in the 'name' column are arbitrary, and are just suggestions in the
//    OpenDMTP protocol.  As such, they may change at any time without notice.  If you
//    rely on the specified property 'name' in your property files, make sure you double 
//    check your property files against the names specified here at the time you plan to 
//    upgrade your code to the newest version.

UInt32 property_refresh_flag = 0;
extern void protocolScheduleResetURL(void);

static utBool     binarySearchOK = utFalse; // false, until verified

static KeyValue_t properties[] = {

#ifdef CUSTOM_PROPERTIES
#include "custprop.h"
#endif

    //KeyCode                      Name               Type                   Attr       Ndx  Init
    // --- local serial port configuration
//	{PROP_CFG_XPORT_PORT,	"cfg.xpo.port",		KVT_STRING,	RO,	 1,  ""},
//	{PROP_CFG_XPORT_BPS,	"cfg.xpo.bps",		KVT_UINT32,	RO,	 1,  ""},
//	{PROP_CFG_XPORT_DEBUG,	"cfg.xpo.debug",	KVT_BOOLEAN,RO,	 1,  "0"},
	{PROP_CFG_GPS_PORT,		"cfg.gps.port",		KVT_STRING,	RO,	 1,  "ttyS3"},
	{PROP_CFG_GPS_BPS,		"cfg.gps.bps",		KVT_UINT32,	RO,	 1,  "9600"},
	{PROP_CFG_GPS_MODEL,	"cfg.gps.model",	KVT_STRING,	RO,	 1,  "sirf"},
//	{PROP_CFG_GPS_DEBUG,	"cfg.gps.debug",	KVT_BOOLEAN,RO,	 1,  "0"},
//	{PROP_CFG_SERIAL0_PORT,	"cfg.sp0.port",		KVT_STRING,	RO,	 1,  ""},
//	{PROP_CFG_SERIAL0_BPS,	"cfg.sp0.bps",		KVT_UINT32,	RO,	 1,  ""},
//	{PROP_CFG_SERIAL0_DEBUG,"cfg.sp0.debug",	KVT_BOOLEAN,RO,	 1,  "0"},
//	{PROP_CFG_SERIAL1_PORT,	"cfg.sp1.port",		KVT_STRING,	RO,	 1,  ""},
//	{PROP_CFG_SERIAL1_BPS,	"cfg.sp1.bps",		KVT_UINT32,	RO,	 1,  ""},
//	{PROP_CFG_SERIAL1_DEBUG,"cfg.sp1.debug",	KVT_BOOLEAN,RO,	 1,  "0"},
//	{PROP_CFG_SERIAL2_PORT,	"cfg.sp2.port",		KVT_STRING,	RO,	 1,  ""},
//	{PROP_CFG_SERIAL2_BPS,	"cfg.sp2.bps",		KVT_UINT32,	RO,	 1,  ""},
//	{PROP_CFG_SERIAL2_DEBUG,"cfg.sp2.debug",	KVT_BOOLEAN,RO,	 1,  "0"},
//	{PROP_CFG_SERIAL3_PORT,	"cfg.sp3.port",		KVT_STRING,	RO,	 1,  ""},
//	{PROP_CFG_SERIAL3_BPS,	"cfg.sp3.bps",		KVT_UINT32,	RO,	 1,  ""},
//	{PROP_CFG_SERIAL3_DEBUG,"cfg.sp3.debug",	KVT_BOOLEAN,RO,	 1,  "0"},
	// ---iBox properties
	{PROP_IBOX_PORT, "ibox.port",	KVT_STRING,	SAVE,	1,	"/dev/ttyS1"},
	{PROP_IBOX_MID, "ibox.mid",	KVT_UINT32,	SAVE,	1,	"147"},
	{PROP_IBOX_96_REQUEST, "ibox.96.request",	KVT_UINT32,	SAVE,	2,	"0,1"},
	{PROP_IBOX_168_REQUEST, "ibox.168.request",	KVT_UINT32,	SAVE,	2,	"0,1"},
	{PROP_IBOX_171_REQUEST, "ibox.171.request",	KVT_UINT32,	SAVE,	2,	"0,1"},
	{PROP_IBOX_200_REQUEST, "ibox.200.request",	KVT_UINT32,	SAVE,	2,	"0,1"},
	{PROP_IBOX_201_REQUEST , "ibox.201.request",	KVT_UINT32,	SAVE,	2,	"0,1"},
	{PROP_IBOX_202_REQUEST , "ibox.202.request",	KVT_UINT32,	SAVE,	2,	"0,1"},
	{PROP_IBOX_203_REQUEST , "ibox.203.request",	KVT_UINT32,	SAVE,	2,	"0,1"},
	{PROP_IBOX_207_REQUEST , "ibox.207.request",	KVT_UINT32,	SAVE,	2,	"0,1"},
	{PROP_IBOX_234_REQUEST , "ibox.234.request",	KVT_UINT32,	SAVE,	2,	"0,1"},
	{PROP_IBOX_235_REQUEST , "ibox.235.request",	KVT_UINT32,	SAVE,	2,	"0,1"},
	{PROP_IBOX_243_REQUEST , "ibox.243.request",	KVT_UINT32,	SAVE,	2,	"0,1"},
	{PROP_IBOX_246_REQUEST , "ibox.246.request",	KVT_UINT32,	SAVE,	2,	"0,1"},
	{PROP_IBOX_247_REQUEST , "ibox.247.request",	KVT_UINT32,	SAVE,	2,	"0,1"},
	{PROP_IBOX_205_COMMAND, "ibox.205.command",	KVT_STRING,	WO,	1,	""},
	{PROP_IBOX_205_COMMAND_TIMEOUT, "ibox.205.cmd.timeout",	KVT_UINT32,	SAVE, 1,	"1"},
	{PROP_IBOX_206_COMMAND, "ibox.206.command",	KVT_STRING,	WO,	1,	""},
	{PROP_IBOX_206_COMMAND_TIMEOUT, "ibox.206.cmd.timeout",	KVT_UINT32,	SAVE, 1,	"1"},
	{PROP_IBOX_208_COMMAND, "ibox.208.command",	KVT_STRING,	WO,	1,	""},
	{PROP_IBOX_208_COMMAND_TIMEOUT, "ibox.208.cmd.timeout",	KVT_UINT32,	SAVE, 1,	"1"},
	// ---RFID properties
	{PROP_RFID_READER_ENABLE,	"rfid.reader.enable",	KVT_UINT8,	SAVE,	1,	"1"},
	{PROP_RFID_READER_PORT,	"rfid.reader.port",	KVT_STRING,	SAVE,	1,	"ttyS1"},
	{PROP_RFID_READER_BPS,	"rfid.reader.bps",	KVT_UINT32,	SAVE,	1,	"115200"},
	{PROP_RFID_COMPANY_ID_RANGE,	"rfid.company.id.range",	KVT_UINT32,	SAVE,	2,	"1,0xFFFFFF"},
	{PROP_RFID_PRIMARY_ID_DIVISOR,	"rfid.primary.id.divisor",	KVT_UINT32,	SAVE,	2,	"1,0"},
	{PROP_RFID_IN_MOTION,			"rfid.in.motion",			KVT_UINT32,	SAVE,	2,	"6,60"},
	{PROP_RFID_PRIMARY_ID,			"rfid.primary.id",			KVT_UINT32,	SAVE,	1,	"0"},
	{PROP_RFID_PRIMARY_ID_RANGE,	"rfid.primary.id.range",	KVT_UINT32,	SAVE,	5,	"0,0,30,45,120"},
	{PROP_RFID_LOCK_ID_RANGE,		"rfid.lock.id.range",		KVT_UINT32,	SAVE,	5,	"0,0,10,15,40"},
	{PROP_RFID_PRIMARY_RSSI_TIMER,	"rfid.primary.rssi.timer",	KVT_UINT32,	SAVE,	1,	"120"},
	{PROP_RFID_PRIMARY_RSSI,		"rfid.primary.rssi",		KVT_UINT8,	SAVE,	2,	"10,100"},
	{PROP_RFID_SWITCH_ID_RANGE,		"rfid.switch.id.range",		KVT_UINT32,	SAVE,	5,	"0,0,30,45,120"},
	{PROP_RFID_CARGO_MIN_RSSI,		"rfid.cargo.rssi",			KVT_UINT8,	SAVE,	1,	"0"},
	{PROP_RFID_CARGO_ID_RANGE,		"rfid.cargo.id.range",		KVT_UINT32,	SAVE,	5,	"0,0,30,45,120"},
	{PROP_RFID_CARGO_SAMPLE_MODE,	"rfid.cargo.sample.mode",	KVT_UINT8,	SAVE,	1,	"0"},
	{PROP_RFID_HIGHTEMP_ID_RANGE,	"rfid.hightemp.id.range",		KVT_UINT32,	SAVE,	5,	"0,0,30,45,120"},
	{PROP_RFID_BATTERY_LIFE_MAX,	"rfid.battery.runtime.max",	KVT_UINT8,	SAVE,	1,	"255"},	
	{PROP_RFID_LOCK_REPORT_INTRVL,	"rfid.lock.rpt.intrvl",		KVT_UINT32,	SAVE,	3,	"60,60,1"},
	{PROP_RFID_HIGHTEMP_REPORT_INTRVL,	"rfid.hightemp.update.intrvl",	KVT_UINT32,	SAVE,	1,	"60"},
	{PROP_RFID_BATTERY_ALARM_INTRVL,"rfid.battery.alarm.intrvl",KVT_UINT32,	SAVE,	1,	"3600"},
	{PROP_RFID_SWITCH_REPORT_INTRVL,"rfid.switch.rpt.intrvl",	KVT_UINT32,SAVE,	2,	"0,0"},
	{PROP_RFID_CARGO_REPORT_INTRVL,	"rfid.in.range.update.intrvl",KVT_UINT32,SAVE,	1,	"30"},
	{PROP_RFID_HIGHTEMP_ID_RANGE_2,	"rfid.hightemp.id.range_2",		KVT_UINT32,	SAVE,	2,	"0,0"},
	{PROP_RFID_MOTION_ID_RANGE, 		"rfid.motion.id.range",		KVT_UINT32,	SAVE,	5,	"0,0,30,45,120"},
	{PROP_RFID_MOTION_REPORT_RATE,	"rfid.motion.rpt.rate",		KVT_UINT32,SAVE,		1,	"0"},
	{PROP_RFID_SENSOR_ID_RANGE, 		"rfid.sensor.id.range",		KVT_UINT32,	SAVE,	5,	"0,0,30,45,120"},
	{PROP_RFID_SENSOR_REPORT_INTRVL,	"rfid.sensor.rpt.intrvl",		KVT_UINT32,SAVE,		1,	"30"},
	{PROP_RFID_HUMIDITY_ID_RANGE,	"rfid.humidity.id.range", 	KVT_UINT32,	SAVE,	5,	"0,0,30,45,120"},
	{PROP_RFID_HUMIDITY_REPORT_INTRVL,	"rfid.humidity.rpt.intrvl",		KVT_UINT32,SAVE,		1,	"30"},
//================================================================================
	// QDAC properties
	{PROP_QDAC_UNKNOWN_TAG, "qdac.unknown.tag", KVT_UINT32, SAVE, 3, "30,45,120"},
	{PROP_QDAC_UNKNOWN_REPORT_INTRVL, "qdac.unknown.rpt.intrvl", KVT_UINT32, SAVE, 1, "30"},
	{PROP_QDAC_LOWTEMP_TAG, "qdac.lowtemp.tag", KVT_UINT32, SAVE, 3, "30,45,120"},
	{PROP_QDAC_LOWTEMP_REPORT_INTRVL, "qdac.lowtemp.rpt.intrvl", KVT_UINT32, SAVE, 1, "30"},
	{PROP_QDAC_PRIMARY_ID, "qdac.primary.id", KVT_UINT32, SAVE, 2, "0,0"},
	{PROP_QDAC_ZONE_0, "qdac.zone.0", KVT_UINT32, SAVE, 2, "0,0"},
	{PROP_QDAC_ZONE_1, "qdac.zone.1", KVT_UINT32, SAVE, 2, "0,0"},
	{PROP_QDAC_ZONE_2, "qdac.zone.2", KVT_UINT32, SAVE, 2, "0,0"},
	{PROP_QDAC_ZONE_3, "qdac.zone.3", KVT_UINT32, SAVE, 2, "0,0"},
	{PROP_QDAC_ZONE_4, "qdac.zone.4", KVT_UINT32, SAVE, 2, "0,0"},
	{PROP_QDAC_ZONE_5, "qdac.zone.5", KVT_UINT32, SAVE, 2, "0,0"},
	{PROP_QDAC_ZONE_6, "qdac.zone.6", KVT_UINT32, SAVE, 2, "0,0"},
	{PROP_QDAC_ZONE_7, "qdac.zone.7", KVT_UINT32, SAVE, 2, "0,0"},
	{PROP_QDAC_GFORCESENSOR_TAG, "qdac.gforcesensor.tag", KVT_UINT32,	SAVE,	3,	"30,45,120"},
	{PROP_QDAC_GFORCESENSOR_RPT_THRESHOLD, "qdac.gforcesensor.rpt.threshold", KVT_UINT32, SAVE, 1, "11"},
	{PROP_QDAC_CURRENTSENSOR_TAG, "qdac.currentsensor.tag", KVT_UINT32,	SAVE,	3,	"30,45,120"},
	{PROP_QDAC_CURRENTSENSOR_REPORT_INTRVL, "qdac.currentsensor.rpt.intrvl", KVT_UINT32, SAVE, 1, "30"},
	{PROP_QDAC_TMP_TYPE1, 	"qdac.tmp.type1", 	KVT_UINT32,	SAVE, 1,	"0"}, 							
	{PROP_QDAC_TMP_TYPE2,	"qdac.tmp.type2", 	KVT_UINT32,	SAVE, 1,	"0"},						
	{PROP_QDAC_TMP_TAG,	"qdac.tmp.tag",	KVT_UINT32, SAVE, 3, "30,45,120"},	
	{PROP_QDAC_TMP_REPORT_INTRVL,	"qdac.tmp.rpt.intrvl", KVT_UINT32, SAVE, 1, "30"},	
	{PROP_QDAC_SWITCH_TAG, "qdac.switch.tag", KVT_UINT32, SAVE, 3, "30,45,120"},
	{PROP_QDAC_SWITCH_REPORT_INTRVL, "qdac.switch.rpt.intrvl", KVT_UINT32, SAVE, 1, "30"},
	{PROP_QDAC_PRESSURE_TAG, "qdac.pressure.tag", KVT_UINT32, SAVE, 3, "30,45,120"},
	{PROP_QDAC_PRESSURE_REPORT_INTRVL, "qdac.pressure.rpt.intrvl",  KVT_UINT32, SAVE, 1, "30"},
	{PROP_QDAC_INIT_WAIT_SEC, "qdac.init.wait.sec", KVT_UINT32, SAVE, 1, "6"},
	{PROP_QDAC_RESET_MIN, "qdac.reset.min", KVT_UINT32, SAVE, 1, "3"},
	{PROP_QDAC_FIRMWARE, "qdac.firm",  KVT_STRING,	RO,	 1,  ""},
//================================================================================
    // --- miscellaneous commands
	{PROP_CMD_SAVE_PROPS,	"cmd.saveprops",	KVT_COMMAND,WO,	 1,  0},
	{PROP_CMD_UPDATE,		"cmd.update",		KVT_COMMAND,WO,	 1,  0},
	{PROP_CMD_UPLOAD_LOG,	"cmd.upload.log",	KVT_COMMAND,WO,	 1,  0},
	{PROP_CMD_UPLOAD_DEBUGLOG,		"cmd.upload.debuglog",		KVT_COMMAND,WO,	 1,  0},
//	{PROP_CMD_STATUS_EVENT,	"cmd.status",		KVT_COMMAND,WO,	 1,  0},
//	{PROP_CMD_SET_OUTPUT,	"cmd.output",		KVT_COMMAND,WO,	 1,  0},
	{PROP_CMD_RESET,		"cmd.reset",		KVT_COMMAND,WO,	 1,  0},
	
	
    // --- retained state properties
	{PROP_STATE_PROTOCOL,	"sta.proto",		KVT_UINT8,	SAVE, 1, "3"},
	{PROP_STATE_FIRMWARE,	"sta.firm",			KVT_STRING,	RO,	 1,  ""},
//	{PROP_STATE_COPYRIGHT,	"sta.copyright",	KVT_STRING,	RO,	 1,  ""},
	{PROP_STATE_SERIAL,		"sta.serial",		KVT_STRING,	HR,	 1,  ""},
	{PROP_STATE_UNIQUE_ID,	"sta.uniq",			KVT_BINARY,	RO,	30,  ""},
	{PROP_STATE_ACCOUNT_ID,	"sta.account",		KVT_STRING,	SAVE,	 1,  " "},
	{PROP_STATE_DEVICE_ID,	"sta.device",		KVT_STRING,	SAVE,	 1,  "default0"},
#if defined(SECONDARY_SERIAL_TRANSPORT)
//	{PROP_STATE_DEVICE_BT,	"sta.device.bt",	KVT_STRING,	SAVE,	 1,  ""},
#endif
//	{PROP_STATE_USER_ID,	"sta.user",			KVT_STRING,	SAVE,	 1,  ""},
//	{PROP_STATE_USER_TIME,	"sta.user.time",	KVT_UINT32,	RO|SAVE, 1,  "0"},
	{PROP_STATE_TIME,		"sta.time",			KVT_UINT32,	RO,	 1,  "0"},
	{PROP_STATE_GPS,		"sta.gpsloc",		KVT_GPS,	RO|SAVE, 1,  ""}, 
	{PROP_STATE_GPS_DIAGNOSTIC,"sta.gpsdiag",	KVT_UINT32,	RO, 5,  "0,0,0,0,0"}, 
//	{PROP_STATE_SAVED_EVENTS,"sta.saved.events",KVT_UINT32,	HS, 2,  "0,0"}, 
//	{PROP_STATE_DIAGNOSTIC,	"sta.diagnostic",	KVT_UINT32,	SAVE,	3,  "0,65535,0"}, 
	{PROP_STATE_DIAGNOSTIC,	"sta.diagnostic",	KVT_UINT32,	HS,	3,  "0,65535,0"},
	{PROP_STATE_AP_DIAGNOSTIC,		"sta.ap.diagnostic",		KVT_UINT32, SAVE,	 1,  "0"},
	{PROP_STATE_DIAGNOSTIC_LEVEL,	"sta.diagnostic.level",	KVT_UINT32, SAVE,	 1,  "1"},
	{PROP_STATE_NETWORK_DEBUG,	"sta.network.debug",	KVT_UINT32, SAVE,	 2,  "0,0"},
	{PROP_STATE_BOOTUP_REPORT, "sta.bootup.report",	KVT_UINT32, SAVE,	 1,  "1"},
	{PROP_STATE_CELL_TEMP_RPT_SETUP, "sta.cell.temp.rpt.setup",	KVT_UINT32, SAVE,	 3,  "30,-30,70"},
	{PROP_STATE_STUCK_TIMEOUT,	"sta.lib.stuck.timeout",	KVT_UINT32, SAVE,	 1,  "900"},
	{PROP_STATE_CHECKNETWORK_TIMEOUT,	"sta.lib.checknetwork.timeout",	KVT_UINT32, SAVE,	 1,  "79"}, 
	{PROP_STATE_NETWORK_CHECK_WAIT_TIMES, "sta.network.check.wait.times", KVT_UINT32, SAVE,	 1,  "3"},
	{PROP_STATE_RTS_CHECK, "sta.rts.check", KVT_UINT32, SAVE,	 2,  "0,1"}, // second arg is in mintue	
	{PROP_STATE_iBOX_ENABLE, "sta.ibox.enable", KVT_UINT32, SAVE, 1, "0"},
	{PROP_STATE_ALIVE_INTRVL, "sta.alive.intrvl", KVT_UINT32, SAVE, 1, "30"},
	
	
    // --- Communication protocol properties
	{PROP_COMM_SPEAK_FIRST,	"com.first",		KVT_BOOLEAN,RO,	 1,  "1"},
	{PROP_COMM_FIRST_BRIEF,	"com.brief",		KVT_BOOLEAN,RO,	 1,  "0"},
	{PROP_COMM_SAVE_RATE,	"com.saverate",		KVT_UINT16,	SAVE,	1,	"0"},
	{PROP_COMM_MAX_DELAY,	"com.maxdelay",		KVT_UINT16,	SAVE,	1,	"24"},
	{PROP_COMM_MIN_XMIT_RATE,"com.minrate",		KVT_UINT32,	SAVE,	1,	"60"},
	{PROP_COMM_MAX_XMIT_RATE,"com.maxrate",		KVT_UINT32,	SAVE,	1,	"600"},
	{PROP_COMM_MAX_DUP_EVENTS,"com.maxduplex",	KVT_UINT8,	SAVE,	1,	"255"},
	{PROP_COMM_MAX_SIM_EVENTS,"com.maxsimplex",	KVT_UINT8,	SAVE,	1,	"0"},
	{PROP_COMM_NET_IDLE_MINUTES,	"com.idle.minutes",		KVT_UINT16,	SAVE,	1,  "30"},
	{PROP_COMM_MTU,			"com.mtu.size",		KVT_UINT32,	SAVE,	1,  "1500"},
	{PROP_COMM_UDP_TIMER,	"com.udp.cfg",		KVT_UINT32,	SAVE,	2,  "50,3"},
    // --- Communication connection properties
	{PROP_COMM_HOST_B,		"com.hostb",		KVT_STRING,	SAVE,	 1,  DFT_COMM_HOSTB},
	{PROP_COMM_PORT_B,		"com.portb",		KVT_UINT16,	SAVE,	 1,  DFT_COMM_PORT},
	{PROP_COMM_POWER_SAVING,"com.power.saving",	KVT_UINT32,	SAVE,	 1,  "0"},
	{PROP_COMM_HOST,		"com.host",			KVT_STRING,	SAVE,	 1,  DFT_COMM_HOST},
	{PROP_COMM_PORT,		"com.port",			KVT_UINT16,	SAVE,	 1,  DFT_COMM_PORT},
//	{PROP_COMM_DNS_1,		"com.dns1",			KVT_STRING,	SAVE,	 1,  "31000"},
//	{PROP_COMM_DNS_2,		"com.dns2",			KVT_STRING,	SAVE,	 1,  ""},
//	{PROP_COMM_CONNECTION,	"com.connection",	KVT_STRING,	SAVE,	 1,  ""},
//	{PROP_COMM_APN_NAME,	"com.apnname",		KVT_STRING,	SAVE,	 1,  ""},
//	{PROP_COMM_APN_SERVER,	"com.apnserv",		KVT_STRING,	SAVE,	 1,  ""},
//	{PROP_COMM_APN_USER,	"com.apnuser",		KVT_STRING,	SAVE,	 1,  ""},
//	{PROP_COMM_APN_PASSWORD,"com.apnpass",		KVT_STRING,	SAVE,	 1,  ""},
//	{PROP_COMM_APN_PHONE,	"com.apnphone",		KVT_STRING,	SAVE,	 1,  ""}, // "*99***1#"
//	{PROP_COMM_APN_SETTINGS,"com.apnsett",		KVT_STRING,	SAVE,	 1,  ""},
//	{PROP_COMM_MIN_SIGNAL,	"com.minsignal",	KVT_INT16,	SAVE,	 1,  "7"},
//	{PROP_COMM_ACCESS_PIN,	"com.pin",			KVT_BINARY,	SAVE,	 8,  DFT_ACCESS_PIN},

    // --- Packet/Data format properties
	{PROP_COMM_CUSTOM_FORMATS,"com.custfmt",	KVT_UINT8,	SAVE,	 1,  "0"},
	{PROP_COMM_ENCODINGS,	"com.encodng",		KVT_UINT8,	SAVE,	 1,  "0"},
	{PROP_COMM_BYTES_READ,	"com.rdcnt",		KVT_UINT32,	SAVE,	 1,  "0"},
	{PROP_COMM_BYTES_WRITTEN,"com.wrcnt",		KVT_UINT32,	SAVE,	 1,  "0"},

    // ---log upload properties 
	{PROP_LOGGING_SERVER,	"com.logging.server",	KVT_POINTER,SAVE, 1, ""},
	{PROP_LOGGING_USER,	"com.logging.user",		KVT_STRING,SAVE, 1, "dmtpclient"},
	{PROP_LOGGING_PASS,	"com.logging.pass",		KVT_STRING,SAVE, 1, "clientunderdebug"},

    // --- GPS properties
	{PROP_GPS_POWER_SAVING,	"gps.power.saving",	KVT_UINT32,	SAVE,	2,  "0,0"},
	{PROP_GPS_SAMPLE_RATE,	"gps.smprate",		KVT_UINT16,	SAVE,	1,  "10"},
	{PROP_GPS_AQUIRE_WAIT,	"gps.aquwait",		KVT_UINT16,	SAVE,	1,  "60"},
	{PROP_GPS_EXPIRATION,	"gps.expire",		KVT_UINT16,	SAVE,	1,  "1200"},
	{PROP_GPS_CLOCK_DELTA,	"gps.updclock",		KVT_UINT32, SAVE,	2,  "10,1"},
	{PROP_GPS_LOST_COUNTER, "gps.lost.counter", KVT_UINT32,	 SAVE,	1,  "5"}, 
//	{PROP_GPS_ACCURACY,	"gps.accuracy",			KVT_UINT16,	SAVE,	1,  "0"},
	{PROP_GPS_MIN_SPEED,	"gps.minspd",		KVT_UINT16|KVT_DEC(1), SAVE, 1, "1.7"},
	{PROP_GPS_DISTANCE_DELTA,"gps.dstdelt",		KVT_UINT32,	SAVE,	1,  "150"},
	{PROP_GPS_HIGH_ACCURACY,"gps.high.accuracy",		KVT_UINT32,	SAVE,	1,  "1"},
    // --- GeoZone properties
	{PROP_CMD_GEOF_ADMIN,	"gf.admin",		KVT_COMMAND,	WO,	 1,  0},
	{PROP_GEOF_COUNT,	"gf.count",		KVT_UINT16,	RO,	 1,  "0"},
	{PROP_GEOF_VERSION,	"gf.version",		KVT_STRING,	SAVE,	 1,  ""},
	{PROP_GEOF_ARRIVE_DELAY,"gf.arr.delay",		KVT_UINT32,	SAVE,	 1,  "30"}, 
	{PROP_GEOF_DEPART_DELAY,"gf.dep.delay",		KVT_UINT32,	SAVE,	 1,  "10"}, 
	{PROP_GEOF_CURRENT,	"gf.current",		KVT_UINT32,	SAVE,	 1,  "0"}, 

    // --- GeoCorr properties
//	{PROP_CMD_GEOC_ADMIN,	"gc.admin",		KVT_COMMAND,	WO,	 1,  0},
//	{PROP_GEOC_ACTIVE_ID,	"gc.active",		KVT_UINT32,	SAVE,	 1,  "0"}, 
//	{PROP_GEOC_VIOLATION_INTRVL,"gc.vio.rate",	KVT_UINT16,	SAVE,	 1,  "300"}, 
//	{PROP_GEOC_VIOLATION_COUNT,"gc.vio.cnt",	KVT_UINT16,	SAVE,	 1,  "0"}, 

    // --- Motion properties
	{PROP_MOTION_START_TYPE,"mot.start.type",	KVT_UINT8,	SAVE,	 1,  "0"},
	{PROP_MOTION_START,	"mot.start",		KVT_UINT16|KVT_DEC(1),SAVE,1,"10"},
	{PROP_MOTION_IN_MOTION,	"mot.inmotion",		KVT_UINT16,	SAVE,	 1,  "60"},
	{PROP_MOTION_STOP,	"mot.stop",		KVT_UINT16,	SAVE,	 1,  "180"},
	{PROP_MOTION_STOP_TYPE,	"mot.stop.type",	KVT_UINT8,	SAVE,	 1,  "1"},
	{PROP_MOTION_DORMANT_INTRVL,"mot.dorm.rate",	KVT_UINT32,	SAVE,	 1,  "180"},
	{PROP_MOTION_DORMANT_COUNT,"mot.dorm.cnt",	KVT_UINT16,	SAVE,	 1,  "1"},
	{PROP_MOTION_EXCESS_SPEED,"mot.exspeed",	KVT_UINT16|KVT_DEC(1),SAVE, 1,"150"},
//	{PROP_MOTION_MOVING_INTRVL,"mot.moving",	KVT_UINT16,	SAVE,	 1,  "0"},

    // --- Odometer properties
	{PROP_ODOMETER_0_VALUE,	"odo.0.value",		KVT_UINT32,	SAVE,	 1,  "0"},
	{PROP_ODOMETER_1_VALUE,	"odo.1.value",		KVT_UINT32,	SAVE,	 1,  "0"},
	{PROP_ODOMETER_2_VALUE,	"odo.2.value",		KVT_UINT32,	SAVE,	 1,  "0"},
	{PROP_ODOMETER_3_VALUE,	"odo.3.value",		KVT_UINT32,	SAVE,	 1,  "0"},
	{PROP_ODOMETER_4_VALUE,	"odo.4.value",		KVT_UINT32,	SAVE,	 1,  "0"},
	{PROP_ODOMETER_5_VALUE,	"odo.5.value",		KVT_UINT32,	SAVE,	 1,  "0"},
	{PROP_ODOMETER_6_VALUE,	"odo.6.value",		KVT_UINT32,	SAVE,	 1,  "0"},
	{PROP_ODOMETER_7_VALUE,	"odo.7.value",		KVT_UINT32,	SAVE,	 1,  "0"},
	{PROP_ODOMETER_0_LIMIT,	"odo.0.limit",		KVT_UINT32,	SAVE,	 1,  "0"},
	{PROP_ODOMETER_1_LIMIT,	"odo.1.limit",		KVT_UINT32,	SAVE,	 1,  "0"},
	{PROP_ODOMETER_2_LIMIT,	"odo.2.limit",		KVT_UINT32,	SAVE,	 1,  "0"},
	{PROP_ODOMETER_3_LIMIT,	"odo.3.limit",		KVT_UINT32,	SAVE,	 1,  "0"},
	{PROP_ODOMETER_4_LIMIT,	"odo.4.limit",		KVT_UINT32,	SAVE,	 1,  "0"},
	{PROP_ODOMETER_5_LIMIT,	"odo.5.limit",		KVT_UINT32,	SAVE,	 1,  "0"},
	{PROP_ODOMETER_6_LIMIT,	"odo.6.limit",		KVT_UINT32,	SAVE,	 1,  "0"},
	{PROP_ODOMETER_7_LIMIT,	"odo.7.limit",		KVT_UINT32,	SAVE,	 1,  "0"},
	{PROP_ODOMETER_0_GPS,	"odo.0.gps",		KVT_GPS,	RO|SAVE, 1,  "0"},
	{PROP_ODOMETER_1_GPS,	"odo.1.gps",		KVT_GPS,	RO|SAVE, 1,  "0"},
	{PROP_ODOMETER_2_GPS,	"odo.2.gps",		KVT_GPS,	RO|SAVE, 1,  "0"},
	{PROP_ODOMETER_3_GPS,	"odo.3.gps",		KVT_GPS,	RO|SAVE, 1,  "0"},
	{PROP_ODOMETER_4_GPS,	"odo.4.gps",		KVT_GPS,	RO|SAVE, 1,  "0"},
	{PROP_ODOMETER_5_GPS,	"odo.5.gps",		KVT_GPS,	RO|SAVE, 1,  "0"},
	{PROP_ODOMETER_6_GPS,	"odo.6.gps",		KVT_GPS,	RO|SAVE, 1,  "0"},
	{PROP_ODOMETER_7_GPS,	"odo.7.gps",		KVT_GPS,	RO|SAVE, 1,  "0"},

    // --- Digital input properties
//	{PROP_INPUT_STATE,	"inp.state",		KVT_UINT32,	RO,	 1,  "0"}, // DON"T SAVE
//	{PROP_INPUT_CONFIG_0,	"inp.0.conf",		KVT_UINT32,	SAVE,	 2,  "0,0"},
//	{PROP_INPUT_CONFIG_1,	"inp.1.conf",		KVT_UINT32,	SAVE,	 2,  "0,0"},
//	{PROP_INPUT_CONFIG_2,	"inp.2.conf",		KVT_UINT32,	SAVE,	 2,  "0,0"},
//	{PROP_INPUT_CONFIG_3,	"inp.3.conf",		KVT_UINT32,	SAVE,	 2,  "0,0"},
//	{PROP_INPUT_CONFIG_4,	"inp.4.conf",		KVT_UINT32,	SAVE,	 2,  "0,0"},
//	{PROP_INPUT_CONFIG_5,	"inp.5.conf",		KVT_UINT32,	SAVE,	 2,  "0,0"},
//	{PROP_INPUT_CONFIG_6,	"inp.6.conf",		KVT_UINT32,	SAVE,	 2,  "0,0"},
//	{PROP_INPUT_CONFIG_7,	"inp.7.conf",		KVT_UINT32,	SAVE,	 2,  "0,0"},

    // --- Digitaloutput properties
//	{PROP_OUTPUT_CONFIG_0,	"out.0.conf",		KVT_UINT32,	SAVE,	 2,  "0,0"},
//	{PROP_OUTPUT_CONFIG_1,	"out.1.conf",		KVT_UINT32,	SAVE,	 2,  "0,0"},
//	{PROP_OUTPUT_CONFIG_2,	"out.2.conf",		KVT_UINT32,	SAVE,	 2,  "0,0"},
//	{PROP_OUTPUT_CONFIG_3,	"out.3.conf",		KVT_UINT32,	SAVE,	 2,  "0,0"},
//	{PROP_OUTPUT_CONFIG_4,	"out.4.conf",		KVT_UINT32,	SAVE,	 2,  "0,0"},
//	{PROP_OUTPUT_CONFIG_5,	"out.5.conf",		KVT_UINT32,	SAVE,	 2,  "0,0"},
//	{PROP_OUTPUT_CONFIG_6,	"out.6.conf",		KVT_UINT32,	SAVE,	 2,  "0,0"},
//	{PROP_OUTPUT_CONFIG_7,	"out.7.conf",		KVT_UINT32,	SAVE,	 2,  "0,0"},

    // --- Elapsed time properties
//	{PROP_ELAPSED_0_VALUE,	"ela.0.value",		KVT_UINT32,	SAVE,	 1,  "0"},
//	{PROP_ELAPSED_1_VALUE,	"ela.1.value",		KVT_UINT32,	SAVE,	 1,  "0"},
//	{PROP_ELAPSED_2_VALUE,	"ela.2.value",		KVT_UINT32,	SAVE,	 1,  "0"},
//	{PROP_ELAPSED_3_VALUE,	"ela.3.value",		KVT_UINT32,	SAVE,	 1,  "0"},
//	{PROP_ELAPSED_4_VALUE,	"ela.4.value",		KVT_UINT32,	SAVE,	 1,  "0"},
//	{PROP_ELAPSED_5_VALUE,	"ela.5.value",		KVT_UINT32,	SAVE,	 1,  "0"},
//	{PROP_ELAPSED_6_VALUE,	"ela.6.value",		KVT_UINT32,	SAVE,	 1,  "0"},
//	{PROP_ELAPSED_7_VALUE,	"ela.7.value",		KVT_UINT32,	SAVE,	 1,  "0"},
//	{PROP_ELAPSED_0_LIMIT,	"ela.0.limit",		KVT_UINT32,	SAVE,	 1,  "0"},
//	{PROP_ELAPSED_1_LIMIT,	"ela.1.limit",		KVT_UINT32,	SAVE,	 1,  "0"},
//	{PROP_ELAPSED_2_LIMIT,	"ela.2.limit",		KVT_UINT32,	SAVE,	 1,  "0"},
//	{PROP_ELAPSED_3_LIMIT,	"ela.3.limit",		KVT_UINT32,	SAVE,	 1,  "0"},
//	{PROP_ELAPSED_4_LIMIT,	"ela.4.limit",		KVT_UINT32,	SAVE,	 1,  "0"},
//	{PROP_ELAPSED_5_LIMIT,	"ela.5.limit",		KVT_UINT32,	SAVE,	 1,  "0"},
//	{PROP_ELAPSED_6_LIMIT,	"ela.6.limit",		KVT_UINT32,	SAVE,	 1,  "0"},
//	{PROP_ELAPSED_7_LIMIT,	"ela.7.limit",		KVT_UINT32,	SAVE,	 1,  "0"},

    // --- Generic sensor properties
//	{PROP_UNDERVOLTAGE_LIMIT,"bat.limit",		KVT_UINT32,	SAVE,	 1,  "0"},
//	{PROP_SENSOR_CONFIG_0,	"sen.0.conf",		KVT_UINT32,	SAVE,	 2,  "0,0"},
//	{PROP_SENSOR_CONFIG_1,	"sen.1.conf",		KVT_UINT32,	SAVE,	 2,  "0,0"},
//	{PROP_SENSOR_CONFIG_2,	"sen.2.conf",		KVT_UINT32,	SAVE,	 2,  "0,0"},
//	{PROP_SENSOR_CONFIG_3,	"sen.3.conf",		KVT_UINT32,	SAVE,	 2,  "0,0"},
//	{PROP_SENSOR_CONFIG_4,	"sen.4.conf",		KVT_UINT32,	SAVE,	 2,  "0,0"},
//	{PROP_SENSOR_CONFIG_5,	"sen.5.conf",		KVT_UINT32,	SAVE,	 2,  "0,0"},
//	{PROP_SENSOR_CONFIG_6,	"sen.6.conf",		KVT_UINT32,	SAVE,	 2,  "0,0"},
//	{PROP_SENSOR_CONFIG_7,	"sen.7.conf",		KVT_UINT32,	SAVE,	 2,  "0,0"},
//	{PROP_SENSOR_RANGE_0,	"sen.0.range",		KVT_UINT32,	SAVE,	 2,  "0,0"},
//	{PROP_SENSOR_RANGE_1,	"sen.1.range",		KVT_UINT32,	SAVE,	 2,  "0,0"},
//	{PROP_SENSOR_RANGE_2,	"sen.2.range",		KVT_UINT32,	SAVE,	 2,  "0,0"},
//	{PROP_SENSOR_RANGE_3,	"sen.3.range",		KVT_UINT32,	SAVE,	 2,  "0,0"},
//	{PROP_SENSOR_RANGE_4,	"sen.4.range",		KVT_UINT32,	SAVE,	 2,  "0,0"},
//	{PROP_SENSOR_RANGE_5,	"sen.5.range",		KVT_UINT32,	SAVE,	 2,  "0,0"},
//	{PROP_SENSOR_RANGE_6,	"sen.6.range",		KVT_UINT32,	SAVE,	 2,  "0,0"},
//	{PROP_SENSOR_RANGE_7,	"sen.7.range",		KVT_UINT32,	SAVE,	 2,  "0,0"},

    // --- Temperature properties
//	{PROP_TEMP_SAMPLE_INTRVL,"tmp.smprate",		KVT_UINT32,	SAVE,	 2,  "0,0"},
	{PROP_TEMP_REPORT_INTRVL,"tmp.rptrate",		KVT_UINT32,	SAVE,	 4,  "60,30,120,999"},
//	{PROP_TEMP_CONFIG_0,	"tmp.0.conf",		KVT_INT16,	SAVE,	 2,  "0,0"},
//	{PROP_TEMP_CONFIG_1,	"tmp.1.conf",		KVT_INT16,	SAVE,	 2,  "0,0"},
//	{PROP_TEMP_CONFIG_2,	"tmp.2.conf",		KVT_INT16,	SAVE,	 2,  "0,0"},
//	{PROP_TEMP_CONFIG_3,	"tmp.3.conf",		KVT_INT16,	SAVE,	 2,  "0,0"},
//	{PROP_TEMP_CONFIG_4,	"tmp.4.conf",		KVT_INT16,	SAVE,	 2,  "0,0"},
//	{PROP_TEMP_CONFIG_5,	"tmp.5.conf",		KVT_INT16,	SAVE,	 2,  "0,0"},
//	{PROP_TEMP_CONFIG_6,	"tmp.6.conf",		KVT_INT16,	SAVE,	 2,  "0,0"},
//	{PROP_TEMP_CONFIG_7,	"tmp.7.conf",		KVT_INT16,	SAVE,	 2,  "0,0"},
	{PROP_TEMP_RANGE_0,	"tmp.0.range",		KVT_INT16|KVT_DEC(1),SAVE,2,"-50,50"},
	{PROP_TEMP_RANGE_1,	"tmp.1.range",		KVT_INT16|KVT_DEC(1),SAVE,2,"-50,50"},
	{PROP_TEMP_RANGE_2,	"tmp.2.range",		KVT_INT16|KVT_DEC(1),SAVE,2,"-50,50"},
	{PROP_TEMP_RANGE_3,	"tmp.3.range",		KVT_INT16|KVT_DEC(1),SAVE,2,"-50,50"},
	{PROP_TEMP_RANGE_4,	"tmp.4.range",		KVT_INT16|KVT_DEC(1),SAVE,2,"-50,50"},
	{PROP_TEMP_RANGE_5,	"tmp.5.range",		KVT_INT16|KVT_DEC(1),SAVE,2,"-50,50"},
	{PROP_TEMP_RANGE_6,	"tmp.6.range",		KVT_INT16|KVT_DEC(1),SAVE,2,"-50,50"},
	{PROP_TEMP_RANGE_7,	"tmp.7.range",		KVT_INT16|KVT_DEC(1),SAVE,2,"-50,50"},
};

#define PROP_COUNT      (sizeof(properties) / sizeof(properties[0]))

// ----------------------------------------------------------------------------
// forward definitions

static void _propInitKeyValueFromString(KeyValue_t *kv, const char *s, utBool internal);

// ----------------------------------------------------------------------------
// Property table accessor functions

/* get KeyValue_t entry at specified index */
KeyValue_t *propGetKeyValueAt(int ndx)
{
    if ((ndx >= 0) && (ndx < PROP_COUNT)) {
        return &properties[ndx];
    }
    return (KeyValue_t*)0;
}

/* get KeyValue entry for specified key */
KeyValue_t *propGetKeyValueEntry(Key_t key)
{
    if (binarySearchOK) {
        // binary search (PROPERTY KEYS MUST BE IN ASCENDING ORDER!)
        int s = 0, e = PROP_COUNT;
        for (;s < e;) {
            int b = (e + s) / 2;
            KeyValue_t *kv = &properties[b];
            if (kv->key == key) {
                return kv;
            } else
            if (key < kv->key) {
                e = b;
            } else {
                s = b + 1;
            }
        }
    } else {
        // linear search
        int i;
        for (i = 0; i < PROP_COUNT; i++) {
            KeyValue_t *kv = &properties[i];
            if (kv->key == key) {
                return kv;
            }
        }
    }
    logWARNING(LOGSRC,"Key not found: 0x%04X", key);
    return (KeyValue_t*)0;
}

/* get KeyValue entry for specified key */
KeyValue_t *propGetKeyValueEntryByName(const char *keyName)
{
    // could stand some optimizing
    int i;
    for (i = 0; i < PROP_COUNT; i++) {
        KeyValue_t *kv = &properties[i];
        if (strEqualsIgnoreCase(kv->name,keyName)) {
            return kv;
        }
    }
    return (KeyValue_t*)0;
}

// ----------------------------------------------------------------------------

static KeyData_t *_propGetData(KeyValue_t *kv)
{
	return &(kv->data);
}

static UInt16 _propGetDataCapacity(KeyValue_t *kv)
{
	return sizeof(kv->data);
}

// ----------------------------------------------------------------------------
// Property table flag modification

// set KVA_READONLY state
utBool propSetReadOnly(Key_t key, utBool readOnly)
{
    KeyValue_t *kv = propGetKeyValueEntry(key);
    if (kv) {
        if (readOnly) {
            kv->attr |= KVA_READONLY;
        } else {
            kv->attr &= ~KVA_READONLY;
        }
        return utTrue;
    }
    return utFalse;
}

// set KVA_SAVE state
utBool propSetSave(Key_t key, utBool save)
{
    KeyValue_t *kv = propGetKeyValueEntry(key);
    if (kv) {
        if (save) {
            kv->attr |= KVA_SAVE;
        } else {
            kv->attr &= ~KVA_SAVE;
        }
        return utTrue;
    }
    return utFalse;
}

// ----------------------------------------------------------------------------
/*set up large string*/
static utBool _propSetPointerToString(KeyValue_t *kv, const char *str, int Len)
{
	if (kv->data.ptr.b != NULL && Len > 0) {
		int len;
		strncpy(kv->data.ptr.b, str, Len); 
		if (Len < MAX_PAYLOAD_SIZE)
			memset((unsigned char *)kv->data.ptr.b + Len, 0, MAX_PAYLOAD_SIZE - Len);
		len = strnlen((char*)kv->data.ptr.b, MAX_PAYLOAD_SIZE);
		kv->data.ptr.l = len;
		kv->lenNdx = 1; 
		kv->dataSize = len;
		kv->attr |= KVA_NONDEFAULT; 
		return utTrue;
	}
	else
		return utFalse;
}
void propInitPointerFromString(Key_t key, char *str)
{
	int len;
    KeyValue_t *kv = propGetKeyValueEntry(key);
    if (kv && KVT_IS_POINTER(kv->type)) {
		len = strnlen(str, MAX_PAYLOAD_SIZE);
		kv->data.ptr.l = len;
		kv->data.ptr.b = str;
		kv->lenNdx = 1; // a single string is defined
		kv->dataSize = len;
	}
}
utBool propSetPointerToString(Key_t key, char *str)
{
    KeyValue_t *kv = propGetKeyValueEntry(key);
    if (!kv || !KVT_IS_POINTER(kv->type))
		return utFalse;
	return _propSetPointerToString(kv, str, strnlen(str, MAX_PAYLOAD_SIZE));
}
char * propGetPointerToString(Key_t key)
{
    KeyValue_t *kv = propGetKeyValueEntry(key);
    if (kv && KVT_IS_POINTER(kv->type))
		return (char *)kv->data.ptr.b;
	return NULL;
}
// ----------------------------------------------------------------------------
// type conversion functions
/* encode GPS value into byte array */
static int _propEncodeGPS(GPSOdometer_t *gps, FmtBuffer_t *dest)
{
    if (BUFFER_DATA(dest) && (BUFFER_DATA_SIZE(dest) >= 4)) {
        binFmtPrintf(dest, "%4u", (UInt32)(gps?gps->fixtime:0L));
        int destLen = BUFFER_DATA_SIZE(dest);
        if ((destLen >= 16) || ((destLen >= 12) && (destLen < 14))) {
            gpsPointEncode8(BUFFER_DATA(dest), (gps?&(gps->point):0));
            binAppendFmtField(dest, 8, 'g');
            binAdvanceFmtBuffer(dest, 8);
            if (destLen >= 16) {
                binFmtPrintf(dest, "%4u", (UInt32)(gps?gps->meters:0L));
            }
        } else
        if ((destLen >= 14) || ((destLen >= 10) && (destLen < 12))) {
            gpsPointEncode6(BUFFER_DATA(dest), (gps?&(gps->point):0));
            binAppendFmtField(dest, 6, 'g');
            binAdvanceFmtBuffer(dest, 6);
            if (destLen >= 14) {
                binFmtPrintf(dest, "%4u", (UInt32)(gps?gps->meters:0L));
            }
        }
        return BUFFER_DATA_LENGTH(dest);
    } else {
        // invalid data length
        return -1;
    }
}

/* decode GPS value from byte array */
static int _propDecodeGPS(GPSOdometer_t *gps, const UInt8 *data , int dataLen)
{
    // Valid lengths:
    //   10/14 - low resolution GPS fix
    //   12/16 - high resolution GPS fix
    if (gps && data && (dataLen >= 4)) {
        gps->fixtime = binDecodeInt32(data, 4, utFalse);
        gpsPointClear(&(gps->point));
        gps->meters = 0L;
        int len = 4;
        if ((dataLen == 10) || (dataLen == 14)) {
            gpsPointDecode6(&(gps->point), &data[len]);
            len += 6;
            if (dataLen == 14) {
                gps->meters = binDecodeInt32(&data[len], 4, utFalse);
                len += 4;
            }
        } else
        if ((dataLen == 12) || (dataLen == 16)) {
            gpsPointDecode8(&(gps->point), &data[len]);
            len += 8;
            if (dataLen == 16) {
                gps->meters = binDecodeInt32(&data[len], 4, utFalse);
                len += 4;
            }
        }
        return len;
    } else {
        return -1;
    }
}

// ----------------------------------------------------------------------------
// Property value refresh/change notification

static void (*propFtn_PropertyGet)(PropertyRefresh_t mode, Key_t key, UInt8 *args, int argLen);
static void (*propFtn_PropertySet)(PropertyRefresh_t mode, Key_t key, UInt8 *args, int argLen);

static void _propRefresh(PropertyRefresh_t mode, KeyValue_t *kv, UInt8 *args, int argLen)
{
    if (kv) {
        if (propFtn_PropertyGet && (mode & PROP_REFRESH_GET)) {
            (*propFtn_PropertyGet)(PROP_REFRESH_GET, kv->key, args, argLen);
        }
        if (propFtn_PropertySet && (mode & PROP_REFRESH_SET)) {
            (*propFtn_PropertySet)(PROP_REFRESH_SET, kv->key, (UInt8*)0, 0);
        }
    }
}

void propSetNotifyFtn(PropertyRefresh_t mode, void (*ftn)(PropertyRefresh_t,Key_t,UInt8*,int))
{
    if (mode & PROP_REFRESH_GET) {
        propFtn_PropertyGet = ftn;
    }
    if (mode & PROP_REFRESH_SET) {
        propFtn_PropertySet = ftn;
    }
}

// ----------------------------------------------------------------------------
// Property table command handler support

utBool propSetCommandFtn(Key_t key, CommandError_t (*cmd)(int pi, Key_t key, const UInt8 *data, int dataLen))
{
    KeyValue_t *kv = propGetKeyValueEntry(key);
    if (kv && (KVT_TYPE(kv->type) == KVT_COMMAND)) {
        KeyData_t *keyDataBuf = _propGetData(kv);
        keyDataBuf->cmd = cmd;
        kv->lenNdx      = 1;
        kv->dataSize    = sizeof(keyDataBuf->cmd); // sizeof(void*)
        return utTrue;
    }
    return utFalse;
}

// This is a sample command type handler
int _sampleCommandHandler(Key_t key, const void *data, int dataLen)
{
    logDEBUG(LOGSRC,"\nExecute command for key 0x%04X ...\n", key);
    return 0; // bytes used from 'data'
}

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// This section implements functions used to access property values from within code.

/* return maximum index size */
Int16 propGetIndexSize(Key_t key)
{
    KeyValue_t *kv = propGetKeyValueEntry(key);
    return kv? (Int16)kv->maxNdx : -1;
}

// ----------------------------------------------------------------------------

/* get a 32-bit value for the specified KeyValue entry */
static utBool _propGetUInt32Value(KeyValue_t *kv, int ndx, UInt32 *val)
{
    if (!val) {
        // no place to put value
        return utFalse;
    } else
    if ((ndx < 0) || (ndx >= kv->lenNdx)) {
        // invalid index
        return utFalse;
    } else
    if (KVT_IS_UINT(kv->type)) {
        KeyData_t *keyDataBuf = _propGetData(kv);
        UInt16     keyDataSize = _propGetDataCapacity(kv);
        if (keyDataSize >= ((ndx + 1) * sizeof(keyDataBuf->i[0]))) {
            *val = keyDataBuf->i[ndx];
            return utTrue;
        } else {
            // insufficient size for index
            //logWARNING(LOGSRC,"Insufficient size: %d 0x%04X\n", ndx, keyDataSize);
            return utFalse;
        }
    } else {
        // unsupported type
        //logWARNING(LOGSRC,"Unsupported type: 0x%04X\n", kv->type);
        return utFalse;
    }
}

/* set a 32-bit value for the specified KeyValue entry */
static utBool _propSetUInt32Value(KeyValue_t *kv, int ndx, UInt32 *val)
{
    if (!val) {
        // no specified value
        return utFalse;
    } else
    if ((ndx < 0) || (ndx >= kv->maxNdx)) {
        // invalid index
        return utFalse;
    } else
    if (KVT_IS_UINT(kv->type)) {
        KeyData_t *keyDataBuf = _propGetData(kv);
        UInt16     keyDataSize = _propGetDataCapacity(kv);
        if (keyDataSize >= ((ndx + 1) * sizeof(keyDataBuf->i[0]))) {
            keyDataBuf->i[ndx] = *val;
            if (kv->lenNdx <= ndx) { 
                kv->lenNdx = ndx + 1;
                kv->dataSize = kv->lenNdx * KVT_UINT_SIZE(kv->type);
            }
            /*kv->attr |= KVA_NONDEFAULT;  */
            kv->attr |= KVA_CHANGED; // set changed
            return utTrue;
        } else {
            // insufficient size
            return utFalse;
        }
    } else {
        // unsupported type
        return utFalse;
    }
}

/* get a 32-bit value for the specified key at the specified index */
UInt32 propGetUInt32AtIndex(Key_t key, int ndx, UInt32 dft)
{
    KeyValue_t *kv = propGetKeyValueEntry(key);
    if (!kv) {
        return dft;
    } else {
        UInt32 val32 = dft;
        PROP_LOCK {
            _propRefresh(PROP_REFRESH_GET, kv, (UInt8*)0, 0); // no args
            if (kv->lenNdx < (ndx + 1)) {
                val32 = dft;
            } else {
                UInt32 v;
                utBool ok = _propGetUInt32Value(kv, ndx, &v);
                val32 = ok? v : dft;
            }
        } PROP_UNLOCK
        return val32;
    }
}

/* set a 32-bit value for the specified key at the specified index */
utBool propSetUInt32AtIndexRefresh(Key_t key, int ndx, UInt32 val, utBool refresh)
{
    KeyValue_t *kv = propGetKeyValueEntry(key);
    if (kv) {
        if (kv->maxNdx < (ndx + 1)) {
            return utFalse;
        } else {
            utBool ok;
            PROP_LOCK {
                ok = _propSetUInt32Value(kv, ndx, &val);
                if (ok && refresh) { 
                    _propRefresh(PROP_REFRESH_SET, kv, (UInt8*)0, 0);
                }
            } PROP_UNLOCK
            return ok;
        }
    } else {
        return utFalse;
    }
}

/* set a 32-bit value for the specified key at the specified index */
utBool propSetUInt32AtIndex(Key_t key, int ndx, UInt32 val)
{
    return propSetUInt32AtIndexRefresh(key, ndx, val, utTrue);
}

/* add a 32-bit value to the specified key at the specified index */
utBool propAddUInt32AtIndex(Key_t key, int ndx, UInt32 *val)
{
    KeyValue_t *kv = propGetKeyValueEntry(key);
    if (!kv) {
        return utFalse;
    } else {
        utBool ok = utFalse;
        PROP_LOCK {
            _propRefresh(PROP_REFRESH_GET, kv, (UInt8*)0, 0); // no args
            if (kv->lenNdx < (ndx + 1)) {
                // index is beyond available entries
                ok = utFalse;
            } else {
                UInt32 oldVal;
                utBool ok = _propGetUInt32Value(kv, ndx, &oldVal);
                if (ok) {
                    *val += oldVal;
                    ok = _propSetUInt32Value(kv, ndx, val);
                    if (ok) { _propRefresh(PROP_REFRESH_SET, kv, (UInt8*)0, 0); }
                } else {
                    ok = utFalse;
                }
            }
        } PROP_UNLOCK
        return ok;
    }
}

/* get a 32-bit value for the specified key */
UInt32 propGetUInt32(Key_t key, UInt32 dft)
{
    return propGetUInt32AtIndex(key, 0, dft);
}

/* set a 32-bit value for the specified key */
utBool propSetUInt32(Key_t key, UInt32 val)
{
    return propSetUInt32AtIndexRefresh(key, 0, val, utTrue);
}

/* set a 32-bit value for the specified key */
utBool propSetUInt32Refresh(Key_t key, UInt32 val, utBool refresh)
{
    return propSetUInt32AtIndexRefresh(key, 0, val, refresh);
}

/* add a 32-bit value to the specified key */
utBool propAddUInt32(Key_t key, UInt32 val)
{
    return propAddUInt32AtIndex(key, 0, &val);
}

// ----------------------------------------------------------------------------

/* get a boolean value for the specified key at the specified index */
utBool propGetBooleanAtIndex(Key_t key, int ndx, utBool dft)
{
    return propGetUInt32AtIndex(key, ndx, (UInt32)dft)? utTrue : utFalse;
}

/* set a boolean value for the specified key at the specified index */
utBool propSetBooleanAtIndex(Key_t key, int ndx, utBool val)
{
    return propSetUInt32AtIndexRefresh(key, ndx, (UInt32)val, utTrue);
}

/* get a boolean value for the specified key */
utBool propGetBoolean(Key_t key, utBool dft)
{
    return propGetBooleanAtIndex(key, 0, dft);
}

/* set a boolean value for the specified key */
utBool propSetBoolean(Key_t key, utBool val)
{
    return propSetBooleanAtIndex(key, 0, val);
}

// ----------------------------------------------------------------------------

/* get a double value for the KeyValue entry at the specified index */
static utBool _propGetDoubleValue(KeyValue_t *kv, int ndx, double *val)
{
    if (kv) {
        UInt32 uval;
        utBool ok = _propGetUInt32Value(kv, ndx, &uval);
        if (ok) {
            if (KVT_IS_SIGNED(kv->type)) {
                *val = UInt32_to_Double((Int32)uval, kv->type);
            } else {
                *val = UInt32_to_Double(uval, kv->type);
            }
            return utTrue;
        } else {
            return utFalse;
        }
    } else {
        return utFalse;
    }
}

/* set a double value for the KeyValue entry at the specified index */
static utBool _propSetDoubleValue(KeyValue_t *kv, int ndx, double *val)
{
    UInt32 uval = Double_to_UInt32(*val, kv->type);
    return _propSetUInt32Value(kv, ndx, &uval);
}

/* get a double value for the specified key at the specified index */
double propGetDoubleAtIndex(Key_t key, int ndx, double dft)
{
    KeyValue_t *kv = propGetKeyValueEntry(key);
    if (!kv) {
        return dft;
    } else {
        double valDbl = dft;
        PROP_LOCK {
            _propRefresh(PROP_REFRESH_GET, kv, (UInt8*)0, 0); // no args
            if (kv->lenNdx < (ndx + 1)) {
                valDbl = dft;
            } else {
                double v;
                utBool ok = _propGetDoubleValue(kv, ndx, &v);
                valDbl = ok? v : dft;
            }
        } PROP_UNLOCK
        return valDbl;
    }
}

/* set a double value for the specified key at the specified index */
utBool propSetDoubleAtIndex(Key_t key, int ndx, double val)
{
    KeyValue_t *kv = propGetKeyValueEntry(key);
    if (kv) {
        if (kv->maxNdx < (ndx + 1)) {
            return utFalse;
        } else {
            utBool ok = utFalse;
            PROP_LOCK {
                ok = _propSetDoubleValue(kv, ndx, &val);
                if (ok) { _propRefresh(PROP_REFRESH_SET, kv, (UInt8*)0, 0); }
            } PROP_UNLOCK
            return ok;
        }
    }
    return utFalse;
}

/* add a double value to the specified key at the specified index */
utBool propAddDoubleAtIndex(Key_t key, int ndx, double *val)
{
    KeyValue_t *kv = propGetKeyValueEntry(key);
    if (!kv) {
        return utFalse;
    } else {
        utBool ok = utFalse;
        PROP_LOCK {
            _propRefresh(PROP_REFRESH_GET, kv, (UInt8*)0, 0); // no args
            if (kv->lenNdx < (ndx + 1)) {
                ok = utFalse;
            } else {
                double oldVal;
                utBool ok = _propGetDoubleValue(kv, ndx, &oldVal);
                if (ok) {
                    *val += oldVal;
                    ok = _propSetDoubleValue(kv, ndx, val);
                    if (ok) { _propRefresh(PROP_REFRESH_SET, kv, (UInt8*)0, 0); }
                } else {
                    ok = utFalse;
                }
            }
        } PROP_UNLOCK
        return ok;
    }
}

/* get a double value for the specified key */
double propGetDouble(Key_t key, double dft)
{
    return propGetDoubleAtIndex(key, 0, dft);
}

/* set a double value for the specified key */
utBool propSetDouble(Key_t key, double val)
{
    return propSetDoubleAtIndex(key, 0, val);
}

/* add a double value to the specified key */
utBool propAddDouble(Key_t key, double val)
{
    return propAddDoubleAtIndex(key, 0, &val);
}

// ----------------------------------------------------------------------------

/* get a null-terminated string value for the KeyValue entry */
static utBool _propGetStringValue(KeyValue_t *kv, const char *(*val))
{
    if (!kv || !val) {
        // no place to get value
        return utFalse;
    } else
    if (KVT_TYPE(kv->type) == KVT_STRING) {
        KeyData_t *keyDataBuf = _propGetData(kv);
      //UInt16     keyDataSize = _propGetDataCapacity(kv);
        *val = (char*)(keyDataBuf->b); // assume already terminated
        return utTrue;
    } else {
        // unsupported type
        return utFalse;
    }
}

/* set a null-terminated string value for the KeyValue entry */
static utBool _propSetStringValue(KeyValue_t *kv, const char *(*val))
{
    if (!kv || !val) {
        // no specified value
        return utFalse;
    } else
    if (KVT_TYPE(kv->type) == KVT_STRING) {
        KeyData_t *keyDataBuf = _propGetData(kv);
        UInt16 keyDataSize = _propGetDataCapacity(kv);
        if (keyDataSize > 0) {
            int len = strlen(*val);
            if (len > keyDataSize)
				len = keyDataSize;
            if ((char*)keyDataBuf->b != *val) {
                strncpy((char*)keyDataBuf->b, *val, keyDataSize);
            }
            kv->lenNdx = 1; // a single string is defined
            kv->dataSize = len; // not including terminating null
            kv->attr |= KVA_CHANGED; 
            return utTrue;
        } else {
            // insufficient size
            return utFalse;
        }
    } else {
        // unsupported type
        return utFalse;
    }
}

/* get a null-terminated string value for the specified key */
const char *propGetString(Key_t key, const char *dft)
{
    KeyValue_t *kv = propGetKeyValueEntry(key);
    if (!kv) {
        return dft;
    } else {
        const char *valChar = dft;
        PROP_LOCK {
            _propRefresh(PROP_REFRESH_GET, kv, (UInt8*)0, 0); // no args
            if (kv->lenNdx <= 0) {
                valChar = dft;
            } else {
                const char *v;
                utBool ok = _propGetStringValue(kv, &v);
                valChar = ok? v : dft;
            }
        } PROP_UNLOCK
        return valChar;
    }
}

utBool propSetString(Key_t key, const char *val)
{
    KeyValue_t *kv = propGetKeyValueEntry(key);
    if (!kv) {
        return utFalse;
    } else {
        utBool ok = utFalse;
        PROP_LOCK {
            ok = _propSetStringValue(kv, &val);
            if (ok) { _propRefresh(PROP_REFRESH_SET, kv, (UInt8*)0, 0); }
        } PROP_UNLOCK
        return ok;
    }
}

// ----------------------------------------------------------------------------

/* get a null-terminated string value for the KeyValue entry */
static utBool _propGetBinaryValue(KeyValue_t *kv, const UInt8 *(*val), UInt16 *maxLen, UInt16 *dtaLen)
{
    if (!kv || !val || !maxLen || !dtaLen) {
        // no place to get value
        return utFalse;
    } else
    if (KVT_TYPE(kv->type) == KVT_BINARY) {
        KeyData_t *keyDataBuf = _propGetData(kv);
        UInt16     keyDataSize = _propGetDataCapacity(kv);
        *val    = keyDataBuf->b;
        *maxLen = keyDataSize;
        *dtaLen = kv->dataSize;
        return utTrue;
    } else {
        // unsupported type
        return utFalse;
    }
}

/* set a null-terminated string value for the KeyValue entry */
static utBool _propSetBinaryValue(KeyValue_t *kv, const UInt8 *(*val), UInt16 dtaLen)
{
    if (!kv || !val) {
        // no specified value
        return utFalse;
    } else
    if (KVT_TYPE(kv->type) == KVT_BINARY) {
        KeyData_t *keyDataBuf = _propGetData(kv);
        UInt16     keyDataSize = _propGetDataCapacity(kv);
        if (keyDataSize > 0) {
            int len = (dtaLen < keyDataSize)? dtaLen : keyDataSize;
            if (keyDataBuf->b != *val) {
                memcpy(keyDataBuf->b, *val, len);
            }
            kv->lenNdx = len;
            kv->dataSize = len;
/*            kv->attr |= KVA_NONDEFAULT; */
            kv->attr |= KVA_CHANGED; 
            return utTrue;
        } else {
            // insufficient size
            return utFalse;
        }
    } else {
        // unsupported type
        return utFalse;
    }
}

/* get a binary value for the specified key */
const UInt8 *propGetBinary(Key_t key, const UInt8 *dft, UInt16 *maxLen, UInt16 *dtaLen)
{
    KeyValue_t *kv = propGetKeyValueEntry(key);
    if (!kv) {
        // maxLen. dtaLen left as-is
        return dft;
    } else {
        const UInt8 *valChar = dft;
        PROP_LOCK {
            _propRefresh(PROP_REFRESH_GET, kv, (UInt8*)0, 0); // args?
            const UInt8 *v = (UInt8*)0;
            UInt16 max = 0, dta = 0;
            utBool ok = _propGetBinaryValue(kv, &v, &max, &dta);
            valChar = ok? v : dft;
            if (maxLen) { *maxLen = ok? max : 0; }
            if (dtaLen) { *dtaLen = ok? dta : 0; }
        } PROP_UNLOCK
        return valChar;
    }
}

/* set a binary value */
utBool propSetBinary(Key_t key, const UInt8 *val, UInt16 dtaLen)
{
    KeyValue_t *kv = propGetKeyValueEntry(key);
    if (!kv) {
        return utFalse;
    } else {
        utBool ok = utFalse;
        PROP_LOCK {
            ok = _propSetBinaryValue(kv, &val, dtaLen);
            if (ok) { _propRefresh(PROP_REFRESH_SET, kv, (UInt8*)0, 0); }
        } PROP_UNLOCK
        return ok;
    }
}

// ----------------------------------------------------------------------------

/* get a GPSOdometer_t value for the KeyValue entry */
static utBool _propGetGPSValue(KeyValue_t *kv, const GPSOdometer_t *(*gps))
{
    if (!kv || !gps) {
        // no place to get value
        return utFalse;
    } else
    if (KVT_TYPE(kv->type) == KVT_GPS) {
        KeyData_t *keyDataBuf = _propGetData(kv);
        UInt16     keyDataSize = _propGetDataCapacity(kv);
        if (keyDataSize >= sizeof(GPSOdometer_t)) {
            *gps = &(keyDataBuf->gps);
            return utTrue;
        } else {
            // insufficient size
            return utFalse;
        }
    } else {
        // unsupported type
        return utFalse;
    }
}

/* set a GPSOdometer_t value for the KeyValue entry */
static utBool _propSetGPSValue(KeyValue_t *kv, const GPSOdometer_t *(*gps))
{
    if (!kv || !gps) { // 'gps' should not be null, however *gps may be
        // no specified value
        return utFalse;
    } else
    if (KVT_TYPE(kv->type) == KVT_GPS) {
        KeyData_t *keyDataBuf = _propGetData(kv);
        UInt16     keyDataSize = _propGetDataCapacity(kv);
        if (keyDataSize >= sizeof(GPSOdometer_t)) {
            if (!*gps) {
                // GPS is null, clear KeyData_t GPS point
                memset(&(keyDataBuf->gps), 0, sizeof(GPSOdometer_t));
                gpsPointClear((GPSPoint_t*)&(keyDataBuf->gps));
            } else
            if (&(keyDataBuf->gps) != *gps) {
                // copy iff the pointers are not the same
                memcpy(&(keyDataBuf->gps), *gps, sizeof(GPSOdometer_t));
            }
            kv->lenNdx = 1; // a single GPS point is defined
            kv->dataSize = sizeof(GPSOdometer_t);
            /*kv->attr |= KVA_NONDEFAULT; */
            kv->attr |= KVA_CHANGED; 
            return utTrue;
        } else {
            // insufficient size
            return utFalse;
        }
    } else {
        // unsupported type
        return utFalse;
    }
}

/* get a GPSOdometer_t value for the KeyValue entry */
const GPSOdometer_t *propGetGPS(Key_t key, const GPSOdometer_t *dft)
{
    KeyValue_t *kv = propGetKeyValueEntry(key);
    if (!kv) {
        return dft;
    } else {
        const GPSOdometer_t *valGPS = dft;
        PROP_LOCK {
            _propRefresh(PROP_REFRESH_GET, kv, (UInt8*)0, 0); // no args
            if (kv->lenNdx <= 0) {
                valGPS = dft;
            } else {
                const GPSOdometer_t *gps;
                utBool ok = _propGetGPSValue(kv, &gps);
                valGPS = ok? gps : dft;
            }
        } PROP_UNLOCK
        return valGPS;
    }
}

/* set GPS property value */
utBool propSetGPS(Key_t key, const GPSOdometer_t *gps)
{
    KeyValue_t *kv = propGetKeyValueEntry(key);
    if (!kv) {
        // key not found
        return utFalse;
    } else {
        utBool ok = utFalse;
        PROP_LOCK {
            ok = _propSetGPSValue(kv, &gps); // gps may be null
            if (ok) { _propRefresh(PROP_REFRESH_SET, kv, (UInt8*)0, 0); }
        } PROP_UNLOCK
        return ok;
    }
}

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// This section implements conversion of properties to/from binary values used
// for getting/setting properties

/* set the KeyValue entry value to the specified bytes */
// The type of KeyValue determines how the data will be interpreted
static PropertyError_t _propSetValue(KeyValue_t *kv, const UInt8 *data, int dataLen)
{
	int it_len, len, n;
	UInt16 keyDataSize;
	UInt32 val, type;
	KeyData_t *keyDataBuf;

	type = KVT_TYPE(kv->type);
	keyDataBuf = _propGetData(kv);
	keyDataSize = _propGetDataCapacity(kv);
	switch (type) {
	case KVT_UINT8:
	case KVT_UINT16:
	case KVT_UINT24:
	case KVT_UINT32:
		it_len = KVT_UINT_SIZE(type); // bytes per type
		if (dataLen > kv->dataSize || dataLen % it_len != 0)
			return PROP_ERROR_INVALID_LENGTH;
		for (n = 0; n < dataLen / it_len; n++) {
			val = binDecodeInt32(data + n * it_len, it_len, KVT_IS_SIGNED(kv->type));
			keyDataBuf->i[n] = val;
		}
		kv->attr |= KVA_NONDEFAULT;
		return PROP_ERROR(PROP_ERROR_OK, dataLen);
	case KVT_BINARY:
		kv->lenNdx = (dataLen < keyDataSize)? dataLen : keyDataSize;
		kv->dataSize = kv->lenNdx;
		if ((void*)data != (void*)keyDataBuf) {
		    memset((void*)keyDataBuf, 0, keyDataSize);
		    memcpy((void*)keyDataBuf, (void*)data, kv->lenNdx);
		}
		kv->attr |= KVA_NONDEFAULT;
		return PROP_ERROR(PROP_ERROR_OK, kv->lenNdx);
	case KVT_STRING: 
		if (dataLen > keyDataSize)
			return PROP_ERROR_INVALID_LENGTH;
		strncpy((char*)keyDataBuf->b, (char*)data, dataLen); 
		if (dataLen < keyDataSize)
			memset(&keyDataBuf->b[dataLen], 0, keyDataSize - dataLen);
		len = strnlen((char*)keyDataBuf->b, keyDataSize);
		// strlen may be < len if 'data' was terminated
		kv->lenNdx = 1; // a string has been defined
		kv->dataSize = len; // not including terminating null
		kv->attr |= KVA_NONDEFAULT;
		return PROP_ERROR(PROP_ERROR_OK, len);
	case KVT_POINTER: 
		if (_propSetPointerToString(kv, (char *)data, dataLen))
			return PROP_ERROR(PROP_ERROR_OK, dataLen);
		else
			return PROP_ERROR_COMMAND_INVALID;
	case KVT_GPS: //binary format
		len = _propDecodeGPS(&(keyDataBuf->gps), data, dataLen);
		if (len >= 0) {
			kv->dataSize = len;
			kv->lenNdx = 1; // GPS defined
			kv->attr |= KVA_NONDEFAULT;
			return PROP_ERROR(PROP_ERROR_OK, len);
		} else {
			// invalid length
			return PROP_ERROR_INVALID_LENGTH;
		}
		break;
	}
	return PROP_ERROR_INVALID_TYPE;
} // _propSetValue

/* set the key value to the specified bytes (from the server) */
// This function obeys the 'KVA_IS_READONLY' flag.
int propSetValueCmd(int protoNdx, Packet_t *pkt, const UInt8 *data, UInt32 dataLen, UInt32 timestamp)
{
	KeyValue_t *kv = (KeyValue_t*)0;
	Key_t key = 0;
	UInt16	status, index = 0;
	ClientError_t code_err = 0;
	if (dataLen < 4)
		code_err = ERROR_PROPERTY_INVALID_VALUE;
	else {
		key = (data[0] << 8) | data[1];
		index = (data[dataLen - 2] << 8) | data[dataLen - 1];
	}
#if defined(SECONDARY_SERIAL_TRANSPORT)
    if ((key == PROP_STATE_DEVICE_ID) && (protoNdx == 1)) {
        kv = propGetKeyValueEntry(PROP_STATE_DEVICE_BT);
    } else {
        kv = propGetKeyValueEntry(key);
    }
#else
	kv = propGetKeyValueEntry(key);
#endif
	if (!kv) {
		// invalid key
		code_err = ERROR_PROPERTY_INVALID_ID;
	} else
	if (KVA_IS_READONLY(kv->attr) || KVA_IS_HIDDEN(kv->attr)) {
		// cannot read a write-only property
		code_err = ERROR_PROPERTY_READ_ONLY;
	} 

	pktInit(pkt, PKT_CLIENT_DMTSP_FORMAT_4, (char*)0); // payload filled-in below
	FmtBuffer_t bb, *bf = pktFmtBuffer(&bb, pkt);

	if (code_err != 0) {
		status = STATUS_PROPERTY_SET_1;
		binFmtPrintf(bf, "%2x%4x%2X%2x%2x%2x", status, timestamp, key, index, code_err, key); 
		pkt->dataLen = (UInt8)BUFFER_DATA_LENGTH(bf); // remember to save buffer length
		return -1;
	}

	if (KVT_TYPE(kv->type) == KVT_COMMAND) {
		CommandError_t cmd_err;
		KeyData_t *keyDataBuf = _propGetData(kv);
		if (keyDataBuf->cmd) {
			cmd_err = (*keyDataBuf->cmd)(protoNdx, key, data + 2, dataLen - 4);
			if (cmd_err != COMMAND_OK)
				code_err = ERROR_COMMAND_ERROR;
		} else {
		// the command has not been initialized (internal error)
			logERROR(LOGSRC,"Command not implemented: 0x%04X", (int)key);
			code_err = ERROR_COMMAND_INVALID;
		}
	} else {
		PropertyError_t prop_err;
		PROP_LOCK {
		prop_err = _propSetValue(kv, data + 2, dataLen - 4);
		if (PROP_ERROR_OK_LENGTH(prop_err) >= 0)
			_propRefresh(PROP_REFRESH_SET, kv, (UInt8*)0, 0); 
		} PROP_UNLOCK
		if (PROP_ERROR_CODE(prop_err) == PROP_ERROR_OK) {
			if (key == PROP_TEMP_REPORT_INTRVL)
				property_refresh_flag |= PROP_REFRESH_TEMPERATURE;
			else if (key >=PROP_GPS_POWER_SAVING || key <= PROP_GPS_DISTANCE_DELTA)
				property_refresh_flag |= PROP_REFRESH_GPS;
			else if (key == PROP_COMM_HOST || key == PROP_COMM_PORT)
				protocolScheduleResetURL();
		} else {
			code_err = ERROR_PROPERTY_INVALID_VALUE;
		}
	}

	if (code_err != 0) {
		status = STATUS_PROPERTY_SET_1;
		binFmtPrintf(bf, "%2x%4x%2X%2x%2x%2x", status, timestamp, key, index, code_err, key); 
	} else {
		status = STATUS_PROPERTY_SET_0;
		binFmtPrintf(bf, "%2x%4x%2X%2x", status, timestamp, key, index); 
	}
	pkt->dataLen = (UInt8)BUFFER_DATA_LENGTH(bf); // remember to save buffer length
	return 0;
}

/* set the key value to the specified bytes (from the server) */
// This function obeys the 'KVA_IS_READONLY' flag.
int propSetValueCmdTCP(int protoNdx, const UInt8 *data, UInt32 dataLen, UInt32 *error_code)
{
	KeyValue_t *kv = (KeyValue_t*)0;
	Key_t key = 0;
	ClientError_t code_err = 0;
	if (dataLen < 2)
		code_err = ERROR_PROPERTY_INVALID_VALUE;
	else
		key = (data[0] << 8) | data[1];
#if defined(SECONDARY_SERIAL_TRANSPORT)
    if ((key == PROP_STATE_DEVICE_ID) && (protoNdx == 1)) {
        kv = propGetKeyValueEntry(PROP_STATE_DEVICE_BT);
    } else {
        kv = propGetKeyValueEntry(key);
    }
#else
	kv = propGetKeyValueEntry(key);
#endif
	if (!kv) {
		// invalid key
		code_err = ERROR_PROPERTY_INVALID_ID;
	} else
	if (KVA_IS_READONLY(kv->attr) || KVA_IS_HIDDEN(kv->attr)) {
		// cannot read a write-only property
		code_err = ERROR_PROPERTY_READ_ONLY;
	} 

	if (code_err != 0) {
		return (int)code_err;
	}

	if (KVT_TYPE(kv->type) == KVT_COMMAND) {
		CommandError_t cmd_err;
		KeyData_t *keyDataBuf = _propGetData(kv);
		if (keyDataBuf->cmd) {
			cmd_err = (*keyDataBuf->cmd)(protoNdx, key, data + 2, dataLen - 2);
			if (cmd_err != COMMAND_OK) {
				code_err = ERROR_COMMAND_ERROR;
				*error_code = cmd_err;
			}
		} else {
		// the command has not been initialized (internal error)
			logWARNING(LOGSRC,"Command not implemented:%04X", (int)key);
			code_err = ERROR_COMMAND_INVALID;
		}
	} else {
		PropertyError_t prop_err;
		PROP_LOCK {
		prop_err = _propSetValue(kv, data + 2, dataLen - 2);
		if (PROP_ERROR_OK_LENGTH(prop_err) >= 0)
			_propRefresh(PROP_REFRESH_SET, kv, (UInt8*)0, 0); 
		} PROP_UNLOCK
		if (PROP_ERROR_CODE(prop_err) == PROP_ERROR_OK) {
			if (key == PROP_TEMP_REPORT_INTRVL)
				property_refresh_flag |= PROP_REFRESH_TEMPERATURE;
			else if (key >=PROP_GPS_POWER_SAVING || key <= PROP_GPS_DISTANCE_DELTA)
				property_refresh_flag |= PROP_REFRESH_GPS;
			else if (key == PROP_COMM_HOST || key == PROP_COMM_PORT)
				protocolScheduleResetURL();
		} else {
			code_err = ERROR_PROPERTY_INVALID_VALUE;
		}
	}
	return (int)code_err;
}

// ----------------------------------------------------------------------------

/* get the byte array data for the KeyValue entry */
static PropertyError_t _propGetValue(KeyValue_t *kv, FmtBuffer_t *bf)
{
	if (KVA_IS_WRITEONLY(kv->attr)) {
		return PROP_ERROR_WRITE_ONLY;
	} else
	if (KVT_TYPE(kv->type) == KVT_COMMAND) {
		// should not be here (pre-validated)
		return PROP_ERROR_WRITE_ONLY;
	} else
	if (!BUFFER_DATA(bf) || (BUFFER_DATA_SIZE(bf) <= 0)) {
		// no place to put the data
		return PROP_ERROR_OK; // return ok anyway
	} else {
		KeyData_t *keyDataBuf = _propGetData(kv);
		UInt32 type = KVT_TYPE(kv->type);
		int len = 0, maxBpe = 0;
		if (bf->fmt && (bf->fmtLen > 0)) { *bf->fmt = 0; /* init format */ }
		switch (type) {
			case KVT_UINT8:
			case KVT_UINT16:
			case KVT_UINT24:
			case KVT_UINT32:
				maxBpe = KVT_UINT_SIZE(type); // bytes per type
				if (BUFFER_DATA_SIZE(bf) >= (kv->maxNdx * maxBpe)) {
				   	utBool isSigned = KVT_IS_SIGNED(kv->type);
					char fmtCh = isSigned? 'i' : (KVT_IS_HEX(kv->type)? 'x' : 'u');
					int n;
					for (n = 0; n < kv->maxNdx; n++) {
						binEncodeInt32(BUFFER_DATA(bf), maxBpe, keyDataBuf->i[n], isSigned);
						binAppendFmtField(bf, maxBpe, fmtCh);
						binAdvanceFmtBuffer(bf, maxBpe);
					}
					return BUFFER_DATA_LENGTH(bf);
				} else {
					// invalid data length (internal error)
					return PROP_ERROR_INVALID_LENGTH;
				}
			case KVT_BINARY:
				len = (BUFFER_DATA_SIZE(bf) < kv->lenNdx)? BUFFER_DATA_SIZE(bf) : kv->lenNdx;
				memcpy(BUFFER_DATA(bf), keyDataBuf->b, len);
				binAppendFmtField(bf, len, 'b');
				binAdvanceFmtBuffer(bf, len);
				return PROP_ERROR(PROP_ERROR_OK, len);
			case KVT_STRING:
			case KVT_POINTER:
				len = kv->dataSize + 1;
				if (len > BUFFER_DATA_SIZE(bf))
					len = BUFFER_DATA_SIZE(bf);
				if (len > 0) {
					if (type == KVT_STRING)
						strncpy((char*)BUFFER_DATA(bf), (char*)keyDataBuf->b, len);
					else
						strncpy((char*)BUFFER_DATA(bf), keyDataBuf->ptr.b, len);
				}
				else if (len == 0) {
					len = sprintf((char*)BUFFER_DATA(bf), "%c", ' ');
				}
				binAppendFmtField(bf, len, 's');
				binAdvanceFmtBuffer(bf, len);
				return PROP_ERROR(PROP_ERROR_OK, len);
			case KVT_GPS: 
				len = _propEncodeGPS(&(keyDataBuf->gps), bf);
				return PROP_ERROR(PROP_ERROR_OK, len);
		}
	}
	// unsupported type (internal error)
	return PROP_ERROR_INVALID_TYPE;
} // _propGetValue

/* get the specified property value as a byte array */
// returns the number of bytes placed into the buffer
// or -1, if there was an error
PropertyError_t propGetValue(Key_t key, UInt8 *data, int dataLen)
{
	KeyValue_t *kv = propGetKeyValueEntry(key);
	if (!kv) {
		// invalid key
		return PROP_ERROR_INVALID_KEY;
	} else
	if (KVA_IS_WRITEONLY(kv->attr)) {
		// cannot read a write-only property
		return PROP_ERROR_WRITE_ONLY;
	} else
	if (KVT_TYPE(kv->type) == KVT_COMMAND) {
		// cannot read from a command
		// should not be here (all commands should be write-only)
		return PROP_ERROR_WRITE_ONLY;
	} else {
		PropertyError_t err;
		PROP_LOCK {
			_propRefresh(PROP_REFRESH_GET, kv, (UInt8*)0, 0); // args?
			FmtBuffer_t bb, *bf = binFmtBuffer(&bb, data, dataLen, 0, 0);
			err = _propGetValue(kv, bf);
		} PROP_UNLOCK
		return err;
	}
}

/* create a packet containing the specified property value (for sending to the server) */
int propGetPropertyPacket(int protoNdx, Packet_t *pkt, UInt8 *args, UInt32 dataLen, UInt32 timestamp)
{
	PropertyError_t prop_err = PROP_ERROR_INVALID_KEY;
	KeyValue_t *kv = (KeyValue_t*)0;
	Key_t key = PROP_STATE_PROTOCOL;
	UInt16	status, index = 0;
	ClientError_t code_err = 0;
	if (dataLen < 4)
		code_err = ERROR_PROPERTY_INVALID_VALUE;
	else {
		key = (args[0] << 8) | args[1];
		index = (args[2] << 8) | args[3];
	}
#if defined(SECONDARY_SERIAL_TRANSPORT)
	if ((key == PROP_STATE_DEVICE_ID) && (protoNdx == 1)) {
		kv = propGetKeyValueEntry(PROP_STATE_DEVICE_BT);
	} else {
		kv = propGetKeyValueEntry(key);
	}
#else
	kv = propGetKeyValueEntry(key);
#endif
	if (!kv) {
		// invalid key
		code_err = ERROR_PROPERTY_INVALID_ID;
	} else
	if ((KVA_IS_WRITEONLY(kv->attr)) || KVA_IS_HIDDEN(kv->attr)) {
		// cannot read a write-only property
		code_err = ERROR_PROPERTY_WRITE_ONLY;
	} 

	pktInit(pkt, PKT_CLIENT_DMTSP_FORMAT_4, (char*)0); // payload filled-in below
	FmtBuffer_t bb, *bf = pktFmtBuffer(&bb, pkt);

	if (code_err != 0) {
		status = STATUS_PROPERTY_GET_1;
		binFmtPrintf(bf, "%2x%4x%2x%2x%2x%2x", status, timestamp, key, index, code_err, key); 
		pkt->dataLen = (UInt8)BUFFER_DATA_LENGTH(bf); // remember to save buffer length
		return -1;
	}
	else {
		status = STATUS_PROPERTY_GET_0;
		binFmtPrintf(bf, "%2x%4x%2X%2x", status, timestamp, key, index); 
	}
	PROP_LOCK {
	prop_err = _propGetValue(kv, bf);
	} PROP_UNLOCK
	if (PROP_ERROR_CODE(prop_err) != PROP_ERROR_OK) {
		binEncodeInt32(BUFFER_PTR(bf), 2, STATUS_PROPERTY_GET_1, utFalse);
		code_err = ERROR_PROPERTY_INVALID_VALUE;
		binFmtPrintf(bf, "%2x%2x", code_err, key); 
	}
	pkt->dataLen = (UInt8)BUFFER_DATA_LENGTH(bf); // remember to save buffer length
	return 0;
}
/* create a packet containing the specified property value (for sending to the server) */
int propGetPropertyPacketTCP(int protoNdx, Packet_t *pkt, UInt8 *args, UInt32 dataLen)
{
	PropertyError_t prop_err = PROP_ERROR_INVALID_KEY;
	KeyValue_t *kv = (KeyValue_t*)0;
	Key_t key = PROP_STATE_PROTOCOL;
	ClientError_t code_err = 0;
	if (dataLen < 2)
		code_err = ERROR_PROPERTY_INVALID_VALUE;
	else
		key = (args[0] << 8) | args[1];
#if defined(SECONDARY_SERIAL_TRANSPORT)
	if ((key == PROP_STATE_DEVICE_ID) && (protoNdx == 1)) {
		kv = propGetKeyValueEntry(PROP_STATE_DEVICE_BT);
	} else {
		kv = propGetKeyValueEntry(key);
	}
#else
	kv = propGetKeyValueEntry(key);
#endif
	if (!kv) {
		// invalid key
		code_err = ERROR_PROPERTY_INVALID_ID;
	} else
	if ((KVA_IS_WRITEONLY(kv->attr)) || KVA_IS_HIDDEN(kv->attr)) {
		// cannot read a write-only property
		code_err = ERROR_PROPERTY_WRITE_ONLY;
	} 
	if (code_err != 0)
		return (int)code_err;

	pktInit(pkt, PKT_CLIENT_PROPERTY_VALUE, (char*)0); // payload filled-in below
	FmtBuffer_t bb, *bf = pktFmtBuffer(&bb, pkt);
	binFmtPrintf(bf, "%2x", key); 

	PROP_LOCK {
	prop_err = _propGetValue(kv, bf);
	} PROP_UNLOCK
	if (PROP_ERROR_CODE(prop_err) != PROP_ERROR_OK) {
		code_err = ERROR_PROPERTY_INVALID_VALUE;
		return (int)code_err;
	}
	else {
		pkt->dataLen = (UInt8)BUFFER_DATA_LENGTH(bf); 
		return 0;
	}
}
// ----------------------------------------------------------------------------
/* start-up initialization for individual KeyValue entry */
static void _propInitKeyValueFromString(KeyValue_t *kv, const char *s, utBool internal)
{
	/* get key data buffer */
	char num_buf[128];
	char delim[4] = ", "; 
	char *saveptr, *value, *endptr;
	UInt32 val;
	int n;
	int len = 0;
	long lv;
	double dv;
	UInt16 keyDataSize;
	KeyData_t *keyDataBuf = _propGetData(kv);
	if (s == NULL)
		return;
	/* reset attributes/length */
	kv->lenNdx = 0; // reset actual length
	kv->dataSize = 0;
	if (internal) {
		kv->attr &= ~KVA_NONDEFAULT; // set to 'default' (clear non-default flag)
	}
	/* scan through types */
	if (KVT_TYPE(kv->type) == KVT_COMMAND) {
		if (internal) {
			// reset/clear command type (value of string is ignored)
			keyDataBuf->cmd = NULL;
		}
		return /*PROP_ERROR_OK*/;
	}
	keyDataSize = _propGetDataCapacity(kv);
	if (!KVT_IS_POINTER(kv->type))
		memset(keyDataBuf, 0, keyDataSize); // clear data buffer
	/* parse for specific type */
	switch (KVT_TYPE(kv->type)) {
		case KVT_UINT8:
		case KVT_UINT16:
		case KVT_UINT24:
		case KVT_UINT32:
			strncpy(num_buf, s, 128);
			value = strtok_r(num_buf, delim, &saveptr);
			n = 0;
			while (value != NULL) {
				if (KVT_DEC(kv->type) > 0) {
					errno = 0;
					dv = strtod(value, &endptr);
					if (endptr == value || errno != 0)
						goto next_value;
					lv = (long)round(dv * 10.0 * KVT_DEC(kv->type));
					keyDataBuf->i[n] = lv;
				}
				else {
					if (strchr(value, 'x') != NULL || strchr(value, 'X') != NULL)
						val = strtol(value, &endptr, 16);
					else
						val = strtol(value, &endptr, 10);
					if (endptr != value)
						keyDataBuf->i[n] = val;
				}
next_value:			value = strtok_r(NULL, delim, &saveptr);
				n++;
			}
			kv->lenNdx = n;
			kv->dataSize = n * KVT_UINT_SIZE(kv->type);
			break;
		case KVT_BINARY:
			len = (kv->maxNdx < keyDataSize)? kv->maxNdx : keyDataSize;
			kv->lenNdx = strParseHex(s, -1, keyDataBuf->b, len); // size of binary data
			kv->dataSize = kv->lenNdx;
			break;
		case KVT_STRING: 
			len = strlen(s);
			if (len > keyDataSize)
				len = keyDataSize;
			strncpy((char*)keyDataBuf->b, s, keyDataSize);
			/*keyDataBuf->b[len] = 0; // terminate */
			kv->lenNdx = 1; // only one string
			kv->dataSize = len;
			break;
		case KVT_POINTER: 
			len = strlen(s);
			if (!internal) {
				_propSetPointerToString(kv, s, len);
			}
			break;
		case KVT_GPS: 
			// GPSOdometer_t only! Expected to be in the format "<time>,<latitude>,<longitude>,<meters>"
			gpsOdomParseString(&(keyDataBuf->gps), s);
			kv->lenNdx = 1; // only 1 GPS point
			kv->dataSize = sizeof(GPSOdometer_t);
			break;
	}
	return /*PROP_ERROR_OK*/;
}

/* start-up initialization for all KeyValue entries */
// This function can be called at any time to reset the properties to their start-up 
// defaults by calling this function with a 'true' argument.  Calling this function
// with a 'false' argument will cause the properties to be initialized only if they
// have not yet been initialized.
static utBool _propsDidInit = utFalse;
void propInitialize(utBool forceReset)
{
    if (!_propsDidInit || forceReset) {
        logDEBUG(LOGSRC,"Property table size: %d entries, %lu bytes", PROP_COUNT, (UInt32)sizeof(properties));
        utBool allInSequence = utTrue;
        Key_t lastKey = 0x0000;
        int i;
        PROP_LOCK {
            for (i = 0; i < PROP_COUNT; i++) {
                KeyValue_t *kv = &properties[i];
                if (kv->key < lastKey) {
                    logWARNING(LOGSRC,"Property key out of sequence: 0x%04X %s", kv->key, kv->name);
                    allInSequence = utFalse;
                }
                lastKey = kv->key;
                _propInitKeyValueFromString(kv, kv->dftInit, utTrue); // initialize
            }
        } PROP_UNLOCK
        _propsDidInit = utTrue;
        binarySearchOK = allInSequence;
    }
}

/* initialize KeyValue from string (local initialization) */
utBool propInitFromString(Key_t key, const char *s)
{
    KeyValue_t *kv = propGetKeyValueEntry(key);
    if (kv) {
        PROP_LOCK {
            _propInitKeyValueFromString(kv, s, utTrue); // initialize
            // [DO NOT CALL "_propRefresh(PROP_REFRESH_SET...)" ]
        } PROP_UNLOCK
        return utTrue;
    }
    return utFalse;
}
	
#define STRING			0
#define UINT8			1
#define UINT32			2
#define UINT16			3
#define INT16_DEC_1 	4

// ----------------------------------------------------------------------------
/* convert property value to string */
static const char *_propType(KeyValue_t *kv, char *buf, int bufLen){
	char *s = buf;
	char *prop_type = (char *)malloc(4*sizeof(char));
	
	sprintf(s, "%s,%d,", kv->name, kv->maxNdx);
	switch(KVT_TYPE(kv->type)) {
		  	case KVT_UINT8:
				sprintf(prop_type, "%d\n", UINT8);
				strcat(s, prop_type);
				break;
                	case KVT_UINT16:
				if (KVT_DEC(kv->type) > 0) 
					sprintf(prop_type, "%d\n", INT16_DEC_1);
				else
					sprintf(prop_type, "%d\n", UINT16);
				strcat(s, prop_type);
				break;
//              	case KVT_UINT24:
//				strcat(s, "24 bits unsigned int (0~372735)\n");
//				break;
                	case KVT_UINT32:
				sprintf(prop_type, "%d\n", UINT32);
				strcat(s, prop_type);
				break;
//			case KVT_BINARY:
//				strcat(s, "Binary\n");
//				break;
			case KVT_STRING:
				sprintf(prop_type, "%d\n", STRING);
				strcat(s, prop_type);
				break;
                	case KVT_POINTER:
				sprintf(prop_type, "%d\n", STRING);
				strcat(s, prop_type);
				break;		
//			case KVT_BOOLEAN:
//				strcat(s, "BOOLEAN ( 0 or 1)\n");
//				break;
//			case  KVT_COMMAND :
//				{
//				switch(kv->key)
//					case PROP_CMD_SAVE_PROPS:
//						strcat(s, "BOOLEAN ( 0 or 1)\n");
//						break;
//					case PROP_CMD_UPDATE:
//						strcat(s, "%d\n", STRING);
//						break;
//					case PROP_CMD_UPLOAD_LOG:
//						strcat(s, "%d\n", STRING);
//						break;
//					case PROP_CMD_RESET:
//						strcat(s, "BOOLEAN ( 0 or 1)\n");
//						break;
//				}
//				break;
	          	default:
                    		break;
	}
	free(prop_type);
	return s;
}

//-----------------------------------------------------------------------------
/* convert property value to string */
static const char *_propToString(KeyValue_t *kv, char *buf, int bufLen)
{
    char *s = buf;
    int slen = bufLen;
    UInt8 tmp[MAX_PAYLOAD_SIZE];

    /* single pass loop */
    for (;;) {
        
        /* refresh */
        _propRefresh(PROP_REFRESH_GET, kv, (UInt8*)0, 0); // no args

        /* create "value" string */
        if (kv->lenNdx > 0) {
            FmtBuffer_t bb, *bf = binFmtBuffer(&bb, tmp, sizeof(tmp), 0, 0); // MUST be FmtBuffer_t
            int n;
            PropertyError_t err;
            switch (KVT_TYPE(kv->type)) {
                case KVT_UINT8:
                case KVT_UINT16:
                case KVT_UINT24:
                case KVT_UINT32:
                    for (n = 0; n < kv->lenNdx; n++) {
                        if (12 > slen) {
                            // overflow
                            return (char*)0;
                        }
                        if (n > 0) {
                            *s++ = ',';
                            slen--;
                        }
                        if (KVT_DEC(kv->type) > 0) {
                            double x = 0.0;
                            _propGetDoubleValue(kv, n, &x);
				dbl_to_str(s, x, KVT_DEC(kv->type), RANG_END);
//                            sprintf(s, "%.*lf", KVT_DEC(kv->type), x);
                        } else {
                            UInt32 x = 0L;
                            _propGetUInt32Value(kv, n, &x);
                            if (KVT_IS_HEX(kv->type)) {
                                int bpe = KVT_UINT_SIZE(kv->type) * 2;
                                sprintf(s, "0x%0*X", bpe, x);
                            } else if (KVT_IS_SIGNED(kv->type)) {
                                sprintf(s, "%d", (int)x);
                            } else {
                                sprintf(s, "%u", x);
                            }
                        }
                        int sn = strlen(s);
                        s += sn;
                        slen -= sn;
                    }
                    break;
                case KVT_BINARY:
                    if ((kv->lenNdx * 2) + 3 > bufLen) {
                        // overflow
                        return (char*)0;
                    }
                    err = _propGetValue(kv, bf);
                    sprintf(s, "0x"); s += strlen(s);
                    strEncodeHex(s, -1, BUFFER_PTR(bf), BUFFER_DATA_LENGTH(bf));
					s += strlen(s);
                    break;
                case KVT_STRING: 
                case KVT_POINTER:
                    err = _propGetValue(kv, bf);
                    sprintf(s, "%.*s", (UInt16)BUFFER_DATA_LENGTH(bf), BUFFER_PTR(bf));
					s += strlen(s);
                    break;
                case KVT_GPS: 
                    // Outputs a string with the format "<fixtime>,<latitude>,<longitude>"
                    if (kv->lenNdx + 10 > bufLen) {
                        // overflow
                        return (char*)0;
                    }
                    err = _propGetValue(kv, bf); // <-- NOTE: Returns encoded GPS data!!!
                    if (PROP_ERROR_OK_LENGTH(err) > 0) { // print iff there is gps data defined
                        // a little redundant (first encode, just to decode)
                        GPSOdometer_t gps;
                        memset(&gps, 0, sizeof(GPSOdometer_t));
                        gpsPointClear(&(gps.point));
                        _propDecodeGPS(&gps, BUFFER_PTR(bf), BUFFER_DATA_LENGTH(bf));
                        gpsOdomToString(&gps, s, (bufLen - (s - buf))); s += strlen(s);
                    }
                    break;
                default:
                    break;
            }
        }
        break;
    }
    return buf;
}

/* print KeyValue to string (local) */
utBool propPrintToString(Key_t key, char *buf, int bufLen)
{
    KeyValue_t *kv = propGetKeyValueEntry(key);
    if (kv) {
        PROP_LOCK {
            _propToString(kv, buf, bufLen);
        } PROP_UNLOCK
        return utTrue;
    }
    return utFalse;
}

// ----------------------------------------------------------------------------

/* print property values */
utBool _propSaveProperties(FILE *file, UInt32 save_type)
{
	char buf[256];
	int k, i, dres, len, alen, flen; 
	char *s;
	Int32 *ival;
	double dval;
	KeyValue_t *kv;

	/* iterate through properties */
	flen = 1;
	for (k = 0; k < PROP_COUNT; k++) {
		kv = &properties[k];
		if (KVA_IS_WRITEONLY(kv->attr) || !KVA_IS_NONDEFAULT(kv->attr))
			continue;
		else if (((save_type == 1) && !KVA_IS_HIDDEN(kv->attr)) ||
				((save_type == 0) && KVA_IS_HIDDEN(kv->attr)))
			continue;
			
		s = buf; 
		alen = 0;
		flen = 0;
		if (KVT_IS_UINT(kv->type)) {
			if (KVT_DEC(kv->type) > 0) {
				dres = KVT_DEC(kv->type);
				ival = (Int32 *)kv->data.i;
				for (i = 0; i < kv->maxNdx; i++) {
					dval = (double)ival[i] / (10.0 * dres);
					if (i == kv->maxNdx - 1)
//						len = snprintf(s, 64, "%.*lf", dres, dval);
						len = dbl_to_str(s, dval, dres, RANG_END);
					else
//						len = snprintf(s, 64, "%.*lf,", dres, dval);
						len = dbl_to_str(s, dval, dres, RANG_START);
					s += len;
					alen += len;
				} 
			}
			else {	
				ival = (Int32 *)(kv->data.i);
				for (i = 0; i < kv->maxNdx; i++) {
					if (i == kv->maxNdx - 1)
						len = snprintf(s, 64, "%d", ival[i]);
					else
						len = snprintf(s, 64, "%d,", ival[i]);
					s += len;
					alen += len;
				}
			}
			flen = fprintf(file, "%s=%*s\n", kv->name, alen, buf); 
			if (flen < 0)
				break;
		}
		else if (KVT_TYPE(kv->type) == KVT_STRING) {
			if (kv->dataSize < sizeof(kv->data))
				flen = fprintf(file, "%s=%*s\n", kv->name, kv->dataSize, (char*)kv->data.b);
			else {
				snprintf(buf, sizeof(kv->data) + 1, "%s", (char*)kv->data.b);
				flen = fprintf(file, "%s=%s\n", kv->name, buf);
			}
			if (flen < 0)
				break;
		}
		else {
			_propToString(kv, buf, 256);
			flen = fprintf(file, "%s=%*s\n", kv->name, strlen(buf), buf);
			if (flen < 0)
				break;
		}
	}
	return ((flen > 0)? utTrue: utFalse);

}

// ----------------------------------------------------------------------------
/* print property values */
void propPrintAll(void)
{
	char pbuf[MAX_PAYLOAD_SIZE];
	KeyValue_t *kv;
	int i;

	memset(pbuf, 0, sizeof(pbuf));
	/* iterate through properties */
	for (i = 0, kv = &properties[0]; i < PROP_COUNT; i++, kv++) {
		if ((kv->key >= PROP_RFID_READER_ENABLE && kv->key < PROP_RFID_UPPER_BOUND) 
			|| (kv->key >= PROP_IBOX_PORT && kv->key <= PROP_IBOX_247_REQUEST)
			|| (kv->key >= PROP_COMM_HOST_B && kv->key <= PROP_COMM_PORT) 
			|| (kv->key >= PROP_LOGGING_SERVER && kv->key <= PROP_GPS_DISTANCE_DELTA) 
			|| (kv->key >= PROP_MOTION_START_TYPE && kv->key <= PROP_MOTION_EXCESS_SPEED) 
			|| (kv->key >= PROP_TEMP_RANGE_0 && kv->key <= PROP_TEMP_RANGE_7) 
			|| (kv->key >= PROP_COMM_SAVE_RATE && kv->key <= PROP_COMM_MAX_XMIT_RATE) 
			|| (kv->key == PROP_CFG_GPS_PORT) || (kv->key == PROP_CFG_GPS_BPS) 
			|| (kv->key == PROP_STATE_PROTOCOL) || (kv->key == PROP_STATE_FIRMWARE) 
			|| (kv->key == PROP_STATE_ACCOUNT_ID) || (kv->key == PROP_STATE_DEVICE_ID) || (kv->key == PROP_STATE_DIAGNOSTIC) 
			|| (kv->key == PROP_STATE_DIAGNOSTIC_LEVEL) || (kv->key == PROP_STATE_STUCK_TIMEOUT)
			|| (kv->key == PROP_STATE_NETWORK_DEBUG)
			|| (kv->key == PROP_STATE_BOOTUP_REPORT)
			|| (kv->key == PROP_STATE_CELL_TEMP_RPT_SETUP)
			|| (kv->key == PROP_STATE_CHECKNETWORK_TIMEOUT)
			|| (kv->key == PROP_STATE_NETWORK_CHECK_WAIT_TIMES)
			|| (kv->key == PROP_STATE_RTS_CHECK)
			|| (kv->key == PROP_STATE_iBOX_ENABLE)
			|| (kv->key == PROP_STATE_ALIVE_INTRVL)
			|| (kv->key == PROP_COMM_MTU) || (kv->key == PROP_COMM_UDP_TIMER)  || (kv->key == PROP_COMM_NET_IDLE_MINUTES)
			|| (kv->key == PROP_TEMP_REPORT_INTRVL)
			|| (kv->key >= PROP_QDAC_UNKNOWN_TAG && kv->key <= PROP_QDAC_UPPER_BOUND)) {
		
			_propToString(kv, pbuf, sizeof(pbuf));
			/* "<Key>=" */
			printf("%s=%s\n", kv->name, pbuf);
		}
	}
}

#define PROP_FILE_SIZE 128
// ----------------------------------------------------------------------------
/* export property types for validation check */
void propExportAll(void)
{
	char *pbuf = NULL;
	const char export_file[]="/tmp/dmtp_cfg_prop.txt";
	FILE *fp;
	KeyValue_t *kv;
	int i;

	/* detect if a file is exist or not */
	fp = fopen(export_file, "r");
	if(fp) {
		fclose(fp);
		remove(export_file);
	}
	
	/* iterate through properties */
	for (i = 0, kv = &properties[0]; i < PROP_COUNT; i++, kv++) {
		if ((kv->key >= PROP_RFID_READER_ENABLE && kv->key < PROP_RFID_UPPER_BOUND) 
//			|| (kv->key >= PROP_CMD_SAVE_PROPS && kv->key <= PROP_CMD_RESET)
			|| (kv->key == PROP_STATE_PROTOCOL)
			|| (kv->key == PROP_STATE_SERIAL)
			|| (kv->key == PROP_STATE_ACCOUNT_ID)
			|| (kv->key == PROP_STATE_DEVICE_ID) 
			|| (kv->key == PROP_STATE_DIAGNOSTIC)
//			|| (kv->key == PROP_STATE_AP_DIAGNOSTIC) 
			|| (kv->key == PROP_STATE_DIAGNOSTIC_LEVEL)
			|| (kv->key == PROP_STATE_NETWORK_DEBUG)
			|| (kv->key == PROP_STATE_BOOTUP_REPORT)
			|| (kv->key == PROP_STATE_CELL_TEMP_RPT_SETUP)
			|| (kv->key == PROP_STATE_STUCK_TIMEOUT)
			|| (kv->key == PROP_STATE_CHECKNETWORK_TIMEOUT)
			|| (kv->key == PROP_STATE_NETWORK_CHECK_WAIT_TIMES)
			|| (kv->key == PROP_STATE_RTS_CHECK)
			|| (kv->key == PROP_STATE_iBOX_ENABLE)
			|| (kv->key == PROP_STATE_ALIVE_INTRVL)
			|| (kv->key >= PROP_IBOX_PORT && kv->key <= PROP_IBOX_247_REQUEST)
			|| (kv->key >= PROP_COMM_SAVE_RATE && kv->key <= PROP_COMM_MAX_XMIT_RATE) 
			|| (kv->key >= PROP_COMM_HOST_B && kv->key <= PROP_COMM_PORT) 
			|| (kv->key >= PROP_COMM_CUSTOM_FORMATS && kv->key <= PROP_COMM_BYTES_WRITTEN) 
			|| (kv->key >= PROP_LOGGING_SERVER && kv->key <= PROP_LOGGING_PASS) 
			|| (kv->key >= PROP_GPS_POWER_SAVING && kv->key <= PROP_GPS_DISTANCE_DELTA) 
			|| (kv->key >= PROP_MOTION_START_TYPE && kv->key <= PROP_MOTION_EXCESS_SPEED) 
			|| (kv->key >= PROP_TEMP_RANGE_0 && kv->key <= PROP_TEMP_RANGE_7) 	
			|| (kv->key == PROP_COMM_MTU) 
			|| (kv->key == PROP_COMM_UDP_TIMER) 
			|| (kv->key == PROP_COMM_NET_IDLE_MINUTES)
			|| (kv->key == PROP_TEMP_REPORT_INTRVL)
			|| (kv->key >= PROP_QDAC_UNKNOWN_TAG && kv->key <= PROP_QDAC_UPPER_BOUND )) {
		
//			_propToString(kv, pbuf, sizeof(pbuf));
			pbuf = (char *)malloc(PROP_FILE_SIZE*sizeof(char));
			memset(pbuf, 0, sizeof(pbuf));
			_propType(kv, pbuf, sizeof(pbuf));
			/* "<Key>=" */
			fp = fopen(export_file, "a+");
			fprintf(fp, "%s", pbuf);
			fclose(fp);
			free(pbuf);
		}
	}
}

/* return true if properties have changed since last saved */
utBool propHasChanged()
{
    int i;
    for (i = 0; i < PROP_COUNT; i++) {
        KeyValue_t *kv = &properties[i];
        if (KVA_IS_SAVE(kv->attr) && KVA_IS_CHANGED(kv->attr)) {
            // property is to be saved, and it has changed since last save
            return utTrue;
        }
    }
    return utFalse;
}

/* clear all property changed flags */
void propClearChanged()
{
    int i;
    for (i = 0; i < PROP_COUNT; i++) {
        KeyValue_t *kv = &properties[i];
        kv->attr &= ~KVA_CHANGED;
    }
}

/* save properties */
utBool propSaveProperties(const char *propFile, UInt32 save_type)
{
    /* open file */
	FILE *propfs;
	propfs = ioOpenStream(propFile, IO_OPEN_WRITE);
	if (propfs == NULL) {
	    // error openning
	    return utFalse;
	}
	/* save to propfs stream */
	utBool rtn = utFalse;
	PROP_LOCK {
		rtn = _propSaveProperties(propfs, save_type);
	} PROP_UNLOCK
	/* close propfs */
	ioCloseStream(propfs);
	return rtn;
}

// ----------------------------------------------------------------------------

/* load property values */
// eg:
//  0xFXXX=<value>
// Notes:
//  - Blank lines and lines beginning with '#' are ignored.
//  - Last line MUST be terminated with a newline, otherwise it will be ignored
//  - DOES NOT CALL "_propRefresh(PROP_REFRESH_SET...)" 
utBool propLoadProperties(const char *propFile, UInt32 method)
{
	char buf[256];
	char pbuf[256];
	FILE *propfs;
	int i, n, d, vlimit;
	long vl;
	double vd;
	char delim[8] = "=\n\r\t ";
	char delim1[8] = "=\n\r\t";
	char delim2[4] = ",;";
	char delim3[4] = ".,;";
	char *kname, *kline, *value, *point, *saveptr, *endptr;
	KeyValue_t *kv;
	KeyAttr_t method_attr;
	
    /* open file */
	if ((propfs = ioOpenStream(propFile, IO_OPEN_READ)) == NULL) {
        // error openning (file may not exist)
		logERROR(LOGSRC,"Unable to open property file: %s", propFile);
		return utFalse;
	}
	if (method == 0)
		method_attr = KVA_CHANGED;
	else
		method_attr = KVA_NONDEFAULT;
	memset(buf, 0, 256);
	kline = fgets(buf, 256, propfs);  
	while (kline != NULL) {
		i = 0;
		while (isblank(kline[i]))	
			i++;
		if (!isalpha(kline[i]))
			goto next_line;
		kname = strtok_r(&kline[i], delim, &saveptr);
		kv = (KeyValue_t*)0;
		if (kname != NULL)
			kv = propGetKeyValueEntryByName(kname);
		if (!kv) {
			logWARNING(LOGSRC,"Unknown key %s, ignored", kname);
			goto next_line;
		}
		value = strtok_r(NULL, delim1, &saveptr);
		if (KVT_IS_UINT(kv->type)) {
			n = 0;
			while (value != NULL) {
				while (value[0] == ',' || value[0] == ';') { 
					value++;
					n++; 
				}
				if (value[0] == '\0')
					break;
				if (n >= kv->maxNdx)
					break;
				point = strpbrk(value, delim3);
				if (point != NULL && *point == '.') {
					errno = 0;
					vd = strtod(value, &endptr);
					if (errno == 0 && (KVT_DEC(kv->type) > 0)) {
						vl = (long)round(vd * 10.0 * KVT_DEC(kv->type));
						vlimit = 1 << ((KVT_UINT_SIZE(kv->type) * 8) - 1);
						if (vl < 0) {
							if (vl < -vlimit) 
								goto next_value;
						}
						else if (vl >= vlimit)
							goto next_value;
						kv->data.i[n] = vl;
					}
					else
						goto next_value;
				}
				else {
					d = 0;
					while (!isdigit(value[d]))
						d++;
					errno = 0;
					if (value[d] == '0' && (value[d+1] == 'x' || value[d+1] == 'X')) {
						vl = strtol(value, &endptr, 16);
					}
					else  {
						vl = strtol(value, &endptr, 10);
					}
					if (errno != 0 || endptr == value)
						goto next_value;
					if (!KVT_IS_SIGNED(kv->type) && (vl < 0))
						goto next_value;
					else if (KVT_UINT_SIZE(kv->type) < 4) {
						vlimit = 1 << (KVT_UINT_SIZE(kv->type) * 8);
						if (vl > 0) {
							vlimit >>= KVT_IS_SIGNED(kv->type)? 1 : 0;
							if (vl >= vlimit)
								goto next_value;
						}
						else {
							vlimit >>= 1;
							if (vl < -vlimit)
								goto next_value;
						}
					}
					if (KVT_DEC(kv->type) == 0)
						kv->data.i[n] = vl;
					else
						kv->data.i[n] = vl * 10 * KVT_DEC(kv->type);
				}
next_value:
				if (endptr != value && *endptr != '\0')
					value = strpbrk(endptr, delim2);
				else if (endptr == value && value[1] != '\0')
					value = strpbrk(value + 1, delim2);
				else
					break;
			}
		}
		else if (value != NULL) 
			_propInitKeyValueFromString(kv, value, utFalse);
		else 
			goto next_line;
		kv->attr |= method_attr;
		if (!KVA_IS_HIDDEN(kv->attr)) {
			_propToString(kv, pbuf, sizeof(pbuf));
			logDEBUG(LOGSRC, "%s=%s", kv->name, pbuf);
		}
next_line:
		kline = fgets(buf, 256, propfs);  
	}
    /* close file */
	ioCloseStream(propfs);
	return utTrue;
}
// ----------------------------------------------------------------------------
utBool propSetUInt32_a(Key_t key, const UInt32 *value, int len)
{
	KeyValue_t *kv = propGetKeyValueEntry(key);
	int n;
	if (!kv)
		return utFalse;
	for (n = 0; n < len; n++) {
		kv->data.i[n] = value[n];
	}
	kv->attr |= KVA_NONDEFAULT;
	return utTrue;
}

// ----------------------------------------------------------------------------

/* return account-id */
const char *propGetAccountID()
{
    return propGetString(PROP_STATE_ACCOUNT_ID,"");
}

/* return device-id */
const char *propGetDeviceID(int protoNdx)
{
#if defined(SECONDARY_SERIAL_TRANSPORT)
    if (protoNdx == 1) {
        return propGetString(PROP_STATE_DEVICE_BT,"");
    }
#endif
    return propGetString(PROP_STATE_DEVICE_ID,"");
}
