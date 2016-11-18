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
//  Property definitions
// ---
// Change History:
//  2006/01/04  Martin D. Flynn
//     -Initial release
//  2006/01/17  Martin D. Flynn
//     -Added PROP_GEOF_ARRIVE_DELAY/PROP_GEOF_DEPART_DELAY properties
//     -Renamed PROP_GEOF_VIOLATION to PROP_GEOF_VIOLATION_INTRVL
//     -Renamed PROP_MOTION_DORMANT to PROP_MOTION_DORMANT_INTRVL
//     -Renamed PROP_ODOMETER_# to PROP_ODOMETER_#_VALUE
//  2006/02/13  Martin D. Flynn
//     -Added PROP_MOTION_STOP_TYPE property.
//  2006/05/07  Martin D. Flynn
//     -Added PROP_CMD_GEOF_ADMIN property.
//     -Added PROP_GEOF_VERSION property.
//     -Added PROP_GEOF_COUNT property.
//     -GeoCorr property types renamed to PROP_GEOC_...
//     -Relocated PROP_GEOC_ACTIVE_ID property key (was 0xF561)
//     -Relocated PROP_GEOC_VIOLATION_INTRVL property key (was 0xF571)
//     -Relocated PROP_GEOC_VIOLATION_COUNT property key (was 0xF572)
//  2007/01/28  Martin D. Flynn
//     -Changes to facilitate WindowsCE port
//     -Relocated PROP_CFG_GPS_MODEL from 0xEF22 to 0xEF2A.  This was done to
//      allow other serial port configurations for 0xEFX2 through 0xEFX9.
//     -Relocated PROP_COMM_APN_SETTINGS from 0xF3AA to 0xF3AC
//     -Relocated PROP_GEOC_ACTIVE_ID (again) property key (was 0xF571)
//     -Added the following properties:
//      PROP_CFG_XPORT_BPS, PROP_CFG_GPS_BPS,
//      PROP_GPS_CLOCK_DELTA, PROP_CFG_SERIAL0_BPS, PROP_CFG_SERIAL1_BPS,
//      PROP_COMM_CONNECTION, PROP_COMM_APN_PHONE, PROP_CMD_AUTHORIZE,
//      PROP_STATE_USER_ID and PROP_STATE_USER_TIME, PROP_TEMP_SAMPLE_INTRVL,
//      PROP_TEMP_REPORT_INTRVL, PROP_CMD_SAVE_PROPS, PROP_CMD_RESET,
//      PROP_STATE_DEV_DIAGNOSTIC, PROP_COMM_MIN_SIGNAL, PROP_INPUT_CONFIG_#,
//      PROP_INPUT_STATE, PROP_OUTPUT_CONFIG_#, PROP_ELAPSED_#_VALUE, 
//      PROP_UNDERVOLTAGE_LIMIT, PROP_COMM_ACCESS_PIN, PROP_CFG_SERIAL2_PORT,
//      PROP_CFG_SERIAL2_BPS, PROP_CFG_SERIAL2_DEBUG, PROP_CMD_GEOC_ADMIN, 
//      PROP_MOTION_MOVING_INTRVL, PROP_CFG_SERIAL3_PORT, PROP_CFG_SERIAL3_BPS, 
//      PROP_CFG_SERIAL3_DEBUG
//     -The odometer unit of measurement has changed from 0.1 meter to 1 meter
//      units for the following properties:
//          PROP_GPS_ACCURACY, PROP_GPS_DISTANCE_DELTA, 
//          PROP_ODOMETER_#_VALUE, PROP_ODOMETER_#_LIMIT
//      A 0.1 meter resolution did not provide a high enough top end value that
//      could be represented in 4 bytes.  The change to 1 meter units provides a
//      maximum odometers value increase from about 267K miles (which many vehicles
//      have been able to attain) to about 2.67M miles (we'll beyond the life 
//      expectancey of most vehicles).  And 1 meter resolution is more than enough
//      to provide a very accurate GPS calculated odometer reading (at least well
//      within the accuracy of currently available GPS receivers).
//     -The elapsed time unit of measurement has changed from milliseconds to
//      seconds for the properties PROP_ELAPSED_#_LIMIT.  The recording of
//      elapsed time with second precision more practical for vehicle requiring 
//      elapsed time measurements, and fits easily into a single 32-bit integer, 
//      which is easier for porting to other platforms.
//     -Added an optional odometer <meters> field to the end of GPS properties
//      PROP_STATE_GPS and PROP_ODOMETER_#_GPS
// ----------------------------------------------------------------------------

#ifndef _PROPERTIES_H
#define _PROPERTIES_H
#ifdef __cplusplus
extern "C" {
#endif

// ----------------------------------------------------------------------------
/* Minimum Unique ID size */
#define MIN_UNIQUE_SIZE         4

// ----------------------------------------------------------------------------
// Custom Property IDs: [00-01 through DF-FF]
//  0001-CFFF : Defined by custom application
// Reserved Property IDs: [00-00, and E0-00 through FF-FF]
//  0000      : Reserved (used to define "No Property")
//  E000-EEFF : Reserved for future use
//  EF00-EFFF : Device configuration
//  F000-F0FF : Misc commands
//  F100-F1FF : Misc, static, read-only information
//  F300-F3FF : Communication
//  F500-F5FF : GPS config
//  F700-F7FF : Motion/Odometer 
//  F900-F9FF : Digital I/O, Elapsed time
//  FB00-FB5F : Analog sensors
//  FB60-FBFF : Temperature sensors
// ----------------------------------------------------------------------------

#ifdef CUSTOM_PROPERTIES
#  include "custprop.h"
#endif

// ----------------------------------------------------------------------------
// Reserved Property IDs: [E0-00 through FF-FF]
// ----------------------------------------------------------------------------
// Properties allow reading state, or setting state, on the device.
// Property attributes typically may be one or more of the following:
//  - Read-Only (cannot be changed by the server)
//  - State (maintains device state, may be read-only by server)
//  - Command (write-only) (invokes an action on the client when set)
//  - Configuration (sets a behaviour attribute)

// ----------------------------------------------------------------------------
// Argument/Value parsing rules:
// 1) For ASCIIZ types, parsing of the ASCII value stops at the first null
//    terminator, or when the end of the data buffer is encountered.  If a null
//    terminator is encountered, then any remaining data after the null may be
//    discarded.
// 2) For numeric/binary types, if the supplied data length is at least the length
//    which is required by the property, then the data is parsed as defined by the 
//    property, and any remaining data may be discarded.
// 3) For numeric types, if the supplied data length is less than what is
//    specified as required by the property, then the supplied data is right 
//    shifted until the proper property required data length is achieved.  If the 
//    property value is signed, then most significant bit of the supplied data is 
//    extended into the bytes comprising the filler.  If the property defines a
//    multiple numeric element, the supplied length is divided evenly by the number 
//    of required elements (any remainder constitutes an error) and the quotient
//    number of bytes is parsed as described above for each element. 0-length data 
//    payloads will be interpreted as '0' numeric values.

// ----------------------------------------------------------------------------
// Notes:
// 1) The client is free to impose whatever limits it deems necessary on the 
//    property values set by the server.

// ----------------------------------------------------------------------------
// Reserved platform configuration properties [E000 through EFFF]

// --- GPS port config
#define PROP_CFG_GPS_PORT               0xEF21
#define PROP_CFG_GPS_BPS                0xEF22
#define PROP_CFG_GPS_MODEL              0xEF2A  // was 0xEF22
// --- iBox config 
#define PROP_IBOX_PORT		0xEF30
#define PROP_IBOX_MID		0xEF31
#define PROP_IBOX_96_REQUEST 0xEF32
#define PROP_IBOX_168_REQUEST 0xEF33
#define PROP_IBOX_171_REQUEST 0xEF34
#define PROP_IBOX_200_REQUEST 0xEF35
#define PROP_IBOX_201_REQUEST 0xEF36
#define PROP_IBOX_202_REQUEST 0xEF37
#define PROP_IBOX_203_REQUEST 0xEF38
#define PROP_IBOX_207_REQUEST 0xEF39
#define PROP_IBOX_234_REQUEST 0xEF3A
#define PROP_IBOX_235_REQUEST 0xEF3B
#define PROP_IBOX_243_REQUEST 0xEF3C
#define PROP_IBOX_246_REQUEST 0xEF3D
#define PROP_IBOX_247_REQUEST 0xEF3E
#define PROP_IBOX_205_COMMAND 0xEF3F
#define PROP_IBOX_205_COMMAND_TIMEOUT 0xEF40
#define PROP_IBOX_206_COMMAND 0xEF41
#define PROP_IBOX_206_COMMAND_TIMEOUT 0xEF42
#define PROP_IBOX_208_COMMAND 0xEF43
#define PROP_IBOX_208_COMMAND_TIMEOUT 0xEF44


// ----------------------------------------------------------------------------
/* Properties for RFID operation */
#define PROP_RFID_READER_ENABLE		0xEF70 /*RO, uint32 x 2 */
#define PROP_RFID_READER_PORT		0xEF72 /*RO, ascii*/
#define PROP_RFID_READER_BPS		0xEF74 /*RO, uint32*/
#define PROP_RFID_COMPANY_ID_RANGE	0xEF75 /*RW, uint32 x 2 */
#define PROP_RFID_PRIMARY_ID_DIVISOR 0xEF78 /*RW, uint32 x 2 */
#define PROP_RFID_IN_MOTION			0xEF7A /*RW, uint32 x 2 */
#define PROP_RFID_PRIMARY_ID		0xEF7B /*RW, uint32*/
#define PROP_RFID_PRIMARY_ID_RANGE	0xEF7C /*RW, uint32* x 5 */
#define PROP_RFID_LOCK_ID_RANGE		0xEF7D /*RW, uint32* x 5 */
#define PROP_RFID_PRIMARY_RSSI_TIMER	0xEF81 /*RW, uint32*/
#define PROP_RFID_PRIMARY_RSSI		0xEF82 /*RW, uint8* x 2*/
#define PROP_RFID_SWITCH_ID_RANGE	0xEF83 /*RW, uint32* x 5 */
#define PROP_RFID_CARGO_MIN_RSSI	0xEF84 /*RW, uint8*/
#define PROP_RFID_CARGO_ID_RANGE	0xEF85 /*RW, uint32* x 5 */
#define PROP_RFID_CARGO_SAMPLE_MODE	0xEF86 /*RW, uint8*/
#define PROP_RFID_BATTERY_LIFE_MAX	0xEF88 /*RW, uint8*/
#define PROP_RFID_LOCK_REPORT_INTRVL	0xEF8A /*RW, uint32 x 3*/
#define PROP_RFID_BATTERY_ALARM_INTRVL	0xEF8C /*RW, uint32*/
#define PROP_RFID_SWITCH_REPORT_INTRVL	0xEF8D /*RW, uint32*/
#define PROP_RFID_CARGO_REPORT_INTRVL	0xEF8E /*RW, uint32*/

/* high temperature tag */
#define PROP_RFID_HIGHTEMP_REPORT_INTRVL	0xEF8B /*RW, uint32*/
#define PROP_RFID_HIGHTEMP_ID_RANGE 0xEF87 /*RW, uint32* x 5 */
#define PROP_RFID_HIGHTEMP_ID_RANGE_2 0xEF8F /*RW, uint32* x 2 */
/* motion tag */
#define PROP_RFID_MOTION_ID_RANGE 			0xEF90  /*RW, uint32* x 5 */
#define PROP_RFID_MOTION_REPORT_RATE		0xEF91 /*RW, uint32*/
/* sensor tag */
#define PROP_RFID_SENSOR_ID_RANGE 			0xEF95  /*RW, uint32* x 5 */
#define PROP_RFID_SENSOR_REPORT_INTRVL		0xEF96 /*RW, uint32*/
/* humidity tag */
#define PROP_RFID_HUMIDITY_ID_RANGE 			0xEF97  /*RW, uint32* x 5 */
#define PROP_RFID_HUMIDITY_REPORT_INTRVL		0xEF98 /*RW, uint32*/

#define PROP_RFID_UPPER_BOUND	0xEF9F
//-------------------------------------------------------------------------------
// Define the properties for the QDAC reader.
// 0. unknown tag
#define PROP_QDAC_UNKNOWN_TAG						0xEFA0
#define PROP_QDAC_UNKNOWN_REPORT_INTRVL			0xEFA1
// 1. low temperature tag
#define PROP_QDAC_LOWTEMP_TAG						0xEFA2
#define PROP_QDAC_LOWTEMP_REPORT_INTRVL				0xEFA3
/* QDAC zone */
#define PROP_QDAC_PRIMARY_ID							0xEFA4
#define PROP_QDAC_ZONE_0								0xEFA5
#define PROP_QDAC_ZONE_1								0xEFA6
#define PROP_QDAC_ZONE_2								0xEFA7
#define PROP_QDAC_ZONE_3								0xEFA8
#define PROP_QDAC_ZONE_4								0xEFA9
#define PROP_QDAC_ZONE_5								0xEFAA
#define PROP_QDAC_ZONE_6								0xEFAB
#define PROP_QDAC_ZONE_7								0xEFAC
// 3. gforce sensor tag
#define PROP_QDAC_GFORCESENSOR_TAG					0xEFB0
#define PROP_QDAC_GFORCESENSOR_RPT_THRESHOLD		0xEFB1
// 4. timer sensor tag
#define PROP_QDAC_CURRENTSENSOR_TAG					0xEFB2
#define PROP_QDAC_CURRENTSENSOR_REPORT_INTRVL		0xEFB3
// 5. 2 Temperary tag
#define PROP_QDAC_TMP_TYPE1							0xEFB4
#define PROP_QDAC_TMP_TYPE2							0xEFB5
#define PROP_QDAC_TMP_TAG								0xEFB6
#define PROP_QDAC_TMP_REPORT_INTRVL					0xEFB7
// 6. Swich tag
#define PROP_QDAC_SWITCH_TAG							0xEFB8
#define PROP_QDAC_SWITCH_REPORT_INTRVL				0xEFB9
// 7. Pressure tag
#define PROP_QDAC_PRESSURE_TAG						0xEFBA
#define PROP_QDAC_PRESSURE_REPORT_INTRVL			0xEFBB
// QDAC revocery regarding properties
#define PROP_QDAC_INIT_WAIT_SEC						0xEFC0
#define PROP_QDAC_RESET_MIN							0xEFC1
#define PROP_QDAC_FIRMWARE							0xEFC2

// QDAC bound
#define PROP_QDAC_UPPER_BOUND						0xEFCF
// ----------------------------------------------------------------------------
// Reserved Command properties (WO = write-only) [F000 through F04F]

#define PROP_CMD_SAVE_PROPS             0xF000
#define PROP_CMD_UPDATE		0xF002
#define PROP_CMD_UPLOAD_LOG	0xF003
#define PROP_CMD_UPLOAD_DEBUGLOG		0xF004
//#define PROP_CMD_STATUS_EVENT           0xF011
//#define PROP_CMD_SET_OUTPUT             0xF031
#define PROP_CMD_RESET                  0xF0FF


// ----------------------------------------------------------------------------
// Read-Only/State properties:

#define PROP_STATE_PROTOCOL             0xF100
#define PROP_STATE_FIRMWARE             0xF101
//#define PROP_STATE_COPYRIGHT            0xF107
#define PROP_STATE_SERIAL               0xF110

#define PROP_STATE_UNIQUE_ID            0xF112
#define PROP_STATE_ACCOUNT_ID           0xF114
#define PROP_STATE_DEVICE_ID            0xF115


#define PROP_STATE_TIME                 0xF121
#define PROP_STATE_GPS                  0xF123
#define PROP_STATE_GPS_DIAGNOSTIC       0xF124
#define PROP_STATE_DIAGNOSTIC       0xF141
#define PROP_STATE_AP_DIAGNOSTIC 	0xF151
#define PROP_STATE_DIAGNOSTIC_LEVEL 0xF161
#define PROP_STATE_NETWORK_DEBUG 0xF162
#define PROP_STATE_BOOTUP_REPORT	0xF163
#define PROP_STATE_CELL_TEMP_RPT_SETUP 0xF164
#define PROP_STATE_STUCK_TIMEOUT	0xF171
#define PROP_STATE_CHECKNETWORK_TIMEOUT 0xF172
#define PROP_STATE_NETWORK_CHECK_WAIT_TIMES	0xF181
#define PROP_STATE_RTS_CHECK 0xF191
#define PROP_STATE_iBOX_ENABLE 0xF201
#define PROP_STATE_ALIVE_INTRVL 0xF205
// ----------------------------------------------------------------------------
// Communication protocol properties:

#define PROP_COMM_SPEAK_FIRST           0xF303
#define PROP_COMM_FIRST_BRIEF           0xF305
#define PROP_COMM_SAVE_RATE			0xF311
#define PROP_COMM_MAX_DELAY			0xF312
#define PROP_COMM_MIN_XMIT_RATE		0xF313 
#define PROP_COMM_MAX_XMIT_RATE         0xF315
#define PROP_COMM_MAX_DUP_EVENTS        0xF317
#define PROP_COMM_MAX_SIM_EVENTS        0xF318
#define PROP_COMM_NET_IDLE_MINUTES	0xF319

// ----------------------------------------------------------------------------
#define PROP_COMM_MTU				0xF321
#define PROP_COMM_UDP_TIMER			0xF322
// Communication connection properties:

#define PROP_COMM_HOST_B                0xF391
#define PROP_COMM_PORT_B                0xF392
#define PROP_COMM_POWER_SAVING              0xF3A0
#define PROP_COMM_HOST                  0xF3A1
#define PROP_COMM_PORT                  0xF3A2

// ----------------------------------------------------------------------------
// Packet/Data format properties:
#define PROP_COMM_CUSTOM_FORMATS        0xF3C0   
#define PROP_COMM_ENCODINGS             0xF3C1
#define PROP_COMM_BYTES_READ            0xF3F1
#define PROP_COMM_BYTES_WRITTEN         0xF3F2

/*upload related*/
#define PROP_LOGGING_SERVER	0xF3F3
#define PROP_LOGGING_USER	0xF3F4
#define PROP_LOGGING_PASS	0xF3F5
// ----------------------------------------------------------------------------
// GPS config properties:
#define PROP_GPS_POWER_SAVING	0xF510
#define PROP_GPS_SAMPLE_RATE	0xF511
#define PROP_GPS_AQUIRE_WAIT	0xF512  
#define PROP_GPS_EXPIRATION	0xF513 
#define PROP_GPS_CLOCK_DELTA	0xF515
#define PROP_GPS_LOST_COUNTER	0xF516
#define PROP_GPS_MIN_SPEED              0xF522
#define PROP_GPS_HIGH_ACCURACY	0xF523
#define PROP_GPS_DISTANCE_DELTA         0xF531
// ----------------------------------------------------------------------------
// GeoZone properties:

#define PROP_CMD_GEOF_ADMIN             0xF542
#define PROP_GEOF_COUNT                 0xF547
#define PROP_GEOF_VERSION               0xF548
#define PROP_GEOF_ARRIVE_DELAY          0xF54A
#define PROP_GEOF_DEPART_DELAY          0xF54D
#define PROP_GEOF_CURRENT               0xF551

// ----------------------------------------------------------------------------
// Motion properties:

#define PROP_MOTION_START_TYPE          0xF711
#define PROP_MOTION_START               0xF712
#define PROP_MOTION_IN_MOTION           0xF713
#define PROP_MOTION_STOP                0xF714
#define PROP_MOTION_STOP_TYPE           0xF715
#define PROP_MOTION_DORMANT_INTRVL      0xF716
#define PROP_MOTION_DORMANT_COUNT       0xF717
#define PROP_MOTION_EXCESS_SPEED        0xF721  // Excess speed (0.1 kph)

// ----------------------------------------------------------------------------
// Odometer properties:

// PROP_ODOMETER_#_VALUE
#define PROP_ODOMETER_0_VALUE           0xF770
#define PROP_ODOMETER_1_VALUE           0xF771
#define PROP_ODOMETER_2_VALUE           0xF772
#define PROP_ODOMETER_3_VALUE           0xF773
#define PROP_ODOMETER_4_VALUE           0xF774
#define PROP_ODOMETER_5_VALUE           0xF775
#define PROP_ODOMETER_6_VALUE           0xF776
#define PROP_ODOMETER_7_VALUE           0xF777

// PROP_ODOMETER_#_LIMIT
#define PROP_ODOMETER_0_LIMIT           0xF780
#define PROP_ODOMETER_1_LIMIT           0xF781
#define PROP_ODOMETER_2_LIMIT           0xF782
#define PROP_ODOMETER_3_LIMIT           0xF783
#define PROP_ODOMETER_4_LIMIT           0xF784
#define PROP_ODOMETER_5_LIMIT           0xF785
#define PROP_ODOMETER_6_LIMIT           0xF786
#define PROP_ODOMETER_7_LIMIT           0xF787

// PROP_ODOMETER_#_GPS
#define PROP_ODOMETER_0_GPS             0xF790
#define PROP_ODOMETER_1_GPS             0xF791
#define PROP_ODOMETER_2_GPS             0xF792
#define PROP_ODOMETER_3_GPS             0xF793
#define PROP_ODOMETER_4_GPS             0xF794
#define PROP_ODOMETER_5_GPS             0xF795
#define PROP_ODOMETER_6_GPS             0xF796
#define PROP_ODOMETER_7_GPS             0xF797

// ----------------------------------------------------------------------------
// Temperature configuration:
#define PROP_TEMP_REPORT_INTRVL	0xFB63
#define PROP_TEMP_RANGE_0               0xFB80  // Set temp 0 high/low
#define PROP_TEMP_RANGE_1               0xFB81  // Set temp 1 high/low
#define PROP_TEMP_RANGE_2               0xFB82  // Set temp 2 high/low
#define PROP_TEMP_RANGE_3               0xFB83  // Set temp 3 high/low
#define PROP_TEMP_RANGE_4               0xFB84  // Set temp 4 high/low
#define PROP_TEMP_RANGE_5               0xFB85  // Set temp 5 high/low
#define PROP_TEMP_RANGE_6               0xFB86  // Set temp 6 high/low
#define PROP_TEMP_RANGE_7               0xFB87  // Set temp 7 high/low

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------

#ifdef __cplusplus
}
#endif
#endif
