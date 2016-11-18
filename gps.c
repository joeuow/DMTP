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
//  Machine interface to GPS module.
//  Abstraction layer for the machine interface to the GPS module.
// Notes:
//  - The code included here is only an example of how to parse NMEA-0183 GPS records.
//  This module should perform sufficient error checking to insure that the GPS module
//  is functioning properly.
//  - Ideally, GPS aquisition should occur in it's own thread in order to return
//  the latest GPS fix to the requestor immediately (without blocking).  In non-thread
//  mode, this implementation will block until a new GPS fix has been aquired (or if a 
//  timeout occurs).
// ---
// Change History:
//  2006/01/04  Martin D. Flynn
//     -Initial release
//  2006/02/09  Martin D. Flynn
//     -This module now maintains its "stale" state.
//     -"gpsGetDiagnostics" now returns a structure.
//  2006/06/08  Martin D. Flynn
//     -Added support for parsing "$GPGSA" 
//      (the DOP values are not currently used for fix discrimination).
//  2007/01/28  Martin D. Flynn
//     -WindowsCE port
//     -Switched to generic thread access methods in 'tools/threads.h'
//     -Initial implementation of a 'power-save' mode feature that closes the GPS 
//      comport if the sampling interval is greater than a minute (or so).  The HP 
//      hw6945 turns off the GPS receiver when the comport has been closed - to save 
//      power.  This feature allows GPS tracking on the HP hw6945 to conserve power
//      (at the expense of some event accuracy).  Note: this feature is still under
//      development and may not currently produce the desired results if used.
// ----------------------------------------------------------------------------

#include "defaults.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <time.h>
#include <math.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <sys/wait.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <poll.h>

#include "log.h"
#include "gps.h"
#include "gpsmods.h"

#include "stdtypes.h"
#include "utctools.h"
#include "strtools.h"
#include "checksum.h"
#include "comport.h"
#include "events.h"
#include "propman.h"
#include "statcode.h"
#include "rfid.h"
#include "float_point_handle.h"
#include "diagnostic.h"
// ----------------------------------------------------------------------------

#if !defined(GPS_COM_DATA_FORMAT)
#  define GPS_COM_DATA_FORMAT           "8N1"
#endif

// ----------------------------------------------------------------------------

/* compiling with thread support? */
#if defined(GPS_THREAD)
//#  warning GPS thread support enabled
#  include "threads.h"
#else
//#  warning GPS thread support disabled
#endif

// ----------------------------------------------------------------------------
// References:
//   "GPS Hackery": http://www.ualberta.ca/~ckuethe/gps/
//   "GPS Applications": http://www.codeproject.com/vb/net/WritingGPSApplications1.asp

// ----------------------------------------------------------------------------

/* default GPS port */
// This default value is not used if the GPS comport property is in effect.
// The real default value is specified in 'propman.c'
#  define DEFAULT_GPS_PORT      "ttyS3"

/* simjulator port name */
#define GPS_SIMULATOR_PORT      "sim"

/* GPS bps */
#define INIT_GPS_SPEED			4800

/* minimum out-of-sync seconds between GPS time and system time */
// delta must be at least this value to cause a system clock update
#define MIN_DELTA_CLOCK_TIME    5L

/* maximum number of allowed seconds in a $GP record 'cycle' */
// In most GPS receivers, the default 'cycle' is 1 second
#define GP_EXPIRE               5L

/* power-save threshold */
// if PROP_GPS_SAMPLE_RATE is >= this value, the GPS port will not be openned
// until 'gpsAquire' is called, and will be closed after a GPS fix is aquired.
#define POWER_SAVE_THRESHOLD    45L // seconds

// ----------------------------------------------------------------------------

/* control characters */
#define ASCII_XON               17 // DC1, CTL_Q
#define ASCII_XOFF              19 // DC3, CTL_S
// ----------------------------------------------------------------------------

#define SUPPORT_TYPE_GPRMC      0x0001 // should always be defined
#define SUPPORT_TYPE_GPGGA      0x0002 // altitude, HDOP
#define SUPPORT_TYPE_GPGSA      0x0004 // PDOP, HDOP, VDOP
// ----------------------------------------------------------------------------

#define BOARD_VERSION "/tmp/board_version"
#define W2SG0084_FILE "/sys/aatsi_devs_ctrl/gps_pw"
#define DMTP_GPS		"/tmp/dmtp_gps"
#define GPS_PUBLISHER	"/tmp/gps_publisher"
#define LED_OFF 	0
#define LED_ON 	1
#define MESSAGE_ID_SUBSCRIBE 0
#define MESSAGE_ID_UNSUBSCRIBE 1

static int gps_led;
UInt32 gpsDistanceDelta;
UInt32 clock_source;
long clock_delta;
double minimumSpeed;
bool gps_power_saving = false;
uint32_t gps_power_saving_cycle = 3600;
static bool gps_read_chip = false;
static bool gps_read_publisher = true;
static int sock_gps;

#if defined(GPS_DEVICE_SIMULATOR)
static utBool			gpsSimulator = utFalse;
#endif
static ComPort_t		gpsCom = {.name = "ttyS3", .bps = 9600,};
static GPS_t			gpsFixLast;
static GPS_t			gpsFixUnsafe;
static struct tm		July2011 = {0, 0, 0, 10, 6, 111, 0};
static utBool			gpsFixValid = utFalse;
static UInt32			gpsSampleCount_A = 0;
static UInt32			gpsSampleCount_V = 0;
static UInt32			gpsRestartCount = 0;
static TimerSec_t		gpsLastSampleTimer = 0;
static TimerSec_t		gpsLastValidTimer = 0;
static uint32_t			gpsExpireInterval;
static long time_delta;

#if defined(GPS_THREAD)
threadCond_t	gpsAquireCond;
static threadMutex_t	gpsSampleMutex;
static threadMutex_t	gpsAquireMutex;
static threadMutex_t	gpsMutex;
static utBool			gpsRunThread = utFalse;
threadThread_t	gpsThread;
#define SAMPLE_LOCK             MUTEX_LOCK(&gpsSampleMutex);
#define SAMPLE_UNLOCK           MUTEX_UNLOCK(&gpsSampleMutex);
#define GPS_LOCK                MUTEX_LOCK(&gpsMutex);
#define GPS_UNLOCK              MUTEX_UNLOCK(&gpsMutex);
#define AQUIRE_LOCK             MUTEX_LOCK(&gpsAquireMutex);
#define AQUIRE_UNLOCK           MUTEX_UNLOCK(&gpsAquireMutex);
//#define AQUIRE_WAIT             CONDITION_WAIT(&gpsAquireCond, &gpsAquireMutex);
#define AQUIRE_WAIT(T)          CONDITION_TIMED_WAIT(&gpsAquireCond, &gpsMutex, (T));
#define AQUIRE_NOTIFY           CONDITION_NOTIFY(&gpsAquireCond);
#else
#define SAMPLE_LOCK     
#define SAMPLE_UNLOCK   
#define GPS_LOCK        
#define GPS_UNLOCK      
#define AQUIRE_LOCK             
#define AQUIRE_UNLOCK           
#define AQUIRE_WAIT             
#define AQUIRE_NOTIFY           
#endif
#define GPS_SENTENCE_SIZE 96
#define GPS_READ_LINES 6
#define GPS_BUF_SIZE GPS_SENTENCE_SIZE * GPS_READ_LINES
#define GPS_PORT_TIMEOUT 86400

static int gps_debug = 0;
// ----------------------------------------------------------------------------

static GPSDiagnostics_t gpsStats = { 0L, 0L, 0L, 0L, 0L};
// ----------------------------------------------------------------------------
int gps_port_timeout;
void * gps_thread_main(void * arg);
static int parse_rmc(char *sentence);
static int parse_gsa(char *sentence);
static int parse_gga(char *sentence);
static void make_nav_init(char * msg);
static void checksum_nema(char * msg);
int synchronize_system_clock(time_t new_time);
int power_up_gps(bool initialize);
int power_down_gps(bool finalize);
int enable_w2sg0084(void);
void w2sg0004_pc15_high(void);
void w2sg0004_pc15_low(void);
void w2sg00xx_i2c_high(void);
void w2sg00xx_i2c_low(void);
//void read_board_version(char *version);
int w2sg0084_high(void);
int w2sg0084_low(void);
void print_data(int n, char *sentence);
static int establish_gps_socket(void);
static int subscribe_gps(void);
static void cancel_gps_subscription(void);
//float str_to_f(char *string);
//double str_to_d(char *string);

GPSDiagnostics_t *gpsGetDiagnostics(GPSDiagnostics_t *stats)
{
#if !defined(GPS_THREAD)
// These values will not be accurate if not running in thread mode
#  warning GPS is not running in thread mode, diagnostic values will not be accurate.
#endif
	GPSDiagnostics_t *s = stats? stats : &gpsStats;
	SAMPLE_LOCK {
		s->lastSampleTime = TIMER_TO_UTC(gpsLastSampleTimer);
		s->lastValidTime  = TIMER_TO_UTC(gpsLastValidTimer);
		s->sampleCount_A  = gpsSampleCount_A;
		s->sampleCount_V  = gpsSampleCount_V;
		s->restartCount   = gpsRestartCount;
	} SAMPLE_UNLOCK
	return s;
}

// ----------------------------------------------------------------------------

static void _gpsConfigGarmin(ComPort_t *com)
{
	// This configures a Garmin receiver to send only GPRMC & GPGGA records,
	// and send them only once every 2 seconds.
	
	// disable all sentences
	comPortWriteString(com, "$PGRMO,,2\r\r\n");
	// "$PGRMO,,4" restores factory default sentences
	comPortFlush(com, 100L);
	
	// Fix mode: Automatic
	// Differential mode: Automatic
	comPortWriteString(com, "$PGRMC,A,,,,,,,,A\r\r\n");
	comPortFlush(com, 100L);
	
	// output every 2 seconds
	// automatic position averaging when stopped
	// NMEA-0183 v2.30 off
	// enable WAAS
	comPortWriteString(com, "$PGRMC1,2,,2,,,,1,W\r\r\n");
	// "$PGRMC1,1" set it back to 1/sec
	// "$PGRMC1E" queries current config
	comPortFlush(com, 100L);

	// enable GPRMC sentence
	comPortWriteString(com, "$PGRMO,GPRMC,1\r\r\n");
	comPortFlush(com, 100L);

	// enable GPGGA sentence
	comPortWriteString(com, "$PGRMO,GPGGA,1\r\r\n");
	comPortFlush(com, 100L);

	/* done */
	logDEBUG(LOGSRC,"Garmin GPS configured");

}

// ----------------------------------------------------------------------------

static void _gpsConfigSiRF(ComPort_t *com)
{
	int n;
	int fd1 = com->read_fd;
	struct pollfd fds1;
	char nmea_buf[GPS_SENTENCE_SIZE];

	memset(nmea_buf, 0, GPS_SENTENCE_SIZE);
	fds1.fd = fd1;
	fds1.events = POLLIN;
	if ((n = poll(&fds1, 1, 4000)) < 0) {
		perror("Poll GPS port");
		return;
	} else if (n > 0) {
		n = read(fd1, nmea_buf, GPS_SENTENCE_SIZE);
		if (n < 0)
			return;
		nmea_buf[GPS_SENTENCE_SIZE - 1] = '\0';
		if (strchr(nmea_buf, '$') != NULL) {
			printf("SiRF Init Info: %s", nmea_buf);
			return;
		}
	}
	
	/* wake up the w2sg0084 chip */	
	enable_w2sg0084();
	change_serial_baudrate(com, INIT_GPS_SPEED);
	usleep(10000);
	n = read(fd1, nmea_buf, GPS_SENTENCE_SIZE);
	if (n < 0)
		return;
	nmea_buf[GPS_SENTENCE_SIZE - 1] = '\0';
	if (strchr(nmea_buf, '$') != NULL) 
		printf("SiRF Init Info: %s", nmea_buf);
	snprintf(nmea_buf, GPS_SENTENCE_SIZE, "$PSRF100,1,9600,8,1,0*FF\r\n\r\n");
	checksum_nema(nmea_buf);
	usleep(100000);
	change_serial_baudrate(com, com->bps);
	write(fd1, nmea_buf, strlen(nmea_buf)); 
	printf("%s", nmea_buf); 
	usleep(100000);
//	change_serial_baudrate(com, com->bps);
	logDEBUG(LOGSRC,"SiRF W2SG0084 is ready");
}

// ----------------------------------------------------------------------------

/* open gps serial port */
static utBool _gpsOpen(bool initialize)
{
	if (gps_read_chip) {
		const char *portName = propGetString(PROP_CFG_GPS_PORT, "");
		const char *chip;
		if (power_up_gps(initialize) < 0)
			return utFalse;

		strncpy(gpsCom.name, portName, PORT_NAME_SIZE);
		gpsCom.bps = propGetUInt32(PROP_CFG_GPS_BPS, 9600);
		if (open_serial_text(&gpsCom) < 0) {
		// The outer loop will retry the open later
		// Generally, this should not occur on the GumStix
			logWARNING(LOGSRC,"Unable to open GPS port '%s'", portName);
			return utFalse;
		}
		logDEBUG(LOGSRC,"Opened GPS port: %s [%ld bps]", portName, gpsCom.bps);

		/* simulator mode? */
	#if defined(GPS_DEVICE_SIMULATOR)
		gpsSimulator = strEqualsIgnoreCase(portName,GPS_SIMULATOR_PORT);
		if (gpsSimulator) {
			// should not be here if we are running in simulator mode
			return utFalse;
		}
	#endif
		/* specific GPS device configuration */
		chip = propGetString(PROP_CFG_GPS_MODEL,"");
		if (strcmp(chip, GPS_RECEIVER_SIRF) == 0) {
		//W2SG0004 or W2SG0084 
			_gpsConfigSiRF(&gpsCom);
		} else if (strcmp(chip, GPS_RECEIVER_GARMIN) == 0) {
		// Garmin 15, 18PC
			_gpsConfigGarmin(&gpsCom);
		} else
			printf("Unknown GPS Chip\n");
		return utTrue;
	}
	if (gps_read_publisher) {
		if ((sock_gps = establish_gps_socket()) < 0)
			return utFalse;
		printf("create dmtp_gps socket success\n");
		if (subscribe_gps() < 0) {
			close(sock_gps);
			return utFalse;
		}
		printf("subscribe to gps_publisher success\n");
		gpsCom.read_fd = sock_gps;
		return utTrue;
	}
}

/* close gps serial port */
static void _gpsClose(bool finalize)
{
	if (gps_read_chip) {
		comPortClose(&gpsCom);
		power_down_gps(finalize);
	}
	if (gps_read_publisher) {
		cancel_gps_subscription();
	}
}

/* indicate thread should stop */
static void _gpsStopThread(void *arg)
{
	gpsRunThread = utFalse;
	GPS_LOCK {
		// nudge thread
		AQUIRE_NOTIFY
	} GPS_UNLOCK
}

/* start the thread */
static utBool _gpsStartThread()
{
	/* create thread */
	gpsRunThread = utTrue; // must set BEFORE we start the thread!
	if (threadCreate(&gpsThread, gps_thread_main, 0, "GPS") == 0) {
		// thread started successfully
		threadAddThreadStopFtn(&_gpsStopThread, 0);
	} else {
		logCRITICAL(LOGSRC,"Unable to start GPS thread");
		gpsRunThread = utFalse;
	}
	return gpsRunThread;
}
// ----------------------------------------------------------------------------
void gpsReloadParameter(void)
{
	gpsEventCycle = propGetUInt32(PROP_GPS_SAMPLE_RATE, 10); 
	if (gpsEventCycle < 2)
		gpsEventCycle = 2;
	gpsDistanceDelta = propGetUInt32(PROP_GPS_DISTANCE_DELTA, 150);
	if (gpsDistanceDelta < 30) 
		gpsDistanceDelta = 30;
	gpsExpireInterval = propGetUInt32(PROP_GPS_EXPIRATION, 1200);
	minimumSpeed = propGetDouble(PROP_GPS_MIN_SPEED, 1.0);
	if (minimumSpeed > 3.0)
		minimumSpeed = 3.0;
	clock_source = propGetUInt32AtIndex(PROP_GPS_CLOCK_DELTA, 1, 7);
	clock_delta = (long)propGetUInt32AtIndex(PROP_GPS_CLOCK_DELTA, 0, 10);
	gps_power_saving = (propGetUInt32AtIndex(PROP_GPS_POWER_SAVING, 0, 0))? true : false;
	if (gps_power_saving) {
		gps_power_saving_cycle = propGetUInt32AtIndex(PROP_GPS_POWER_SAVING, 1, 3600);
		clock_source |= CLOCK_SYNC_GPS;
		time_delta = 3;
	}
	else {
		gps_power_saving_cycle = 0;
		if (clock_source == 0)
			clock_source = CLOCK_SYNC_GPS;
		time_delta = clock_delta;
	}
}
// ----------------------------------------------------------------------------
/* GPS module initialization */
void gpsInitialize(eventAddFtn_t queueEvent)
{
	/* clear gps struct */
	gpsClear(&gpsFixLast);
	gpsClear(&gpsFixUnsafe);

	gpsFixUnsafe.point.latitude = 43.493125;
	gpsFixUnsafe.point.longitude = -80.204365;
	gpsFixUnsafe.altitude = 346.3;
	
	gps_port_timeout = 0;
#if defined(GPS_THREAD)
	/* create mutex's */
	threadMutexInit(&gpsMutex);
	threadMutexInit(&gpsSampleMutex);
	threadMutexInit(&gpsAquireMutex);
	threadConditionInit(&gpsAquireCond);
	gpsReloadParameter();
	/* return success */
	/* start thread */
	_gpsStartThread();
#endif
}

#if !defined(GPS_THREAD)
// ----------------------------------------------------------------------------
/* return true if current GPS fix is stale */
utBool gpsIsFixStale()
{
	// Note: this may be accessed from multiple threads, but since this is not
	// a critical value, no locking is performed.
	return !gpsFixValid;
}
// ----------------------------------------------------------------------------

/* check minimum values in GPS record */
utBool gpsCheckMinimum(GPS_t *gps0, GPS_t *gps1)
{
	GPSPoint_t point0, point1;

	if (gps1->speedKPH >= minimumSpeed)
		return utFalse;

	point0 = gps0->point; 	
	point1 = gps1->point; 	
	double distance = gpsMetersToPoint(&point0, &point1);
	if (distance < gpsDistanceDelta)
		return  utTrue;
	else
		return utFalse;
}

// ----------------------------------------------------------------------------

/* aquire GPS fix */
GPS_t *gpsAquire(GPS_t *gps, UInt32 timeoutMS)
{

	/* null gps pointer specified */
	if (!gps) {
		return (GPS_t*)0;
	}
	
	/* clear gps point */
	gpsClear(gps);


	/* no timeout, return last fix */
	if (timeoutMS <= 0L) {
		// no timeout specified, just return the latest fix that we have (if any)
		// the caller can determine if he wants to use the fix
		return gpsGetLastGPS(gps, -1);
	}
	
	/* indicate to GPS thread that we want a fix */
	// This is only necessary if the gps interval is long and the gps thread is
	// waiting to be told to make a gps aquisition.
	AQUIRE_LOCK {
		gpsAquireTimeoutMS = timeoutMS; // this value is > 0
		gpsAquireRequest   = utTrue;
		AQUIRE_NOTIFY
	} AQUIRE_UNLOCK
	// at this point, the gps thread should be working on getting a fix

	/* wait until a fix is available */
	UInt32 accumTimeoutMS = 0L;
	for (;accumTimeoutMS < timeoutMS;) {
		
		/* get latest fix */
		GPS_t *g = gpsGetLastGPS(gps, -1);
		if (g && (utcGetTimerAgeSec(g->ageTimer) <= 7L)) {
			// The latest fix occurred within the last 7 seconds.
			return g;
		}
		
		/* wait for next fix */
		UInt32 tmo = timeoutMS - accumTimeoutMS;
		if (tmo > 1000L) { tmo = 1000L; }
		threadSleepMS(tmo);
		accumTimeoutMS += tmo;

	}

	/* no fix (timeout?) */
	return (GPS_t*)0;

#if defined(GPS_DEVICE_SIMULATOR)
	const char *gpsPortName = propGetString(PROP_CFG_GPS_PORT, DEFAULT_GPS_PORT);
	gpsSimulator = strEqualsIgnoreCase(gpsPortName, GPS_SIMULATOR_PORT);
	if (gpsSimulator) {
		int rtn = _gpsReadGPSFix(tmo);
		if (rtn > 0) {
			// valid fix aquired
			return gpsGetLastGPS(gps, 15);
		} else {
			// timeout
			return (GPS_t*)0;
		}
	} else
#endif
	if (comPortIsOpen(&gpsCom) || _gpsOpen(false)) {
		int rtn = _gpsReadGPSFix(tmo);
		_gpsClose(true); // always close the port when runnin in non-thread mode
		if (rtn > 0) {
			// valid fix aquired (at least $GPRMC)
			GPS_t *g = gpsGetLastGPS(gps, 15);
			return g;
		} else
		if (rtn < 0) {
			// error
			return (GPS_t*)0;
		} else {
			// timeout 
			return (GPS_t*)0;
		}
	} else {
		// unable to open GPS port
		return (GPS_t*)0;
	}
}
#endif // defined(GPS_THREAD)

/* get last aquired GPS fix */
GPS_t *gpsGetLastGPS(GPS_t *gps, int maxAgeSec)
{
	GPS_t *gps_got;
	/* get latest fix */
	GPS_LOCK {
	if (gpsIsValid(&gpsFixLast)) {
		gps_got = gpsCopy(gps, &gpsFixLast);
		if (gps_power_saving)
			gpsInvalidate(&gpsFixLast);
	} else
	if (gpsPointIsValid(&gpsFixLast.point)) {
		if (utcGetTimerAgeSec(gpsFixLast.ageTimer) > maxAgeSec)
			    // The last fix ('gpsFixLast') is stale.
			gps_got = (GPS_t*)0; // GPSPoint is stale
		else
			gps_got = gpsCopy(gps, &gpsFixLast);
			    // The last fix ('gpsFixLast') is stale, but the caller doesn't care
	}
	else 
		gps_got = (GPS_t*)0; // GPSPoint is stale
	} GPS_UNLOCK
	return gps_got;
}

int gpsAcquireWait(void)
{
	int err = -1;
	struct timespec later;
	GPS_LOCK {
	while (!gpsIsValid(&gpsFixLast) && (gpsRunThread)) {
		if ((err = clock_gettime(CLOCK_REALTIME, &later)) != 0) {
			perror("Realtime Clock");
			return err;
		}
		if (gps_power_saving)
			later.tv_sec += GPS_PORT_TIMEOUT;
		else
			later.tv_sec += 60;
		err = AQUIRE_WAIT(&later);
	}
	} GPS_UNLOCK
	return err;
}
// ----------------------------------------------------------------------------
#if defined(GPS_THREAD)
static char gps_line[GPS_BUF_SIZE];
static char gps_command[GPS_SENTENCE_SIZE];
static int fd1;
// ----------------------------------------------------------------------------
void * gps_thread_main(void * arg)
{
	int i, n, nwake = 0;
	time_t now;
	char * sentence;
	struct sigevent sev3;
	struct itimerspec it3;
	timer_t timer3;

	/*open GPS port*/
	if (!_gpsOpen(true)) {
		logCRITICAL(LOGSRC,"Unable to open GPS"); 
		diagnostic_report(DIAGNOSTIC_MESSAGE, 0, "GPS Open Failed");
		return NULL;
	}
	fd1 = gpsCom.read_fd;
	/*setup timer */
	sev3.sigev_notify = SIGEV_SIGNAL;
	sev3.sigev_signo = SIG_TIMEOUT;
	sev3.sigev_value.sival_int = TIMER_GPS_1;
	i = timer_create(CLOCK_REALTIME, &sev3, &timer3);
	it3.it_interval.tv_sec = GPS_PORT_TIMEOUT;
	it3.it_interval.tv_nsec = 0;
	it3.it_value.tv_sec = GPS_PORT_TIMEOUT;
	it3.it_value.tv_nsec = 0;
	memset(gps_line, 0, GPS_BUF_SIZE);
	sentence = gps_line;
	i = 0;
	gps_led = LED_OFF;
	while (gpsRunThread) {
		timer_settime(timer3, 0, &it3, NULL);
		if ((n = read(fd1, sentence, GPS_SENTENCE_SIZE)) < 0) {
			w2sg0004_pc15_high();    /* GPS LED off if read error*/
			if (errno == EINTR && gps_port_timeout == -1) {
				printf("Reset GPS chip\n");
					gpsClear(&gpsFixLast);
					gpsClear(&gpsFixUnsafe);
				_gpsClose(true);
				sleep(60);
				_gpsOpen(true);
				fd1 = gpsCom.read_fd;
				gps_port_timeout = 0;
				continue;
			}
		}
		print_data(n, sentence);
		if (strncmp(sentence + 1, "GPGGA", 5) == 0) {
//			print_data(n, sentence);
			parse_gga(sentence);
		}
		else if (strncmp(sentence + 1, "GPGSA", 5) == 0) {
//			print_data(n, sentence);
			parse_gsa(sentence);
		}
		else if (strncmp(sentence + 1, "GPRMC", 5) == 0) {
//			print_data(n, sentence);
			parse_rmc(sentence);
			now = time(NULL); 
		}
		if (gpsIsValid(&gpsFixUnsafe)) {
				long tdiff;
				GPS_LOCK
				if (!gpsIsValid(&gpsFixLast))
					AQUIRE_NOTIFY
				gpsCopy(&gpsFixLast, &gpsFixUnsafe);
				GPS_UNLOCK
				if (clock_source & CLOCK_SYNC_GPS) {
					tdiff = gpsFixUnsafe.fixtime - now;
					if (tdiff > time_delta || tdiff < -time_delta) {
						synchronize_system_clock(gpsFixUnsafe.fixtime);
					}
				}
				gpsFixValid = utTrue;
		}
		else {
			if (gpsPointIsValid(&gpsFixLast.point)) {
					GPS_LOCK
					gpsFixLast.nmea = gpsFixUnsafe.nmea;
					if (now - gpsFixLast.fixtime > gpsExpireInterval)  {
						gpsClear(&gpsFixLast);
					}
					GPS_UNLOCK
				}
				if (gps_power_saving)
					nwake++;
		}
	
		if (++i == GPS_READ_LINES) {
			memset(gps_line, 0, GPS_BUF_SIZE);
			i = 0;
			sentence = gps_line;
		}
		else
			sentence += GPS_SENTENCE_SIZE;

		if (gps_power_saving && (nwake > GPS_POWER_SAVING_WAKE_PERIOD || gpsFixValid)) {
			it3.it_value.tv_sec = 0;
			it3.it_value.tv_nsec = 0;
			timer_settime(timer3, 0, &it3, NULL);
			_gpsClose(false);
			if (sleep(gps_power_saving_cycle - nwake) > 0) {
				sleep(2);
				if (!gpsRunThread)
					break;
			}
			_gpsOpen(false);
			fd1 = gpsCom.read_fd;
			/*initialize GPS*/
			make_nav_init(gps_command);
			checksum_nema(gps_command);
			usleep(100000);
			write(fd1, gps_command, strlen(gps_command));
			nwake = 0;
			gpsFixValid = utFalse;
		}
	}
	_gpsClose(true);
	gpsRunThread = 0;
	return NULL;
}
int parse_rmc(char *sentence)
{
	char item[5][32];
	struct tm utctm;
	float times = 0.0;
	float lati = 0.0, longi = 0.0;
	float knots = 0.0, angles = 0.0;
	int idate = 0;
	char flag = '\0', north = '\0', west = '\0';
	char dtype[8];
	int items, itime, iyear;	
	time_t fixtime;
	int result = -1;
	int i = 0;
/*	items = sscanf(sentence, "%6s,%f,%c,%f,%c,%f,%c,%f,%f,%d", dtype,
		&times, &flag, &lati, &north, &longi, &west, &knots, 
		&angles, &idate);
*/
	while(sentence[i] != '\n') {
                if(sentence[i] == ',') sentence[i] = ' ';
                i++;
        }
	items = sscanf(sentence, "%6s %s %c %s %c %s %c %s %s %d", dtype,
		item[0], &flag, item[1], &north, item[2], &west, item[3], 
		item[4], &idate);
	times = str_to_f(item[0]);
	lati = str_to_f(item[1]);
	longi = str_to_f(item[2]);
	knots = str_to_f(item[3]);
	angles = str_to_f(item[4]);

	if (flag == 'A') {
		itime = (int)times;
		utctm.tm_sec = itime % 100;
		itime /= 100;
		utctm.tm_min = itime % 100;
		itime /= 100;
		utctm.tm_hour = itime;
		iyear = idate % 100;
		idate /= 100;
		if (iyear < 70)
			iyear += 100;
		utctm.tm_year = iyear;
		utctm.tm_mon = idate % 100 - 1;
		idate /= 100;
		utctm.tm_mday = idate;
		utctm.tm_isdst = 0;
		utctm.tm_gmtoff = 0;
		fixtime = mktime(&utctm);
		if (fixtime > 0) {
			gpsFixUnsafe.fixtime = fixtime;
			gpsFixUnsafe.speedKPH = knots * KILOMETERS_PER_KNOT;
			gpsFixUnsafe.heading = angles;
			gpsFixUnsafe.ageTimer = utcGetTimer();
			gpsFixUnsafe.nmea |= NMEA0183_GPRMC;
			result = 0;
		}
	}
	else
		gpsFixUnsafe.nmea &= ~NMEA0183_GPRMC;
		
	return result;
}
int parse_gsa(char *sentence)
{
	char item[3][32];
	int items = 0, mode = 0;	
	float pdop = 0.0, hdop = 0.0, vdop = 0.0;
	char dtype[8];
	char manual = '\0';
	char *ppdop;
	int result = -1;
	int i = 0;
	items = sscanf(sentence, "%6s,%c,%d", dtype, &manual, &mode);
	if (mode > 1) {
		ppdop = strchr(sentence, '.');
		if (ppdop == NULL)
			return result;
		while (*(--ppdop) != ',')  ;;
		++ppdop;
		while(sentence[i] != '\n') {
                	if(sentence[i] == ',') sentence[i] = ' ';
                	i++;
        	}
//		items = sscanf(ppdop, "%f,%f,%f", &pdop, &hdop, &vdop);
		items = sscanf(ppdop, "%s %s %s", item[0], item[1], item[2]);
		pdop = str_to_f(item[0]);
		hdop = str_to_f(item[1]);
		vdop = str_to_f(item[2]);
		gpsFixUnsafe.pdop = pdop;
		gpsFixUnsafe.vdop = vdop;
		gpsFixUnsafe.hdop = hdop;
		gpsFixUnsafe.nmea |= NMEA0183_GPGSA;
		result = 0;
	}
	else
		gpsFixUnsafe.nmea &= ~NMEA0183_GPGSA;
	return result;
}
int parse_gga(char *sentence)
{
	char item[5][32];
	float times = 0.0, hdop = 0.0, altitude = 0.0;
	double lati = 0.0, longi = 0.0;
	int items = 0, nsat = 0, fixtype = 0;	
	char dtype[8];
	double lat_m, lon_m;
	long lat_d, lon_d; 
	char north = '\0', west = '\0', unit = '\0';
	int result = -1;
	int i = 0;
	static int report_gps_lost = 0;
	static unsigned int gps_lost_count = 0;
	unsigned int gps_lost_tolerance = propGetUInt32(PROP_GPS_LOST_COUNTER, 0);
	static bool gps_lost = false;
/*	items = sscanf(sentence, "%6s,%f,%lf,%c,%lf,%c,%d,%d,%f,%f,%c",  dtype, 
		&times, &lati, &north, &longi, &west, &fixtype, &nsat, &hdop,
		&altitude, &unit);
*/
	while(sentence[i] != '\n') {
                if(sentence[i] == ',') sentence[i] = ' ';
                i++;
        }
        items = sscanf(sentence, "%6s %s %s %c %s %c %d %d %s %s %c",
                dtype, item[0], item[1], &north, item[2], &west, 
		&fixtype, &nsat, item[3], item[4], &unit);

	times = str_to_f(item[0]);
	lati = str_to_d(item[1]);
	longi = str_to_d(item[2]);
	hdop = str_to_f(item[3]);
	altitude = str_to_f(item[4]);

	if ((fixtype == 1) || (fixtype == 2)) {
		w2sg0004_pc15_low(); /* GPS LED on*/
		if (report_gps_lost) {
			diagnostic_report(DIAGNOSTIC_GPS_LOST, 0, NULL);
			report_gps_lost = 0;
		}
		if (gps_lost) {	
			gps_lost = false;
		}
		gps_lost_count = 0;

		lati /= 100.0;
		longi /= 100.0;
		lat_d = (long)lati;
		lon_d = (long)longi;
		lat_m = lati - lat_d;
		lon_m = longi  - lon_d;
		if (north == 'N')
			gpsFixUnsafe.point.latitude = lat_d + lat_m * (10.0 / 6.0);
		else
			gpsFixUnsafe.point.latitude = -(lat_d + lat_m * (10.0 / 6.0));
		if (west == 'W')
			gpsFixUnsafe.point.longitude = -(lon_d + lon_m * (10.0 / 6.0));
		else
			gpsFixUnsafe.point.longitude = lon_d + lon_m * (10.0 / 6.0);
		gpsFixUnsafe.fixtype = fixtype;
		gpsFixUnsafe.altitude = altitude;
		gpsFixUnsafe.nmea |= NMEA0183_GPGGA;
		result = 0;
	}
	else {					// gps is not registered
		w2sg0004_pc15_high();    /* GPS LED off */
		if(!gps_lost && (gps_lost_count >= gps_lost_tolerance)) {
			printf("GPS lost\n");
			gps_lost = true;
		} else if (!gps_lost && (gps_lost_count < gps_lost_tolerance)) {
			printf("GPS lost times %d\n", gps_lost_count);
			gps_lost_count++;
		}
		if (gps_lost && !report_gps_lost) {
			diagnostic_report(DIAGNOSTIC_GPS_LOST, 1, NULL);
			report_gps_lost = 1;
		}
		gpsFixUnsafe.nmea &= ~NMEA0183_GPGGA;
	}
	return result;
}

int synchronize_system_clock(time_t new_time)
{
	int rc = 0;
	struct timespec ts1;
	UInt32 nowTimer =  utcGetTimer();
	ts1.tv_sec = new_time;
	ts1.tv_nsec = 10000000;
	if ((rc = clock_settime(CLOCK_REALTIME, &ts1)) != 0) {
		perror("Set realtime Clock");
	} 
	utcSetStartupTimeSec(new_time - nowTimer);
	rfid_queue_time_adjust = true;
	usleep(10000);
	return rc;
}
void make_nav_init(char * msg)
{
	time_t now, etime;
	uint32_t elapsed, tow, wn;
	etime = mktime(&July2011);
	now = time(NULL);
	elapsed = now - etime;
	wn = elapsed / 604800 + 1644;
	tow = elapsed % 604800;
	snprintf(msg, GPS_SENTENCE_SIZE, "$PSRF104,%2.6lf,%3.6lf,%.1f,96000,%u,%u,12,3*99",
		gpsFixUnsafe.point.latitude, gpsFixUnsafe.point.longitude, gpsFixUnsafe.altitude,
		tow, wn);
}
void checksum_nema(char * msg)
{
	char *ast, *pc;
	unsigned char chsm = 0;

	if ((ast = strrchr(msg, '*')) == NULL)
		return;

	for (pc = msg + 1; pc != ast; pc++)
		chsm ^= *((unsigned char *)pc);

	sprintf(ast + 1, "%02X\r\n\r\n", chsm);
}
/***************pca9535**********/
struct pca9535_ioctl_arg {
    int off;
    int val;
};
#define PCA9535_IOC_MAGIC 0xF6 
#define PCA9535_READ_PIN _IOR(PCA9535_IOC_MAGIC, 0, struct pca9535_ioctl_arg)
#define PCA9535_WRITE_PIN _IOW(PCA9535_IOC_MAGIC,1, struct pca9535_ioctl_arg)
#define PCA9535_READ_PIO _IOR(PCA9535_IOC_MAGIC, 2, struct pca9535_ioctl_arg)
#define PCA9535_WRITE_PIO _IOW(PCA9535_IOC_MAGIC,3, struct pca9535_ioctl_arg)
static char GPIO_DEVICE[] = "/dev/pca9535";
static int pca_fd = -1;
/***************pca9535**********/
int power_up_gps(bool initialize)
{
//	int rc = -1;
//	struct pca9535_ioctl_arg pca_ioc;
//	struct pca9535_ioctl_arg pca_ioc_read;
//	int i = 15;
	
	
	if (initialize || pca_fd < 0) {
		pca_fd = open(GPIO_DEVICE, O_RDWR);
		if(pca_fd < 0) {
			perror("Open PCA9535");
			return pca_fd;
		}
	} 
/*

sleep(60);
printf("pca_ioc_read: ");
while(i >= 0) {
	pca_ioc_read.off = i;
	if ((rc = ioctl(pca_fd, PCA9535_READ_PIN, &pca_ioc_read)) < 0) {
		perror("PCA9535 IOCTL");
		return rc;
	}
	printf("%d",pca_ioc_read.val);
	i--;
}
printf("\n");
*/

/*
	pca_ioc.off = 1;
	pca_ioc.val = 0;

	if ((rc = ioctl(pca_fd, PCA9535_WRITE_PIN, &pca_ioc)) < 0) {
		perror("PCA9535 IOCTL");
		return rc;
	}

i = 15;
printf("pca_ioc_read: ");
while(i >= 0) {
	pca_ioc_read.off = i;
	if ((rc = ioctl(pca_fd, PCA9535_READ_PIN, &pca_ioc_read)) < 0) {
		perror("PCA9535 IOCTL");
		return rc;
	}
	printf("%d",pca_ioc_read.val);
	i--;
}
printf("\n");

*/
	w2sg00xx_i2c_low();

/*
i = 15;
printf("pca_ioc_read: ");
while(i >= 0) {
	pca_ioc_read.off = i;
	if ((rc = ioctl(pca_fd, PCA9535_READ_PIN, &pca_ioc_read)) < 0) {
		perror("PCA9535 IOCTL");
		return rc;
	}
	printf("%d",pca_ioc_read.val);
	i--;
}
printf("\n");
*/

/*	
	pca_ioc.off = (2 << 5) | 15;
	pca_ioc.val = 0;
	if ((rc = ioctl(pca_fd, PCA9535_WRITE_PIO, &pca_ioc)) < 0) {
		perror("PCA9535 IOCTL");
		return rc;
	}
*/
	return 0;
}
int power_down_gps(bool finalize)
{
//	int rc = -1;
//	struct pca9535_ioctl_arg pca_ioc;

	if (pca_fd < 0) {
		pca_fd = open(GPIO_DEVICE, O_RDWR);
		if(pca_fd < 0) {
			perror("Open PCA9535");
			return pca_fd;
		}
	}
/*
	pca_ioc.off = 1;
	pca_ioc.val = 1;
	if ((rc = ioctl(pca_fd, PCA9535_WRITE_PIN, &pca_ioc)) < 0) {
		perror("PCA9535 IOCTL");
		return rc;
	}
*/	
	w2sg00xx_i2c_high();
/*
	pca_ioc.off = (2 << 5) | 15;
	pca_ioc.val = 1;
	if ((rc = ioctl(pca_fd, PCA9535_WRITE_PIO, &pca_ioc)) < 0) {
		perror("PCA9535 IOCTL");
		return rc;
	}

*/
	if (finalize) {
		close(pca_fd);
		pca_fd = -1;
	}

	
	return 0;
}

/* watchdog flash light */
static int running_light_state = 1;
int flash_running_light(int mode)
{
//	int rc = -1;
	struct pca9535_ioctl_arg pca_ioc;
	
	if (mode == 0 && running_light_state == 0)
		return 0;
	if(pca_fd < 0)
		return pca_fd;
	pca_ioc.off = 25;
	if (mode == 0) {
		pca_ioc.val = 1;
		running_light_state = 0;
//		w2sg0004_power_off();
	}
	else {
		running_light_state ^= 1;
		pca_ioc.val = running_light_state & 1;
//		if (pca_ioc.val == 0)
//			w2sg0004_power_on();
//		if (pca_ioc.val == 1)
//			w2sg0004_power_off();
	}
	
/*
	if ((rc = ioctl(pca_fd, PCA9535_WRITE_PIO, &pca_ioc)) < 0) {
		perror("PCA9535 IOCTL");
		return rc;
	}
*/

	return 0;
}

int enable_w2sg0084(void)
{	
	if(w2sg0084_low() < 0)
		return -1;
	usleep(100);
	if(w2sg0084_high() < 0)
		return -1;
	return 0;
}
#endif

static char w2sg0004_i2c_file[]="/sys/aatsi_devs_ctrl/i2c_io2"; 

void w2sg0004_pc15_high(void) {
	int status;
	
	if (gps_led == LED_ON) {
		pid_t child_pid = fork();
		if(child_pid == 0) 
			execl("/ositech/pin_pc15_ctr", "pin_pc15_ctr", "high", NULL);	
	}
	wait(&status);
	gps_led = LED_OFF;
/*
	w2sg0004_pc15_fd = open(w2sg0004_pc15_file, O_RDWR);
	if(w2sg0004_pc15_fd < 0)
		printf("open file /sys/gps/pin_pc15 error!\n");

	write(w2sg0004_pc15_fd, "high", strlen("high"));
	usleep(1000);
*/
}

void w2sg0004_pc15_low(void) {
	int status;
	
	if (gps_led == LED_OFF) {	
	pid_t child_pid = fork();

	
		if(child_pid == 0) {
			execl("/ositech/pin_pc15_ctr", "pin_pc15_ctr", "low", NULL);	
			usleep(1000);
		}
	}
	wait(&status);
	gps_led = LED_ON;
}

void w2sg00xx_i2c_high(void) {
	int w2sg0004_i2c_fd;

	w2sg0004_i2c_fd = open(w2sg0004_i2c_file, O_RDWR);
	if(w2sg0004_i2c_fd < 0)
		printf("open file /sys/aatsi_devs_ctrl/i2c_io2 error!\n");

	write(w2sg0004_i2c_fd, "high", strlen("high"));
	close(w2sg0004_i2c_fd);
	usleep(1000);
}

void w2sg00xx_i2c_low(void) {
	int w2sg0004_i2c_fd;
	
	w2sg0004_i2c_fd = open(w2sg0004_i2c_file, O_RDWR);
	if(w2sg0004_i2c_fd < 0)
		printf("open file /sys/aatsi_devs_ctrl/i2c_io2 error!\n");

	write(w2sg0004_i2c_fd, "low", strlen("low"));
	close(w2sg0004_i2c_fd);
	usleep(1000);
}

int w2sg0084_high(void) {
	int w2sg0084_fd;
	
	w2sg0084_fd = open(W2SG0084_FILE, O_RDWR);
	if(w2sg0084_fd < 0) {
		printf("open file /sys/aatsi_devs_ctrl/gps_pw error!\n");
		return -1;
		}
	write(w2sg0084_fd, "1", sizeof(char));
	close(w2sg0084_fd);
	usleep(100);
	return 0;
}

int w2sg0084_low(void) {
	int w2sg0084_fd;
	
	w2sg0084_fd = open(W2SG0084_FILE, O_RDWR);
	if(w2sg0084_fd < 0) {
		printf("open file /sys/aatsi_devs_ctrl/gps_pw error!\n");
		return -1;
		}
	write(w2sg0084_fd, "0", sizeof(char));
	close(w2sg0084_fd);
	usleep(100);

/*	int status;
	
	if (gps_led == LED_OFF) {	
	pid_t child_pid = fork();

	
		if(child_pid == 0) {
			execl("/ositech/pin_pc15_ctr", "pin_pc15_ctr", "low", NULL);	
			usleep(1000);
		}
	}
	wait(&status);
	gps_led = LED_ON;

	w2sg0004_pc15_fd = open(w2sg0004_pc15_file, O_RDWR);
	if(w2sg0004_pc15_fd < 0)
		printf("open file /sys/gps/power error!\n");

	write(w2sg0004_pc15_fd, "low", strlen("low"));
	usleep(1000);
*/
	return 0;
}

void print_data(int n, char *sentence) {
	int pos = 0;
	
	if (n > 0 && gps_debug) {
			while(pos < n)
				printf("%c", *(sentence+pos++));
		}
}

/***********************************************************
 * establish_gps_socket
 * Description: establish gps socket
 * Input Argument: NONE
 * Return Value: GPS socket file descriptor, when succeeds
 *              -1 = failed
 **********************************************************/
static int establish_gps_socket(void)
{
	int sock0;
	struct sockaddr_un sockaddr_gps;
	
	if ((sock0 = socket(AF_UNIX, SOCK_DGRAM, 0)) < 0) {
		perror("creating socket");
		return sock0;
	}
	memset(&sockaddr_gps, 0, sizeof(sockaddr_gps));
	sockaddr_gps.sun_family = AF_UNIX;
	strcpy(sockaddr_gps.sun_path, DMTP_GPS);
	unlink(DMTP_GPS);
	if (bind(sock0, (struct sockaddr *)&sockaddr_gps,
									sizeof(sockaddr_gps)) < 0) {
		perror("socket subscriber bind");
		return -1;
	}
	printf("subscribed for GPS\n");
	return sock0;
}

/***********************************************************
 * subscribe_gps
 * Description: subscribe to GPS publishing
 * Input Argument: NONE
 * Return Value: 0 = success
 *              -1 = failed
 **********************************************************/
static int subscribe_gps(void)
{
	int sock_subscriber;
	int slen, my_len, ret;
	struct sockaddr_un addr_publisher;
	char subscription_buf[64];
	
	if ((sock_subscriber = socket(AF_UNIX, SOCK_DGRAM, 0)) < 0) {
		perror("creating socket");
		return -1;
	}
	memset(&addr_publisher, 0, sizeof(addr_publisher));
	addr_publisher.sun_family = AF_UNIX;
	strcpy(addr_publisher.sun_path, GPS_PUBLISHER);
	if (connect(sock_subscriber, (struct sockaddr *)&addr_publisher,
										sizeof(addr_publisher)) < 0) {
		perror("socket connect");
		return -1;
	}
	memset(subscription_buf, 0, sizeof(subscription_buf));
	subscription_buf[0] =  'M';
	subscription_buf[1] =  (MESSAGE_ID_SUBSCRIBE >> 8) & 0xFF;
	subscription_buf[2] =  MESSAGE_ID_SUBSCRIBE & 0xFF;
	my_len = strlen(DMTP_GPS);
	subscription_buf[3] = (char)my_len;
	strcpy(subscription_buf + 4, DMTP_GPS);
	slen = send(sock_subscriber, subscription_buf, my_len + 4, 0);
	if (slen < 0) {
		perror("subscription socket send");
		ret = -1;
	} else {
		ret = 0;
	}
	usleep(100000);
	close(sock_subscriber);
	return ret;
}

/***********************************************************
 * cancel_gps_subscription
 * Description: terminate subscription of GPS publishing
 * Input Argument: NONE
 * Return Value: NONE
 **********************************************************/
static void cancel_gps_subscription(void)
{
	int sock_subscriber;
	int slen, my_len, ret;
	struct sockaddr_un addr_publisher;
	char subscription_buf[64];
	
	if ((sock_subscriber = socket(AF_UNIX, SOCK_DGRAM, 0)) < 0) {
		perror("creating socket");
		return;
	}
	memset(&addr_publisher, 0, sizeof(addr_publisher));
	addr_publisher.sun_family = AF_UNIX;
	strcpy(addr_publisher.sun_path, GPS_PUBLISHER);
	if (connect(sock_subscriber, (struct sockaddr *)&addr_publisher,
										sizeof(addr_publisher)) < 0) {
		perror("socket connect");
		return;
	}
	memset(subscription_buf, 0, sizeof(subscription_buf));
	subscription_buf[0] =  'M';
	subscription_buf[1] =  (MESSAGE_ID_UNSUBSCRIBE >> 8) & 0xFF;
	subscription_buf[2] =  MESSAGE_ID_UNSUBSCRIBE & 0xFF;
	my_len = strlen(DMTP_GPS);
	subscription_buf[3] = (char)my_len;
	strcpy(subscription_buf + 4, DMTP_GPS);
	slen = send(sock_subscriber, subscription_buf, my_len + 4, 0);
	if (slen < 0) {
		perror("subscription socket send");
		ret = -1;
	} else {
		ret = 0;
	}
	usleep(100000);
	close(sock_subscriber);
}