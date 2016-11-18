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
//  Main GPS aquisition/process loop.
// ---
// Change History:
//  2006/01/04  Martin D. Flynn
//     -Initial release
//  2006/02/09  Martin D. Flynn
//     -Improved "stale" GPS fix checking
//  2006/04/10  Martin D. Flynn
//     -Send event on first GPS fix aquisition
//  2007/01/28  Martin D. Flynn
//     -WindowsCE port
// ----------------------------------------------------------------------------

#include "defaults.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <ctype.h>
#include <sys/socket.h>
#include <sys/syslog.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/netlink.h>
#include <linux/rtnetlink.h>
#include <linux/ip.h>
#include <linux/icmp.h>
#include <netdb.h>
#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <openssl/bio.h>
#include <openssl/ssl.h>
#include <openssl/err.h>
#include <openssl/evp.h>
#include <openssl/md5.h>
#include "log.h"
#include "startup.h"
#include "transport.h"
#include "gps.h"
#include "gpsmods.h"
#include "stdtypes.h"
#include "utctools.h"
#include "gpstools.h"
#include "strtools.h"
#include "motion.h"
#include "odometer.h"
#include "propman.h"
#include "statcode.h"
#include "mainloop.h"
#if !defined(PROTOCOL_THREAD)
#include "accting.h"
#endif
#include "protocol.h"
#include "packet.h"
#include "events.h"
#include "base64.h"
#include "pqueue.h"
#include "rfid.h"
#include "libwlcomm.h"
#include "watchdog_f.h"
#include "ap_diagnostic_log.h"
#include "qdac.h"
#include "diagnostic.h"
#include "network_diagnostic.h"
#include "SerialPinMonitor.h"
// ----------------------------------------------------------------------------
#ifdef ENABLE_THREADS
#include "threads.h"
#define MAIN_THREAD

extern int ZPDEBUG = 0;

extern threadThread_t	gpsThread;
extern threadThread_t	protocolThread;
extern threadThread_t	logThread;
threadThread_t	mainThread;
pthread_t	thread0;
pthread_t	thread_update;
extern pthread_t	thread_rfid_main;
extern pthread_t	thread_rfid_reader;
#endif
extern char *get_build_version(void);
extern int stop_watchdog(void);
extern void network_diagnostic_init(void);
extern int udp_recv_timeout;
// ----------------------------------------------------------------------------
static TimerSec_t               lastGPSAquisitionTimer = (TimerSec_t)0L;
static TimerSec_t               lastModuleCheckTimer = (TimerSec_t)0L;
static GPS_t                    lastValidGPSFix; // needs to be initialized
static GPS_t                    newFix; // needs to be initialized
static TimerSec_t               gpsStaleTimer = (TimerSec_t)0L;
static eventAddFtn_t            ftnQueueEvent = 0;
#if !defined(PROTOCOL_THREAD)
static PacketEncoding_t         defaultEncoding = 0;
#endif
/*static int debug_counter = 0; */
// ----------------------------------------------------------------------------
utBool mainRunThread = utFalse;
bool update_pending = false;
bool update_error = false;
uint32_t volatile wireless_stuck = 0;
uint32_t gpsEventCycle;
extern uint32_t network_link_status;
extern bool reboot_pending;
bool link_down_occurred = false;
bool network_manager_running;
static uint32_t link_down_count = 0;
extern pthread_cond_t	network_up_sema;
extern pthread_cond_t	network_down_sema;
extern pthread_mutex_t	network_status_mutex;
char logging_server_url[MAX_PAYLOAD_SIZE] = "216.171.106.21/dmtplog/log_receiver.cgi";
void * download_update(void *arg);
// ----------------------------------------------------------------------------
static void * _mainLoopRun(void *arg);
static void _mainRunLoopStop(void *arg);
void report_mission_status(int mission_status, char *Reason);
bool watchdog_network_monitor(void);
static bool network_status(void);
static void report_bootup_status();
int upload_debuglog(void);
static void setLocalTime(void);

#define RFID_ENABLE 1
#define QDAC_ENABLE 2
#define NETWORK_TERMINATE_IDLE	180
#define NETWORK_DOWN_IDLE	1801
#define NETWORK_MINUTES 60
#define NETWORK_START_IDLE	67
#define NETWORK_MANAGER_LOOP_CYCLE	79	
#define NETWORK_PING_TIMEOUT	79
#define NETWORK_BREAK_PERIOD	10
//#define NETWORK_BREAK_PERIOD	7
#define CLOCK_SYNC_INIT_CYCLE 23
#define CLOCK_SYNC_CYCLE 1093
#define NANO_SECOND 1.0E-9
#define JAN_1970 2208988800UL /* 1970 - 1900 in seconds */
static char update_daemon[48] = "/mnt/flash/titan-data/update.daemon";
static char update_stat_file[48] = "/mnt/flash/titan-data/update.stat";
static char update_file[48] = "/mnt/flash/titan-data/updateme.tgz";

// ----------------------------------------------------------------------------
void interrupt_handler(int sig)
{
	int reader_type = propGetUInt32AtIndex(PROP_RFID_READER_ENABLE, 0, 1);
	printf("User Interrupts\n");
	if (!pthread_equal(pthread_self(), thread0))
		pthread_kill(thread0, sig);
	threadStopThreads();
/*	if READER_TYPE = rfid */
/* if READER_TYPE has rfid enabled */ 
	if (reader_type & RFID_ENABLE) {
  		rfid_stop();
		rfid_port_close();
		}
/* if READER_TYPE has qdac enabled */
	if (reader_type & QDAC_ENABLE) {
  		qdac_stop();
		qdac_port_close();
		}
/* 	if READER_TYPE = qdac 
	qdac_stop();
	qdac_port_close();
*/
/*	if READER_TYPE = both
	rfid_stop();
	rfid_port_close();
	qdac_stop();
	qdac_port_close();
*/	
}
// ----------------------------------------------------------------------------
void terminate_handler(int sig)
{
	if (pthread_equal(pthread_self(), thread0)) {
		network_manager_running = false;
		printf("User Stops Program\n");
		pthread_cond_signal(&network_down_sema);
	}
}
// ----------------------------------------------------------------------------
void timeout_handler(int sig, siginfo_t *sinfo, void *context)
{
	if (sinfo->si_value.sival_int == TIMER_PROTOCOL_1) {
		if (!pthread_equal(pthread_self(), protocolThread.thread)) {
			pthread_kill(protocolThread.thread, sig);
			printf("Redirect timeout signal %d to protocol thread\n", 
					sig);
		}
		network_link_status = NETWORK_STATUS_TIMEOUT;
		pthread_cond_signal(&network_down_sema);
	}
	else if (sinfo->si_value.sival_int == TIMER_RFID_1) {
		if (!pthread_equal(pthread_self(), thread_rfid_reader)) {
			pthread_kill(thread_rfid_reader, sig);
			printf("Redirect timeout signal %d to rfid thread\n", 
					sig);
		}
	}
	else if (sinfo->si_value.sival_int == TIMER_GPS_1) {
		if (!pthread_equal(pthread_self(), gpsThread.thread)) {
			pthread_kill(gpsThread.thread, sig);
			printf("Redirect timeout signal %d to gps thread\n", 
					sig);
		}
		gps_port_timeout = -1;
	}
	else if (sinfo->si_value.sival_int == TIMER_NETWORK_MANAGER) {
		if (!pthread_equal(pthread_self(), thread0)) {
			pthread_kill(thread0, sig);
			printf("Redirect timeout signal %d to network manager\n", 
					sig);
			debuglog(LOG_INFO, "Redirect timeout signal %d to network manager", sig);
		}
//		wireless_stuck++;
		wireless_stuck = 1;
//		printf("%s wireless_stuck set\n", __FUNCTION__);
		debuglog(LOG_INFO,"%s wireless_stuck set", __FUNCTION__);
	}
	else if (sinfo->si_value.sival_int == TIMER_PROTOCOL_2) {
		if (!pthread_equal(pthread_self(), protocolThread.thread)) {
			pthread_kill(protocolThread.thread, sig);
			printf("Redirect timeout signal %d to protocol thread\n", 
					sig);
		}
		udp_recv_timeout = -1;
	}
	else if (sinfo->si_value.sival_int == TIMER_UPDATE) {
		if (!pthread_equal(pthread_self(), thread_update)) {
			pthread_kill(thread_update, sig);
			printf("Redirect timeout signal %d to update thread\n", 
					sig);
		}
	}
	else if (sinfo->si_value.sival_int == TIMER_POWER_SAVING) {
		if (!pthread_equal(pthread_self(), protocolThread.thread)) {
			pthread_kill(protocolThread.thread, sig);
			printf("Redirect timeout signal %d to protocol thread\n", 
					sig);
		}
//		wireless_stuck++;
		wireless_stuck = 1;
	}
	debuglog(LOG_INFO, "Timer %d expired", sinfo->si_value.sival_int); 
	printf("Timer %d expired\n", sinfo->si_value.sival_int); 
}
// ----------------------------------------------------------------------------
/* main loop module initialization */
static utBool didInitialize = utFalse;
void mainLoopInitialize(eventAddFtn_t queueEvent)
{
	struct sigaction sa, sa0;
	int reader_type = 0;
	int SerialRTSMonitor = 0;
	int iBoxMonitor = 0;

	/* adjust the local time first */
    	setLocalTime();

	sa.sa_handler = interrupt_handler;
	sigemptyset(&sa.sa_mask);
	sa.sa_flags = SA_RESETHAND;
	if (sigaction(SIGINT, &sa, NULL) == -1) {
		perror("Sigaction");
	}
	sa.sa_handler = terminate_handler;
	sigemptyset(&sa.sa_mask);
	sa.sa_flags = 0;
	if (sigaction(SIGUSR1, &sa, NULL) == -1) {
		perror("Sigaction");
	}
	sa0.sa_sigaction = timeout_handler;
	sa0.sa_flags = SA_SIGINFO;
	if (sigaction(SIG_TIMEOUT, &sa0, NULL) == -1) {
		perror("sigaction");
	}
	thread0 = pthread_self();
    ftnQueueEvent = queueEvent;
    gpsClear(&lastValidGPSFix);
    gpsClear(&newFix);
    lastGPSAquisitionTimer = (TimerSec_t)0L;
    lastModuleCheckTimer = (TimerSec_t)0L;
    gpsStaleTimer = (TimerSec_t)0L;
    gpsInitialize(ftnQueueEvent); // threaded
    reader_type = propGetUInt32AtIndex(PROP_RFID_READER_ENABLE, 0, 1);
    SerialRTSMonitor = propGetUInt32AtIndex(PROP_STATE_RTS_CHECK, 0, 0);
    iBoxMonitor = propGetUInt32AtIndex(PROP_STATE_iBOX_ENABLE, 0, 0);
/* if READER_TYPE has rfid enabled */ 
	if (reader_type & RFID_ENABLE)
    		rfid_initialize();
/* if READER_TYPE has qdac enabled */
	if (reader_type & QDAC_ENABLE)
    		qdac_initialize();
/* Serial RTS monitor */
	if (SerialRTSMonitor) 
		SerialPinMonitor_initialize();
/* iBox monitor */
	if (iBoxMonitor)
		iBoxMonitor_initialize();
	
    gpsModuleInitialize(ftnQueueEvent); // threaded
#ifdef MAIN_THREAD
	mainRunThread = utTrue;
	if (threadCreate(&mainThread, _mainLoopRun, 0, "MainLoop") == 0) {
		// thread started successfully
		threadAddThreadStopFtn(&_mainRunLoopStop, 0);
	} else {
		logCRITICAL(LOGSRC, "Unable to create main thread!!");
		mainRunThread = utFalse;
	}
#endif
    /* did initialize */
    didInitialize = utTrue;
}

// ----------------------------------------------------------------------------

/* add a motion event to the event queue */
static void _queueMotionEvent(PacketPriority_t priority, StatusCode_t code, const GPS_t *gps)
{
    if (gps && ftnQueueEvent) {
        Event_t evRcd;
        evSetEventDefaults(&evRcd, code, 0L, gps);
        (*ftnQueueEvent)(priority, DEFAULT_EVENT_FORMAT, &evRcd);
    }
}

// ----------------------------------------------------------------------------

#ifdef MAIN_THREAD
/* indicate main thread should stop */
void _mainRunLoopStop(void *arg)
{
	mainRunThread = utFalse;
}
#endif

#ifndef MAIN_THREAD
// GPS aquisition & data transmission loop
utBool mainLoopRun(PacketEncoding_t dftEncoding, utBool runInThread)
{
    defaultEncoding = dftEncoding;
    /* start main loop */
    mainRunThread = utTrue;
    if (runInThread) {
        if (threadCreate(&mainThread,_mainLoopRun,0,"MainLoop") == 0) {
            // thread started successfully
            threadAddThreadStopFtn(&_mainRunLoopStop,0);
            return utTrue;
        } else {
            logCRITICAL(LOGSRC,"Unable to create main thread!!");
            mainRunThread = utFalse;
            return utFalse;
        }
    } else {
        /* main loop */
        _mainLoopRun((void*)0);
        // does not return (unless requested to stop)
        return utFalse;
    }
    /* main loop */
    _mainLoopRun((void*)0);
    // does not return (unless requested to stop)
    return utFalse;

}
#endif

// GPS aquisition & data transmission loop
static void * _mainLoopRun(void *arg)
{
	UInt32 gpsAquireTimeoutSec;
	utBool gpsAcquired = utFalse;
	GPS_t *gps = (GPS_t*)0; 
	// acquire valid GPS fix to start with
	while (mainRunThread) {
		gpsAquireTimeoutSec = propGetUInt32(PROP_GPS_AQUIRE_WAIT, 0L);
		if (!gpsAcquired) {
			gpsAcquireWait();
			if (!mainRunThread)
				break;
			gps = gpsGetLastGPS(&lastValidGPSFix, gpsAquireTimeoutSec);
			logINFO(LOGSRC,"GPS fix: %.5lf/%.5lf", 
					gps->point.latitude, gps->point.longitude);
			_queueMotionEvent(PRIORITY_NORMAL, STATUS_INITIALIZED, &lastValidGPSFix);
			sleep(gpsEventCycle);
			if (gps_power_saving) {
				protocolStartSession();
				continue;
			}
			else
				gpsAcquired = utTrue;
		}
		// acquire GPS fix 
		gps = gpsGetLastGPS(&newFix, gpsAquireTimeoutSec);
		if (gps) {
			gpsModuleCheckEvents(&lastValidGPSFix, &newFix);
			gpsCopy(&lastValidGPSFix, &newFix);
		}
		else {
		/*We have previously received at least 1 valid GPS fix.  Set the timer to
		the last valid fix, and compare the age of the fix to the GPS expiration interval */
			lastValidGPSFix.speedKPH = 0;
			gpsModuleCheckEvents(&lastValidGPSFix, &lastValidGPSFix);
			gpsAcquired = utFalse;
			motionResetStatus();
			logINFO(LOGSRC,"Lost GPS signal"); 
		} 
		// miscellaneous housekeeping items should go here
		startupMainLoopCallback();
        
#if !defined(PROTOCOL_THREAD)
		// -----------------
		// time to transmit? (we have data and/or minimum times have expired)
		// If the protocol transport is NOT running in a separate thread, this
		// function will block until the connection is closed.
		protocolTransport(0,defaultEncoding);
#endif
		sleep(gpsEventCycle);
	}
    // only reaches here when thread terminates
    // Note: this _may_ be the *main* thread
	/*protocolStartSession(); */
	return NULL;
}
// ----------------------------------------------------------------------------

int startNetwork(timer_t timer_a, struct itimerspec *it_a)
{
	int status, request;
//	system("dmtp_ap_start &");
	print_critical("Start Wireless Link\n");
	debuglog(LOG_INFO,"Start Wireless Link");
//	it_a->it_value.tv_sec = (NETWORK_PING_TIMEOUT << 2);
	it_a->it_value.tv_sec =propGetUInt32(PROP_STATE_STUCK_TIMEOUT, 900); 
//	it_a->it_value.tv_sec = (NETWORK_PING_TIMEOUT << 3);	// to create more time for the connection
	it_a->it_value.tv_nsec = 0;
	it_a->it_interval.tv_sec = NETWORK_BREAK_PERIOD;
	it_a->it_interval.tv_nsec = 0;
	timer_settime(timer_a, 0, it_a, NULL);
	request = WIFI_ESTABLISHING;
//	if (is_wifi_logging())
//		request |= 0x10000;
	status = libwlcomm_main(request, NULL);
	CheckCarrier();
	if (wireless_stuck) {
		debuglog(LOG_INFO, "%s calling lib returned, now reset wireless_stuck to 0",
			__FUNCTION__);
		wireless_stuck = 0;
	}
	if ((status == WIFI_RET_ESTABLISHED) || (status == WIFI_RET_LIVE)) {
		debuglog(LOG_INFO,"Connection establish SUCCESS");
		status = 0;
	}
	else {
		debuglog(LOG_INFO,"Connection establish FAIL");
		status = -1;
	}
	it_a->it_value.tv_sec = 0;
	it_a->it_value.tv_nsec = 0;
	it_a->it_interval.tv_sec = 0;
	it_a->it_interval.tv_nsec = 0;
	timer_settime(timer_a, 0, it_a, NULL);
	system("dmtp_ap_start &");
	return status;
}
int terminateNetwork(timer_t timer_a, struct itimerspec *it_a)
{
	int status = 0, request;

	print_critical("Terminate Wireless Link\n");
	debuglog(LOG_INFO,"Terminate Wireless Link");
	it_a->it_value.tv_sec = NETWORK_PING_TIMEOUT;
	it_a->it_value.tv_nsec = 0;
	it_a->it_interval.tv_sec = NETWORK_BREAK_PERIOD;
	it_a->it_interval.tv_nsec = 0;
	timer_settime(timer_a, 0, it_a, NULL);
	request = WIFI_STOP;
//	if (is_wifi_logging())
//		request |= 0x10000;
	status = libwlcomm_main(request, NULL);
	if (wireless_stuck) {
		debuglog(LOG_INFO, "%s calling lib returned, now reset wireless_stuck to 0",
			__FUNCTION__);
		wireless_stuck = 0;
	}
	if (status != WIFI_RET_DOWNOK) {
		debuglog(LOG_INFO,"Terminate Wireless Link ERROR");
		print_critical("WiFi_down error %x", status);
	} else 
		debuglog(LOG_INFO,"Terminate Wireless Link SUCCESS");
	it_a->it_value.tv_sec = 0;
	it_a->it_value.tv_nsec = 0;
	it_a->it_interval.tv_sec = 0;
	it_a->it_interval.tv_nsec = 0;
	timer_settime(timer_a, 0, it_a, NULL);
	request_log_collect();
	return status;
}
bool checkNetworkDown(timer_t timer_a, struct itimerspec *it_a)
{
	bool result;
	int request;
	print_critical("Checking Wireless Link!\n");
	debuglog(LOG_INFO,"Cannot talk to server & Checking Wireless Link!");
//	it_a->it_value.tv_sec = NETWORK_PING_TIMEOUT;
	it_a->it_value.tv_sec =propGetUInt32(PROP_STATE_CHECKNETWORK_TIMEOUT, 79); 
//	it_a->it_value.tv_sec = 45;
	it_a->it_value.tv_nsec = 0;
	it_a->it_interval.tv_sec = NETWORK_BREAK_PERIOD;
	it_a->it_interval.tv_nsec = 0;
	timer_settime(timer_a, 0, it_a, NULL);
	request = WIFI_EXISTING;
//	if (is_wifi_logging())
//		request |= 0x10000;
//	if (libwlcomm_main(request, NULL) == WIFI_RET_EXISTING)
	result = network_status();
	if (wireless_stuck) {
		debuglog(LOG_INFO, "%s calling lib returned, now reset wireless_stuck to 0",
			__FUNCTION__);
		wireless_stuck = 0;
		diagnostic_report(DIAGNOSTIC_LIB_STUCK, wireless_stuck, NULL);
	}
//printf("!!%d %s\n", result, (result == false)?"existing":"not existing");
	it_a->it_value.tv_sec = 0;
	it_a->it_value.tv_nsec = 0;
	it_a->it_interval.tv_sec = 0;
	it_a->it_interval.tv_nsec = 0;
	timer_settime(timer_a, 0, it_a, NULL);
	return result;
}
#ifdef ENABLE_SNTP
struct tstamp {
	uint32_t sec;
	uint32_t frac;
};
struct ntp_packet {
	unsigned char li_vn_mode;
	unsigned char stratum;
	unsigned char poll;
	unsigned char precision;
	uint32_t	root_delay;
	uint32_t	root_disp;
	uint32_t	refid;
	struct tstamp ref;
	struct tstamp org;
	struct tstamp rec;
	struct tstamp xmt;
} __attribute__((__aligned__));

static struct ntp_packet ntp_request;
static struct ntp_packet ntp_reply;
static char server1[64] = "0.north-america.pool.ntp.org";
static char service1[16] = "ntp";
int check_time(void)
{	
	struct addrinfo hints, sainfo, *result, *painfo;
	struct sockaddr_in saddr_in;
	struct timespec time1, time2, time3;
	double fraction; 
	uint64_t nsec64;
	int32_t rtrip, ndiff;
	int sockfd, ret, myerr, rlen, tdiff, accuracy;
	uint32_t old, na2;

	memset(&hints, 0, sizeof hints);
	hints.ai_family = AF_INET;
	hints.ai_socktype = SOCK_DGRAM;
	if ((ret = getaddrinfo(server1, service1, &hints, &result)) != 0) {
		fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(ret));
		return -1;
	}
	sainfo = *result;
	painfo = &sainfo;
	memcpy(&saddr_in, painfo->ai_addr, painfo->ai_addrlen);
	freeaddrinfo(result);

	if ((sockfd = socket(painfo->ai_family, painfo->ai_socktype, painfo->ai_protocol)) == -1) {
		perror("Socket");
		return sockfd;
	}
	ret = -1;
	if ((ret = connect(sockfd, (struct sockaddr *)(&saddr_in), painfo->ai_addrlen)) < 0) {
		myerr = errno;
		perror("connect");
		if (myerr == EHOSTUNREACH)
			printf("Host is unreachable\n");
		else if (myerr == ENETUNREACH)
			printf("Internet is unreachable\n");
		goto exit2;
	}
	memset(&ntp_request, 0, sizeof(ntp_request));
	if ((myerr = clock_gettime(CLOCK_REALTIME, &time1)) < 0) {
		perror("Realtime clock");
		goto exit2;
	}
	ntp_request.xmt.sec = htonl(time1.tv_sec + JAN_1970);
	fraction =(double)(time1.tv_nsec) * NANO_SECOND * (double)0x10000;
	ntp_request.xmt.frac = htonl((uint32_t)fraction << 16);
	ntp_request.li_vn_mode = (3 << 3) | 3;
	if ((ret = send(sockfd, (unsigned char *)&ntp_request, sizeof(ntp_request), 0)) < 0) {
		perror("Send:");
		goto exit2;
	}
	memset(&ntp_reply, 0, sizeof(ntp_reply));
	if ((rlen = recvfrom(sockfd, (unsigned char *)&ntp_reply, sizeof(ntp_reply), 0,
						(struct sockaddr *)(&saddr_in), &painfo->ai_addrlen)) < 0) {
												 				
		perror("Receive:");
		goto exit2;
	} else if (rlen == 0) {
		printf("Peer closed\n");
		ret = 0;
		goto exit2;
	}
	ret = 0;
	if ((myerr = clock_gettime(CLOCK_REALTIME, &time3)) < 0) {
		perror("Realtime clock");
		goto exit2;
	}
	rtrip = time3.tv_sec - time1.tv_sec;
	ndiff = time3.tv_nsec - time1.tv_nsec;
	if (ndiff < 0) {
		--rtrip;
		ndiff += 1000000000;
	} 
	ndiff >>= 1;
	if ((rtrip & 1) == 1)
		ndiff += 500000000;
	rtrip >>= 1;
	time2.tv_sec = ntohl(ntp_reply.xmt.sec) + rtrip - JAN_1970 ;
	tdiff = time2.tv_sec -  time3.tv_sec;
	accuracy = (int)propGetUInt32(PROP_GPS_CLOCK_DELTA, 3600);
	if (tdiff > accuracy || tdiff < -accuracy) {
		nsec64 = (uint64_t)(ntohl(ntp_reply.xmt.frac)) * 1000000000;
		na2 = (uint32_t)(nsec64 >> 32) + ndiff;
		if (na2 > 1000000000) {
			time2.tv_nsec = na2 - 1000000000;
			time2.tv_sec++;
		} else
			time2.tv_nsec = na2;
		old = utcGetTimer();
		if (clock_settime(CLOCK_REALTIME, &time2) != 0) {
			perror("Set Realtime Clock:");
			goto exit2;
		} 
		utcSetStartupTimeSec(time2.tv_sec - old);
		update_timestamp(tdiff);
		rfid_queue_time_adjust = true;
		usleep(10000);
	}
exit2:
	close(sockfd);
	return ret;
}
static int clock_sync_timer = 0;
#endif

/* for signal testing */
static bool turn_down_network = false;
void turn_down_connection(int param) {
	printf("turn_down_network signal received, handling it...");
	turn_down_network = true;
}
/****************/

static struct sigevent sev5;
static struct itimerspec it5;
static timer_t timer5;

int network_manager(void)
{
	struct timespec later;
	int err, child_id;
	int fd, res;
	char network_pid[8] = {};
	bool network_link_down = false;
	bool power_saving = propGetUInt32(PROP_COMM_POWER_SAVING, 0L)? true : false;
	int reader_type = propGetUInt32AtIndex(PROP_RFID_READER_ENABLE, 0, 1);
	int network_down_idle = propGetUInt32AtIndex(PROP_COMM_NET_IDLE_MINUTES, 0, 30);
	int min_rate = propGetUInt32(PROP_COMM_MIN_XMIT_RATE, 1);
	UInt32 report_bootup = propGetUInt32(PROP_STATE_BOOTUP_REPORT, 0);
	
	network_down_idle *= NETWORK_MINUTES;
	int idle_minutes = 0;
	
	struct tm tm1; 
	struct tm *ptm;
	time_t now;
	
	signal(SIGCHLD, SIG_IGN);

/* for signal testing */
	fd = open("/tmp/dmtp_network_pid", O_WRONLY | O_CREAT);
	if (fd < 0) {
		perror("open");
		goto exit5;
	}
	snprintf(network_pid, sizeof(pid_t)+1, "%d", getpid());
//	sprintf(network_pid, "%d", getpid());
	printf("%s\n", network_pid);
	res = write(fd, network_pid, sizeof(network_pid));
	signal(SIGUSR2, turn_down_connection);
/****************/

	
	/*setup timer */
	sev5.sigev_notify = SIGEV_SIGNAL;
	sev5.sigev_signo = SIG_TIMEOUT;
	sev5.sigev_value.sival_int = TIMER_NETWORK_MANAGER;
	err = timer_create(CLOCK_REALTIME, &sev5, &timer5);
	it5.it_interval.tv_sec = 0; 
	it5.it_interval.tv_nsec = 0;
	it5.it_value.tv_sec = 0;
	it5.it_value.tv_nsec = 0;
	add_watchdog_func(watchdog_network_monitor);
	if (network_link_status == NETWORK_STATUS_INIT) {
		printf("calling startNetwork at the beginning\n");
		network_link_status = startNetwork(timer5, &it5);
		debuglog(LOG_INFO, "return from startNetwork with network_link_status %d\n", network_link_status);
		if (network_link_status == 0) {
			pthread_mutex_lock(&network_status_mutex);
			pthread_cond_broadcast(&network_up_sema);
			pthread_mutex_unlock(&network_status_mutex);
		}
	}
	network_diagnostic_init();
		
	if(report_bootup)	report_bootup_status();
//	system("run_led_on &");
	network_manager_running = true;
	while (network_manager_running) {
		pthread_mutex_lock(&network_status_mutex);
//		while ((network_link_status == 0) && network_manager_running) {

/* add the check of turn_down_network signal */
		while (!turn_down_network && (network_link_status == 0) && network_manager_running) {
			if ((err = clock_gettime(CLOCK_REALTIME, &later)) != 0) {
				perror("Realtime Clock:");
				goto exit5;
			}
//org			later.tv_sec += NETWORK_MANAGER_LOOP_CYCLE;
			later.tv_sec += min_rate;
			err = pthread_cond_timedwait(&network_down_sema, &network_status_mutex, &later);

			if (network_link_status == 0) {
				pthread_mutex_unlock(&network_status_mutex);
				property_maintenance();
				if (update_pending && update_error) {
					pthread_join(thread_update, NULL);
					update_pending = false;
					update_error = false;
					printf("Update Thread Ended\n");
				}
				if (reboot_pending)
					network_manager_running = false;
				link_down_occurred = false;
				pthread_mutex_lock(&network_status_mutex);
			}
		}
		pthread_mutex_unlock(&network_status_mutex);
		if (!network_manager_running)
			break;
		/*check network's integraty*/
		/* if power saving mode is not being enabled *
		
/* add the check of turn_down_network signal */
	if (!turn_down_network) {
		if (power_saving == false) {
			printf("calling checkNetworkDown\n");
			diagnostic_report(DIAGNOSTIC_CNNCT_CHECK, 0, NULL);
			network_link_down = checkNetworkDown(timer5, &it5);
			pthread_mutex_lock(&network_status_mutex);
			if (network_link_down) {
				debuglog(LOG_INFO,"Network is down");
				diagnostic_report(DIAGNOSTIC_CNNCT_DOWN, 1, NULL);
				network_link_status |= NETWORK_STATUS_ERROR;
				link_down_occurred = true;
				pthread_mutex_unlock(&network_status_mutex);
				sleep(NETWORK_BREAK_PERIOD);
			} else {
				debuglog(LOG_INFO,"Network is alive");
				diagnostic_report(DIAGNOSTIC_CNNCT_DOWN, 0, NULL);
				network_link_status = 0;
				pthread_cond_broadcast(&network_up_sema);
				pthread_mutex_unlock(&network_status_mutex);
				continue;
			}
		} else {
			network_link_status |= NETWORK_STATUS_ERROR;
			link_down_occurred = true;
			pthread_mutex_unlock(&network_status_mutex);
			network_link_down = 1;
		}
	}
		while ((turn_down_network || network_link_down) && network_manager_running) {
			/* turn_down_network signal is received */
			if (turn_down_network) {
				terminateNetwork(timer5, &it5);
				turn_down_network = false;
				printf("Network is down!Rebuild a network in 3 seconds\n");
				sleep(3);
			} else {
			/* if power saving mode is not being enabled */
				if (power_saving == false) {
					network_link_status |= NETWORK_STATUS_ERROR;
					printf("calling terminateNetwork if network is down\n");
//				diagnostic_report(DIAGNOSTIC_CNNCT_DOWN, idle_minutes);
				
					terminateNetwork(timer5, &it5);
					if (link_down_count++ < 3)
						idle_minutes = NETWORK_TERMINATE_IDLE;
//					sleep(NETWORK_TERMINATE_IDLE);
					else
						idle_minutes = network_down_idle;
					printf("Network is down!Rebuild a network in %d mins\n", idle_minutes /NETWORK_MINUTES);
					sleep(idle_minutes);
					if (!network_manager_running)
						goto exit5;
					system("dmtp_ap_stop");
					sleep(1);
				}	
			}
			now = time(NULL);
			ptm = localtime_r(&now, &tm1);

			print_debug("%02d/%02d/%4d %02d:%02d:%02d: Start Network\n", 
						ptm->tm_mon+1, ptm->tm_mday, ptm->tm_year+1900, 
						ptm->tm_hour, ptm->tm_min, ptm->tm_sec); 
			printf("calling startNetwork while network is down\n");
			diagnostic_report(DIAGNOSTIC_CNNCT_REBUILT, 0, NULL);
			network_link_down = (startNetwork(timer5, &it5) < 0);
			if (network_link_down) {
				if (power_saving)
					printf("Network is down\n");
				diagnostic_report(DIAGNOSTIC_CNNCT_DOWN, 1, NULL);
				continue;
			//	sleep(NETWORK_START_IDLE);
			}
			else {
				diagnostic_report(DIAGNOSTIC_CNNCT_REBUILT, 1, NULL);
				link_down_count = 0;
			}
		}
		pthread_mutex_lock(&network_status_mutex);
		network_link_status = 0;
		pthread_cond_broadcast(&network_up_sema);
		pthread_mutex_unlock(&network_status_mutex);
	}
exit5:
	if (update_pending) {
		pthread_join(thread_update, NULL);
		if ((child_id = fork()) == 0) {
			if (execl(update_daemon, update_daemon, NULL) == -1)
				perror("execl");
		}
	}
	threadStopThreads();
	if (network_link_status != 0)
		pthread_cond_broadcast(&network_up_sema);
	sleep(1);
/* 	if READER_TYPE = rfid */
	if(reader_type & RFID_ENABLE) 
		rfid_stop();
/*	if READER_TYPE = qdac */
	if(reader_type & QDAC_ENABLE)
		qdac_stop();
	
	usleep(20000);
	pthread_kill(thread_rfid_reader, SIGUSR1);
	pthread_kill(thread_rfid_main, SIGUSR1);
	pthread_kill(gpsThread.thread, SIGUSR1);
	pthread_kill(mainThread.thread, SIGUSR1);
	pthread_kill(protocolThread.thread, SIGUSR1);
	pthread_kill(logThread.thread, SIGUSR1);
	pthread_join(thread_rfid_main, NULL);
	pthread_join(thread_rfid_reader, NULL);
	pthread_join(gpsThread.thread, NULL);
	pthread_join(protocolThread.thread, NULL);
	pthread_join(mainThread.thread, NULL);
	pthread_join(logThread.thread, NULL);
	printf("calling terminateNetwork at exit of netwokr manager\n");
	terminateNetwork(timer5, &it5); 
	stop_watchdog();
	return 0;
}
// ----------------------------------------------------------------------------
#define RECV_SIZE 2048
#define UNIT_SIZE 2048
#define REQ_SIZE 1024
#define RESP_SIZE 1024
#define CAT_SIZE 512
#define REQ_START 512
#define ERR_MSG_SIZE 76
void report_mission_status(int mission_status, char *Reason)
{
	Packet_t up_pkt;
	time_t now;
	size_t	len;
	int l1;
	const char *ser_id;
	FILE *fp_update;
	char *build_version_s;
	char host[64] = {};
	char reason[80];
	char status_str[80];
	UInt16	status = STATUS_CLIENT_REBOOT;
	UInt8	seq = 0;

	if (mission_status <= 0x200) {
		memset(reason, '\0', sizeof(reason));
		build_version_s = get_build_version();
		ser_id = propGetString(PROP_STATE_SERIAL, "0");
		//snprintf(reason, sizeof(reason), "%s @%s", ser_id, build_version_s);
		osGetHostname(host, sizeof(host));
		snprintf(reason, sizeof(reason), "%s @%s", ser_id, host);
		len = strlen(reason);
		if (mission_status > 0)	 {
			l1 = snprintf(reason + len, ERR_MSG_SIZE - len, " events restored");
			len += l1;
		}
		if ((fp_update = fopen(update_stat_file, "r")) != NULL) {
			memset(status_str, '\0', 32);
			status = STATUS_CLIENT_UPDATE_FAILED;
			if (fgets(status_str, sizeof(status_str), fp_update) != NULL) {
				if ((strstr(status_str, "SUCCESS")) != NULL) {
					status = STATUS_CLIENT_UPDATE_OK;
					snprintf(reason + len, ERR_MSG_SIZE - len, " updated successfully");
				}
			}
			fclose(fp_update);
			if (status == STATUS_CLIENT_UPDATE_FAILED) {
				snprintf(reason, sizeof(reason), "%s UPDATE Failed: %s", ser_id, status_str);
			}
			usleep(100000);
			unlink(update_stat_file);
		}
	}
	else {
		strncpy(reason, Reason, ERR_MSG_SIZE);
		status = mission_status;
	} 

	len = strnlen(reason, ERR_MSG_SIZE);
	if (len >= ERR_MSG_SIZE)
		--len;
	now = time(&now);
	pktInit(&up_pkt, PKT_CLIENT_DMTSP_FORMAT_3, "%2U%4U%*s%1U", status, (UInt32)now, 
									len, reason, seq); 
	up_pkt.seqPos = 6 + len;
	up_pkt.seqLen = 1;
	evAddEncodedPacket(&up_pkt);
	
	
	if (status == STATUS_CLIENT_UPDATE_OK || status == STATUS_CLIENT_UPDATE_FAILED) {
		unlink(update_file);
		unlink(update_daemon);
	}
}
// ----------------------------------------------------------------------------
struct resource_t {
	char path[256];
	char user_pass[68];
	unsigned char digest[16];
	char port[8];
	char host[164];
};
struct resource_t update_info;
int trigger_update(char *cline, int arg_len)
{
	char hex_str[8];
	char *sdigest, *slash, *colon, *endptr, *atsign;
	int i, l, l1, nport; 
	int ret = -1;

	if (update_pending) 
		return 0;
	if (arg_len < 39) 
		return ret;
	if ((atsign = strchr(cline, '@')) == NULL) 
		return ret;
	if ((colon = strchr(cline, ':')) == NULL) 
		return ret;

	l1 = atsign - cline;
	++atsign;
	if ((slash = strchr(atsign, '/')) == NULL)
		return ret;
	l = arg_len - (slash - cline) - 32;
	if (l < 3 || l1 < 3)
		return ret;
	memset(&update_info, 0, sizeof(struct resource_t));
	strncpy(update_info.user_pass, cline, l1);
	strncpy(update_info.path, slash, l);  
	update_info.path[l] = '\0';
	if ((colon = strchr(atsign, ':')) != NULL) {
		if (slash - colon < 3) {
			printf("Invalid port number\n");
			return ret;
		}
		strncpy(hex_str, colon + 1, 7);
		hex_str[7] = '\0'; 
		errno = 0;
		nport = strtol(hex_str, &endptr, 10);
		if (endptr == hex_str) {
			printf("Invalid port number\n");
			return ret;
		}
		sprintf(update_info.port, "%d", nport);
	}
	else {
		colon = slash;
		snprintf(update_info.port, 8, "443");
	}
	strncpy(update_info.host, atsign, colon - atsign);
	update_info.host[colon - atsign] = '\0';

	hex_str[2] = ' ';
	hex_str[3] = '\0';
	sdigest = cline + arg_len - 32;
	for (i = 0; i < 16; i++) {
		hex_str[0] = sdigest[i * 2];
		hex_str[1] = sdigest[i * 2 + 1];
		l = sscanf(hex_str, "%hhx", &update_info.digest[i]);
		if (l <= 0)
			return ret;
	}
	if (pthread_create(&thread_update, NULL, download_update, &update_info) == 0) {
		update_error = false;
		update_pending = true;
		ret = 0;
	}
	return ret;
}
void * download_update(void *arg)
{
	int rlen, offset, alen, elen, wlen;  
	int res_status, recvfd, err;
	int status = -1;
	char error_str[ERR_MSG_SIZE];
	char header_close[32] = "Connection: close\r\n\r\n";
	char header_auth[32] = "Authorization: Basic ";
	char proto_ver[16] = " HTTP/1.1\r\n";
	unsigned char digest[16];
	struct sigevent sev4;
	struct itimerspec it4;
	struct addrinfo hints, *servinfo;
	timer_t timer4;
	bool is_chunk0 = true; 
	char *rp, *pbody, *pcon, *endptr;
	char *response_buf, *request_buf, *basic_encoded, *cipher_des;
	BIO *sbio = NULL;
	SSL_CTX *ctx; 
	SSL *ssl;
	/*SSL_CIPHER *ccipher; */
	MD5_CTX	md5c;
	struct stat daemon_stat;
	struct resource_t *uri = (struct resource_t *)arg;

	sev4.sigev_notify = SIGEV_SIGNAL;
	sev4.sigev_signo = SIG_TIMEOUT;
	sev4.sigev_value.sival_int = TIMER_UPDATE;
	timer_create(CLOCK_REALTIME, &sev4, &timer4);
	it4.it_interval.tv_sec = 0;
	it4.it_interval.tv_nsec = 0;
	if ((response_buf = malloc(RECV_SIZE)) == NULL) {
		perror("reponse_buf allocation"); 
		snprintf(error_str, ERR_MSG_SIZE, "out of memory");
		goto exit_f;
	}
	request_buf = response_buf + REQ_START;
	basic_encoded = request_buf + REQ_SIZE;
	cipher_des = basic_encoded;
	memset(response_buf, 0, RECV_SIZE);
	if ((recvfd = open(update_file, O_WRONLY | O_CREAT, S_IRUSR | S_IWUSR)) < 0) { 
		perror("Open File");
		snprintf(error_str, ERR_MSG_SIZE, "Can create the file");
		goto exit0;
	}
	sleep(2);
	it4.it_value.tv_sec = NETWORK_PING_TIMEOUT << 1;
	it4.it_value.tv_nsec = 0;
	timer_settime(timer4, 0, &it4, NULL);
	/* Test the URL*/
	memset(&hints, 0, sizeof(hints));
	hints.ai_family = AF_INET;
	hints.ai_socktype = SOCK_STREAM;
	if ((err = getaddrinfo(uri->host, uri->port, &hints, &servinfo)) != 0) {
		snprintf(error_str,  ERR_MSG_SIZE, "%s", gai_strerror(err));
		fprintf(stderr,  "%s\n", gai_strerror(err));
		goto exit1;
	}
	freeaddrinfo(servinfo);
	/* Global system initialization*/
	SSL_library_init();
	SSL_load_error_strings();
	OpenSSL_add_all_algorithms();
	OpenSSL_add_all_digests();

	alen = 0;
	rlen = 0;
	elen = 15;
	error_str[0] = '\0';
	ctx = SSL_CTX_new(SSLv23_client_method());
	if (ctx == NULL) {
		perror("Can\'t allocate SSL context");
		snprintf(error_str, ERR_MSG_SIZE, "Can\'t allocate SSL context");
		goto closing;
	}
	sbio = BIO_new_ssl_connect(ctx);
	BIO_get_ssl(sbio, &ssl);
	if(!ssl) {
		perror("Can\'t locate SSL pointer");
		snprintf(error_str, ERR_MSG_SIZE, "Can\'t locate SSL pointer");
		goto closing;
	}
	SSL_set_mode(ssl, SSL_MODE_AUTO_RETRY);

	snprintf(cipher_des, CAT_SIZE, "%s:%s", uri->host, uri->port);
	BIO_set_conn_hostname(sbio, cipher_des);
	if ((status = BIO_do_connect(sbio)) <= 0) {
		if (errno != 0)	
			strerror_r(errno, error_str, ERR_MSG_SIZE);
		perror("Connect");
		goto closing;
	}
	if ((BIO_do_handshake(sbio)) <= 0) {
		ERR_error_string_n(ERR_get_error(), error_str, ERR_MSG_SIZE);
		printf("SSL Handshake: %s\n", error_str);
		goto closing;
	}
/*	if ((ccipher = (SSL_CIPHER*)SSL_get_current_cipher(ssl)) != NULL) {
		rp = SSL_CIPHER_description(ccipher, cipher_des, CAT_SIZE);
		if (rp != NULL) {
			printf("Cipher:%s\n", cipher_des);
		}
	}
*/
	snprintf(request_buf, REQ_SIZE, "HEAD %s%sHOST: %s\r\n\r\n",
					uri->path, proto_ver, uri->host);
	/*printf("%s", request_buf); */
	status = -1;
	if ((status = BIO_write(sbio, request_buf, strlen(request_buf))) <= 0) {
		perror("Send:");
		snprintf(error_str, ERR_MSG_SIZE, "SSL write error");
		goto closing;
	}	
	if ((status = BIO_read(sbio, response_buf, RECV_SIZE)) == 0) {
		snprintf(error_str, ERR_MSG_SIZE, "Peer closed unexpectedly\n");
		status = 1;
		goto closing;
	} else if (status < 0) {
		perror("Receive");
		snprintf(error_str, ERR_MSG_SIZE, "SSL receiving error");
		goto closing;
	}
	/*printf("%s\n", response_buf); */
	res_status = (int)strtol(response_buf + 9, &endptr, 10);
	if (endptr == response_buf + 8) 
		goto closing;
	if (res_status == 401 || res_status == 200) {
		base64Encode(basic_encoded, CAT_SIZE, (unsigned char *)uri->user_pass, 
												strnlen(uri->user_pass, 68));
		snprintf(request_buf, REQ_SIZE, "GET %s%sHost: %s\r\n%s%s\r\n%s",
					uri->path, proto_ver, uri->host, header_auth, 
						basic_encoded, header_close); 
		printf("%s", request_buf);
	}
	else {
		if ((pbody = strstr(response_buf, "\r\n")) != NULL)
			alen = ((pbody - response_buf - 8) < ERR_MSG_SIZE)? 
				(pbody - response_buf - 8) : ERR_MSG_SIZE;
		else 
			alen = 33;
		strncpy(error_str, response_buf + 9, alen);
		goto closing;
	}

	if ((status = BIO_write(sbio, request_buf, strlen(request_buf))) <= 0) {
		perror("Send:");
		snprintf(error_str, ERR_MSG_SIZE, "SSL write error");
		goto closing;
	}	
	memset(response_buf, 0, RECV_SIZE);
	status = -1;
	while(alen < elen) {
		rp = response_buf;
		if ((rlen = BIO_read(sbio, rp, RECV_SIZE)) == 0) {
			snprintf(error_str, ERR_MSG_SIZE, "Peer closed unexpectedly\n");
			status = 1;
			break;
		} else if (rlen < 0) {
			perror("Receive");
			snprintf(error_str, ERR_MSG_SIZE, "SSL receiving error");
			break;
		}
		if (is_chunk0) {
			res_status = (int)strtol(response_buf + 9, &endptr, 10);
			if (endptr == response_buf + 8) 
				break;
			if (res_status != 200) {
				if ((pbody = strstr(response_buf, "\r\n")) != NULL)
					alen = ((pbody - response_buf - 8) < ERR_MSG_SIZE)? 
						(pbody - response_buf - 8) : ERR_MSG_SIZE;
				else 
					alen = 33;
				strncpy(error_str, response_buf + 9, alen);
				break;
			}
			pcon = strstr(response_buf, "Content-Length:");
			if (pcon == NULL)
				break;
			pbody = strstr(pcon, "\r\n\r\n");
			if (pbody == NULL)
				break;
			pcon += strlen("Content-Length:");
			elen = (int)strtol(pcon, &endptr, 10);
			MD5_Init(&md5c);
			offset = pbody - response_buf + 4;
			alen = rlen - offset;
			MD5_Update(&md5c, response_buf + offset, alen);
			is_chunk0 = false;
			if ((wlen = write(recvfd, response_buf + offset, alen)) < 0) {
				perror("File Write");
				snprintf(error_str, ERR_MSG_SIZE, "file  write error");
				break;
			}
		} else {
			alen += rlen;
			MD5_Update(&md5c, response_buf, rlen);
			if ((wlen = write(recvfd, response_buf, rlen)) < 0) {
				perror("Receiving");
				snprintf(error_str, ERR_MSG_SIZE, "SSL receiving error");
				break;
			}
		}
	}	
	if (alen == elen) {
		MD5_Final(digest, &md5c);
		if (memcmp(&uri->digest, &digest, 16) == 0) {
			printf("Update Download Success\n");
			status = elen;
		}
		else {
			printf("File Digest wrong\n");
			snprintf(error_str, ERR_MSG_SIZE, "MD5 Digest verification error");
			status = -1;
		}
	}
	else if (rlen > 0)
		printf("%s\n", response_buf);
closing:	
	BIO_free_all(sbio);
	SSL_CTX_free(ctx);
	ERR_free_strings();
exit1:
	it4.it_value.tv_sec = 0;
	it4.it_value.tv_nsec = 0;
	timer_settime(timer4, 0, &it4, NULL);
	close(recvfd);
	if (status > 0) {
		snprintf(request_buf, REQ_SIZE, "tar xz -f %s -C /mnt/flash/titan-data/", update_file);
		if ((res_status = system(request_buf)) != 0) {
			status = -1;
			snprintf(error_str, ERR_MSG_SIZE, "Cannot decompress update pacakge");
			goto exit0;
		}
		if ((res_status = stat(update_daemon, &daemon_stat)) < 0) {
			status = -1;
			snprintf(error_str, ERR_MSG_SIZE, "Cannot find update daemon");
			goto exit0;
		}
		pthread_kill(thread0, SIGUSR1);
	}
exit0:
	free(response_buf);
exit_f:
	if (status <= 0) {
		printf("Update Download Error\n"); 
		if (strnlen(error_str, ERR_MSG_SIZE) == 0)
			snprintf(error_str, ERR_MSG_SIZE, "Update Download Error");
		report_mission_status(STATUS_CLIENT_UPDATE_FAILED, error_str);
		update_error = true;
		pthread_cond_signal(&network_down_sema); 
	}
	return NULL;
}

static struct upload_request_t log_req = {.request_size = 0, .status = 0};
int http_uploader(void *arg)
{
	struct addrinfo hints, sainfo, *servinfo, *painfo;
	struct upload_request_t *ur = (struct upload_request_t *)arg;
	char header_auth[64] = "Authorization: Basic ";
	char content_type[64] = "Content-Type: application/x-gzip\r\n";
	char proto_ver[32] = " HTTP/1.1\r\n";
	char content_length[32] = "Content-Length: ";
	char *request_buf, *response_buf, *basic_encoded, *description;
	int rlen, offset, clen, slen, len, alen = 0;  
	int err, status = 0, result = -501;
	int sendfd;
	/*char s_ip_addr[INET6_ADDRSTRLEN]; */
	char *endptr, *rp, *eor;
	struct sockaddr_in saddr_in;
	BIO *sbio;
	SSL_CTX *ctx; 
	SSL *ssl;
	/*SSL_CIPHER *ccipher; */

	memset(&hints, 0, sizeof hints);
	hints.ai_family = AF_INET;
	hints.ai_socktype = SOCK_STREAM;
	if ((err = getaddrinfo(ur->host, ur->port, &hints, &servinfo)) != 0) {
		snprintf(ur->err_msg,  ERR_MSG_SIZE, "%s", gai_strerror(err));
		fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(err));
		return result;
	}
	sainfo = *servinfo;
	painfo = &sainfo;
	memcpy(&saddr_in, painfo->ai_addr, painfo->ai_addrlen);
	freeaddrinfo(servinfo);
	/*inet_ntop(painfo->ai_family, &saddr_in.sin_addr, s_ip_addr, sizeof(s_ip_addr)); */
	/*open send file*/
	if ((sendfd = open(ur->upfile, O_RDONLY)) < 0) { 
		perror("Open File");
		snprintf(ur->err_msg, ERR_MSG_SIZE, "Can create the file");
		return result;
	}
	if ((clen = lseek(sendfd, 0, SEEK_END)) < 0) { 
		perror("lseek"); 
		goto exit0;
	} 
	/*allocate memory*/
	if ((request_buf = malloc(UNIT_SIZE)) == NULL) {
		perror("reponse_buf allocation"); 
		goto exit0;
	}
	response_buf = request_buf + REQ_SIZE;
	basic_encoded = response_buf + (RESP_SIZE - CAT_SIZE);
	description = basic_encoded;
	memset(request_buf, 0, UNIT_SIZE);
	/* Global system initialization*/
	SSL_library_init();
	SSL_load_error_strings();
	OpenSSL_add_all_algorithms();
	ctx = SSL_CTX_new(SSLv23_client_method());
	if (ctx == NULL) {
		perror("Can't allocate SSL context");
		goto exit1;
	}
	sbio = BIO_new_ssl_connect(ctx);
	BIO_get_ssl(sbio, &ssl);
	if(!ssl) {
		perror("Can't locate SSL pointer");
		goto closing;
	}
	SSL_set_mode(ssl, SSL_MODE_AUTO_RETRY);
	snprintf(description, CAT_SIZE, "%s:%s", ur->host, ur->port);
	BIO_set_conn_hostname(sbio, description);
	/*SSL socket operation*/
	if(BIO_do_connect(sbio) <= 0) {
		if (errno != 0)	
			strerror_r(errno, ur->err_msg, ERR_MSG_SIZE);
		perror("Connect");
		goto closing;
	}
	if(BIO_do_handshake(sbio) <= 0) {
		ERR_error_string_n(ERR_get_error(), ur->err_msg, ERR_MSG_SIZE);
		printf("SSL Handshake: %s\n", ur->err_msg);
		goto closing;
	}
/*	if ((ccipher = (SSL_CIPHER*)SSL_get_current_cipher(ssl)) != NULL) {
		rp = SSL_CIPHER_description(ccipher, description, CAT_SIZE);
		if (rp != NULL) {
			printf("Cipher Suite:\n");
			printf("%s", description);
		}
	}
*/
	/*first request*/
	snprintf(request_buf, REQ_SIZE, "PUT %s?%s%sHOST: %s\r\n%s\r\n",
					ur->path, ur->query, proto_ver, ur->host,
					content_type);
	/*printf("%s", request_buf);  */
	rlen = strnlen(request_buf, REQ_SIZE);
	if ((err = BIO_write(sbio, request_buf, rlen)) <= 0) {
		perror("Send");
		snprintf(ur->err_msg, ERR_MSG_SIZE, "SSL write error");
		goto closing;
	}	
	if ((rlen = BIO_read(sbio, response_buf, RESP_SIZE)) == 0) {
		snprintf(ur->err_msg, ERR_MSG_SIZE, "Peer closed unexpectedly\n");
		goto closing;
	} else if (rlen < 0) {
		strerror_r(errno, ur->err_msg, ERR_MSG_SIZE);
		perror("Receive");
		goto closing;
	}
	printf("%s", response_buf);
	status = (int)strtol(response_buf + 9, &endptr, 10); 
	if ((rp = strstr(response_buf, "Content-Length")) != NULL) {
		len = (int)strtol(rp + 15, &endptr, 10); 
		if ((eor = strstr(response_buf, "\r\n\r\n")) != NULL) {
			if ((rlen - (eor - response_buf) - 4) >= len) 
				goto second_request;
		}
	}
	if ((rlen = BIO_read(sbio, response_buf + rlen, RESP_SIZE - rlen)) == 0) {
		snprintf(ur->err_msg, ERR_MSG_SIZE, "Peer closed unexpectedly\n");
		goto closing;
	} else if (rlen < 0) {
		perror("Receive");
		snprintf(ur->err_msg, ERR_MSG_SIZE, "SSL receiving error");
		goto closing;
	}
	/*printf("%s", response_buf); */
second_request:
	usleep(100000);
	if (status == 401) {
		memset(basic_encoded, 0, CAT_SIZE);
		base64Encode(basic_encoded, CAT_SIZE, (unsigned char *)ur->user_pass, 
							strnlen(ur->user_pass, sizeof(ur->user_pass)));
		snprintf(request_buf, UNIT_SIZE, "PUT %s?%s%sHost: %s\r\n%s%s\r\n%s%s%d\r\n\r\n",
				ur->path, ur->query, proto_ver, ur->host, header_auth, 
				basic_encoded, content_type, content_length, clen); 
		printf("%s", request_buf);
		if ((err = lseek(sendfd, 0, SEEK_SET)) < 0) { 
			perror("lseek"); 
			goto closing;
		}
		offset = strnlen(request_buf, UNIT_SIZE);
		if ((slen = read(sendfd, request_buf + offset, UNIT_SIZE - offset)) < 0) {
			perror("Read");
			goto closing;
		}
		rlen = slen + offset;
		clen += offset;
	}
	else {
		if ((eor = strstr(response_buf, "\r\n")) != NULL)
			alen = ((eor - response_buf - 8) < ERR_MSG_SIZE)? 
				(eor - response_buf - 8) : ERR_MSG_SIZE;
		else 
			alen = 33;
		strncpy(ur->err_msg, response_buf + 9, alen);
		goto closing;
	}

	alen = 0;
	do {
		slen = 0;
		while (slen < rlen) {
			if ((len = BIO_write(sbio, request_buf + slen, rlen)) < 0) {
				perror("Send");
				snprintf(ur->err_msg, ERR_MSG_SIZE, "SSL write error");
				goto end_sending;
			} 
			slen += len;
		}
		alen += slen;
		if ((rlen = read(sendfd, request_buf, UNIT_SIZE)) < 0) {
			perror("Read");
			break;
		}
	} while (alen < clen);
end_sending:
	if ((rlen = BIO_read(sbio, response_buf, RESP_SIZE)) == 0) {
		snprintf(ur->err_msg, ERR_MSG_SIZE, "Peer closed unexpectedly\n");
		goto closing;
	} else if (rlen < 0) {
		perror("Receive");
		snprintf(ur->err_msg, ERR_MSG_SIZE, "SSL receiving error");
		goto closing;
	}
	alen = rlen;
	status = (int)strtol(response_buf + 9, &endptr, 10); 
	if (status == 201) {
		printf("Succeeded uploading %s\n", ur->query);
		result = 0;
	}
	else {
		if ((eor = strstr(response_buf, "\r\n")) != NULL)
			alen = ((eor - response_buf - 8) < ERR_MSG_SIZE)? 
				(eor - response_buf - 8) : ERR_MSG_SIZE;
		else 
			alen = 33;
		strncpy(ur->err_msg, response_buf + 9, alen);
	}
	if ((rp = strstr(response_buf, "Content-Length")) != NULL) {
		len = (int)strtol(rp + 15, &endptr, 10); 
		if ((eor = strstr(response_buf, "\r\n\r\n")) != NULL) {
			if ((rlen - (eor - response_buf) - 4) >= len) 
				goto closing;
		}
	}
	if ((rlen = BIO_read(sbio, response_buf + rlen, RESP_SIZE - rlen)) == 0) {
		snprintf(ur->err_msg, ERR_MSG_SIZE, "Peer closed unexpectedly\n");
		goto closing;
	} else if (rlen < 0) {
		strerror_r(errno, ur->err_msg, ERR_MSG_SIZE);
		perror("Receive");
		goto closing;
	}
	alen += rlen;
closing:	
	if (result != 0) {
	 	if (ur->err_msg[0] == '\0')
			sprintf(ur->err_msg, "Error in HTTP PUT operation");
		else if (alen > 0 && alen < RESP_SIZE) {
			response_buf[alen] = '\0';
			printf("%s", response_buf);
		}
	}
	BIO_free_all(sbio);
	SSL_CTX_free(ctx);
	ERR_free_strings();
exit1:
	free(request_buf);
exit0:
	close(sendfd);
	return result;
}
int upload_log(uint32_t file_size, char * upload_file)
{
	char working_str[MAX_PAYLOAD_SIZE];
	const char *s1, *s2;
	char *pt;
	time_t now;
	uint32_t l1, l2;
	int status = -1;

	if (network_link_status != 0)
		return -1;
	if (log_req.status != 0) {
		if ((strncmp(log_req.upfile, upload_file, strlen(upload_file)) == 0)
									&& (log_req.request_size == file_size))
			goto coming_back;
	}
	strncpy(working_str, logging_server_url, MAX_PAYLOAD_SIZE);  
	if ((s1 = strchr(working_str, '/')) == NULL)
		goto failure;
	if ((s2 = strchr(working_str, ':')) != NULL) {
		if ((s1 - s2) < 3)
			goto failure;
		strncpy(log_req.port, s2 + 1, (s1 - s2 - 1));
	}
	else {
		snprintf(log_req.port, 8, "443");
		s2 = s1;
	}
	strncpy(log_req.host, working_str, (s2 - working_str));
	strncpy(log_req.path, s1, sizeof(log_req.path));
	s1 = propGetString(PROP_LOGGING_USER, "");
	strncpy(log_req.user_pass, s1, MAX_ID_SIZE);
	s2 = propGetString(PROP_LOGGING_PASS, "");
	snprintf(log_req.user_pass + strnlen(s1, MAX_ID_SIZE), MAX_ID_SIZE + 2, ":%s", s2);
	sprintf(working_str, "gzip -c %s > %s.gz", upload_file, upload_file);
	if ((status = system(working_str)) != 0)
		goto failure;
	snprintf(log_req.upfile, sizeof(log_req.upfile), "%s.gz", upload_file);
coming_back:
	now = time(&now);
	s1 = propGetAccountID();
	l1 = strnlen(s1, MAX_ID_SIZE);
	strncpy(log_req.query, s1, MAX_ID_SIZE);
	s2 = propGetDeviceID(0);
	snprintf(log_req.query + l1, MAX_ID_SIZE + 1, "_%s", s2);
	l2 = strnlen(log_req.query, sizeof(log_req.query));
	sprintf(log_req.query + l2, "_%08x.%1d.log.gz", (unsigned)now, get_log_type());
	status = http_uploader(&log_req);
failure:
	if (status == 0) {
		strncpy(working_str, log_req.query, MAX_PAYLOAD_SIZE);
		pt = strstr(working_str, ".gz");
		sprintf(pt, " size:%d", file_size);
		report_mission_status(STATUS_CLIENT_LOGGING_OK, working_str);
		usleep(100000);
		unlink(log_req.upfile);
		unlink(upload_file);
	}
	else if (status != -1) {
		report_mission_status(STATUS_CLIENT_LOGGING_FAILED, log_req.err_msg);
	}
	log_req.request_size = file_size;
	log_req.status = status;
	return status;
}
bool watchdog_network_monitor(void)
{
	static int stuck_times = 0;
	if (wireless_stuck > 0) {
		stuck_times += 1;
		print_critical("Stuck in communication library %d times\n", stuck_times);
		debuglog(LOG_INFO,"Stuck in communication library %d times", stuck_times);
		diagnostic_report(DIAGNOSTIC_LIB_STUCK, stuck_times, NULL);
		if(stuck_times >= 3)
			diagnostic_report(DIAGNOSTIC_CLIENT_REBOOT, REBOOT_LIBRARY_STUCK, NULL);
		return true;
	}
	else {
		stuck_times = 0;
		return false;
	}
}
void watchdog_alert(void)
{
	pquePreserveQueue(evGetEventQueue());
}
void watchdog_reboot(void)
{
	pthread_kill(thread0, SIGUSR1);
}

/* false = exist
 * true  = down
 */
static bool network_status(void) {
        FILE *fp;
        char buffer[256];
        char ip_addr[15];
//        if (backhaul_test == 0) {
//                return 0;
//        }
        fp = fopen("/etc/resolv.conf", "r");
        if ( fp == NULL)
                return -1;
        while (fgets(buffer, sizeof(buffer), fp) != NULL) {
                int i = 0 , j = 0;

             //  if (wifi_live() < 0)
             //           break;

                if (strstr(buffer, "nameserver") == NULL)
                        continue;

                for ( i = 0 ; i < strlen(buffer); i++) {
                        if ( ((buffer[i] >= 0x30) && (buffer[i] <= 0x39)) || buffer[i] == 0x2E) {
                                ip_addr[j] = buffer[i];
                                j++;
                        }
                }
                ip_addr[j] = '\0';
                if (check_ip_connection1(ip_addr) >=0) {
                        fclose(fp);
                        return false;
                }
        }
        fclose(fp);
        return true;
}


#define DMSG_FILE "/tmp/dmesg.txt"
#define STRING_SIZE 1024
#define SRCH_STRING "AT91: Starting after "

static void report_bootup_status() {
	int dmsg_fd;
	int str_pos = 0;
	char read_string[STRING_SIZE];
	char read_char = 0;
	char *bootup_status;

	system("dmesg > /tmp/dmesg.txt");
	if (access(DMSG_FILE, F_OK) == -1) 
		return -1;
	
	dmsg_fd = open(DMSG_FILE, O_RDONLY);
	if (dmsg_fd < 0) {
//		perror("Open /var/log/messages fail");	
		printf("%s\n", strerror(errno));
		debuglog(LOG_INFO, "[ositechdmtp] Open /tmp/dmesg.txt fail: %s\n", strerror(errno));
		exit(-1);
	}
	
	while (1) {
		if(read(dmsg_fd, &read_char, sizeof(char)))
		{
			if (read_char != '\n') {	
				read_string[str_pos] = read_char;
				str_pos++;
				continue;
			}
		}
		else
			break;

		read_string[str_pos] = '\0';
		bootup_status = strstr(read_string, SRCH_STRING);
		if (bootup_status) {
			bootup_status += (sizeof(SRCH_STRING)-1);
//			printf("%s\n", bootup_status);
			break;
		}
		memset(read_string, 0, STRING_SIZE*sizeof(char));
		str_pos = 0;
	}

	close(dmsg_fd);
	unlink(DMSG_FILE);
	diagnostic_report(DIAGNOSTIC_MESSAGE, 0, bootup_status);
}

static void setLocalTime(void) {
		int n = 0;
		time_t t1, t0 = 0;
		UInt32 seq;
		n = pqueRestoreQueue(evGetEventQueue());
		if (n > 0) {
			t1 = time(NULL);
			seq = pqueGetLastSequence(evGetEventQueue(), &t0);
			evSetSequence(seq + 1);
			if (t0 != 0 && t1 < t0)
				synchronize_system_clock(t0 + 5);
		}
		if (n > 0x200)
			n = 0x200;
		report_mission_status(n, NULL);
	}
