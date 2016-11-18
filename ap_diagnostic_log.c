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
//  Debug/Info logging.
//  - Since each platform may provide its own type of logging facilities, this
//  module has been placed in the 'custom/' folder.
//  - The logging facility provided here now includes support for Linux 'syslog'
//  output (which may be overkill for some embedded systems).
// ---
// Change History:
//  2006/01/04  Martin D. Flynn
//     -Initial release
//  2006/01/12  Martin D. Flynn
//     -Upgraded with syslog support
//  2007/01/28  Martin D. Flynn
//     -WindowsCE port
//     -Added Aux storage logging
//     -Added threaded logging (to prevent logging from blocking other threads)
// ----------------------------------------------------------------------------

#include "defaults.h"

// For auxiliary message logging, the following must be defined:
//   "LOGGING_MESSAGE_FILE"
//   "LOGGING_AUX_DIRECTORY"

// ----------------------------------------------------------------------------

// uncomment to include syslog support
#if defined(TARGET_LINUX) || defined(TARGET_GUMSTIX)
//#  define INCLUDE_SYSLOG_SUPPORT
#endif

// uncomment to display log messages in a separate thread
//#define SYSLOG_THREAD

// ----------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <ctype.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

#if defined(INCLUDE_SYSLOG_SUPPORT)
#  include <syslog.h>
#endif
#include "log.h"
#include "ap_diagnostic_log.h"
#include "stdtypes.h"
#include "strtools.h"
#include "statcode.h"
#include "utctools.h"
#include "threads.h"
#include "buffer.h"
#include "propman.h"
#include "startup.h"
#include "io.h"
#include "libwlcomm.h"
#include "protocol.h"
#include <sys/socket.h>		/* For AF_INET & struct sockaddr */
#include <netinet/in.h>
#include <netinet/ip_icmp.h>
#include <resolv.h>
#include <netdb.h>


#define PACKETSIZE	64
// This value may be overridden in "defaults.h" by defining "LOGGING_DEFAULT_LEVEL"

#define MAX_MESSAGE_LENGTH	256
#define LOG_COLLECT_SIZE	1024
#define CONSOLE_OUTPUT		stderr
#define FLUSH_MODULO		5L
#define LOG_BUF_ALERT	LOG_BUFFER_SIZE - MAX_MESSAGE_LENGTH
#define LOG_BUF_FULL	((LOG_BUFFER_SIZE >> 1) + (LOG_BUFFER_SIZE >> 2))
#define LOG_MAIN_PERIOD	30
#define SYSLOG_NORMAL	(SYSLOG_GENERIC | SYSLOG_WIFI)
#define SYSLOG_TYPE_MASK	7
// ----------------------------------------------------------------------------

//static int			syslogLevel = LOGGING_DEFAULT_LEVEL;

//static utBool		syslogRunThread = utFalse;
//static UInt32		syslogSwitch = 0;
static UInt32		current_log_index = 0;
//static UInt32		log_control_props[3] = {0, (SYSLOG_DEFAULT_SIZE << 12), 0};
//static int			log_maximum_size = (SYSLOG_DEFAULT_SIZE << 12);
//static char			sys_log_name[2][64] = {SYSLOG_NAME0, SYSLOG_NAME1};
//static char			cell_log_name[64];

static threadMutex_t        syslogMutex;
static threadCond_t         syslogCond;
#define LOG_LOCK            MUTEX_LOCK(&syslogMutex);
#define LOG_UNLOCK          MUTEX_UNLOCK(&syslogMutex);
#define LOG_WAIT(T)         CONDITION_TIMED_WAIT(&syslogCond, &syslogMutex, T);
#define LOG_NOTIFY          CONDITION_NOTIFY(&syslogCond);

#define AP_DIAG_LOG_FILE 	"/tmp/ap_diagnostic.log"
#define BR_MODE_FILE 		"/etc/titanlib/mode.conf"
#define DHCP_LEASE_FILE		"/var/run/dhcp.leases"
#define CLIENT_IP_FILE 		"/tmp/client_ip"
#define AP_EXIST_FILE		"/tmp/ap_exist"
#define RADIO_SWITCH_FILE	"/tmp/ap_radio"
#define DNSMASQ_RUNNING_FILE		"/tmp/ap_dnsmasq_running"
#define CNET_CLIENT_FILE 		"/tmp/ap_connected_client"
#define PING_RES_FILE		"/tmp/ping_client_res"
#define NAT_CHECK_FILE		"/tmp/nat_check_res"
#define DNS_CHECK_FILE		"/tmp/dns_check_res"

//#define LOG_BUFFER_SIZE				4096/2
#define AP_DIAG_STRING_SIZE 			1024
#define AP_DIAG_TIME_STRING_SIZE 		32
#define AP_DIAG_RESULT_STRING_SIZE 	64
#define NUM_OF_END_CHAR					20  /* combined with 18 '+'s, 1 '\n' & 1 '\0'*/ 

threadThread_t       ap_diagnostic_logThread;
extern UInt32 network_link_status;
char ap_diag_log_server_url[MAX_PAYLOAD_SIZE] = "216.171.106.21/dmtplog/log_receiver.cgi";
static struct upload_request_t ap_diag_log_req = {.request_size = 0, .status = 0};
// ----------------------------------------------------------------------------
//extern int upload_log(UInt32 l, char *s);
extern utBool startupSaveProperties(UInt32 save_type);
// ----------------------------------------------------------------------------
static void * ap_diagnostic_log_thread_main(void * arg);
char *ap_diag_time_update(void);
char *ap_diag_check_network(int *pass);
char *ap_diag_end_test(void);
char *ap_diag_check_br_mode(int *pass);
char *ap_diag_check_AP_exist(int *pass) ;
char *ap_diag_check_AP_radio(int *pass) ;
char *ap_diag_check_dnsmasq(int *pass);
char *ap_diag_check_client(int *pass);
char *ap_diag_ping_client(int *pass);
char *ap_diag_check_NAT(int *pass);
char *ap_diag_check_dns(int *pass);
char *ap_print_cur_dns(void);
void fill_the_string(char *diag_string, char *res_string) ;
char *fetch_client_ip(char *client_mac);
char *fetch_client_mac(char *read_string);
int ping_client(char * client_ip);
int upload_ap_diag_log(uint32_t file_size, char * upload_file);
// ----------------------------------------------------------------------------

/* maintain debug mode */
/* set debug mode */

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------


/* call-back to stop thread */
static void ap_diagnostic_logStopThread(void *arg)
{
	LOG_LOCK
	LOG_NOTIFY 
	LOG_UNLOCK
}

void * ap_diagnostic_log_thread_main(void * arg)  
{
/* used variable */	

	FILE *log_file;
	static bool file_sent = true;
	int pass = 0;
	uint32_t ap_log_file_size = 0;
	static int current_pos = 0;
	static int pos_init = 0;
	ssize_t wlen;
	
	char *ap_diag_result_string = NULL;
	char *ap_diag_string = NULL;
	char *ap_diag_end_test_string = NULL;

/* Joe's code starts from herer */
	UInt32 report_cycle, ap_diagnostic = 0;
	report_cycle = propGetUInt32(PROP_COMM_MIN_XMIT_RATE, 1);
/* 	check out the "sta.ap.diagnostic" value each "com.min.rate" intervals. Since "com.min.rate" is less than or equale to the time
	that client connects to the server, it indicates the intervals that the "sta.ap.diagnostic" value being changed.   */
//	ap_diagnostic_logInitialize();

	while( 1 ) {
		ap_diagnostic = propGetUInt32AtIndex(PROP_STATE_AP_DIAGNOSTIC, 0, 0);

/*		if "sta.ap.diagnostic" is 1, then check the status informations regarding to the AP then sleep,
		otherwise sleep directly */
		if (ap_diagnostic) { 
		/* wait until the network is established */	
			if (network_link_status == NETWORK_STATUS_INIT)  {
				sleep(LOG_MAIN_PERIOD);
				continue;
			} else {	
				file_sent = false;
				ap_diag_string = (char *)malloc(AP_DIAG_STRING_SIZE*sizeof(char));
				if (ap_diag_string == NULL) {
					printf("malloc ap_diag_string fail\n");
					exit(-1);
				}
				memset(ap_diag_string, 0, AP_DIAG_STRING_SIZE*sizeof(char));

			/* 	the contents of the test result		*/
			/*	date and time						*/
				ap_diag_result_string = ap_diag_time_update();	
				fill_the_string(ap_diag_string, ap_diag_result_string);
			/* 	1. check if Wifi in AP mode 			*/
				printf("1. check if Wifi in AP mode\n");
				ap_diag_result_string = ap_diag_check_br_mode(&pass);
//			strcat(ap_diag_string, ap_diag_result_string);
//			free(ap_diag_result_string);
				fill_the_string(ap_diag_string, ap_diag_result_string);
				if (!pass) 	
					goto res;
			
			/*	if yes, 2. check if "uap0" existed 		*/
				printf("2. check if \"uap0\" existed\n");
				ap_diag_result_string = ap_diag_check_AP_exist(&pass); 
//			strcat(ap_diag_string, ap_diag_result_string);
//			free(ap_diag_result_string);
				fill_the_string(ap_diag_string, ap_diag_result_string);
				if (!pass) 	
					goto res;
				
			/*	if yes, 3. check if AP radio is on 		*/
				printf("3. check if AP radio is on \n");
				ap_diag_result_string = ap_diag_check_AP_radio(&pass);
//				strcat(ap_diag_string, ap_diag_result_string);
//			free(ap_diag_result_string);
				fill_the_string(ap_diag_string, ap_diag_result_string);
				if (!pass)
					goto res;
			
			/*		   4. check if dnamasq is running 	*/
				printf("4. check if dnamasq is running \n");
				ap_diag_result_string = ap_diag_check_dnsmasq(&pass);
				fill_the_string(ap_diag_string, ap_diag_result_string);
				if (!pass)
					goto res;
			
			/*	          5. check if there is client on AP	*/
				printf("5. check if there is client on AP\n");
				ap_diag_result_string = ap_diag_check_client(&pass);
				fill_the_string(ap_diag_string, ap_diag_result_string);
				if (!pass)
					goto res;
			
			/*		   6. check if client is pinged-able 	*/
				printf("6. check if client is ping-able\n");
				ap_diag_result_string = ap_diag_ping_client(&pass);
				fill_the_string(ap_diag_string, ap_diag_result_string);
				if (!pass)
					goto res;
			
			/*		   7. check if network can go out	*/
				printf("7. check if network can go out\n");
				ap_diag_result_string = ap_diag_check_network(&pass);
				fill_the_string(ap_diag_string, ap_diag_result_string);
				if (!pass) {				
					goto res;
				}
			/*		   8. check if DNS working fine	*/			
				printf("8. check if DNS working fine\n");
				ap_diag_result_string = ap_print_cur_dns();
				fill_the_string(ap_diag_string, ap_diag_result_string);
				ap_diag_result_string = ap_diag_check_dns(&pass);
				fill_the_string(ap_diag_string, ap_diag_result_string);
				printf("%s\n", ap_diag_string);
				if (!pass) {				
					goto res;
				}
			/*		   9. check if NAT is enable		*/
				printf("9. check if NAT is enable\n");
				ap_diag_result_string = ap_diag_check_NAT(&pass);
				fill_the_string(ap_diag_string, ap_diag_result_string);
				if (!pass)
					goto res;
			/*		10. print the cur ip routing table 	*/
/*				ap_diag_result_string = ap_print_cur_routing();
				fill_the_string(ap_diag_string, ap_diag_result_string);
				if (!pass)
					goto res;
*/
				strcat(ap_diag_string, "** All AP regarding test passed **\n");
res:
				/*   end of the test					*/
				ap_diag_end_test_string = ap_diag_end_test();
				strcat(ap_diag_string, ap_diag_end_test_string);
				free(ap_diag_end_test_string);

//				log_fd = open(AP_DIAG_LOG_FILE, O_CREAT | O_APPEND | O_RDWR);
				/* 	log file open to write	 */
				log_file = fopen(AP_DIAG_LOG_FILE, "w");
				if (log_file == NULL) {
					printf("Open Log failed");
					sleep(LOG_MAIN_PERIOD);
					continue;
				} else
					printf("open %s success\n", AP_DIAG_LOG_FILE);
			
				if (fwrite(ap_diag_string, sizeof(char), strlen(ap_diag_string), log_file) != strlen(ap_diag_string)) 
					perror("Writing log");

				ap_log_file_size = ftell(log_file);
//				if(upload_ap_diag_log(ap_log_file_size, AP_DIAG_LOG_FILE) > 0) 
//					file_sent = true;
				fclose(log_file);
				free(ap_diag_string);
				ap_diag_string = NULL;
			}
//			printf("set ap diagnostic to 0\n");			
			propSetUInt32(PROP_STATE_AP_DIAGNOSTIC, 0);
		}
/*	check network status & log file status 						*/
/* 	if network is live and the log file is never been send before 	*/
/* 	send it, otherwise do nothing								*/
		if(!file_sent) {		
			if (ap_log_file_size > 0) {
				if(upload_ap_diag_log(ap_log_file_size, AP_DIAG_LOG_FILE) > 0) 
				file_sent = true;
				ap_log_file_size = 0;
			}
		}
		sleep(report_cycle+2);
	}

	return NULL;
}

/* start serial communication thread */
utBool ap_diagnostic_logStartThread()
{
	/* init log mutex */
	threadMutexInit(&syslogMutex);
	threadConditionInit(&syslogCond);
	if (threadCreate(&ap_diagnostic_logThread, &ap_diagnostic_log_thread_main, &current_log_index, "ap_diagnostic_log") == 0) {
		// thread started successfully
		threadAddThreadStopFtn(&ap_diagnostic_logStopThread, 0);
		return utTrue;
	} else {
		return utFalse;
	}
}

char *ap_diag_time_update(void) {
	time_t now;
	struct tm tm1, *ptm;
	char *time_string = NULL;
	
	now = time(NULL);
	ptm = localtime_r(&now, &tm1);
	time_string = (char *)malloc(AP_DIAG_TIME_STRING_SIZE*sizeof(char));
	if (time_string== NULL)
		exit(-1);
	sprintf(time_string, "%02d/%02d/%4d %02d:%02d:%02d: \n",
				ptm->tm_mon+1, ptm->tm_mday, ptm->tm_year+1900, 
				ptm->tm_hour, ptm->tm_min, ptm->tm_sec); 
	
	return time_string;
}
char *ap_diag_check_network(int *pass) {
	int request;
	char *res_string = (char *)malloc(AP_DIAG_RESULT_STRING_SIZE*sizeof(char));
	if (res_string == NULL)
		exit(-1);
	request = WIFI_EXISTING;
	if (libwlcomm_main(request, NULL) == WIFI_RET_EXISTING) {
		sprintf(res_string, "7. Network is Existed!\n");
		*pass = 1;
	} else { 
		sprintf(res_string, "** Error: Network is Down!**\n");
		*pass = 0;
	}
	
	return res_string;
}

char *ap_diag_end_test(void) {
	int i = 0;
	char *end_string =  (char *)malloc(NUM_OF_END_CHAR*sizeof(char));
	
	if (end_string == NULL)
		exit(-1);
	while(i < NUM_OF_END_CHAR-2) *(end_string+i++) = '+';
	*(end_string+i) = '\n';
	*(end_string+i+1) = '\0';

	return end_string;
}		

char *ap_diag_check_br_mode(int *pass) {
	int tmp_fd; 
	char *res_string = (char *)malloc(AP_DIAG_RESULT_STRING_SIZE*sizeof(char));
	char *read_string = (char *)malloc(16*sizeof(char));
	memset(read_string, 0, 16*sizeof(char));
	char *br_mode = NULL;
	if (res_string == NULL)
		exit(-1);

	tmp_fd = open(BR_MODE_FILE, O_RDONLY);
	if (tmp_fd == -1) {
		perror("open /etc/titanlib/mode.conf failed");
		exit(-1);
	}
	read(tmp_fd, read_string, 16);

	br_mode = strstr(read_string, "br_mode=NO");
	if(br_mode) {
		*pass = 1;
		sprintf(res_string, "1. Wifi work in AP mode.\n");
	} else {
		*pass = 0;
		sprintf(res_string, "1. ** Error: Wifi is working in BR mode. **\n");
	}
	free(read_string);
	return res_string;
}

char *ap_diag_check_AP_exist(int *pass) {
	int tmp_fd; 
	char *res_string = (char *)malloc(AP_DIAG_RESULT_STRING_SIZE*sizeof(char));
	char *read_string = (char *)malloc(8*sizeof(char));
	memset(read_string, 0, 8*sizeof(char));
	int ap_exist;
	if (res_string == NULL)
		exit(-1);
	system("ap_dev_exist");
	sleep(1);
	
	tmp_fd = open(AP_EXIST_FILE, O_RDONLY);
	if (tmp_fd == -1) {
		perror("open /tmp/ap_exist failed");
		exit(-1);
	}
	read(tmp_fd, read_string, 8);
	ap_exist = strncmp(read_string, "1", sizeof(char));
	if(!ap_exist) {
		*pass = 1;
		sprintf(res_string, "2. AP interface \"uap0\" is Enabled.\n");
	} else {
		*pass = 0;
		sprintf(res_string, "2. ** Error: AP interface \"uap0\" is not present. **\n");
	}
	free(read_string);
	return res_string;
}

char *ap_diag_check_AP_radio(int *pass) {
	int tmp_fd; 
	char *res_string = (char *)malloc(AP_DIAG_RESULT_STRING_SIZE*sizeof(char));
	char *read_string = (char *)malloc(8*sizeof(char));
	memset(read_string, 0, 8*sizeof(char));
	int ap_radio;
	if (res_string == NULL)
		exit(-1);
	system("ap_radio");
	sleep(1);
	
	tmp_fd = open(RADIO_SWITCH_FILE, O_RDONLY);
	if (tmp_fd == -1) {
		perror("open /tmp/ap_radio failed");
		exit(-1);
	}
	read(tmp_fd, read_string, 8);
	ap_radio = strncmp(read_string, "on", strlen("on"));
	if(!ap_radio) {
		*pass = 1;
		sprintf(res_string, "3. AP's radio is on.\n");
	} else {
		*pass = 0;
		sprintf(res_string, "3. ** Error: AP's radio is off. **\n");
	}
	free(read_string);
	return res_string;
}
void fill_the_string(char *diag_string, char *res_string) {
	char *ap_diag_time_string = NULL;
	/* get the real time to write */
//	printf("update time\n");
	strcat(diag_string, res_string);
	free(res_string);
	res_string = NULL;
//	printf("strlen of diag_string %d\n", strlen(diag_string));
}

char *ap_diag_check_dnsmasq(int *pass) {
	int tmp_fd; 
	char *res_string = (char *)malloc(AP_DIAG_RESULT_STRING_SIZE*sizeof(char));
	char *read_string = (char *)malloc(8*sizeof(char));
	memset(read_string, 0, 8*sizeof(char));
	int ap_radio;
	if (res_string == NULL)
		exit(-1);
	system("ap_check_dnsmasq");
	sleep(1);
	tmp_fd = open(DNSMASQ_RUNNING_FILE, O_RDONLY);
	if (tmp_fd == -1) {
		perror("open /tmp/ap_dnsmasq_running");
		exit(-1);
	}
	read(tmp_fd, read_string, 8);
	ap_radio = strncmp(read_string, "1", sizeof(char));
	if(!ap_radio) {
		*pass = 1;
		sprintf(res_string, "4. Dnsmasq is running.\n");
	} else {
		*pass = 0;
		sprintf(res_string, "4. ** Error: Dnsmasq is not running. **\n");
	}
	free(read_string);
	return res_string;
}

char *ap_diag_check_client(int *pass)
{
	int tmp_fd; 
	char *res_string = (char *)malloc(AP_DIAG_RESULT_STRING_SIZE*sizeof(char));
	char *read_string = (char *)malloc(256*sizeof(char));
	memset(read_string, 0, 256*sizeof(char));
	char *client = NULL;
	int num_client;
	if (res_string == NULL)
		exit(-1);
	system("ap_list_client");
	sleep(1);

	tmp_fd = open(CNET_CLIENT_FILE, O_RDONLY);
	if (tmp_fd == -1) {
		perror("open /tmp/ap_connected_client");
		exit(-1);
	}
	read(tmp_fd, read_string, 256);
	client = strstr(read_string, "Number of STA =");
	num_client = (int)(*(client+strlen("Number of STA = ")) - '0');
	if( num_client > 0) {
		*pass = 1;
		sprintf(res_string, "5. %d Client%c is connected to AP\n", num_client, (num_client>1? 's' : ' '));
	} else {
		*pass = 0;
		sprintf(res_string, "5. ** Error: No Client is connecting to AP **\n");
	}

	free(read_string);
	return res_string;
}

char *ap_diag_ping_client(int *pass)
{
	int tmp_fd; 
	char *res_string = (char *)malloc(AP_DIAG_RESULT_STRING_SIZE*sizeof(char));
	char *read_string = (char *)malloc(256*sizeof(char));
	memset(read_string, 0, 256*sizeof(char));
	char *client_mac= NULL, *client_ip = NULL;
//	if (res_string == NULL)
//		exit(-1);
//	system("ap_list_client");
//	sleep(1);
	tmp_fd = open(CNET_CLIENT_FILE, O_RDONLY);
	if (tmp_fd == -1) {
		perror("open /tmp/ap_connected_client");
		exit(-1);
	}
	read(tmp_fd, read_string, 256);
	/* get mac address */
	client_mac = fetch_client_mac(read_string);
	/* get ip address */
	if( client_mac != NULL) 
		client_ip = fetch_client_ip(client_mac);
	/* client ping test */
	if (client_ip != NULL) 
		*pass = ping_client(client_ip);
	
	if (*pass == 1)
		sprintf(res_string, "6. ping Client %s Success\n", client_ip);
	else 
		sprintf(res_string, "6. ** Error: ping Client %s fail **\n", client_ip);

	free(client_ip);
	free(read_string);
	return res_string;
}

char *ap_diag_check_NAT(int *pass) {
	int tmp_fd; 
	char *res_string = (char *)malloc(AP_DIAG_RESULT_STRING_SIZE*sizeof(char));
	char *read_string = (char *)malloc(8*sizeof(char));
	memset(read_string, 0, 8*sizeof(char));

	if (res_string == NULL)
		exit(-1);
	system("ap_check_NAT");

	tmp_fd = open(NAT_CHECK_FILE, O_RDONLY);
	if (tmp_fd == -1) {
		perror("open /tmp/nat_check_res");
		exit(-1);
	}
	read(tmp_fd, read_string, 8);
	
	if( atoi(read_string) == 1) {
		*pass = 1;
		sprintf(res_string, "9. NAT is enable\n");
	} else {
		*pass = 0;
		sprintf(res_string, "9. ** Error: NAT is Disable **\n");
	}

	free(read_string);
	return res_string;
}

char *ap_print_cur_dns(void) {
	int tmp_fd; 
	char *res_string = (char *)malloc((AP_DIAG_RESULT_STRING_SIZE*2)*sizeof(char));
	char *read_string = (char *)malloc((AP_DIAG_RESULT_STRING_SIZE*2)*sizeof(char));
	memset(read_string, 0, AP_DIAG_RESULT_STRING_SIZE*2*sizeof(char));
	tmp_fd = open("/etc/resolv.conf", O_RDONLY);
	if (tmp_fd == -1) {
		perror("open \"etc/resolv.conf\" ");
		sprintf(res_string, "8. ** Error: Open DNS file \"etc/resolv.conf\" fail. \n");
	}
	read(tmp_fd, read_string, 80*sizeof(char));
	sprintf(res_string, "8. DNS server:\n%s", read_string);

	free(read_string);
	return res_string;
}

/*
char *ap_print_cur_routing(void) {
	int tmp_fd; 
	char *res_string = (char *)malloc((AP_DIAG_RESULT_STRING_SIZE*2)*sizeof(char));

	tmp_fd = open("/tmp/ip_routing_table", O_RDONLY);
}
*/

char *ap_diag_check_dns(int *pass) {
	int tmp_fd; 
	char *res_string = (char *)malloc(AP_DIAG_RESULT_STRING_SIZE*sizeof(char));
	char *read_string = (char *)malloc(8*sizeof(char));
	memset(read_string, 0, 8*sizeof(char));

	if (res_string == NULL)
		exit(-1);
	system("check_dns_working");
	tmp_fd = open(DNS_CHECK_FILE, O_RDONLY);
	if (tmp_fd == -1) {
		perror("open /tmp/dns_check_res");
		exit(-1);
	}
	read(tmp_fd, read_string, 8*sizeof(char));
	if( atoi(read_string) == 1) {
		*pass = 1;
		sprintf(res_string, "DNS is working well.\n");
	} else {
		*pass = 0;
		sprintf(res_string, "** Error: DNS is not working **\n");
	}

	free(read_string);
	return res_string;
}

char *fetch_client_mac(char *read_string) {
	char *client_mac = NULL;
	int i = 0;
	
	client_mac = strstr(read_string, "MAC Address");
	if( client_mac != NULL) {
		client_mac = client_mac + strlen("MAC Address: ");
		while(*(client_mac+i) != '\n') i++;
		*(client_mac+i) = '\0';
	}

	return client_mac;
}
#define RETRY_WRITE 3
char *fetch_client_ip(char *client_mac) {
	int i = 0;
	int dhcp_fd = open(DHCP_LEASE_FILE, O_RDONLY);
	int ip_fd = open(CLIENT_IP_FILE, O_CREAT | O_RDWR);
	int times = 0;
	
	char *read_string = (char *)malloc(256*sizeof(char));
	char *client_ip = NULL;

	if (dhcp_fd == -1) {
		perror("open /var/run/dhcp.leases");
		exit(-1);
	}
	read(dhcp_fd, read_string, 256);	
	client_ip = strstr(read_string, client_mac);	
	if( client_ip != NULL) {
		client_ip = client_ip + strlen(client_mac) + 1;	
		while(*(client_ip+i) != ' ') i++;
		*(client_ip+i) = '\0';
	}
	/* write the ip to a file */
	if (ip_fd == -1) {
		perror("open /tmp/client_ip fail");
		exit(-1);
	}
	while ( times++ < RETRY_WRITE) {
		if(write(ip_fd, client_ip, strlen(client_ip)) == strlen(client_ip)) 
			break;
	}
		
	return client_ip;
}

int ping_client(char *client_ip) {
	int ping_res_fd;
	int res = 0;
	char *ping_client_res = (char *)malloc(8*sizeof(char));
	memset(ping_client_res, 0, 8);

	system("ping_client");
	ping_res_fd = open(PING_RES_FILE, O_RDONLY);
 	if (ping_res_fd == -1) {
		perror("open /tmp/client_ip_res fail");
		exit(-1);
	}
	read(ping_res_fd, ping_client_res, 8*sizeof(char));
	res = atoi(ping_client_res);

	free(ping_client_res);
	return res;
}

int upload_ap_diag_log(uint32_t file_size, char * upload_file)
{
	char working_str[MAX_PAYLOAD_SIZE];
	const char *s1, *s2;
	char *pt;
	time_t now;
	uint32_t l1, l2;
	int status = -1;
	if (network_link_status != 0)
		return -1;
	if (ap_diag_log_req.status != 0) {
		if ((strncmp(ap_diag_log_req.upfile, upload_file, strlen(upload_file)) == 0)
									&& (ap_diag_log_req.request_size == file_size))
			goto coming_back;
	}
	strncpy(working_str, ap_diag_log_server_url, MAX_PAYLOAD_SIZE);  
	if ((s1 = strchr(working_str, '/')) == NULL)
		goto failure;
	if ((s2 = strchr(working_str, ':')) != NULL) {
		if ((s1 - s2) < 3)
			goto failure;
		strncpy(ap_diag_log_req.port, s2 + 1, (s1 - s2 - 1));
	}
	else {
		snprintf(ap_diag_log_req.port, 8, "443");
		s2 = s1;
	}
	strncpy(ap_diag_log_req.host, working_str, (s2 - working_str));
	strncpy(ap_diag_log_req.path, s1, sizeof(ap_diag_log_req.path));
	s1 = propGetString(PROP_LOGGING_USER, "");
	strncpy(ap_diag_log_req.user_pass, s1, MAX_ID_SIZE);
	s2 = propGetString(PROP_LOGGING_PASS, "");
	snprintf(ap_diag_log_req.user_pass + strnlen(s1, MAX_ID_SIZE), MAX_ID_SIZE + 2, ":%s", s2);
	sprintf(working_str, "gzip -c %s > %s.gz", upload_file, upload_file);
	if ((status = system(working_str)) != 0)
		goto failure;
	snprintf(ap_diag_log_req.upfile, sizeof(ap_diag_log_req.upfile), "%s.gz", upload_file);
coming_back:
	now = time(&now);
	s1 = propGetAccountID();
	l1 = strnlen(s1, MAX_ID_SIZE);
	strncpy(ap_diag_log_req.query, s1, MAX_ID_SIZE);
	s2 = propGetDeviceID(0);
	snprintf(ap_diag_log_req.query + l1, MAX_ID_SIZE + 1, "_%s", s2);
	l2 = strnlen(ap_diag_log_req.query, sizeof(ap_diag_log_req.query));
	sprintf(ap_diag_log_req.query + l2, "_ap_diag_%08x.log.gz", (unsigned)now);
	status = http_uploader(&ap_diag_log_req);
failure:
	if (status == 0) {
		strncpy(working_str, ap_diag_log_req.query, MAX_PAYLOAD_SIZE);
		pt = strstr(working_str, ".gz");
		sprintf(pt, " size:%d", file_size);
		report_mission_status(STATUS_CLIENT_LOGGING_OK, working_str);
		usleep(100000);
		unlink(ap_diag_log_req.upfile);
		unlink(upload_file);
	}
	else if (status != -1) {
		report_mission_status(STATUS_CLIENT_LOGGING_FAILED, ap_diag_log_req.err_msg);
	}
	ap_diag_log_req.request_size = file_size;
	ap_diag_log_req.status = status;
	return status;
}