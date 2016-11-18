#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <stdbool.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <pthread.h>
#include "diagnostic.h"
#include "propman.h"

#define SERIAL_DEVICE	"/dev/ttyS1"
#define MINUTE	60

pthread_t RTSmonitor_thread_main;

static int open_serial(void);
static void close_serial(int serial_fd);
static int check_RTS(int serial_fd);
static bool cmp_RTS(int cur_rts, int *pre_rts);
static void report_RTS(int cur_rts);
void * thread_RTSmonitor_main(void *args);
static void report_RTS_init(void);

int SerialPinMonitor_initialize(void)
{
	if (pthread_create(&RTSmonitor_thread_main, NULL, thread_RTSmonitor_main, NULL) != 0) {
		printf("Creating Thread Error\n");
		return -1;
	}
	
	return 0;
}

void * thread_RTSmonitor_main(void *args) {
	int serial_fd;
	int cur_rts_status, pre_rts_status;
	unsigned int check_intvl = propGetUInt32AtIndex(PROP_STATE_RTS_CHECK, 1, 1);
	check_intvl *= MINUTE;
	int init_report = 0;
	
	while(1) {
		serial_fd = open_serial();
		cur_rts_status = check_RTS(serial_fd);
		if (!init_report) {
			report_RTS(cur_rts_status);
			init_report = 1;
			pre_rts_status = cur_rts_status;
		} else  {
			if(cmp_RTS(cur_rts_status, &pre_rts_status)) 
				report_RTS(cur_rts_status);
		}
		
		close_serial(serial_fd);
		sleep(check_intvl);
	}

	return NULL;
}

static int open_serial(void)
{
	int serial_fd;

	serial_fd = open(SERIAL_DEVICE, O_RDWR); 
	if (serial_fd < 0) {
		printf("open failed (%d)\n", errno);
		exit(-1);
	}

	return serial_fd;
}

static void close_serial(int serial_fd) {
	close(serial_fd);
}

/* return status to indicate the current level of the RTS */
static int check_RTS(int serial_fd) {
	int status;
	int RTS_status;

	if(ioctl(serial_fd, TIOCMGET, &status) == -1) {
		printf("ioctl failed (%d)\n", errno);
		return -1;
	}

	RTS_status = status & TIOCM_RTS;

//	printf("RTS_status: %d\n", RTS_status);

	return RTS_status;
}

static bool cmp_RTS(int cur_rts, int *pre_rts) {
	if (*pre_rts != cur_rts)	 {
		*pre_rts = cur_rts;
		return true;
	}
	
	return false;
}

static void report_RTS(int cur_rts) {
	char message[64];
	
	sprintf(message, "RTS Powered %s", cur_rts?"ON":"OFF");
	printf("%s\n", message);
	diagnostic_report(DIAGNOSTIC_MESSAGE, 0, message);
}
