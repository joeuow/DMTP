#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdbool.h>
#include <unistd.h>
#include <errno.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/syslog.h>
#include <pthread.h>
#include <fcntl.h>
#include "diagnostic.h"
#include "propman.h"

#define DMTP_ALIVE "/tmp/dmtp_alive"

pthread_t dmtp_alive = 0;
void *dmtp_alive_main(void *arg);

int start_dmtp_alive(void)
{
	if (pthread_create(&dmtp_alive, NULL, dmtp_alive_main, NULL) != 0) {
		perror("Creating Thread:");
		return -1;
	}
	return 0;
}

void *dmtp_alive_main(void *arg) {
	int fd;
	unsigned int intrvl =  propGetUInt32AtIndex(PROP_STATE_ALIVE_INTRVL, 0, 30);
	unsigned int counter = 0; 

	do {
		if (counter++ >= intrvl) {
			if ((fd = open(DMTP_ALIVE, O_RDWR | O_CREAT)) < 0) {
				perror("open /tmp/dmtp_alive");
				return NULL;
			}

			close(fd);
			counter = 0;
		}
		sleep(1);
	} while(1);
}