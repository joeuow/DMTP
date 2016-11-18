#define _GNU_SOURCE
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
#include <linux/watchdog.h>
#include "watchdog_f.h"
#include "debuglog.h"

pthread_t thread_watchdog = 0;
bool watchdog_running = false;
bool reboot_pending = false;
void * watchdog_main(void * thread_args);
watchdog_watch_func watch_function1 = NULL;
watchdog_watch_func watch_function2 = NULL;
watchdog_recurrent_func recurrent_function1 = NULL;
extern void watchdog_reboot(void);
extern void watchdog_alert(void);
extern int flash_running_light(int mode);

int start_watchdog(void)
{
	if (pthread_create(&thread_watchdog, NULL, watchdog_main, NULL) != 0) {
		perror("Creating Thread:");
		return -1;
	}
	return 0;
}

int stop_watchdog(void)
{
	if (watchdog_running) {
		watchdog_running = false;
		pthread_kill(thread_watchdog, SIGUSR1);
	}
	else
		return 0;
	pthread_join(thread_watchdog, NULL);
	return 0;
}

void add_watchdog_func(watchdog_watch_func func)
{
	if (watch_function1 == NULL)
		watch_function1 = func;
	else
		watch_function2 = func;
}

void add_recurrent_func(watchdog_recurrent_func func)
{
	if (recurrent_function1 == NULL)
		recurrent_function1 = func;
}

void * watchdog_main(void * thread_args)
{
	int watchdog_fd = -1;
	int alert = 0, alert1 = 0, alert2 = 0;
	int counter1 = 0;
	unsigned int watchdog_dummy = 0;
/* not going to take control of the watchdog from the kernel */
/*
	if ((watchdog_fd = open("/dev/watchdog", O_RDWR)) < 0) {
		printf("Cannot Open hardware Watchdog\n");
		return NULL;
	}
	else
		printf("Watchdog started\n");
*/

	watchdog_running = true;
	while (watchdog_running) {
		if (sleep(2) != 0)
			continue;
		if (++counter1 >= 5) {
			if (watch_function1 != NULL) {
				if (watch_function1())
					alert1++;
				else {
					if (alert1 != 0)
						alert1 = 0;	
				}
			}
			if (watch_function2 != NULL) {
				if (watch_function2())
					alert2++;
				else {
					if (alert2 != 0)
						alert2 = 0;
				}
			}
			alert = alert1 + alert2;
//			printf(" alert: %d = %d + %d\n", alert, alert1, alert2);
			if (alert == 1)
				watchdog_alert();
			else if (alert > 3) {
				reboot_pending = true;
				debuglog(LOG_INFO, "Unit REBOOT!!");
				watchdog_reboot();	
				break;
			}

//			if (write(watchdog_fd, &watchdog_dummy, sizeof(int)) < 0)
//				perror("Writing to watchdog");
			/*if (recurrent_function1 != NULL)
				recurrent_function1(); */
			counter1 = 0;
		}
		flash_running_light(1);
	}
/* not going to take control of the watchdog from the kernel */
/*
	if (!reboot_pending) {
		if (ioctl(watchdog_fd, WDIOC_SETPRETIMEOUT, &watchdog_dummy) < 0)
			printf("Watchdog ioctl error\n");
		else
			printf("Watchdog ioctl\n");
	}
*/
	if (reboot_pending) {		/* reboot */	
		int pid = fork();
		if (pid < 0)
			printf("fork() Error\n");
		else if (pid == 0) {
//			if (write(watchdog_fd, &watchdog_dummy, sizeof(int)) < 0) // before reboot, write the watchdog once to gain another 16 seconds.
//				perror("Writing to watchdog");
			printf("Going to reboot the unit\n");
			execl("/ositech/reboot_unit", "reboot_unit", 0, 0);
			printf("reboot error\n");
		}
	}
/*	if ((close(watchdog_fd)) < 0)
		printf("Cannot Close Watchdog\n");
	else
		printf("Watchdog stopped\n");
*/	
	return NULL;
}
