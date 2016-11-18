/***********************************************************************
* Original Author:   Joe Wei
* File Creation Date: 2014/07/09
* Project: Titan III
* Description:  This is the program to get the signal strength
* File Name: gobi_debug.c
* Last Modified:  2014/08/28
* Changes:
**********************************************************************/
/*
 * This file contains proprietary information and is subject to the terms and
 * conditions defined in file 'OSILICENSE.txt', which is part of this source
 * code package.
 */



/* Copyright Option International 2010 */

#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
//#include "../GobiConnectionMgmt/GobiConnectionMgmtAPI.h"
#include <getopt.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <errno.h>
#include <ctype.h>
#include <syslog.h>
#include <time.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/un.h>
#include <sys/uio.h>
#include <sys/time.h>
#include <sys/wait.h>
#include <sys/stat.h>
#include <stdarg.h>
#include <dirent.h>
#include <pthread.h>
#include <fcntl.h>
#include <debuglog.h>
#include "GobiConnectionMgmtAPI.h"

#define GobiEthName1                    "gobi0"
#define GOBINODE0 "qcqmi0"
#define GOBINODE1 "qcqmi1"
#define QMI_GET_MEID_IOCTL 0x8BE0 + 3

#define SIGNL_FIRMWARE_FIFO		"/tmp/cell_firmware_info"

#define SIGNL_FIRMWARE_LENGTH 128

/***********************************************************************
*
* Description: This function check the gobi cell firmware information
*
* Calling Arguments:
* Name               Mode      Description
* None

* Return Value:
*    Type      Description
*     int      Success . See below.
*
* Return Codes/Exceptions:
* 0     Get the gobi cell signal strength.
*
******************************************************************************/
static int CheckFirmware(void)
{
	int ret;
	int res;
	int fd;
	int handle;

	ULONG RegistrationState;
	ULONG CSDomain;
	ULONG PSDomain;
	ULONG RAN;
	BYTE RadioIfacesSize;
	BYTE RadioIfaces;
	ULONG Roaming;
	WORD MCC;
	WORD MNC;
	BYTE nameSize = 50;
	CHAR pName[32];
	char message[SIGNL_FIRMWARE_LENGTH] = {};
	
	char s[16];
       memset(&s[1],0,16);

	if ((handle=open("/dev/qcqmi0", O_RDWR))< 0) {
		perror("open /dev/qcqmi0");
		return -1;
	}
	if(ioctl(handle, QMI_GET_MEID_IOCTL, &s[0]) !=0) {
		perror("ioctl /dev/qcqmi0");
		return -1;
	}

	close(handle);

       ret = QCWWANConnect(GOBINODE1, s);
        if(ret != 0)
        {
              ret = QCWWANConnect(GOBINODE0, s);
              if(ret != 0) {
              	printf("Can not connect device with driver. ");
                        	return -1;
              }
       }

	ret = GetServingNetwork(
						&RegistrationState,
						&CSDomain,
						&PSDomain,
						&RAN,
						&RadioIfacesSize,
						&RadioIfaces,
						&Roaming,
				 		&MCC,
				 		&MNC,
				 		nameSize,
				 		pName);
	QCWWANDisconnect();
	
	if (!ret) {							
		fd = open(SIGNL_FIRMWARE_FIFO, O_WRONLY);
		if (fd < 0) {
			perror("open fifo");
			exit(-1);
		}

		res = write(fd, &pName, strlen(pName));
		close(fd);
	}
		
	return 0;
}

int main(int argc, char *argv[])
{
        ULONG ret;
//       ret = QCWWANDisconnect();
	   
	ret = CheckFirmware();

	return ret;
}

