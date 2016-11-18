#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <errno.h>
#include <sys/types.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <dirent.h>
#include <debuglog.h>
#include <termios.h> 
#include "diagnostic.h"
#include "GobiConnectionMgmtAPI.h"
#include "libwlcomm.h"
#include "propman.h"

#define SIGNL_STRENGTH_FIFO		"/tmp/cell_signl_strength"
#define SIGNL_FIRMWARE_FIFO		"/tmp/cell_firmware_info"

#define DNSMASQ_DEBUG_FILE		"/tmp/dmtp_debug_dnsmasq.txt"
#define CARRIER_FILE				"/tmp/current_carrier.txt"
#define GOBI_USB_FILE 				"/tmp/gobi_usb_interface"
#define TTYUSB1_OPEN				"/tmp/ttyUSB1_port_open"

#define CONTENT_LENGTH				256
#define STRING_SIZE					64
#define BUFF_SIZE					256
#define ATCMD_MESSAGE_LENGTH		256
//#define LOOP_CYCLE					30

/*-- Debug events --*/
#define CELL_SIGNAL_CHANGE_EVENT		0x1
#define CELL_CARRIER_CHANGE_EVENT		0x2
#define DHCP_ASSIGNING_STATE_EVENT 	0x4
#define SERVER_NO_RESPONDING_EVENT 	0x8
#define CELL_TEMPERATURE_EVENT		0x10

#define GOBINODE0 "qcqmi0"
#define GOBINODE1 "qcqmi1"
#define QMI_GET_MEID_IOCTL 0x8BE0 + 3

#define CARRIER_NAME_SIZE 	32
#define REGION_SIZE			32
/* Region Code */
#define USA		310
#define Canada	302
/* Carrier Code */

static void check_dnsmasq_state(void);
static bool check_cell_sig_change(int signl_threshold);

extern UInt32 network_link_status;
//static bool check_cell_sig_bearer(void);
void *network_diagnostic(void *thread);
static bool network_down(void);
static bool check_gobi_power(void);
static int get_signl_strength(void);
static int get_signal_info(char *message);
static int open_gobi_serial_port(void);
static int sendATcmd(int fd, char *atcmd);
static void getATresponse(int fd, int toSec, char *message);
static int ATcmd_operate(int fd, char *atcmd, char *message);
static int OpenGobi(void);
static int getRFInfo(unsigned int *band, unsigned int *channel);
static void getBand(int band, char *band_string);
static int get_carrier_from_generic(char *message);
static void get_current_carrier(void);
static void get_current_cell_temperature(int *pre_temp);

pthread_t thread_network_diag_main;

static char *dataBearer[] = {
        "Unknown",
        "CDMA 1xRTT",
        "CDMA 1xEV-DO Rev 0",
        "GPRS",
        "WCDMA",
        "CDMA 1xEV-DO Rev A",
        "EDGE",
        "HSDPA DL, WCDMA UL",
        "WCDMA DL, HSUPA UL",
        "HSDPA DL, HSUPA UL"
};

void network_diagnostic_init(void) {
	if (pthread_create(&thread_network_diag_main, NULL, network_diagnostic, NULL) != 0) {
		printf("Creating Thread Error\n");
		return;
	}
}

#define NORM_RPT_INTRVL 30
#define CELL_TEMP_INIT	0xffff
void *network_diagnostic(void *thread) {
	bool sig_vary = false;
	bool res;
	int events;
	int arg;
	int signl_change;
	int dhcp_assign;
	int carrier_change;
	int cell_temperature;
	static unsigned int counter = 1, cell_temp_rpt_counter = 1;
	bool norm_rpt = false;
	UInt32 cell_temp_rpt_intrvl = propGetUInt32AtIndex(PROP_STATE_CELL_TEMP_RPT_SETUP, 0, 30);
	static int pre_temp = CELL_TEMP_INIT;
	
//	system("/ositech/cell_signl_monitor 1>/dev/null 2>/dev/null &");
	usleep(100);
	while(1) {		
		events = propGetUInt32AtIndex(PROP_STATE_NETWORK_DEBUG, 0, 0);
		signl_change = (events& CELL_SIGNAL_CHANGE_EVENT);
		carrier_change = (events & CELL_CARRIER_CHANGE_EVENT);
		dhcp_assign =  (events & DHCP_ASSIGNING_STATE_EVENT);
		cell_temperature =  (events & CELL_TEMPERATURE_EVENT);

		if (events & 0xF) {
			if (counter >= NORM_RPT_INTRVL) {
				norm_rpt = true;
				counter = 1;
			} else
				counter++;
		}
		
		if (norm_rpt) {			
			if (dhcp_assign)
				check_dnsmasq_state();
			if (signl_change) {
				arg = propGetUInt32AtIndex(PROP_STATE_NETWORK_DEBUG, 1, 0);
				sig_vary = check_cell_sig_change(arg);
				if (sig_vary) {
					res = network_down();
					if (!res) {
						diagnostic_report(DIAGNOSTIC_CELL_DOWN, 0, NULL);
					} else {
						diagnostic_report(DIAGNOSTIC_CELL_DOWN, 1, NULL);
					}
				}
			}
			if (carrier_change)
				get_current_carrier();

			norm_rpt = false;
		} 

		if (cell_temperature) {
			if(cell_temp_rpt_counter >= cell_temp_rpt_intrvl) {
				get_current_cell_temperature(&pre_temp);
				cell_temp_rpt_counter = 1;
			} else
				cell_temp_rpt_counter++;
		} else
			pre_temp = CELL_TEMP_INIT;
		
		sleep(1);
	}
	return;
}

static bool check_cell_sig_change(int signl_threshold) {
	int ret;
//	int fd;
	char message[CONTENT_LENGTH];
	int cur_signl_streng = 0;
	static int pre_signl_streng = 0;
	bool res = false;

	ret = check_gobi_power();
	if (ret) {
		cur_signl_streng = get_signl_strength();
		if (cur_signl_streng && (pre_signl_streng - signl_threshold >= cur_signl_streng || cur_signl_streng - signl_threshold >= pre_signl_streng)) {
			memset(message, 0, CONTENT_LENGTH*sizeof(char));
			sprintf(message, "RSSI: %ddB,", cur_signl_streng);
			if(!OpenGobi()) {
				get_signal_info(message);
//			printf("message leng %d\n", strlen(message));
				diagnostic_report(DIAGNOSTIC_MESSAGE, 0, message);
				pre_signl_streng = cur_signl_streng;
					
				res = true;
			}
		}
	}	
	else 
		pre_signl_streng = 0;	/* if gobi is powered off, reset the last signal strength to 0,
								  * and ready to report the next available signal strength. */
//	QCWWANDisconnect();
	return res;
}

static void check_dnsmasq_state(void) {
	char dhcp_contents[1024] = {};
	char *string;
	int fd = 0;
	size_t res = 0;
	
	if (access(DNSMASQ_DEBUG_FILE, F_OK) == -1)
		return;
	
	fd = open(DNSMASQ_DEBUG_FILE, O_RDONLY);
	if (fd < 0) {
		printf("%s: %s\n", __FUNCTION__, strerror(errno));
		return;
	}

	res = read(fd, dhcp_contents, 1024*sizeof(char));
	string = strtok (dhcp_contents,"\n");
	while(string) {		
//		printf("[%s] %s\n", __FUNCTION__, string);
		diagnostic_report(DIAGNOSTIC_DHCP, 0, string);
		string = strtok (NULL, "\n");
	}
	close(fd);
	unlink(DNSMASQ_DEBUG_FILE);
}

static bool network_down(void) {
        FILE *fp;
        char buffer[256];
        char ip_addr[15];

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

static bool check_gobi_power(void) {
	static bool report_off = true;
	struct stat fileStat;
	
	system("find_gobi_usb");
	if (stat(GOBI_USB_FILE, &fileStat) < 0) {
		printf("%s\n", strerror(errno));
//		exit(-1);
/*	unexpected state and should just returns false */
		return false;
	}
	if (fileStat.st_size <= 0) {
//	if (access("/sys/bus/usb/devices/1-1:1.0/GobiQMI:qcqmi0", F_OK) < 0) {
		if(report_off) {
             		debuglog(LOG_INFO, "[ositechdmtp] Gobi is not on.\n");
			report_off = false;
		}
              return false;
       }

	report_off = true;
	return true;
}

/***********************************************************************
*
* Description: This function check the gobi cell signal strength
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
static int OpenGobi(void)
{
	int ret;
	int handle;
	char s[16];
       memset(&s[1],0,16);

//  	ret = QCWWANDisconnect();	
	
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
		
	return 0;
}

/***********************************************************************
*
* Description: This function check the gobi data bearer
*
* Calling Arguments:
* Name               Mode      Description
* None

* Return Value:
*    Type      Description
*     int      Success . See below.
*
* Return Codes/Exceptions:
* -1     error
*
******************************************************************************/
static int get_signal_info(char *message)
{
	ULONG bearer = 0;
	unsigned int band = -1;
	unsigned int channel = 0;
	int res = 0;
	char dataBand[32] = {};

//	OpenGobi();
	
	if(getRFInfo(&band, &channel)) {
		printf("Failed to getRFInfo\n");
		res = -1;
	}
	getBand(band, dataBand);
	sprintf(message+strlen(message), " %s, Chnl.%d,", dataBand, channel);
	
	if(GetDataBearerTechnology(&bearer)) {
		printf("Failed to GetDataBearerTechnology\n");
		res = -1;	
	}
	sprintf(message+strlen(message), " Bearer=%s", dataBearer[bearer]);
	
	QCWWANDisconnect();
	return res;
}

static int open_gobi_serial_port(void) {
	struct termios newtio, oldtio;
//	static bool shut_echo = false;
        int fd;
	char atcmd_message[ATCMD_MESSAGE_LENGTH] = {};

	if((fd = open("/dev/ttyUSB1", O_RDWR|O_NONBLOCK|O_NOCTTY|O_NDELAY)) < 0) {
		perror("Gobi Serial Port Open Fail\n");
		return -1;
	} 
	
//	if (fd > 0){
//    		tcgetattr(fd,&oldtio); /* save current serial port settings */
//     		bzero(&newtio, sizeof(newtio)); /* clear struct for new port settings */
     
/* 
     BAUDRATE: Set bps rate. You could also use cfsetispeed and cfsetospeed.
     CS8     : 8n1 (8bit,no parity,1 stopbit)
     CLOCAL  : local connection, no modem contol
     CREAD   : enable receiving characters No hardware control
     */
    	 	newtio.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
     	/* Raw output. */
    		newtio.c_oflag &= ~OPOST;
	/* raw input */
		newtio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	/*  now clean the modem line and activate the settings for the port  */
		tcflush(fd, TCIFLUSH);
		tcsetattr(fd,TCSANOW,&newtio);	
/*		if(ATcmd_operate(fd, "AT\r", atcmd_message)) {
		memset(atcmd_message, 0, ATCMD_MESSAGE_LENGTH*sizeof(char));
		if (shut_echo && ATcmd_operate(fd, "ATE0\r", atcmd_message))
			shut_echo = true;
		if (shut_echo)		
		}
	}
*/	
	return fd;
}

static int sendATcmd(int fd, char *atcmd) {
	int wfd;
	fd_set  wset;
	struct timeval timeout;
	ssize_t len;

	FD_ZERO(&wset);
	FD_SET(fd,&wset);

	timeout.tv_sec = 1;
	timeout.tv_usec = 0;
	
	wfd = select(FD_SETSIZE,  NULL, &wset, NULL, &timeout);
	if (wfd < 0) {
		perror("select");
		exit(EXIT_FAILURE);
	} else if (wfd > 0) {
		len = write(fd, atcmd, strlen(atcmd));
	} else {
		printf("Write to QDAC Port Timeout\n");
		return 0;
	}

//       printf("[ositechdmtp] sendATcmd: %d - %s\n", len, atcmd);
	debuglog(LOG_INFO, "[ositechdmtp] sendATcmd: %s\n", atcmd);

   	return len;
}

static void getATresponse(int fd, int toSec, char *message) {
	fd_set  rset;
	struct timeval tv;
	int ret = 0;
	int rfd;
	char *buff;

	tv.tv_sec = toSec;
   	tv.tv_usec = 0;

	buff = (char *)malloc(BUFF_SIZE*sizeof(char));
	if (buff)
 		memset(buff, 0 , BUFF_SIZE*sizeof(char));
	else {
		perror("malloc response buff error");
		exit(-1);
	}
   	//always need to zero it first, then add our new descripto
   	FD_ZERO(&rset);
   	FD_SET(fd, &rset);

   	//wait specified time for input to arrive
   	rfd = select(FD_SETSIZE, &rset, NULL, NULL, &tv);
	if (rfd < 0) {
		perror("select");
		exit(EXIT_FAILURE);
	} else if (rfd > 0) {
      		if(FD_ISSET(fd, &rset)) {
             		ret = read(fd, buff, BUFF_SIZE);
			if (ret >= 0) {
//				debuglog(LOG_INFO, "[ositechdmtp] %s: %s\n", __FUNCTION__, buff);
				strcpy(message, buff);
			}
      		} else
          		printf("Timed out waiting for response!\n");
  	} 
	
  	free(buff);
}

static int get_signl_strength(void) {
	int res;
	int fd;
	signed char cur_signl = 0;
	fd_set read_fd_set;
	struct timeval timeout; 

	FD_ZERO(&read_fd_set);

	
	if(access(SIGNL_STRENGTH_FIFO, F_OK) == -1) {
		res = mkfifo(SIGNL_STRENGTH_FIFO, 0777);
		if (res) {
			perror("mkfifo");
			exit(-1);
		}
	}
	
	system("/ositech/get_signl_strength &");

/* open FIFO with nonblock mode to get the return from open right way */
	fd = open(SIGNL_STRENGTH_FIFO, O_RDONLY | O_NONBLOCK);
	if (fd < 0) {
		perror("open fifo");
		exit(-1);
	}
	
	FD_SET(fd, &read_fd_set);
	timeout.tv_sec = 10;
	timeout.tv_usec = 0;
	
	res = select(FD_SETSIZE , &read_fd_set, (fd_set *) 0, 
			  (fd_set *) 0, &timeout);

	if (res < 0) {
		perror("select");
		exit(EXIT_FAILURE);
	} else if (res) {
		if(FD_ISSET(fd, &read_fd_set)) {
			res = read(fd, &cur_signl, sizeof(cur_signl));
//	if (res)
//		printf("cur_signl = %d\n", cur_signl);
			close(fd);
		}
	}
	
	return cur_signl; // 0 as fail
}

static int ATcmd_operate(int fd, char *atcmd, char *message) {
	int i,k;
	
	i = sendATcmd(fd, atcmd);
     	if ( i > 0) {
		for ( k = 0; k < 3; k++) {
        		getATresponse(fd, 15, message);
        		if (*message == 0) { // no contents
				usleep(500);
				continue;
			} else {
				if (strstr(message, "OK") != NULL) {

					debuglog(LOG_INFO,"[ositechdmtp] %s response: %s\n", atcmd, message);
					return 1;
				} else {
					memset(message, 0 , ATCMD_MESSAGE_LENGTH*sizeof(char));
					usleep(500);
					continue;
				}
			}
		}
	}else {
			debuglog(LOG_INFO,"[ositechdmtp] %s send AT command fails!\n", __FUNCTION__);
	}

	return 0;
}

static int getRFInfo(unsigned int *band, unsigned int *channel)
{
   	ULONG ret;
    	ULONG gi[3];
   	 ULONG byte = 1;
//	 BYTE gi[16];

    	ret=GetRFInfo((BYTE *)&byte, (BYTE *)&gi[0]);
//	ret=GetRFInfo(byte, gi);
    	if(!ret) {
//		printf("GetRFInfo ret=%d array nb=%d radioIf=%d,active band class=%d, active chanel=%d\n",(int)ret,(unsigned int)byte[0],(unsigned int)gi[0],(unsigned int)gi[1],(unsigned int)gi[2]);
		*band = (unsigned int)gi[1];
		*channel = (unsigned int)gi[2];
    	}
	else
		return -1;
	
	return 0;	
  //	return ret;
}

static void getBand(int band, char *band_string) {
	if(band >= 0 && band <= 16)	sprintf(band_string, "CDMA Band Class %d", band);
	else {
		switch(band) {
			case 40:
				strcpy(band_string, "GSM 450");
				break;
			case 41:
				strcpy(band_string, "GSM 480");
				break;
			case 42:
				strcpy(band_string, "GSM 750");
				break;
			case 43:
				strcpy(band_string, "GSM 850");
				break;
			case 44:					
				strcpy(band_string, "GSM 900 (Extended)");
				break;
			case 45:
				strcpy(band_string, "GSM 900(Primary)");
				break;
			case 46:
				strcpy(band_string, "GSM 900(Railways)");
				break;
			case 47:
				strcpy(band_string, "GSM 1800");
				break;
			case 48:
				strcpy(band_string, "GSM 1900");
				break;
			case 80:
				strcpy(band_string, "WCDMA 2100");
				break;
			case 81:
				strcpy(band_string, "WCDMA PCS 1900");
				break;
			case 82:
				strcpy(band_string, "WCDMA DCS 1800");
				break;
			case 83:
				strcpy(band_string, "WCDMA 1700 (US)");
				break;
			case 84:
				strcpy(band_string, "WCDMA 850");
				break;
			case 85:
				strcpy(band_string, "WCDMA 800");
				break;
			case 86:
				strcpy(band_string, "WCDMA 2600");
				break;
			case 87:
				strcpy(band_string, "WCDMA 900");
				break;
			case 88:
				strcpy(band_string, "WCDMA 1700 (Japan)");
				break;
			default:
				strcpy(band_string, "Unknown");
				break;
		}
	}
}

static int get_carrier_from_generic(char *message) {
		int res;
		int fd;
		char carrier_name[32];
		
		if(access(SIGNL_FIRMWARE_FIFO, F_OK) == -1) {
			res = mkfifo(SIGNL_FIRMWARE_FIFO, 0777);
			if (res) {
				perror("mkfifo");
				exit(-1);
			}
		}
	
		system("/ositech/get_firmware_info &");

		fd = open(SIGNL_FIRMWARE_FIFO, O_RDONLY);
		if (fd < 0) {
			perror("open fifo");
			exit(-1);
		}

		res = read(fd, carrier_name, sizeof(carrier_name));
		close(fd);		
		if (res) {
			sprintf(message, "Carrier: %s", carrier_name);
		} else {
			strcpy(message, "Cannot Get Carrier");
		}

		return 0;
}

#define CARRIER_LENGTH 64
int CheckCarrier(void) {
	char message[CARRIER_LENGTH]= {};
	int fd, res;
	
	ULONG FirmwareID;
	ULONG Technology;
	ULONG Carrier;
	ULONG Region;
	ULONG GPSCapability;
	ULONG ret;
	
	if(!OpenGobi()) {
		ret = GetFirmwareInfo(&FirmwareID, &Technology, &Carrier, &Region, &GPSCapability);
		QCWWANDisconnect();
		if(ret) {
			printf("get GetFirmwareInfo failed\n");	
			strcpy(message, "Cannot Get Carrier\n");
		}
	}
	switch(Carrier) {
		case 1:
			get_carrier_from_generic(message);
			break;
		case 101:
			strcpy(message, "Carrier: Verizon");
			break;
		case 102:
			strcpy(message, "Carrier: Sprint");
			break;
		case 104:
			strcpy(message, "Carrier: Bell");
			break;
		case 105:
			strcpy(message, "Carrier: Telus");
			break;
		case 201:
			strcpy(message, "Carrier: AT&T");
			break;
		case 202:
			strcpy(message, "Carrier: Vodafone");
			break;
		case 203:
			strcpy(message, "Carrier: T-Mobile");
			break;
		case 204:
			strcpy(message, "Carrier: Orange");
			break;
		case 205:
			strcpy(message, "Carrier: Telefonica");
			break;
		case 206:
			strcpy(message, "Carrier: Telecom Italia");
			break;
		default:
			strcpy(message, "Carrier: Unknown");
			break;
	}

	fd = open(CARRIER_FILE, O_WRONLY | O_CREAT | O_TRUNC);
	if (fd < 0) {
		printf("%s: %s\n", __FUNCTION__, strerror(errno));
		return -1;
	}

	res = write(fd, message, CARRIER_LENGTH*sizeof(char));
	close(fd);
	
	return 1;

}

int Server_NO_Response(void) {
		int events = propGetUInt32AtIndex(PROP_STATE_NETWORK_DEBUG, 0, 0);
		int server_no_responding = (events & SERVER_NO_RESPONDING_EVENT);
		if (server_no_responding)
		diagnostic_report(DIAGNOSTIC_MESSAGE, 0, "DMTP Server Not Responding");

		return 0;
}

static void get_current_carrier(void) {
	char carrier[CARRIER_LENGTH] = {};

	int fd = 0;
	size_t res = 0;
	
	if (access(CARRIER_FILE, F_OK) == -1)
		return;
	
	fd = open(CARRIER_FILE, O_RDONLY);
	if (fd < 0) {
		printf("%s: %s\n", __FUNCTION__, strerror(errno));
		return;
	}

	res = read(fd, carrier, CARRIER_LENGTH*sizeof(char));
	if (res > 0)
		diagnostic_report(DIAGNOSTIC_MESSAGE, 0, carrier);

	close(fd);
	unlink(CARRIER_FILE);
}

static void get_current_cell_temperature(int *pre_temp) {
	char string[64] = {};
	char atcmd_msg[ATCMD_MESSAGE_LENGTH] = {};
	char temp_string[8] = {};
	char *pOK = NULL;
	int cur_temp;
	int i = 0, j=0;
	int fd, ttyUSB1_fd;
	bool res = false;
	int ATcmd_ret;

	//UInt32 cell_temp_low = propGetUInt32AtIndex(PROP_STATE_CELL_TEMP_RPT_SETUP, 1, 0);
	int cell_temp_low = propGetUInt32AtIndex(PROP_STATE_CELL_TEMP_RPT_SETUP, 1, 0);
	UInt32 cell_temp_high = propGetUInt32AtIndex(PROP_STATE_CELL_TEMP_RPT_SETUP, 2, 60);

	res = check_gobi_power();
	if (res) {
		fd = open_gobi_serial_port();
		if (fd < 0) {
			printf("Error: Open Gobi Serial Port Fail.\n");
// if Gobi is powered off after calling check_gobi_power, it could lead open gobi serial port to be an error
// So it should return but not exit.
			return;
//			exit(-1);
		} else {
			ttyUSB1_fd = open(TTYUSB1_OPEN, O_CREAT);
			if (ttyUSB1_fd < 0) {
				debuglog(LOG_INFO, "Create ttyUSB1_open Error %s.\n", strerror(errno));
				printf("Create ttyUSB1_open Error.\n");
				close(fd);
				return; 
			} 
			close(ttyUSB1_fd);
			
			ATcmd_ret = ATcmd_operate(fd, "AT$QCTEMP\r", atcmd_msg);
			close(fd);
			unlink(TTYUSB1_OPEN);

			if(ATcmd_ret) {
				pOK=strstr(atcmd_msg, "OK");
				while ((atcmd_msg + i) < pOK) {
					if(*(atcmd_msg + i) >= '0' && *(atcmd_msg + i) <= '9') {	
						*(temp_string + j) = *(atcmd_msg + i);
						j++;
					}
					i++;
				}
				*(temp_string + j) = '\0';
				cur_temp = atoi(temp_string);
				printf("Current Temperature %d\n", cur_temp);
				if (*pre_temp == CELL_TEMP_INIT) { 					
					sprintf(string, "Current Temperature: %d.", cur_temp);
					diagnostic_report(DIAGNOSTIC_MESSAGE, 0, string);
					*pre_temp = cur_temp;
				} else {
					if (cur_temp >= cell_temp_high || cur_temp <= cell_temp_low) {
						if (cur_temp != *pre_temp) {
							sprintf(string, "WARNING: Currnet Temperature: %d.", cur_temp);
							diagnostic_report(DIAGNOSTIC_MESSAGE, 0, string);
							*pre_temp = cur_temp;
						}
					}
				}
			}
		}  
	}

	return;
}

