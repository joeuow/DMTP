#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include "thermo.h"
#include "thermo_error.h"
#include "props.h"
#include "propman.h"
#include "diagnostic.h"
#include "statcode.h"
#include "events.h"

#define IBOX_DEVICE	"/dev/ttyS1"
#define MSG_LENGTH 256
#define MSG_MAX_SIZE 90

#define IBOX_POWER_OFF 0
#define IBOX_POWER_ON 1

typedef struct __ibox_pid {
	int pid;
	int rate;
	int timeout;
	char *raw_packet;
	time_t last_sample;
	struct __ibox_pid *pnext;
} ibox_pid;

pthread_t iBoxMonitor_thread_main;

void * thread_iBoxMonitor_main(void *args);

static ibox_pid *init_node(int req_pid, uint32_t samp_rate, uint32_t timeout) {
	ibox_pid *tmp = NULL;

	tmp = (ibox_pid *)malloc(sizeof(ibox_pid));
	if(tmp == NULL) {
		printf("ERROR: %s malloc ibox_pid\n", __FUNCTION__);
		exit(-1);
	}
	tmp->pid = req_pid;
	tmp->rate = samp_rate;
	tmp->last_sample = 0;
	tmp->timeout = timeout;
	tmp->raw_packet = (char *)malloc(MSG_LENGTH*sizeof(char));
	if(tmp->raw_packet == NULL) {
		printf("Error: %s (%d) malloc\n", __FUNCTION__, __LINE__);
		exit(-1);
	}
	tmp->pnext = NULL;

	return tmp;
}
static void link_add_node(ibox_pid **head, ibox_pid *node) {
	ibox_pid *tmp = *head;
	
	if(tmp == NULL)
		*head = node;
	else {
		while(tmp->pnext) tmp = tmp->pnext;
		tmp->pnext = node;
	}
}
static void link_del_node(ibox_pid **head, int pid_num) {
	ibox_pid *tmp = *head;
	ibox_pid *bef = tmp;
	
	while(tmp) {
		if (tmp->pid == pid_num) {
			if (tmp == *head)
				*head = tmp->pnext;
			else
				bef->pnext = tmp->pnext;
			free(tmp);
			tmp = NULL;
		} else {
			bef = tmp;
			tmp = tmp->pnext;
		}
	}
}
/* 
	if node found in the link, return 1
	if node not found in the link, return 0
*/	
static int find_node(ibox_pid *head, int req_pid) {
	ibox_pid *tmp = head;
	
	while(tmp) {
		if (tmp->pid == req_pid)
			return 1;
		else 
			tmp = tmp->pnext;
	}

	return 0;
}

static void hex_to_asc(char *dest, char *src, int src_size) {
	int index = 0;
	char req_pid = src[0];

	/* if src_size < 0, use a "?" append to the pid to indicates the error of iBox reading */
	if (src_size < 0)
		sprintf(dest, "%.3d?", req_pid);
	else {
		sprintf(dest, "%.3d,", req_pid);
		while(index < src_size-1) {
			sprintf((dest+4)+index*2, "%.2x", *(src+1+index));
			index++;
		}
	}
}

static int asc_to_hex(char *dest, char *src) {
	int index=0;
	int leng = strlen(src);
	leng = (leng+1)/2;
	char tmp1, tmp2;

	while(index < leng) {
		tmp1=0;
		tmp2=0;
		if(src[index*2] >= '0' && src[index*2] <= '9')
			tmp1 = (src[index*2] - '0')*16;
		if(src[index*2] >= 'a' && src[index*2] <= 'f')
			tmp1 = (src[index*2] - 'a' + 10)*16;
		if(src[index*2] >= 'A' && src[index*2] <= 'F')
			tmp1 = (src[index*2] - 'A' + 10)*16;
		
		if(src[index*2+1] >= '0' && src[index*2+1] <= '9')
			tmp2 = src[index*2+1] - '0';
		if(src[index*2+1] >= 'a' && src[index*2+1] <= 'f')
			tmp2 = src[index*2+1] - 'a' + 10;
		if(src[index*2+1] >= 'A' && src[index*2+1] <= 'F')
			tmp2 = src[index*2+1] - 'A' + 10;
		
		*(dest+index) = tmp1+tmp2;
		index ++;
	}

	return leng;
}

static void iBox_status_report(char *arg){
	Packet_t up_pkt;
	time_t now;
	size_t len;
	char msg_description[MSG_MAX_SIZE];
	UInt16	status = STATUS_IBOX;
	UInt8	seq = 0;
	
	sprintf(msg_description, "%s\n", arg);
	debuglog(LOG_INFO, "[ositechdmtp] iBox response: %s\n", arg);
	printf("iBox response: %s\n", arg);
	
	len = strlen(msg_description);
	now = time(&now);
	pktInit(&up_pkt, PKT_CLIENT_DMTSP_FORMAT_3, "%2U%4U%*s%1U", status, (UInt32)now, 
									len, msg_description, seq); 
	up_pkt.seqPos = 6 + len;
	up_pkt.seqLen = 1;
	evAddEncodedPacket(&up_pkt);
	
	return;
}


/*
	find out if any PID being enabled or disabled 
	if PID is being enabled but not in the link, add it
	if PID is being disabled and existed in the link, delete it
*/
static void scan_request_pid(ibox_pid **head) {
	uint32_t pid_rate, timeout;
	ibox_pid *pid_req = NULL;
	int pid_num;
	int pid_exist;
	bool pid_valid;

	for(pid_num = FUEL_LEVEL; pid_num <= ENGINE_HOURS; pid_num++) {
		switch(pid_num) {
			case FUEL_LEVEL:
				pid_rate = propGetUInt32AtIndex(PROP_IBOX_96_REQUEST, 0, 0);
				timeout = propGetUInt32AtIndex(PROP_IBOX_96_REQUEST, 1, 1);
				pid_valid = true;
				if (pid_rate == 0xFFFF)
					propSetUInt32AtIndex(PROP_IBOX_96_REQUEST, 0, 0);
				break;
			case BATTERY_VOLTAGE:
				pid_rate = propGetUInt32AtIndex(PROP_IBOX_168_REQUEST, 0, 0);
				timeout = propGetUInt32AtIndex(PROP_IBOX_168_REQUEST, 1, 1);
				pid_valid = true;
				if (pid_rate == 0xFFFF)
					propSetUInt32AtIndex(PROP_IBOX_168_REQUEST, 0, 0);
				break;
			case AMBIENT_AIR_TEMPERATURE:
				pid_rate = propGetUInt32AtIndex(PROP_IBOX_171_REQUEST, 0, 0);
				timeout = propGetUInt32AtIndex(PROP_IBOX_171_REQUEST, 1, 1);
				pid_valid = true;
				if (pid_rate == 0xFFFF)
					propSetUInt32AtIndex(PROP_IBOX_171_REQUEST, 0, 0);
				break;
			case CAR_TEMPERATURE_ZONE_1:
				pid_rate = propGetUInt32AtIndex(PROP_IBOX_200_REQUEST, 0, 0);
				timeout = propGetUInt32AtIndex(PROP_IBOX_200_REQUEST, 1, 1);
				pid_valid = true;
				if (pid_rate == 0xFFFF)
					propSetUInt32AtIndex(PROP_IBOX_200_REQUEST, 0, 0);
				break;
			case CAR_TEMPERATURE_ZONE_2:
				pid_rate = propGetUInt32AtIndex(PROP_IBOX_201_REQUEST, 0, 0);
				timeout = propGetUInt32AtIndex(PROP_IBOX_201_REQUEST, 1, 1);
				pid_valid = true;
				if (pid_rate == 0xFFFF)
					propSetUInt32AtIndex(PROP_IBOX_201_REQUEST, 0, 0);
				break;
			case CAR_TEMPERATURE_ZONE_3:
				pid_rate = propGetUInt32AtIndex(PROP_IBOX_202_REQUEST, 0, 0);
				timeout = propGetUInt32AtIndex(PROP_IBOX_202_REQUEST, 1, 1);
				pid_valid = true;
				if (pid_rate == 0xFFFF)
					propSetUInt32AtIndex(PROP_IBOX_202_REQUEST, 0, 0);
				break;
			case CARGOWATCH_SENSOR_READ:
				pid_rate = propGetUInt32AtIndex(PROP_IBOX_203_REQUEST, 0, 0);
				timeout = propGetUInt32AtIndex(PROP_IBOX_203_REQUEST, 1, 1);
				pid_valid = true;
				if (pid_rate == 0xFFFF)
					propSetUInt32AtIndex(PROP_IBOX_203_REQUEST, 0, 0);
				break;

			case MULTI_ALARM_READ_CAPABILITY:
				pid_rate = propGetUInt32AtIndex(PROP_IBOX_207_REQUEST, 0, 0);
				timeout = propGetUInt32AtIndex(PROP_IBOX_207_REQUEST, 1, 1);
				pid_valid = true;
				if (pid_rate == 0xFFFF)
					propSetUInt32AtIndex(PROP_IBOX_207_REQUEST, 0, 0);
				break;

			case SOFTWARE_ID:
				pid_rate = propGetUInt32AtIndex(PROP_IBOX_234_REQUEST, 0, 0);
				timeout = propGetUInt32AtIndex(PROP_IBOX_234_REQUEST, 1, 1);
				pid_valid = true;
				if (pid_rate == 0xFFFF)
					propSetUInt32AtIndex(PROP_IBOX_234_REQUEST, 0, 0);
				break;
			case TOTAL_ELEC_HOURS:
				pid_rate = propGetUInt32AtIndex(PROP_IBOX_235_REQUEST, 0, 0);
				timeout = propGetUInt32AtIndex(PROP_IBOX_235_REQUEST, 1, 1);
				pid_valid = true;
				if (pid_rate == 0xFFFF)
					propSetUInt32AtIndex(PROP_IBOX_235_REQUEST, 0, 0);
				break;
			case COMPONENT_ID_PARA	:
				pid_rate = propGetUInt32AtIndex(PROP_IBOX_243_REQUEST, 0, 0);
				timeout = propGetUInt32AtIndex(PROP_IBOX_243_REQUEST, 1, 1);
				pid_valid = true;
				if (pid_rate == 0xFFFF)
					propSetUInt32AtIndex(PROP_IBOX_243_REQUEST, 0, 0);
				break;
			case VEHICLE_HOURS:
				pid_rate = propGetUInt32AtIndex(PROP_IBOX_246_REQUEST, 0, 0);
				timeout = propGetUInt32AtIndex(PROP_IBOX_246_REQUEST, 1, 1);
				pid_valid = true;
				if (pid_rate == 0xFFFF)
					propSetUInt32AtIndex(PROP_IBOX_246_REQUEST, 0, 0);
				break;
			case ENGINE_HOURS:
				pid_rate = propGetUInt32AtIndex(PROP_IBOX_247_REQUEST, 0, 0);
				timeout = propGetUInt32AtIndex(PROP_IBOX_247_REQUEST, 1, 1);
				pid_valid = true;
				if (pid_rate == 0xFFFF)
					propSetUInt32AtIndex(PROP_IBOX_247_REQUEST, 0, 0);
				break;
			default:
				pid_rate = 0;
				pid_valid = false;
				break;
		}
		if(pid_valid) {
			pid_exist = find_node(*head, pid_num);
			//printf("PID %d exist? %s\n", pid_num, pid_exist? "yes":"no");
			if (pid_rate && !pid_exist) {
				pid_req = init_node(pid_num, pid_rate, timeout);
				link_add_node(head, pid_req);
				printf("Add PID %d\n", pid_num);
			}
			if (!pid_rate && pid_exist) {
				printf("Del PID %d\n", pid_num);
				link_del_node(head, pid_num);
			}
		}
	}	
}

int iBoxMonitor_initialize(void)
{
	if (pthread_create(&iBoxMonitor_thread_main, NULL, thread_iBoxMonitor_main, NULL) != 0) {
		printf("Creating Thread Error\n");
		return -1;
	}
	
	return 0;
}

void * thread_iBoxMonitor_main(void *args) {
	bool pid_valid = false;
	int ibox_power = IBOX_POWER_OFF;
	int resp_leng;
	int fd;
	int timeout;
	int cmd_leng;
	int cmd_pid_num;
	char resp_msg[MSG_LENGTH];
	const char *cmd_msg_tmp;
	char cmd_msg[MSG_LENGTH],cmd_msg_asc[MSG_LENGTH], cmd_resp[MSG_LENGTH];
	ibox_pid *request_pid_head = NULL, *tmp;
	uint32_t ibox_mid = propGetUInt32(PROP_IBOX_MID, 147);
	const char *ibox_port = propGetString(PROP_IBOX_PORT, "");
	static int time_beats = 0;

	if ((fd = init_ibox(ibox_port, ibox_mid)) < 0) {
			printf("%s: open %s FAIL\n", __FUNCTION__, ibox_port);
			return NULL;
	} else
		ibox_power = IBOX_POWER_ON;
	
	ibox_debug = 1;
	while(1) {
		/* request pid */
		scan_request_pid(&request_pid_head);
		for(tmp = request_pid_head; tmp != NULL; tmp=tmp->pnext) {
			/* if sample rate is 0xFF, the PID will be reported only once */
			if ((time_beats - tmp->last_sample >= tmp->rate) || (tmp->rate == 0xFFFF)) {
				printf("Send PID %d request to iBox: ", tmp->pid);
				memset(resp_msg, 0, MSG_LENGTH);

				if(ibox_power == IBOX_POWER_OFF) {
					if ((fd = init_ibox(ibox_port, ibox_mid)) < 0) {
						printf("%s: open %s FAIL\n", __FUNCTION__, ibox_port);
						return NULL;
					}
					ibox_power = IBOX_POWER_ON;
				}
				
				if((resp_leng=request_pid(fd, tmp->pid,resp_msg, tmp->timeout, NULL, 0)) < 0) {
					hex_to_asc(tmp->raw_packet, (char *)&(tmp->pid), resp_leng);
					printf("PID %d request Error (%d)\n", tmp->pid, resp_leng);
				} else {
					if(ibox_debug) {
						printf("%s: ", __FUNCTION__);
						print_resp(resp_msg, resp_leng);
					} 
					hex_to_asc(tmp->raw_packet, resp_msg, resp_leng);
				}	

				iBox_status_report(tmp->raw_packet);
				tmp->last_sample = time_beats;
			}
		}
	
		/* command pid */
		for(cmd_pid_num = POWER_CONTROL;
			cmd_pid_num <= EXTEN_PARA_ID_CAPABILITY;
			cmd_pid_num++) {

			switch(cmd_pid_num) {
				case POWER_CONTROL:
					cmd_msg_tmp = propGetString(PROP_IBOX_205_COMMAND, "");
					timeout = propGetUInt32(PROP_IBOX_205_COMMAND_TIMEOUT, 1);
					pid_valid = true;
					if (*cmd_msg_tmp) {
						memcpy(cmd_msg_asc, cmd_msg_tmp, strlen(cmd_msg_tmp));
						propSetString(PROP_IBOX_205_COMMAND, "");
					}
					break;
				case UNIT_CONTROL_CAPABILITY:
					cmd_msg_tmp = propGetString(PROP_IBOX_206_COMMAND, "");
					timeout = propGetUInt32(PROP_IBOX_206_COMMAND_TIMEOUT, 1);
					pid_valid = true;
					if (*cmd_msg_tmp) {
						memcpy(cmd_msg_asc, cmd_msg_tmp, strlen(cmd_msg_tmp));
						propSetString(PROP_IBOX_206_COMMAND, "");
					}
					break;
				case EXTEN_PARA_ID_CAPABILITY:
					cmd_msg_tmp = propGetString(PROP_IBOX_208_COMMAND, "");
					timeout = propGetUInt32(PROP_IBOX_208_COMMAND_TIMEOUT, 1);
					pid_valid = true;
					if (*cmd_msg_tmp) {
						memcpy(cmd_msg_asc, cmd_msg_tmp, strlen(cmd_msg_tmp));
						propSetString(PROP_IBOX_208_COMMAND, "");
					}
					break;
				default:
					pid_valid = false;
					break;
			}
			if(pid_valid && *cmd_msg_asc) {
				printf("Receive PID %d command: %s\n ", cmd_pid_num, cmd_msg_asc);
				cmd_leng = asc_to_hex(cmd_msg, cmd_msg_asc);
				memset(resp_msg, 0, MSG_LENGTH);	

				if(ibox_power == IBOX_POWER_OFF) {
					if ((fd = init_ibox(ibox_port, ibox_mid)) < 0) {
						printf("%s: open %s FAIL\n", __FUNCTION__, ibox_port);
						return NULL;
					}
					ibox_power = IBOX_POWER_ON;
				}
				
				if((resp_leng=request_pid(fd, cmd_pid_num, resp_msg, timeout, cmd_msg, cmd_leng)) < 0) {
					printf("PID %d request Error (%d)\n", cmd_pid_num, resp_leng);
					hex_to_asc(cmd_resp, (char*)&cmd_pid_num, resp_leng);
				} else {
					if(ibox_debug) {
						printf("%s: ", __FUNCTION__);
						print_resp(resp_msg, resp_leng);
					}
					hex_to_asc(cmd_resp, resp_msg, resp_leng);
				}
				iBox_status_report(cmd_resp);
				/* power off the ibox if the TK is being powered off already */
				printf("cmd_msg %x\n", *cmd_msg);
				if (cmd_pid_num == POWER_CONTROL && *cmd_msg == POWER_OFF_THK) { 
					printf("%s(%d) Power off TK and then iBox\n", __FUNCTION__, __LINE__);
					shut_ibox(fd);
					ibox_power = IBOX_POWER_OFF;
				}
					
				memset(cmd_msg_asc, 0, MSG_LENGTH*sizeof(char));
				memset(cmd_msg, 0, MSG_LENGTH*sizeof(char));
				memset(cmd_resp, 0, MSG_LENGTH*sizeof(char));
			}
		}
		time_beats++;
		sleep(1);
	}
	
	shut_ibox(fd);
	return NULL;
}

