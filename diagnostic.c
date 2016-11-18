#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <ctype.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <errno.h>
#include <netinet/in.h>
#include <netinet/ip_icmp.h>
#include <fcntl.h>
#include <debuglog.h>
#include <errno.h>
#include "log.h"
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
#include "diagnostic.h"
#include "defaults.h"
#include "packet.h"

#define MSG_MAX_SIZE 90


extern uint32_t volatile wireless_stuck;

void diagnostic_report(int diagnostic_status, int arg1, char *arg2){
	Packet_t up_pkt;
	time_t now;
	size_t len;
	char msg_description[MSG_MAX_SIZE];
	UInt16	status = STATUS_CLIENT_DIAGNOSTIC_MSG;
	UInt8	seq = 0;
	UInt32 diag_level = propGetUInt32(PROP_STATE_DIAGNOSTIC_LEVEL, 1);

	if(diag_level == 1) {
//		msg_description = (char *)malloc(MSG_MAX_SIZE*sizeof(char));
		switch(diagnostic_status) {
		case DIAGNOSTIC_CNNCT_DOWN:
		{
//			int rebuild_mins = arg;
//			sprintf(msg_description, "Connection DOWN & Re-build in %d minutes", rebuild_mins/60);
			if (arg1)
				sprintf(msg_description, "Connection DOWN");
			else if (!arg1)
				sprintf(msg_description, "Connection ALIVE");
			break;
		}
		case DIAGNOSTIC_CNNCT_REBUILT:
		{
			if (arg1)
				sprintf(msg_description, "Connection REBUILT");
			else if (!arg1)
				sprintf(msg_description, "TRY REBUILD Connection ");
			break;
		}
		case DIAGNOSTIC_GPS_LOST:
			{
			if (arg1)
				sprintf(msg_description, "GPS signal LOST");
			else if (!arg1)
				sprintf(msg_description, "GPS signal BACK");
			break;
			}
		case DIAGNOSTIC_CLIENT_REBOOT:
		{
			UInt32 down_hours;
			if(arg1 == REBOOT_LIBRARY_STUCK)
				sprintf(msg_description, "Reboot ALERT due to stuck in library");
			else if (arg1 == REBOOT_DOWN_TOO_LONG) {
				down_hours = propGetUInt32(PROP_COMM_MAX_DELAY, 0);
				sprintf(msg_description, "Reboot ALERT due to connection DOWN for %d hours", down_hours);
			}
			break;
		}
		case DIAGNOSTIC_CNNCT_CHECK:
			sprintf(msg_description, "Check Network status");
			break;
		case DIAGNOSTIC_LIB_STUCK:
			if (arg1)
				sprintf(msg_description, "Wireless Stuck set");
			else if (!arg1)
				sprintf(msg_description, "Wireless Stuck cleared");
			break;
		case DIAGNOSTIC_DHCP:
			{
//			char *string = arg2;
			sprintf(msg_description, "%s\n", arg2);
			debuglog(LOG_INFO, "[ositechdmtp] %s\n", arg2);
			break;
			}
		case DIAGNOSTIC_MESSAGE:
			{
//			char *string = (char *)arg;
			sprintf(msg_description, "%s\n", arg2);
			debuglog(LOG_INFO, "[ositechdmtp] %s\n", arg2);
			break;
			}
		case DIAGNOSTIC_CELL_DOWN:
			{
			if (arg1)
				sprintf(msg_description, "Cellular Connection DOWN");
			else if (!arg1)
				sprintf(msg_description, "Cellular Connection ALIVE");
			break;
			}
		default:
			printf("the diagnostic_status: %d is ERROR\n", diagnostic_status);
			return;
		}

		len = strlen(msg_description);
	
		now = time(&now);
		pktInit(&up_pkt, PKT_CLIENT_DMTSP_FORMAT_3, "%2U%4U%*s%1U", status, (UInt32)now, 
									len, msg_description, seq); 
		up_pkt.seqPos = 6 + len;
		up_pkt.seqLen = 1;
		evAddEncodedPacket(&up_pkt);
	}
	
	return;
}
