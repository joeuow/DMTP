#include "thermo.h"
#include "thermo_error.h"

#define MSG_SIZE 		256
#define REQUEST_SIZE 	4
#define RESP_SIZE 		140
#define MINIMUN_MSG_LENG 4

static int ibox_mid;

void print_resp(char *resp, int readn) {
	int index = 0;
	
	printf("iBox Msg: ");
	while(index < readn) printf("%.2x ", *(resp+index++));
	printf("\n");
}

static int open_serial_port(const char *port) {
	int fd;

/*	O_NOCTTY flag tells UNIX that this program doesn't want to be the "controlling terminal" for that port. */
/*	O_NDELAY flag tells UNIX that this program doesn't care what state the DCD signal line is in - 		*/

//	if((fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY )) < 0) { 
	if((fd = open(port, O_RDWR| O_NOCTTY)) < 0) { 
		printf("Error: Cannot Open device %s (%d)\n", port, errno);
		return -1;
	} else
		return fd;
}

/* Sets up a serial port for RTU communications */
static int configure_serial_port(int serial_fd) {

    	struct termios tios;
    	speed_t speed;

    	memset(&tios, 0, sizeof(struct termios));

/* The baud rate is set to be 9600 */
        speed = B9600;
    	if ((cfsetispeed(&tios, speed) < 0) ||
        	(cfsetospeed(&tios, speed) < 0)) {
        	printf("Error: Cannot set baud rate to be 9600 (%d)\n", errno);
		close(serial_fd);
        	return -1;
	}

	/* 	C_CFLAG      Control options
       	CLOCAL       Local line - do not change "owner" of port
       	CREAD        Enable receiver
	*/
    	tios.c_cflag |= (CREAD | CLOCAL);

	/* No parity (8N1) */
	tios.c_cflag &= ~PARENB;
	tios.c_cflag &= ~CSTOPB;
	tios.c_cflag &= ~CSIZE;
	tios.c_cflag |= CS8;

    	/* C_LFLAG      Line options

       ISIG Enable SIGINTR, SIGSUSP, SIGDSUSP, and SIGQUIT signals
       ICANON       Enable canonical input (else raw)
       XCASE        Map uppercase \lowercase (obsolete)
       ECHO Enable echoing of input characters
       ECHOE        Echo erase character as BS-SP-BS
       ECHOK        Echo NL after kill character
       ECHONL       Echo NL
       NOFLSH       Disable flushing of input buffers after
       interrupt or quit characters
       IEXTEN       Enable extended functions
       ECHOCTL      Echo control characters as ^char and delete as ~?
       ECHOPRT      Echo erased character as character erased
       ECHOKE       BS-SP-BS entire line on line kill
       FLUSHO       Output being flushed
       PENDIN       Retype pending input at next read or input char
       TOSTOP       Send SIGTTOU for background output

       Canonical input is line-oriented. Input characters are put
       into a buffer which can be edited interactively by the user
       until a CR (carriage return) or LF (line feed) character is
       received.

       Raw input is unprocessed. Input characters are passed
       through exactly as they are received, when they are
       received. Generally you'll deselect the ICANON, ECHO,
       ECHOE, and ISIG options when using raw input
    	*/

    	/* Raw input */
    	tios.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

   	 /* C_IFLAG      Input options */
        /* No Parity check*/
        tios.c_iflag &= ~INPCK;


    	/* Software flow control is disabled */
   	 tios.c_iflag &= ~(IXON | IXOFF | IXANY);

    	/* C_OFLAG      Output options
	       OPOST        Postprocess output (not set = raw output)
       	ONLCR        Map NL to CR-NL
       	ONCLR ant others needs OPOST to be enabled
    	*/

    	/* Raw ouput */
    	tios.c_oflag &=~ OPOST;

	/* C_CC         Control characters
       	VMIN         Minimum number of characters to read
       	VTIME        Time to wait for data (tenths of seconds)
       Unused because we use open with the NDELAY option */
    	tios.c_cc[VMIN] = MINIMUN_MSG_LENG;
    	tios.c_cc[VTIME] = 1;

    	if (tcsetattr(serial_fd, TCSANOW, &tios) < 0) {
       	close(serial_fd);
		printf("Error: Set serial port\n");
       	serial_fd = -1;
        	return -1;
    }

    return 0;
}

static int verify_pid(int pid) {
	if (pid == REQUEST_PARAMETER
		|| pid == FUEL_LEVEL
		|| pid == BATTERY_VOLTAGE
		|| pid == AMBIENT_AIR_TEMPERATURE
		|| pid == CAR_TEMPERATURE_ZONE_1
		|| pid == CAR_TEMPERATURE_ZONE_2
		|| pid == CAR_TEMPERATURE_ZONE_3
		|| pid == CARGOWATCH_SENSOR_READ
		|| pid == POWER_CONTROL
		|| pid == UNIT_CONTROL_CAPABILITY
		|| pid == MULTI_ALARM_READ_CAPABILITY
		|| pid == EXTEN_PARA_ID_CAPABILITY
		|| pid == SOFTWARE_ID
		|| pid == TOTAL_ELEC_HOURS
		|| pid == COMPONENT_ID_PARA
		|| pid == VEHICLE_HOURS
		|| pid == ENGINE_HOURS)
			return 0;
	else
		return INVALID_PID;
}

static void add_checksum(char *msg) {
	char checksum;
	char sum;

	/* cal checksum */
	sum = *(msg+0) + *(msg+1);
	sum	+= *(msg+2);
	checksum = 0xff-sum+1;

	*(msg+3) = checksum;
}

/* the sum of all the characters, neglecting the CARRY, should be 0 */ 
/* 	
	corret returns 1;
	incorrect returns 0;
*/
static int verify_checksum(char *msg, int msg_leng) {
	int index = 0;
	char sum = 0x0;

	while(index < msg_leng)	{
		if (ibox_debug && index > 0)
			printf("=>");
		sum += *(msg+index++);
		if (ibox_debug)
			printf("%.2x", sum);
	}
	if (ibox_debug)
		printf("\n");
	
	if (!sum)
		return 1;
	else
		return 0;
}

static int find_MID(char *msg) {
	return (*msg == ibox_mid);
}
static int verify_PID(char *msg, int pid) {
	return (*(msg + 1) == pid);
}

/* 	1. Find the MID first, which is the START of the msg 
	2. and then the following byte should be the PID
	3. if they are both correct, the last byte should be checksum
	4. if the checksum is correct, then the msg is correct.
	5. if msg is correct, the index of the MID in the msg would be returned.
if Any error occured during the above verification process, it could indicate that 
the receiving msg is incorrect and it will be discarded. So a -1 will be returned.
*/
static int verify_msg(char *rec_msg, int *msg_leng, int pid) {
	int index = 0;
	int MID_found = 0, PID_found = 0;

/* 
	the minimum length for a msg should be 4.
	thus, when msg less than 4 bytes, it would be invalid
*/
	while(*msg_leng - index >= MINIMUN_MSG_LENG) {
		MID_found = find_MID(rec_msg+index);
		if(!MID_found) {
			printf("ERROR: Cannot find MID\n");
			index++;
			continue;
		} else {
			PID_found = verify_PID(rec_msg+index, pid);
			if (!PID_found) {
				printf("ERROR: PID incorrect. Keep searching\n");
				index++;
				continue;
			} else
				break;
		}
	}
	*msg_leng -= index;
	if (ibox_debug) {
		printf("%s: ", __FUNCTION__);
		print_resp(rec_msg+index, *msg_leng);
	}
	
	if(verify_checksum(rec_msg+index, *msg_leng))
		return index;
	else 
		return -1;
}

static void fill_req_msg(int pid, char *msg, char *cmd, int cmd_leng) {
	int index = 0;
	
	msg[0] = ibox_mid;
	msg[1] = 0;
	msg[2] = (char) pid;
	while(index < cmd_leng) {
		*(msg+3+index) = *(cmd+index);
		index++;
	}
	add_checksum(msg);
}
	
/*********************************************************************** 
 
Description:  
	The API to initialize the serial port of communicating with the ibox. It including the 		 
	process of open, configuring and etc. 	 

Calling Arguments: 
 	Name    	       Mode      Description 
 	port			Input	The serial port of the ibox  
 
Return Value: 
	Type      		Description 
 	int			The fd of the open ibox

Exception:
 	If the open of the ibox serial port is failed, -1 should be returned to indicate it.
******************************************************************************/
int init_ibox(const char *port, int mid) {
	int serial_fd;

	serial_fd = open_serial_port(port);
	if (serial_fd>= 0) {
		printf("Open Serial Port Successed\n");
		if (configure_serial_port(serial_fd) < 0) {
			if (ibox_debug)
				printf("Configure Serial Port Failed\n");
			shutdown_ibox(serial_fd);
			return -1;
		}
		printf("Init Serial Port Successed\n");
	}
	ibox_mid = mid;

	return serial_fd;
}


/*********************************************************************** 
 
Description:  
	The API to send the request to the ibox 	 

Calling Arguments: 
 	Name    	       Mode      Description 
 	serial_fd		Input	The fd of the serial port of the ibox  
 	pid			Input	The requested PID number
 	cmd 		Input	The command sent to configure the iBox. it could be NULL
Return Value: 
	Type      		Description 
 	int			bytes write to the iBox

Exception:
 	If the response sending of the ibox serial port is failed, a negative should be returned
 	to indicate the error.
******************************************************************************/
static int send_request(int serial_fd, int pid, char *cmd, int cmd_leng) {
	int writen;
	char req_msg[REQUEST_SIZE];
	int wfd;
	int index = 0;
	fd_set  wset;
	struct timeval timeout;

	FD_ZERO(&wset);
	FD_SET(serial_fd, &wset);
	timeout.tv_sec = 1;
	timeout.tv_usec = 0;
	
	memset(req_msg, 0, REQUEST_SIZE*sizeof(char));
	fill_req_msg(pid, req_msg, cmd, cmd_leng);

	while(index < cmd_leng+4) printf("%.2x ", *(req_msg+index++));
	
	wfd = select(FD_SETSIZE,  NULL, &wset, NULL, &timeout);
	if (wfd < 0) {
		perror("send_request: select");
		exit(-1);
	} else if (wfd > 0) {
		writen = write(serial_fd, req_msg, REQUEST_SIZE*sizeof(char));
	} else {
		printf("Write to iBox Port Timeout\n");
		return 0;
	}
	
	if (writen == REQUEST_SIZE)
		return writen;
	else {
		printf("ERR: Only %d bytes being written\n", writen);
		return WRITE_ERR;
	}
}

/*********************************************************************** 
 
Description:  
	The API to read the response from the ibox 	 

Calling Arguments: 
 	Name    	       Mode      Description 
 	serial_fd		Input	The fd of the serial port of the ibox  
 	buff			Output	The buffer contains the reading response of the latest sending request from the ibox  
Return Value: 
	Type      		Description 
 	int			the length of the response

Exception:
 	If the response reading of the ibox serial port is failed, a negative should be returned
 	to indicate the error.
******************************************************************************/
static int read_response(int serial_fd, char *ret_buff, int timeout) {
	fd_set  rset;
	struct timeval tv;
	int readn = 0;
	int rfd;
	char resp[RESP_SIZE];

	memset(resp, 0 , RESP_SIZE*sizeof(char));
	
	tv.tv_sec = timeout;
   	tv.tv_usec = 0;

   	//always need to zero it first, then add our new descripto
   	FD_ZERO(&rset);
   	FD_SET(serial_fd, &rset);

   	//wait specified time for input to arrive
   	rfd = select(FD_SETSIZE, &rset, NULL, NULL, &tv);
	if (rfd < 0) {
		perror("read_response: select\n");
		exit(-1);
	} else if (rfd > 0) {
      		if(FD_ISSET(serial_fd, &rset)) {
            		readn = read(serial_fd, resp, RESP_SIZE);
			if (readn > 0) {
				if (ibox_debug)
					print_resp(resp, readn);	
				memcpy(ret_buff, resp, readn);
			}
      		} 
	} else {
          	printf("Read From iBox Port Timeout\n");
		return NO_DATA;
	}
      	 
	
	
	if (ibox_debug)
		printf("Read %d bytes in total\n", readn);
	return readn;
}

/*********************************************************************** 
Description:  
	The API to require the PID request from the ibox 	 

Calling Arguments: 
 	Name    	       Mode      Description 
 	serial_fd		Input	The fd of the serial port of the ibox 
 	pid			Input	The request PID
 	resp_size		Output	The buffer size of the responding message
 	args			Input	The arguments sent to the iBox if necessary
 
Return Value: 
	Type      		Description 
 	int			request result

Exception:
 	If the PID request to the ibox is failed, a negative value should be
 	returned to indicate the error.
******************************************************************************/
int request_pid(int serial_fd, int pid, char *buff, int timeout, char *cmd, int cmd_leng) {
	int ret;
	int index = 0;
	char ret_msg[MSG_SIZE];

	if(verify_pid(pid)) {
		printf("Valid PID\n");
		return INVALID_PID;
	}

	ret = send_request(serial_fd, pid, cmd, cmd_leng);
	if (ret < 0) {
		printf("ERR: Send request to the iBox. (%d)\n", ret);
		return WRITE_ERR;
	}	
	memset(ret_msg, 0, MSG_SIZE*sizeof(char));
	
	ret = read_response(serial_fd, ret_msg, timeout);
	if (ret < 0) {
		printf("ERR: Send request to the iBox. (%d)\n", ret);
		return READ_ERR;
	}
	
	if((index = verify_msg(ret_msg, &ret, pid)) < 0) {
		return VERIFY_ERR;
	}
	
	/* just pass the PID and data, exclude MID and checksum */
	memcpy(buff, ret_msg+index+1, ret-2); 
	if (ibox_debug)
		print_resp(buff, ret);
	
	return ret-2;
}

void shutdown_ibox(int serial_fd) {
	close(serial_fd);
}

void print_iBox_err(int error) {
	if ( error == INVALID_MID )
		printf("Invalid MID\n");
	else if (error == INVALID_PID)
		printf("Invalid PID\n");
	else if (error == INVALID_PAR)
		printf("Invalid Parameter\n");
	else if (error == WRITE_ERR)
		printf("Write Error\n");
	else if (error == READ_ERR)
		printf("READ Error\n");
}