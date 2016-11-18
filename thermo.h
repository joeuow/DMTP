#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <debuglog.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>
#include <termios.h>

/* supported PID number */
#define REQUEST_PARAMETER 			0
#define FUEL_LEVEL 					96
#define BATTERY_VOLTAGE 			168
#define AMBIENT_AIR_TEMPERATURE 	171
#define CAR_TEMPERATURE_ZONE_1	200
#define CAR_TEMPERATURE_ZONE_2	201
#define CAR_TEMPERATURE_ZONE_3	202
#define CARGOWATCH_SENSOR_READ	203
#define POWER_CONTROL				205
#define UNIT_CONTROL_CAPABILITY	206
#define MULTI_ALARM_READ_CAPABILITY 207
#define EXTEN_PARA_ID_CAPABILITY	208
#define SOFTWARE_ID				234
#define TOTAL_ELEC_HOURS			235
#define COMPONENT_ID_PARA			243
#define VEHICLE_HOURS				246
#define ENGINE_HOURS				247

// for PID 205
#define POWER_OFF_THK			0x0
int ibox_debug = 0;

int init_ibox(const char *port, int mid);
int shut_ibox(int fd);
int request_pid(int serial_fd, int pid, char *buff, int timeout, char *cmd, int cmd_leng);
int config_ibox(int serial_fd, int pid, char *buff);
void print_iBox_err(int error);
void print_resp( char *resp, int readn);