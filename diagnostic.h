#include <stdarg.h>
#include "stdtypes.h"

// diagnostic_status
/* Network Status */
#define DIAGNOSTIC_CNNCT_DOWN	1
#define DIAGNOSTIC_CNNCT_REBUILT	2
#define DIAGNOSTIC_CNNCT_CHECK	3
/* Other Status */
#define DIAGNOSTIC_GPS_LOST		4
#define DIAGNOSTIC_CLIENT_REBOOT	5
#define DIAGNOSTIC_LIB_STUCK		6
/* DHCP Status */
#define DIAGNOSTIC_DHCP			7
/* Cellular signal strength */
#define DIAGNOSTIC_MESSAGE		8
/* Cellular Connection */
#define DIAGNOSTIC_CELL_DOWN		9

// reboot reason
#define REBOOT_LIBRARY_STUCK		1
#define REBOOT_DOWN_TOO_LONG		2
extern void diagnostic_report(int diagnostic_status, int arg, char *arg2);

