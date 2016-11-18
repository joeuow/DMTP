#define WIFI_ESTABLISHING 0
#define WIFI_STOP 1
#define WIFI_EXISTING 2
#define WIFI_AP_STATUS 3

#define WIFI_RET_ESTABLISHED 0
#define WIFI_RET_NONETCOVERAGE 1
#define WIFI_RET_NOIPCONNECTION 12
#define WIFI_RET_NONEXISTING 5
#define WIFI_RET_EXISTING    6
#define WIFI_RET_DOWNERROR   7
#define WIFI_RET_DOWNOK      8 
#define WIFI_RET_LIVE        9
#define WIFI_RET_AP_OK        10
#define WIFI_RET_AP_NONE        11

int libwlcomm_main(int state, char *server);
void add_cell_log_function(int (*func1)(const char *format, va_list ap));
void get_cell_log_name(char *log_name, size_t log_name_size);
void ppp_debug_func(int (*new_func)(const char *format, va_list ap));
int check_ip_connection1(char *pIP);
