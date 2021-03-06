#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <unistd.h>
#include <math.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/queue.h>
#include <time.h>
#include <pthread.h>
#include <signal.h>
#include <fcntl.h>
#include <limits.h>
#include "stdtypes.h"
#include "log.h"
#include "comport.h"
#include "propman.h"
#include "gpstools.h"
#include "gps.h"
#include "packet.h"
#include "protocol.h"
#include "qdac.h"
#include "reader_type.h"
#define QDAC_MAIN_PERIOD 10 
#define QDAC_IF_TIMEOUT 3600
#define ONE_HOUR 3600
//#define QDAC_FUNCTION_ENABLE 0x80000000
#define NUM_RSSI 8
#define RAW_READ_SIZE 32
#define MAX_READ_LEN 24
#define TAG_PACKET_SIZE_19 19
#define TAG_PACKET_SIZE_20 20
#define TAG_PACKET_SIZE_HALF 10
#define TAG_FRESH 1
#define TAG_SENIOR 2

#define TAG_MOTION_OPEN 0x10
#define TAG_STATE_BURST 0xC0
#define MAX_TEMPERATURE_RECORDS 8
#define STATUS_QDAC_INIT_FAIL				0xFA00
//#define STATUS_QDAC_INIT_SUCCESS 		0xFA01
// Temperly set it to be the same as the GPS Initialization event.
#define STATUS_QDAC_INIT_SUCCESS 			0xF010
//Temperly set them to be the same as the protrac events
#define STATUS_QDAC_TAG_IN 				0xF500
#define STATUS_QDAC_TAG_OUT 				0xF501
#define STATUS_QDAC_BATTERY_ALERT 		0xF503
#define STATUS_QDAC_PRIMARY_IN 			0xF504
#define STATUS_QDAC_PRIMARY_OUT 			0xF505
#define STATUS_QDAC_TEMPERATURE 			0xF7F1
#define STATUS_QDAC_TEMPERATURE_HIGH	0xF730
#define STATUS_QDAC_TEMPERATURE_LOW		0xF731

#define NUM_TAGS_PER_POOL 128

#define RESPONSE_MAX_SIZE 					259	// QDAC response data packet length
#define RESPONSE_MIN_SIZE 					5	// QDAC response data packet length
#define PKTS_QUEUE_LENGTH					RESPONSE_MAX_SIZE*6
//#define PKTS_QUEUE_LENGTH					130
#define QDAC_VERSION_LENGTH 				64
#define RESP_HEAD_END_TOTAL				5
#define READ_HUB_VERSION_RESP_LENGTH		9+RESP_HEAD_END_TOTAL
#define HUB_MODE_SET_RESP_LENGTH			0+RESP_HEAD_END_TOTAL
#define STREAM_MODE_RESP_LENGTH			1+RESP_HEAD_END_TOTAL
/* QDAC Primary Tach */
#define DETACHED 	0
#define LATCHED		1
/* QDAC power */
#define POWER_ON 	1
#define POWER_OFF	0
/* QDAC hub command */
#define GET_HUB_VERSION			0
#define WRITE_HUB_CONFIGUATION 	2
#define READ_TAG_DATA				10
#define ENABLE_HUB_STREAM			11
/* QDAC hub mode */
#define NORMAL_MODE 			0
#define SNOOP_MODE				1
#define PROMISCUOUS_MODE 		2
#define PROMISCUOUS_MODE_DLM	3
/* QDAC Error Code */
#define EDATA		0
#define EADDRESS	1
#define ENONE		2
#define EFUNCTION	3
#define ECRC			4

/* QDAC tag type */
//#define UNKNOWN_TAG 0
#define LOW_TEMP_TAG 			1
#define HIGH_TEMP_TAG 			2
#define GFORCE_SENSOR_TAG 		3
#define CURRENT_SENSOR_TAG 	4
#define SWITCH_TAG				5
#define PRESSURE_TAG			6
#define KNOWN_TAG_TYPE 		PRESSURE_TAG

/* QDAC packet */
#define QDAC_DATA_MAX_LENGTH 255-23
#define TIMER_LENGTH 8
/* QDAC zone */
#define QDAC_NUM_ZONES 8
#define QDAC_NUM_TEMPERATURES 10
#define QDAC_ALARM_NORM 	0
#define QDAC_ALARM_LOW	1
#define QDAC_ALARM_HIGH 	2
/* QDAC sending packet protocol */
#define QDAC_PROTOCOL		1
#define PROTRAC_PROTOCOL	2
/* QDAC Temperary Tag */
#define TMP_TAG_NUMBER 2
/* QDAC Verify PKT Status */
#define VERIFY_SUCCESS	0
/* QDAC Debug Method */
#define DEBUG_PORT	1


#define	MINUTES 60
#define	GPS_TRAIL_LENGTH 32 
#define	RSSI_BITS 5 
#define	RSSI_TOTAL (1 << RSSI_BITS)
#define	RSSI_INDEX_M (RSSI_TOTAL - 1)
#define	RSSI_WORDS  (RSSI_TOTAL >> 2)
#define 	QDAC_PW_FILE "/sys/aatsi_devs_ctrl/qdac"
#define 	QDAC_COM_DIR                "/dev/"     


#define READ_ERR_TYPE 5
struct QDAC_Tag {
	TAILQ_ENTRY(QDAC_Tag) link;
	uint32_t status;
	uint32_t qdac_tag_serial;
	uint16_t qdac_tag_model;
	uint16_t qdac_tag_id;
	uint16_t qdac_hub_id;
	uint32_t qdac_tag_contact;
	time_t recent;
	long qdac_nsec;
	uint16_t qdac_tag_beacon_period;
	uint16_t qdac_tag_rev_period;
	uint16_t qdac_tag_type;
	uint8_t qdac_rssi;
	uint8_t qdac_tag_battery;
	uint8_t qdac_flag;
//	short qdac_temp[QDAC_NUM_TEMPERATURES];
	short cur_temp;
	uint16_t qdac_gforce;
	uint8_t qdac_switch;
	unsigned long qdac_current_4_3;
	unsigned long qdac_current_2_1;
	long	qdac_pressure;
	int raw_data_length;
	unsigned char *raw_data;
};

static TAILQ_HEAD(tq_head, QDAC_Tag) tag_queue_1;

struct qdac_tag_control {
//	uint16_t qdac_tag_type;
	uint32_t primary_id;
	pthread_mutex_t mutex_hub_ctrl;
	pthread_mutex_t mutex_qdac_lowtemp;
	pthread_mutex_t mutex_qdac_hightemp;
	pthread_mutex_t mutex_qdac_unknown;
	pthread_mutex_t mutex_qdac_gforcesensor;
	pthread_mutex_t mutex_qdac_currentsensor;
	pthread_mutex_t mutex_qdac_switchsensor;
	pthread_mutex_t mutex_qdac_pressuresensor;
	pthread_mutex_t mutex_qdac_tmp;
	pthread_mutex_t mutex_recycle;
	pthread_mutex_t mutex_qdac_pkts;
	struct tq_head qdac_lowtemp_queue;
	struct tq_head recycle_queue;
	struct tq_head qdac_hightemp_queue;	
	struct tq_head qdac_gforcesensor_queue;
	struct tq_head qdac_currentsensor_queue;
	struct tq_head qdac_switchsensor_queue;
	struct tq_head qdac_pressuresensor_queue;
	struct tq_head qdac_unknown_queue;
	struct tq_head qdac_tmp_queue;
//	uint16_t customer_id;
};

struct qdac_thread_args {
	int tty_fd;
	struct qdac_tag_control * tag_c;
};

struct qdac_temp_zone_tag {
	uint32_t qdac_serial;
	uint16_t qdac_tag_type;
	uint16_t zone_id;
	short qdac_zone_temp_max;
	short qdac_zone_temp_min;
	struct qdac_temp_spec *zone_own_spec;
//	struct qdac_temp_zone *qdac_zone_general;
};

struct qdac_temp_zone {
	uint32_t primary_id;
	uint16_t primary_tag_type;
	uint32_t report_cycle;
//	uint32_t alarm_summary;
//	uint32_t alarm_count;
	uint32_t alarm_cycle;
	uint32_t alarm_delay;
	uint32_t alarm_times;
	struct qdac_temp_zone_tag qdac_zone_tag[QDAC_NUM_ZONES];
	struct qdac_lowtemp_tag *low_temp_tag;
	struct qdac_hightemp_tag *high_temp_tag;
}; 

struct qdac_temp_spec {
	uint32_t qdac_tag_serial;
	uint16_t zone_id;
	uint16_t qdac_tag_model;
	uint16_t qdac_tag_type;
	uint16_t temp_index;
	uint16_t temp_count;
	uint16_t alarm_type;
	short set_high;
	short set_low;
	short cur_high;
	short cur_low;
	short cur_avg;
	short *qdac_temp;
//	struct qdac_temp_zone *qdac_zone_general;
	struct qdac_temp_spec *qdac_temp_spec_next;
};

struct qdac_lowtemp_tag {
	int tag_in_time;
	int tag_out_time;
	uint32_t beacon_cycle;
	uint32_t report_cycle;
	struct qdac_temp_spec *qdac_lowtemp_tag_spec;
};

struct qdac_hightemp_tag {
	int tag_in_time;
	int tag_out_time;
	uint32_t beacon_cycle;
	uint32_t report_cycle;
	struct qdac_temp_spec *qdac_hightemp_tag_spec;
};


struct qdac_gforcesensor_tag {
	int tag_in_time;
	int tag_out_time;
//	uint32_t min_rssi;
	uint32_t beacon_cycle;
	uint32_t report_threshold;
	uint32_t primary_id;
};

struct qdac_currentsensor_tag {
	int tag_in_time;
	int tag_out_time;
//	uint32_t min_rssi;
	uint32_t beacon_cycle;
	uint32_t report_cycle;
};

struct qdac_switch_tag {
	int tag_in_time;
	int tag_out_time;
//	uint32_t min_rssi;
	uint32_t beacon_cycle;
	uint32_t report_cycle;
};

struct qdac_pressure_tag {
	int tag_in_time;
	int tag_out_time;
//	uint32_t min_rssi;
	uint32_t beacon_cycle;
	uint32_t report_cycle;
};

struct qdac_unknown_tag {
	int tag_in_time;
	int tag_out_time;
//	uint32_t min_rssi;
	uint32_t beacon_cycle;
	uint32_t report_cycle;
};

struct qdac_tmp_tag {	
	int tag_in_time;
	int tag_out_time;
//	uint32_t min_rssi;
	uint32_t beacon_cycle;
	uint32_t report_cycle;
	uint32_t tmp_tag_type1;
	uint32_t tmp_tag_type2;
};
struct qdac_recv_pkts {
	unsigned char *qdac_pkts_queue;
	int read_index;				// point to the begin of the next read pkt	
	int parse_index;				// point to the begin of the next parsed pkt
	uint32_t queue_unparse_length;
};

struct qdac_hightemp_tag qdac_high_temp;
struct qdac_lowtemp_tag qdac_low_temp;
struct qdac_gforcesensor_tag qdac_gforce_sensor;
struct qdac_currentsensor_tag qdac_current_sensor;
struct qdac_switch_tag qdac_switch_sensor;
struct qdac_pressure_tag qdac_pressure_sensor;
struct qdac_unknown_tag qdac_unknown;
struct qdac_temp_zone qdac_zone;
struct qdac_tmp_tag qdac_tmp;
//struct qdac_tmp_tag qdac_tmp;

struct qdac_recv_pkts qdac_pkts;
/* claim the qdac relate variables */
static struct qdac_tag_control qdac_recv_tag = {
//			.qdac_reader_type = 0, 
			.mutex_qdac_lowtemp = PTHREAD_MUTEX_INITIALIZER, 
			.mutex_qdac_hightemp = PTHREAD_MUTEX_INITIALIZER, 
			.mutex_qdac_unknown = PTHREAD_MUTEX_INITIALIZER,
			.mutex_qdac_tmp = PTHREAD_MUTEX_INITIALIZER,
			.mutex_qdac_gforcesensor = PTHREAD_MUTEX_INITIALIZER,
			.mutex_qdac_currentsensor = PTHREAD_MUTEX_INITIALIZER,
			.mutex_qdac_switchsensor = PTHREAD_MUTEX_INITIALIZER,
			.mutex_qdac_pressuresensor = PTHREAD_MUTEX_INITIALIZER,
			.mutex_recycle = PTHREAD_MUTEX_INITIALIZER,
			.mutex_qdac_pkts = PTHREAD_MUTEX_INITIALIZER,
			.mutex_hub_ctrl = PTHREAD_MUTEX_INITIALIZER};
			
static struct qdac_thread_args qdac_args =  {0, &qdac_recv_tag};
static struct QDAC_Tag * qdac_tag_pool1;
static struct QDAC_Tag * qdac_tag_pool2;
static int qdac_reader_running = 0;
static int qdac_main_running = 0;
static bool qdac_hub_reset = false;
int qdac_debug = 1;
//int qdac_debug = 0;
int qdac_power = POWER_ON;			// at init, qdac in on
//short protocol = PROTRAC_PROTOCOL;
short protocol = QDAC_PROTOCOL;
//short qdac_test = DEBUG_PORT;
short qdac_test = 0;
static ComPort_t QDACCom = {.name = "ttyS4", .bps = 115200, .read_len = TAG_PACKET_SIZE_19,};

static unsigned char command[260];
pthread_t thread_qdac_main;
pthread_t thread_qdac_reader;

//static struct temp_record temp_sorted[NUM_ZONES];
/* qdac GPS variable */
static GPS_t gpsFresh;
static GPS_t gpsFleeting;
static GPSPoint_t gps_trail[GPS_TRAIL_LENGTH];

static Packet_t qdac_event_packet;
//static unsigned char qdac_raw_buf[RAW_READ_SIZE * 2];


//static int qdac_highTemp_tag_processing(struct Tag *high_temp_tag);
void * qdac_thread_main(void * thread_args);
void * qdac_thread_reader(void * thread_args);
//static int qdac_verify_tag_packet(unsigned char * pkt);
static int qdac_verify_tag_packet(unsigned char **pkt, int length, int fun_code);
static void qdac_parse_tag(struct qdac_tag_control *tag_c, unsigned char *pkt);
static bool qdac_admit_tag(struct QDAC_Tag *tag, time_t now, int period);
//static bool tag_not_bursting(struct Tag *tag, struct timespec ts1);
static unsigned short qdac_crc16(unsigned char *data, unsigned int n);

//static void battery_time_checking(struct tag_control *tag_c);
static struct QDAC_Tag *qdac_search_tag(struct tq_head *tqueue, uint32_t serial);
static int filter_missing_tag( unsigned char *pkt);
static void update_gps_trail(void);
static GPSPoint_t * backtrack_gps_trail(int backward_time);
//static bool am_i_moving(uint32_t speed);
//static void sample_rssi(struct freezer_control *freezer, uint32_t rssi);
//static uint32_t find_median_rssi(struct freezer_control *freezer);
static void qdac_init_tag_parameter(struct qdac_tag_control *tag_c); 
static int qdac_init_tag_queue(struct qdac_tag_control *tag_c, struct QDAC_Tag *tag_pool1, struct QDAC_Tag *tag_pool2);
static int qdac_temp_processing(struct qdac_tag_control *tag_c, struct qdac_temp_zone *qdac_zone);
static int qdac_hightemp_queue_processing(struct qdac_tag_control *tag_c, struct qdac_hightemp_tag *high_temp_tag, struct qdac_temp_zone_tag *qdac_zone);
static int qdac_lowtemp_queue_processing(struct qdac_tag_control *tag_c, struct qdac_lowtemp_tag *low_temp_tag, struct qdac_temp_zone_tag *qdac_zone);
static void qdac_unknown_queue_processing(struct qdac_tag_control *tag_c, struct qdac_unknown_tag *unknown_tag);
static void qdac_tmp_queue_processing(struct qdac_tag_control *tag_c, struct qdac_tmp_tag *tmp_tag);
static void qdac_gforcesensor_queue_processing(struct qdac_tag_control *tag_c, struct qdac_gforcesensor_tag *rail_sensor_tag);
static void qdac_currentsensor_queue_processing(struct qdac_tag_control *tag_c, struct qdac_currentsensor_tag *current_sensor_tag);
static void qdac_switchsensor_queue_processing(struct qdac_tag_control *tag_c, struct qdac_switch_tag *switch_tag);
static void qdac_pressuresensor_queue_processing(struct qdac_tag_control *tag_c, struct qdac_pressure_tag *pressure_tag);
static void init_temp_parameter(void);
static void init_qdac_lowtemp_parameter(struct qdac_lowtemp_tag *low_tag);
static void init_qdac_hightemp_parameter(struct qdac_hightemp_tag  *high_temp);
static void init_qdac_unknown_parameter(struct qdac_unknown_tag  *unknown_tag);
static void init_qdac_tmp_parameter(struct qdac_tmp_tag *tmp_tag);
static void init_qdac_gforcesensor_parameter(struct qdac_gforcesensor_tag *gforcesensor_tag); 
static void init_qdac_currentsensor_parameter(struct qdac_currentsensor_tag *currentsensor_tag);
static void init_qdac_switchsensor_parameter(struct qdac_switch_tag *switch_tag);
static void init_qdac_pressuresensor_parameter(struct qdac_pressure_tag *pressure_tag);
static void qdac_event_gen(struct qdac_tag_control *tag_c, struct tq_head *tqueue, char *temp_des, int tag_out, int arg);
static struct QDAC_Tag *qdac_temp_tag_event(struct qdac_tag_control *tag_c, struct QDAC_Tag *tag, struct tq_head *tqueue, char *tag_des, int tag_out_time);
static int qdac_temp_event(struct qdac_tag_control *tag_c, struct QDAC_Tag *tag, struct tq_head *tqueue, struct qdac_temp_spec *temp_spec);
//static void qdac_unknown_event_gen(struct qdac_tag_control *tag_c, struct tq_head *tqueue);
//static void init_temperature_parameter(struct freezer_control *freezer);
static void make_qdac_event( struct QDAC_Tag * tag, GPSPoint_t *coord, uint16_t ev_status, time_t timestamp);
static void make_qdac_temperature_event(struct qdac_tag_control *tag_c, GPSPoint_t *coord, struct qdac_temp_spec *temp_spec, int alarm_type);
static void make_qdac_status_event(int status, int firm_ver);
static struct qdac_temp_spec *search_temp_spec(struct qdac_temp_spec **temp_spec, int tag_serial, int tag_type);
static struct qdac_temp_spec *create_temp_spec(void);
//static void quicksort(uint32_t *A, int b, int e);
//static int quick_select(uint32_t *A, int b, int e, int rank);
static int power_on_qdac(void);
static int power_off_qdac(void);
static int set_hub_mode(int mode);
static int stream_mode(unsigned int mode_switch);
static int read_hub_version(void);
static void qdac_littleE_encode2Bytes(unsigned char *pdest, unsigned short val);
static void qdac_littleE_encode4Bytes(unsigned char *pdest, unsigned int val);
static void qdac_bigE_encode2Bytes(unsigned char *pdest, unsigned short val);
static void qdac_bigE_encode4Bytes(unsigned char *pdest, unsigned int val);
static void qdac_bigE_encode3Bytes(unsigned char *pdest, unsigned int val);
static void print_response(unsigned char *resp, int length);
static void print_command(unsigned char *cmd, int cmd_length);
static int tx_cmd_to_hub(unsigned char *cmd);
//static unsigned char *rx_resp_from_hub(int *read_length);
static unsigned char *read_hub_buff(int *actur_rd_len, unsigned int expect_rd_length);
static unsigned char *rx_resp_from_hub(int *length, int expect_rd_length,  int fun_code);
static void qdac_hub_init(void);
static void qdac_init_gps(void);
static void print_time(char *des, int serial);
static void remove_tag(struct qdac_tag_control *tag_c, struct tq_head *tqueue, struct QDAC_Tag *tag); 
static void qdac_sample_temperature(struct qdac_temp_spec *temp_spec, struct QDAC_Tag *tag);
static void qdac_encode_temperature(short high, short low, short avg, unsigned char *temp);
static void qdac_temp_cal(struct qdac_temp_spec *temp_spec);
static int qdac_cmp_temp(struct qdac_temp_spec *temp_spec);
static unsigned char *char_to_hex(unsigned char *response, int *actur_rd_len);
static int copy_recv_data(struct qdac_recv_pkts *pkts_queue, unsigned char **recv_data, int read_length);
static unsigned char *verify_recv_queue(struct qdac_recv_pkts *pkts_queue, int *verify_status);
static unsigned char *find_out_start_byte(unsigned char *pkt, int *parse_error, int queue_length);
static void move_parse_index(struct qdac_recv_pkts *pkts_queue, int pkt_length);
static bool is_pkt_empty(struct qdac_recv_pkts *pkts_queue);
static bool is_pkt_full(struct qdac_recv_pkts *pkts_queue, int read_length);
/* CRC calculation looking up table */
unsigned char CRC_hi[] = {
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
};
unsigned char CRC_low[] = {
0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2,
0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04,
0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E,
0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8,
0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A,
0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,
0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6,
0xD2, 0x12, 0x13, 0xD3, 0x11, 0xD1, 0xD0, 0x10,
0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32,
0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE,
0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38,
0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA,
0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C,
0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0,
0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62,
0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE,
0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA,
0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C,
0xB4, 0x74, 0x75, 0xB5, 0x77, 0xB7, 0xB6, 0x76,
0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54,
0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E,
0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98,
0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A,
0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86,
0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80, 0x40
};

int qdac_initialize(void)
{
	if (pthread_create(&thread_qdac_main, NULL, qdac_thread_main, NULL) != 0) {
		printf("Creating Thread Error\n");
		return -1;
	}
	
	return 0;
}

void * qdac_thread_main(void *args)
{
	int n_alarms;
	int verify_status=0, error=0;
	unsigned char *tag_data = NULL;
	unsigned int reset_min = propGetUInt32AtIndex(PROP_QDAC_RESET_MIN, 1, 3);
	if (!reset_min)
		reset_min = 3;
	
	if(qdac_test == DEBUG_PORT) {
		strncpy(QDACCom.name, "ttyS0", 48);
		open_serial_binary(&QDACCom);
		qdac_args.tty_fd = QDACCom.read_fd;
		qdac_main_running = 1;
		qdac_reader_running = 1;
	} else
		qdac_hub_init();

	if (pthread_create(&thread_qdac_reader, NULL, qdac_thread_reader, &qdac_args) != 0) {
		printf("Creating Thread Error\n");
		return NULL;
	}
	
	qdac_tag_pool1 = malloc(NUM_TAGS_PER_POOL * sizeof(struct QDAC_Tag));
	if (qdac_tag_pool1 == NULL) {
		perror("Outof Memory");
		return NULL;
	}
	qdac_tag_pool2 = malloc(NUM_TAGS_PER_POOL * sizeof(struct QDAC_Tag));
	if (qdac_tag_pool2 == NULL) {
		perror("Outof Memory");
		return NULL;
	}
	memset(qdac_tag_pool1, 0, NUM_TAGS_PER_POOL * sizeof(struct QDAC_Tag));
	memset(qdac_tag_pool2, 0, NUM_TAGS_PER_POOL * sizeof(struct QDAC_Tag));
	
	qdac_init_tag_queue(&qdac_recv_tag, qdac_tag_pool1, qdac_tag_pool2);
	qdac_init_tag_parameter(&qdac_recv_tag);

	qdac_init_gps();
	
	while (qdac_main_running != 0) { 
		update_gps_trail();
//		battery_time_checking(&tag_control_1);
		n_alarms = 0;
//printf("!!qdac_main_running %d\n", qdac_main_running);
		pthread_mutex_lock(&qdac_recv_tag.mutex_qdac_pkts);
		while(!is_pkt_empty(&qdac_pkts)) {
			tag_data=verify_recv_queue(&qdac_pkts,&verify_status);	
			if (tag_data == NULL) {
				if (verify_status < 0) { error -= verify_status; goto error_check;}
				else if (verify_status > 0) break;	// need to read more data
			}else if(tag_data != NULL) {
				error = 0;
				if(filter_missing_tag(tag_data) < 0) {
					free(tag_data);
					continue;
				}

/*			if(qdac_test == DEBUG_PORT)  {
				printf("go to parse the tag\n");
				printf("-----------------\n");
			}
			else
*/				qdac_parse_tag(&qdac_recv_tag, tag_data);
			}	
error_check:
			if(error >= 3) {
				printf("Errors occured: Reset Hub...\n");	
				break;
			}
		}
		pthread_mutex_unlock(&qdac_recv_tag.mutex_qdac_pkts);

		if(error >= 3) {
			error = 0;
			qdac_hub_reset = true;
			if(qdac_test == DEBUG_PORT) {
				printf("reset HUB...\n");
//				pthread_mutex_lock(&qdac_recv_tag.mutex_hub_ctrl);
				sleep(MINUTES);
				open_serial_binary(&QDACCom);
				qdac_args.tty_fd = QDACCom.read_fd;
				printf("reset HUB done\n");
				qdac_hub_reset = false;
//				pthread_mutex_unlock(&qdac_recv_tag.mutex_hub_ctrl);
//				continue;
			}
			else {
				printf("reset HUB\n");
//				pthread_mutex_lock(&qdac_recv_tag.mutex_hub_ctrl);
				power_off_qdac();
				sleep(reset_min*MINUTES);
				qdac_hub_init();
				qdac_hub_reset = false;
//				pthread_mutex_unlock(&qdac_recv_tag.mutex_hub_ctrl);
//				continue;
			}
		}

		n_alarms = qdac_temp_processing(&qdac_recv_tag, &qdac_zone);
		qdac_gforcesensor_queue_processing(&qdac_recv_tag, &qdac_gforce_sensor);
		qdac_currentsensor_queue_processing(&qdac_recv_tag, &qdac_current_sensor);
		qdac_switchsensor_queue_processing(&qdac_recv_tag, &qdac_switch_sensor);
		qdac_pressuresensor_queue_processing(&qdac_recv_tag, &qdac_pressure_sensor);
		qdac_unknown_queue_processing(&qdac_recv_tag, &qdac_unknown);
		qdac_tmp_queue_processing(&qdac_recv_tag, &qdac_tmp);
		
		if (n_alarms > 0)
			protocolStartSession();

		sleep(1);
	}
	free(qdac_tag_pool1);
	free(qdac_tag_pool2);
	
	return NULL;
}

void * qdac_thread_reader(void * thread_args)
{
	struct qdac_thread_args * args = (struct qdac_thread_args *)thread_args;	
	struct qdac_tag_control *tag_c = args->tag_c;
	int read_length = 0;
	int verify_res = -1;
	unsigned char *recv_data = NULL;
	int read_error = 0;
	int hub_alive = 0;
	unsigned int reset_min = propGetUInt32AtIndex(PROP_QDAC_RESET_MIN, 1, 3);
	
	if (qdac_reader_running == 0)
		goto exit_r;

	qdac_pkts.qdac_pkts_queue = (unsigned char *)malloc(PKTS_QUEUE_LENGTH*sizeof(unsigned char));
	if (qdac_pkts.qdac_pkts_queue == NULL) {
		perror("qdac_pkts_queue malloc");
		exit(-1);
	}
	memset(qdac_pkts.qdac_pkts_queue , 0, PKTS_QUEUE_LENGTH*sizeof(unsigned char));
	qdac_pkts.read_index = 0;
	qdac_pkts.parse_index = 0;
	qdac_pkts.queue_unparse_length = 0;
//	while (qdac_reader_running != 0) { 
	while (1) {
//printf("!!qdac_hub_reset: %d\n", qdac_hub_reset);
//		pthread_mutex_lock(&tag_c->mutex_hub_ctrl);
		read_length = 0;
		if (!qdac_hub_reset)
			recv_data=rx_resp_from_hub(&read_length, RESPONSE_MAX_SIZE*sizeof(unsigned char), READ_TAG_DATA);
		else {
			if(qdac_test == DEBUG_PORT) 
				sleep(MINUTES);
			else
				sleep(reset_min*MINUTES);
			continue;
		}
//		pthread_mutex_unlock(&tag_c->mutex_hub_ctrl);
/*		if(recv_data == NULL)  {
//			printf("detect hub if alive\n");
//			pthread_mutex_lock(&tag_c->mutex_hub_ctrl);
			hub_alive = read_hub_version();			// if no data being read in 10 secs, send cmd to read hub version to make sure the hub is alive.
//			pthread_mutex_unlock(&tag_c->mutex_hub_ctrl);
			if (hub_alive < 0) {
//				pthread_mutex_lock(&tag_c->mutex_hub_ctrl);
				power_off_qdac();	
				qdac_hub_init();
//				pthread_mutex_unlock(&tag_c->mutex_hub_ctrl);
				continue;
			} 
		}
*/		if (read_length) {
//print_response(recv_data, read_length);
			do {
				pthread_mutex_lock(&tag_c->mutex_qdac_pkts);
				qdac_reader_running = copy_recv_data(&qdac_pkts, &recv_data, read_length);
				pthread_mutex_unlock(&tag_c->mutex_qdac_pkts);
				if(qdac_debug) {
//					printf("%s: ", __FUNCTION__);
//					printf("Copy to buff Length: %d\n", read_length);
//					printf("Cur read_index: %d\n", qdac_pkts.read_index);
//					printf("Pkt unparse Length: %d\n",qdac_pkts.queue_unparse_length);
//					print_response(recv_data, read_length);
//					printf("add: %p\n", qdac_pkts.qdac_pkts_queue);
				}
				if(!qdac_reader_running)
					sleep(1);
			} while(!qdac_reader_running);
		}
	}
	qdac_reader_running = 0;

exit_r:
	comPortClose(&QDACCom);
	return NULL;
}

static bool is_pkt_full(struct qdac_recv_pkts *pkts_queue, int read_length) {
	uint32_t unparse_length = pkts_queue->queue_unparse_length;
	if(unparse_length + read_length > PKTS_QUEUE_LENGTH)
		return true;
	else
		return false;
}
static bool is_pkt_empty(struct qdac_recv_pkts *pkts_queue) {
	uint32_t unparse_length = pkts_queue->queue_unparse_length;
	if(unparse_length == 0)
		return true;
	else
		return false;
}
static int copy_recv_data(struct qdac_recv_pkts *pkts_queue, unsigned char **recv_data, int read_length) {
	int pkt_rest = 0;
	int read_index = pkts_queue->read_index;
	int parse_index = pkts_queue->parse_index;
//	int unparse_length = pkts_queue->queue_unparse_length;
	unsigned char *queue = pkts_queue->qdac_pkts_queue;
	bool full_res;
//	if (read_index+1 >= parse_index) 		// queue is full
//		goto queue_full;
//	else {
//printf("%s: before copy: read_index %d - parse_index:%d\n", __FUNCTION__, read_index, parse_index);
	full_res = is_pkt_full(pkts_queue, read_length);
	if (full_res) 
		goto queue_full;
	else{
		pkt_rest = PKTS_QUEUE_LENGTH - read_index;
		if (pkt_rest < read_length) {
			memcpy(queue+read_index, *recv_data, pkt_rest*sizeof(unsigned char));
			memcpy(queue, *recv_data+pkt_rest, (read_length-pkt_rest)*sizeof(unsigned char));
			pkts_queue->read_index = read_length-pkt_rest;
		} else {
			memcpy(queue+read_index, *recv_data, read_length*sizeof(unsigned char));
			pkts_queue->read_index += read_length;
			if (pkts_queue->read_index == PKTS_QUEUE_LENGTH)
				pkts_queue->read_index = 0;
		}
	} 
	pkts_queue->queue_unparse_length += read_length;
	free(*recv_data);
	*recv_data = NULL;
	return read_length;

queue_full:
	printf("pkts queue is full already, waiting for space & no more reading\n");
//	qdac_reader_running = 0;
	return 0;
}

// qdac_Verify_tag_packet returns the Error counts: 1 error: -1; 2 error -2;...
// if success return 0: no error;
// if the current pkt is in the mid of a complete packet, return the "reset length" to
// get a full packet.
static int qdac_verify_tag_packet(unsigned char **pkt, int verify_length, int fun_code)
{
	unsigned short crc_cal, crc_recv;
	int crc_index = 0;
//	int index = 0;
//	int res = read_length;
	int payload_length=0,	 pkt_length=0, total_pkt_length=0;
	int error = 0;
	unsigned char *pkt_start = NULL; 
//	unsigned char *tmp_clean = NULL; 

	if(fun_code != READ_TAG_DATA) {
		pkt_start=find_out_start_byte (*pkt, &error, verify_length);				// 1. 0xAA
		if(pkt_start == NULL)
			return -1;

		if (pkt_start != *pkt)
			*pkt = pkt_start;

		payload_length = *(pkt_start+2); 	// found out a expected complete pkt length
		pkt_length = payload_length + 5;

		if (pkt_length > verify_length) {
			printf("need more data");
			return pkt_length -verify_length;
		}
	} else {
		pkt_start = *pkt;
		pkt_length = verify_length;
		payload_length = pkt_length - 5;
	}

print_response(pkt_start, pkt_length);

	if (*(pkt_start+1) != fun_code) {										//  2. function code
		printf("!!Function code error\n");
		error -= 1;				
	} else {
			crc_cal= qdac_crc16(pkt_start+1, 2+payload_length); 				// 3. check CRC
			crc_index = 3 + payload_length;
			crc_recv = *(pkt_start+crc_index+1) << 8 | *(pkt_start+crc_index);
			if (crc_cal != crc_recv) {                 							// if CRC error, assumes that the starting "0xAA" is not expected, find the next one.
				printf("!!CRC Error: recv %02x - cal %02x from packet: \n", crc_recv, crc_cal);
				printf("pkt length %d\n", verify_length);
				print_response(pkt_start, verify_length);
				error -= 1;
			} else {
//				printf("CRC Correct\n");
				return VERIFY_SUCCESS;
			}
	}

	return error;
}

static void qdac_parse_tag(struct qdac_tag_control *tag_c, unsigned char *pkt) 
{
	uint32_t serial; 
	int max_period;
	struct QDAC_Tag *tag = NULL; 
	struct tq_head *tqueue;
	pthread_mutex_t *pmutex;
	struct timespec tsnow;
	uint16_t data = 0xFFFF;
	unsigned short tag_type;
	uint32_t tmp_type1 = propGetUInt32(PROP_QDAC_TMP_TYPE1, 0);
	uint32_t tmp_type2 = propGetUInt32(PROP_QDAC_TMP_TYPE2, 0);


	serial = (pkt[6] << 24 | pkt[5] << 16 | pkt[4] << 8 | pkt[3]); 
	tag_type = ((pkt[22] << 8) | pkt[21]);
	if (tag_type == LOW_TEMP_TAG) {
		max_period = qdac_low_temp.tag_in_time;
		tqueue = &tag_c->qdac_lowtemp_queue;
		pmutex = &tag_c->mutex_qdac_lowtemp;
	} else if (tag_type == HIGH_TEMP_TAG) {
			max_period = qdac_high_temp.tag_in_time;
			tqueue = &tag_c->qdac_hightemp_queue;
			pmutex = &tag_c->mutex_qdac_hightemp;
	} else if (tag_type == GFORCE_SENSOR_TAG) {
			max_period = qdac_gforce_sensor.tag_in_time;
			tqueue = &tag_c->qdac_gforcesensor_queue;
			pmutex = &tag_c->mutex_qdac_gforcesensor;
	} else if (tag_type == CURRENT_SENSOR_TAG) {
			max_period = qdac_current_sensor.tag_in_time;
			tqueue = &tag_c->qdac_currentsensor_queue;
			pmutex = &tag_c->mutex_qdac_currentsensor;
	} else if (tag_type == SWITCH_TAG) {
			max_period = qdac_switch_sensor.tag_in_time;
			tqueue = &tag_c->qdac_switchsensor_queue;
			pmutex = &tag_c->mutex_qdac_switchsensor; 
	} else if (tag_type == PRESSURE_TAG) {
			max_period = qdac_pressure_sensor.tag_in_time;
			tqueue = &tag_c->qdac_pressuresensor_queue;
			pmutex = &tag_c->mutex_qdac_pressuresensor; 	
	} else if (tag_type == tmp_type1 || tag_type ==tmp_type2) { //temperary type of tag
			if (qdac_debug)
				printf("a Tmporary Tag %d is found\n", serial);
			max_period = qdac_tmp.tag_in_time;
			tqueue = &tag_c->qdac_tmp_queue;
			pmutex = &tag_c->mutex_qdac_tmp;
	} else { // unknown type
		if (qdac_debug)
			printf("a Unknown Tag %d is found\n", serial);
		max_period = qdac_unknown.tag_in_time;
		tqueue = &tag_c->qdac_unknown_queue;
		pmutex = &tag_c->mutex_qdac_unknown;
	}

	if (clock_gettime(CLOCK_REALTIME, &tsnow) < 0) {
		perror("Realtime clock");
		return;
	}
	pthread_mutex_lock(pmutex);
	tag = qdac_search_tag(tqueue, serial);	
	if (tag == NULL) {
		if (TAILQ_EMPTY(&tag_c->recycle_queue)) {
			pthread_mutex_unlock(pmutex);
			return;
		}
		pthread_mutex_lock(&tag_c->mutex_recycle);
		tag = tag_c->recycle_queue.tqh_first;
		TAILQ_REMOVE(&tag_c->recycle_queue, tag, link);
		pthread_mutex_unlock(&tag_c->mutex_recycle);		
		TAILQ_INSERT_TAIL(tqueue, tag, link);
/* init some key elements for the struct QDAC_tag */
		tag->raw_data = NULL;
		tag->raw_data_length = 0;
		tag->status = 0;
	} else if (!(tag->status & (TAG_FRESH | TAG_SENIOR))) {
		if (qdac_admit_tag(tag, tsnow.tv_sec, max_period))
			tag->status |= TAG_FRESH;
		// for the unknown tag
//		if (!(tag->qdac_tag_type))
//			printf("status %d\n", tag->status);
	}
	pthread_mutex_unlock(pmutex);	
	/* fill the other field */
	tag->qdac_tag_serial = serial;
//	tag_c->qdac_tag_serial = serial;
	tag->qdac_tag_model = pkt[8] << 8 | pkt[7];
	tag->qdac_tag_id = pkt[10] << 8 | pkt [9];
	tag->qdac_hub_id = pkt[12] << 8 | pkt[11];
	tag->qdac_tag_contact = (pkt[16] << 24 | pkt[15] << 16 | pkt[14] << 8 | pkt[13]); 
	tag->qdac_tag_beacon_period = (pkt[18] << 8 | pkt[17]);
	tag->qdac_tag_rev_period = (pkt[20] << 8 | pkt[19]);
	tag->qdac_tag_type = tag_type;
//	tag_c->qdac_tag_type = tag_type;
	tag->qdac_rssi= pkt[23];
	tag->qdac_tag_battery= pkt[24];
	tag->qdac_flag = pkt[25];

	/* temperature or g-force */
//	tag->qdac_temp = NULL;

	
	if (tag_type == HIGH_TEMP_TAG || tag_type == LOW_TEMP_TAG)  {
		data = pkt[27] << 8 | pkt[26]; 
		tag->cur_temp = data * 5; // (data / 2 * 10): data / 2 to get the temperature value, and then time 10 for sending.
	}
	else if (tag_type == GFORCE_SENSOR_TAG) {
		data = pkt[27] << 8 | pkt[26]; 
		tag->qdac_gforce = data;
	}
	else if (tag_type == CURRENT_SENSOR_TAG) {	
		tag->qdac_current_2_1 = (pkt[29] << 24 | pkt[28] << 16 | pkt[27] << 8 | pkt[26]);
		tag->qdac_current_4_3 = (pkt[33] << 24 | pkt[32] << 16 | pkt[31] << 8 | pkt[30]);
	} else if (tag_type == SWITCH_TAG) { 
		data = pkt[26];
		tag->qdac_switch = data;
	} else if (tag_type == PRESSURE_TAG) { 
		data = (pkt[29] << 24 | pkt[28] << 16 | pkt[27] << 8 | pkt[26]);
//printf("Pressure: %02x -%02x %02x %02x %02x\n", data, pkt[29], pkt[28], pkt[27], pkt[26]);
		tag->qdac_pressure= data;
	} else {		// unknown tag and the temperary tag
			tag->raw_data_length = pkt[2]  - 23; 
			if (tag->raw_data != NULL)
				free(tag->raw_data);
			if (tag->raw_data_length) {
				tag->raw_data = (unsigned char *)malloc((tag->raw_data_length)*sizeof(unsigned char));
				if (tag->raw_data == NULL)
					perror("tag->raw_data malloc");
				memset(tag->raw_data, 0, (tag->raw_data_length)*sizeof(unsigned char));
				memcpy(tag->raw_data, pkt+26, (tag->raw_data_length)*sizeof(unsigned char));
			}
	}
	tag->recent = tsnow.tv_sec;
}

static int filter_missing_tag( unsigned char *pkt)
{
	uint32_t serial; 
	uint16_t tag_type, tag_model, tag_beacon_period,tag_rev_period;
	uint8_t tag_battery, tag_flag;
	int res = 1;
	
	serial = (pkt[6] << 24 | pkt[5] << 16 | pkt[4] << 8 | pkt[3]); 
	tag_model = pkt[8] << 8 | pkt[7];
	tag_beacon_period = (pkt[18] << 8 | pkt[17]);
	tag_rev_period = (pkt[20] << 8 | pkt[19]);
	tag_type = ((pkt[22] << 8) | pkt[21]);
	tag_battery= pkt[24];
	tag_flag = pkt[25];

	if(serial == 0 && tag_model == 0 && tag_beacon_period == 0xFFFF
		&& tag_rev_period == 0 && tag_type == 0 && tag_battery == 0
		&& tag_flag == 0)
			res = -1;

	return res;
}

static struct QDAC_Tag *qdac_search_tag(struct tq_head *tqueue, uint32_t serial)
{
	struct QDAC_Tag *tag = NULL;
	if (TAILQ_EMPTY(tqueue))
		return tag;
	TAILQ_FOREACH(tag, tqueue, link) {
		if (tag->qdac_tag_serial == serial)
			break;
	}
	return tag;
}
static bool qdac_admit_tag(struct QDAC_Tag *tag, time_t now, int period)
{
	return (now - tag->recent <= period);
}

static uint32_t temp_report = 0;
static int qdac_temp_processing(struct qdac_tag_control *tag_c, struct qdac_temp_zone *qdac_zone) {
	int i, upcoming = 0;
	int alarms = 0;
	struct qdac_temp_zone_tag *qdac_zone_tag;
	struct qdac_temp_spec *temp_spec = NULL;
	static uint32_t alarm_flag_pre = 0;
	static uint32_t alarm_flag_cur = 0;
	static uint32_t qdac_temp_alarm_beat = 0;
	static uint32_t qdac_alarm_count = 0;

	if(qdac_zone->primary_id) {		
//		temp_report++;
//		(qdac_zone->low_temp_tag)->report_cycle = qdac_zone->report_cycle;
//		(qdac_zone->high_temp_tag)->report_cycle = qdac_zone->report_cycle;
		alarm_flag_cur = 0;
		
		for(i=0; i<QDAC_NUM_ZONES; i++) {
			qdac_zone_tag = &(qdac_zone->qdac_zone_tag[i]);
			alarms = 0;
			if(qdac_zone_tag->qdac_tag_type == LOW_TEMP_TAG) {
				alarms = qdac_lowtemp_queue_processing(&qdac_recv_tag, qdac_zone->low_temp_tag, qdac_zone_tag);
			} else if (qdac_zone_tag->qdac_tag_type == HIGH_TEMP_TAG) {
				alarms = qdac_hightemp_queue_processing(&qdac_recv_tag, qdac_zone->high_temp_tag, qdac_zone_tag);
			}
			if  (alarms == QDAC_ALARM_HIGH || alarms == QDAC_ALARM_LOW)
				alarm_flag_cur |= 1<<i;
		}

//printf("alarm_flag_cur: %x\n", alarm_flag_cur);
		if (alarm_flag_cur) {
			if (qdac_alarm_count == 0)  {
				upcoming = qdac_zone->alarm_delay;
//				print_time("First alarm", 0);
//				printf("\n");
			}
			else if (qdac_alarm_count < qdac_zone->alarm_times) {
//				printf("alarm_cycle %d\n", qdac_zone->alarm_cycle);
				upcoming = qdac_zone->alarm_cycle;
			}
			else {
				if (alarm_flag_cur == alarm_flag_pre) {
					alarm_flag_pre = alarm_flag_cur;
					return 0;
				}
				else
					qdac_alarm_count = 0;
			}
			alarm_flag_pre = alarm_flag_cur;
/* 	if there is an alarm produced currently, increase the alarm beat 
	else if there is no any alarms being produced, reset alarm beat
*/
//printf("qdac_temp_alarm_beat %d : upcoming %d\n" ,qdac_temp_alarm_beat, upcoming);
			if (++qdac_temp_alarm_beat >= upcoming) {
				i = 0;
				while(i < QDAC_NUM_ZONES) {
					if (alarm_flag_cur & (1 << i)) {
						qdac_zone_tag = &(qdac_zone->qdac_zone_tag[i]);
						if(qdac_zone_tag->zone_own_spec == NULL) {printf("!!zone_spec NULL\n"); break;}
						temp_spec = search_temp_spec(&(qdac_zone_tag->zone_own_spec), qdac_zone_tag->qdac_serial, qdac_zone_tag->qdac_tag_type);
 						make_qdac_temperature_event(tag_c, &gpsFresh.point, temp_spec, temp_spec->alarm_type);
					}
					i++;
				}
				qdac_alarm_count++;
				qdac_temp_alarm_beat = 0;
			} 
			else
				return 0;
		}
		else	
			qdac_temp_alarm_beat = 0;
			
//		if(temp_report >= qdac_zone->report_cycle)
//			temp_report = 0;	
	} else {
		qdac_lowtemp_queue_processing(&qdac_recv_tag, &qdac_low_temp, NULL);
		qdac_hightemp_queue_processing(&qdac_recv_tag, &qdac_high_temp, NULL);
	}

	return alarm_flag_cur;
}

static int qdac_lowtemp_queue_processing (struct qdac_tag_control *tag_c, struct qdac_lowtemp_tag *low_temp_tag, 
	struct qdac_temp_zone_tag *qdac_zone_tag)
{
	int alarm = QDAC_ALARM_NORM;
//	static uint32_t lowtemp_report = 0;
	struct qdac_temp_spec *lowtemp_spec = NULL;
	struct tq_head *tqueue;
	struct QDAC_Tag *tag;
//	struct qdac_temp_zone qdac_zone;
	bool report_norm_temp_event = false;
	time_t now;
	static time_t low_temp_last=0;

	now = time(NULL);
//	printf("time elapse %d\n", now - low_temp_last);
//	printf("report_cycle %d\n", low_temp_tag->report_cycle);
	if(now - low_temp_last >= low_temp_tag->report_cycle) {
		report_norm_temp_event = true;
		low_temp_last = now;
	}

	tqueue = &tag_c->qdac_lowtemp_queue;
	if (TAILQ_EMPTY(tqueue)) {
		return 0;
	}
/* if primary_id is not set, the low/high temperature queue has its own timer */
/*	if(tag_c->primary_id) 
		lowtemp_report = temp_report;
	else
		lowtemp_report++;

	if(lowtemp_report >= low_temp_tag->report_cycle) 
		report_norm_temp_event = true;
*/	
	pthread_mutex_lock(&tag_c->mutex_qdac_lowtemp);
	for (tag = tqueue->tqh_first; tag != NULL; tag = tag->link.tqe_next) {	
		if(tag_c->primary_id) {
			if(qdac_zone_tag->qdac_serial == tag->qdac_tag_serial) {
				lowtemp_spec=search_temp_spec(&(low_temp_tag->qdac_lowtemp_tag_spec), tag->qdac_tag_serial, tag->qdac_tag_type);
				lowtemp_spec->zone_id = qdac_zone_tag->zone_id;
				lowtemp_spec->set_high = qdac_zone_tag->qdac_zone_temp_max;
				lowtemp_spec->set_low = qdac_zone_tag->qdac_zone_temp_min;
				if (qdac_zone_tag->zone_own_spec == NULL)
					qdac_zone_tag->zone_own_spec = lowtemp_spec;
			} else
				continue;
		}
		else {
			lowtemp_spec=search_temp_spec(&(low_temp_tag->qdac_lowtemp_tag_spec), tag->qdac_tag_serial, tag->qdac_tag_type);
		}
		lowtemp_spec->qdac_tag_model = tag->qdac_tag_model;
		tag = qdac_temp_tag_event(tag_c, tag, tqueue, "Low Temperature", low_temp_tag->tag_out_time);
		if (tag == NULL)
				break;
		if (tag->status & (TAG_FRESH | TAG_SENIOR)) {
			qdac_sample_temperature(lowtemp_spec, tag);
			alarm = qdac_temp_event(tag_c, tag, tqueue, lowtemp_spec);
			if(report_norm_temp_event && (alarm == QDAC_ALARM_NORM)) {
				make_qdac_temperature_event(tag_c, &gpsFresh.point, lowtemp_spec, alarm);	
			}	
			if(tag_c->primary_id && (qdac_zone_tag->qdac_serial == tag->qdac_tag_serial))		
				break;
		}
	}
	pthread_mutex_unlock(&tag_c->mutex_qdac_lowtemp);
	
//	if(!tag_c->primary_id && report_norm_temp_event) 
//		lowtemp_report = 0;

	return alarm; 
}

static int qdac_hightemp_queue_processing(struct qdac_tag_control *tag_c, struct qdac_hightemp_tag *high_temp_tag,
	struct qdac_temp_zone_tag *qdac_zone_tag)
{
	struct tq_head *tqueue;
	struct QDAC_Tag *tag;
//	static uint32_t hightemp_report = 0;
//	int hightemp_report_reset = 0;
	int alarm = QDAC_ALARM_NORM;
	bool report_norm_temp_event = false;
	struct qdac_temp_spec *hightemp_spec = NULL;
	time_t now;
	static time_t high_temp_last=0;

	now = time(NULL);
	if(now - high_temp_last >= high_temp_tag->report_cycle) {
		report_norm_temp_event = true;
		high_temp_last = now;
	}

	tqueue = &tag_c->qdac_hightemp_queue;
	if (TAILQ_EMPTY(tqueue))
		return 0;
/*
	if(tag_c->primary_id) 
		hightemp_report = temp_report;
	else
		hightemp_report++;
	if(hightemp_report >= high_temp_tag->report_cycle) 
		report_norm_temp_event = true;
*/
	pthread_mutex_lock(&tag_c->mutex_qdac_hightemp);
	for (tag = tqueue->tqh_first; tag != NULL; tag = tag->link.tqe_next) {			
		if(tag_c->primary_id) {
			if(qdac_zone_tag->qdac_serial == tag->qdac_tag_serial) {
				hightemp_spec = search_temp_spec(&(high_temp_tag->qdac_hightemp_tag_spec),tag->qdac_tag_serial, tag->qdac_tag_type);			
				hightemp_spec->zone_id = qdac_zone_tag->zone_id;
				hightemp_spec->set_high = qdac_zone_tag->qdac_zone_temp_max;
				hightemp_spec->set_low = qdac_zone_tag->qdac_zone_temp_min;
				if (qdac_zone_tag->zone_own_spec == NULL)
					qdac_zone_tag->zone_own_spec = hightemp_spec;
			} else	
				continue;		
		} else {
			hightemp_spec = search_temp_spec(&(high_temp_tag->qdac_hightemp_tag_spec),tag->qdac_tag_serial, tag->qdac_tag_type);
		}
		hightemp_spec->qdac_tag_model = tag->qdac_tag_model;
		tag = qdac_temp_tag_event(tag_c, tag, tqueue, "High Temperature", high_temp_tag->tag_out_time);
		if (tag == NULL)
			break;
		if (tag->status & (TAG_FRESH | TAG_SENIOR)) {
			qdac_sample_temperature(hightemp_spec, tag);
			alarm = qdac_temp_event(tag_c, tag, tqueue, hightemp_spec);
			if(report_norm_temp_event && (alarm == QDAC_ALARM_NORM)) {
				make_qdac_temperature_event(tag_c, &gpsFresh.point, hightemp_spec, alarm);	
			}	
			if(tag_c->primary_id && (qdac_zone_tag->qdac_serial == tag->qdac_tag_serial))	
				break;
		}
	}
	pthread_mutex_unlock(&tag_c->mutex_qdac_hightemp);
	
//	if(!tag_c->primary_id && report_norm_temp_event) 
//		hightemp_report = 0;

	return alarm;
}

static void qdac_gforcesensor_queue_processing(struct qdac_tag_control *tag_c, struct qdac_gforcesensor_tag *gforce_sensor_tag)
{
	struct tq_head *tqueue;
//	static uint32_t gforcesensor_beacon = 0;

// the event store interval is based on the beacon rate instead of the report cycle
// if the payload G-force is beyond 2G (20), store the event.
//	if (++gforcesensor_beacon == gforce_sensor_tag->beacon_cycle) {
//		gforcesensor_beacon = 0;

		tqueue = &tag_c->qdac_gforcesensor_queue;
		if (TAILQ_EMPTY(tqueue))
			return;
//printf("%s\n", __FUNCTION__);
		pthread_mutex_lock(&tag_c->mutex_qdac_gforcesensor);
		qdac_event_gen(tag_c, tqueue, "G-force Sensor", gforce_sensor_tag->tag_out_time, gforce_sensor_tag->report_threshold);
		pthread_mutex_unlock(&tag_c->mutex_qdac_gforcesensor);
//	} else
//		return;
}

static void qdac_currentsensor_queue_processing(struct qdac_tag_control *tag_c, struct qdac_currentsensor_tag *current_sensor_tag)
{
	struct tq_head *tqueue;
	static uint32_t sensor_beat = 0;
	bool report_current_event = false;
	time_t now;
	static time_t current_last=0;

	now = time(NULL);

	if(now - current_last >= current_sensor_tag->report_cycle) {
		report_current_event = true;
		current_last = now;
	}
//	if (++sensor_beat == current_sensor_tag->report_cycle) {
//		sensor_beat = 0;
	if(report_current_event) {
		tqueue = &tag_c->qdac_currentsensor_queue;
		if (TAILQ_EMPTY(tqueue)) return;
	
		pthread_mutex_lock(&tag_c->mutex_qdac_currentsensor);
		qdac_event_gen(tag_c, tqueue, "Current Sensor", current_sensor_tag->tag_out_time, 0);
		pthread_mutex_unlock(&tag_c->mutex_qdac_currentsensor);
	} else
		return;
}

static void qdac_switchsensor_queue_processing(struct qdac_tag_control *tag_c, struct qdac_switch_tag *switch_tag)
{
	struct tq_head *tqueue;
	static uint32_t sensor_beat = 0;
	bool report_switch_event = false;
	time_t now;
	static time_t switch_last=0;

	now = time(NULL);

	if(now - switch_last >= switch_tag->report_cycle) {
		report_switch_event = true;
		switch_last = now;
	}
//	if (++sensor_beat == current_sensor_tag->report_cycle) {
//		sensor_beat = 0;
	if(report_switch_event) {
		tqueue = &tag_c->qdac_switchsensor_queue;
		if (TAILQ_EMPTY(tqueue)) return;
	
		pthread_mutex_lock(&tag_c->mutex_qdac_switchsensor);
		qdac_event_gen(tag_c, tqueue, "Switch Sensor", switch_tag->tag_out_time, 0);
		pthread_mutex_unlock(&tag_c->mutex_qdac_switchsensor);
	} else
		return;
}

static void qdac_pressuresensor_queue_processing(struct qdac_tag_control *tag_c, struct qdac_pressure_tag *pressure_tag)
{
	struct tq_head *tqueue;
	static uint32_t sensor_beat = 0;
	bool report_pressure_event = false;
	time_t now;
	static time_t pressure_last=0;

	now = time(NULL);

	if(now - pressure_last >= pressure_tag->report_cycle) {
		report_pressure_event = true;
		pressure_last = now;
	}
//	if (++sensor_beat == current_sensor_tag->report_cycle) {
//		sensor_beat = 0;
	if(report_pressure_event) {
		tqueue = &tag_c->qdac_pressuresensor_queue;
		if (TAILQ_EMPTY(tqueue)) return;
	
		pthread_mutex_lock(&tag_c->mutex_qdac_pressuresensor);
		qdac_event_gen(tag_c, tqueue, "Pressure Sensor", pressure_tag->tag_out_time, 0);
		pthread_mutex_unlock(&tag_c->mutex_qdac_pressuresensor);
	} else
		return;
}


static void qdac_unknown_queue_processing(struct qdac_tag_control *tag_c, struct qdac_unknown_tag *unknown_tag)
{
	struct tq_head *tqueue;
//	static uint32_t unknown_beat = 0;
	bool report_unknown_event = false;
	time_t now;
	static time_t current_last=0;

	now = time(NULL);
	if(now - current_last >= unknown_tag->report_cycle) {
		report_unknown_event = true;
		current_last = now;
	}

	if(report_unknown_event) {
		tqueue = &tag_c->qdac_unknown_queue;
		if (TAILQ_EMPTY(tqueue))
			return;
		pthread_mutex_lock(&tag_c->mutex_qdac_unknown);
		qdac_event_gen(tag_c, tqueue, "Unknown", unknown_tag->tag_out_time, 0);
		pthread_mutex_unlock(&tag_c->mutex_qdac_unknown);
	} else
		return;
}

static void qdac_tmp_queue_processing(struct qdac_tag_control *tag_c, struct qdac_tmp_tag *tmp_tag)
{
	struct tq_head *tqueue;
//	static uint32_t unknown_beat = 0;
	bool report_tmp_event = false;
	time_t now;
	static time_t current_last=0;

	now = time(NULL);
	if(now - current_last >= tmp_tag->report_cycle) {
		report_tmp_event = true;
		current_last = now;
	}

	if(report_tmp_event) {
		tqueue = &tag_c->qdac_tmp_queue;
		if (TAILQ_EMPTY(tqueue))
			return;
		pthread_mutex_lock(&tag_c->mutex_qdac_tmp);
		qdac_event_gen(tag_c, tqueue, "Temporary", tmp_tag->tag_out_time, 0);
		pthread_mutex_unlock(&tag_c->mutex_qdac_tmp);
	} else
		return;
}

static struct QDAC_Tag *qdac_temp_tag_event(struct qdac_tag_control *tag_c, struct QDAC_Tag *tag, struct tq_head *tqueue, char *tag_des, //int tag_type,
	int tag_out_time) {	
	struct QDAC_Tag *tag1;
	time_t now, past;

	now = time(NULL);
	past = now - tag_out_time;

	if (tag->recent < past) {
		if (tag->status & TAG_SENIOR) {
			make_qdac_event(tag, backtrack_gps_trail(now - tag->recent), STATUS_QDAC_PRIMARY_OUT, now);
			print_time(tag_des, tag->qdac_tag_serial);
			printf("Primary Tag Out\n");	
		}
		tag1 = tag->link.tqe_next;
		remove_tag(tag_c, tqueue, tag);
		if (tag1 == NULL)
			return NULL;
		else
			return tag1;
	} else { /*IN processing */
		if (tag->status & TAG_FRESH) {
			tag->status &= ~TAG_FRESH;
			tag->status |= TAG_SENIOR;
			make_qdac_event(tag, &gpsFresh.point, STATUS_QDAC_PRIMARY_IN, now);
			print_time(tag_des, tag->qdac_tag_serial); 
			print_debug("Primary Tag In\n");
		} 
		return tag;
	}
}

static int qdac_temp_event(struct qdac_tag_control *tag_c, struct QDAC_Tag *tag, struct tq_head *tqueue, struct qdac_temp_spec *temp_spec)
{
	int alarm_type = QDAC_ALARM_NORM;
//	int upcoming = 0;
//	static unsigned int qdac_temp_alarm_beat = 0;
	
	if (tag->status & TAG_SENIOR) {
		qdac_temp_cal(temp_spec);
		if (temp_spec->zone_id != 0xFF) {					// zone is setting
			alarm_type = qdac_cmp_temp(temp_spec);
			temp_spec->alarm_type = alarm_type;
		}
	}
//if (alarm_type)
//printf("!! %s: %d: alarm %s\n", __FUNCTION__, tag->qdac_tag_serial, alarm_type == 1?"Low Alarm":"High Alarm");

	return alarm_type;
}

static void qdac_event_gen(struct qdac_tag_control *tag_c, struct tq_head *tqueue, char *temp_des, int  tag_out_time, int arg) 
{
	struct QDAC_Tag *tag, *tag1;
	time_t now, past;
//	struct tm tm1; 
//	struct tm *ptm;
	now = time(NULL);
	
	past = now -tag_out_time;

	for (tag = tqueue->tqh_first; tag != NULL; tag = tag->link.tqe_next) {	
		if (tag->recent < past) {
			if (tag->status & TAG_SENIOR) {
					make_qdac_event(tag, backtrack_gps_trail(now - tag->recent), STATUS_QDAC_TAG_OUT, now);	
					print_time(temp_des, tag->qdac_tag_serial);
					printf("Tag OUT \n");
			}
			tag1 = tag->link.tqe_next;
			remove_tag(tag_c, tqueue, tag);
			if (tag1 == NULL)
				break;
			tag = tag1;			
		}

		if ((tag->status & TAG_FRESH)) {
			if (tag->qdac_tag_type == GFORCE_SENSOR_TAG) { // for the g-force sensor tag, make event if g-force > 11
				 if(tag->qdac_gforce > arg) {
					make_qdac_event(tag, &gpsFresh.point, STATUS_QDAC_TAG_IN, now);
					print_time(temp_des, tag->qdac_tag_serial); 
					printf("Tag IN\n");
					tag->qdac_gforce = 0;
				 }
			}
			else {
				make_qdac_event(tag, &gpsFresh.point, STATUS_QDAC_TAG_IN, now);
				print_time(temp_des, tag->qdac_tag_serial); 
				printf("Tag IN\n");
			}	
			tag->status &= ~TAG_FRESH;
			tag->status |= TAG_SENIOR;
		} else if (tag->status & TAG_SENIOR) {
			if (tag->qdac_tag_type == GFORCE_SENSOR_TAG) { // for the g-force sensor tag, make event if g-force > 11
				if (tag->qdac_gforce >  arg)  {
					if(qdac_debug) {
						printf("%u: G-force %d\n", tag->qdac_tag_serial, tag->qdac_gforce);
					}
					make_qdac_event(tag, &gpsFresh.point, STATUS_QDAC_TAG_IN, now);
					print_time(temp_des, tag->qdac_tag_serial); 
					printf("Tag IN\n");
					tag->qdac_gforce = 0;
				}
			}
			else {
				make_qdac_event(tag, &gpsFresh.point, STATUS_QDAC_TAG_IN, now);
				print_time(temp_des, tag->qdac_tag_serial); 
				printf("Tag IN\n");
				}
		} 
	}
}

static void make_qdac_status_event(int status, int firm_ver) {
	time_t now;
	Packet_t up_pkt;
	char reason[80];
	size_t len;
	UInt8	seq = 0;

	now = time(&now);
	
	snprintf(reason, sizeof(reason), "Initialize version %d QDAC board Success", firm_ver);
	len = strlen(reason);
	pktInit(&up_pkt, PKT_CLIENT_DMTSP_FORMAT_3, "%2U%4U%*s%1U", status, (UInt32)now, 
									len, reason, seq); 
	up_pkt.seqPos = 6 + len;
	up_pkt.seqLen = 1;
	evAddEncodedPacket(&up_pkt);
}
static void make_qdac_event(struct QDAC_Tag * tag, GPSPoint_t *coord, uint16_t ev_status, time_t timestamp)
{
	int seq_p;
	Packet_t *ppt = &qdac_event_packet;
	
	memset(ppt->data, 0, sizeof(ppt->data));
	qdac_bigE_encode2Bytes(ppt->data, ev_status);
	qdac_bigE_encode4Bytes(ppt->data + 2, (uint32_t)timestamp);
	gpsPointEncode8(ppt->data + 6, coord);

	// the previous rfid packet takes 4 bytes to record the reader type 
	// now in the qdac it only takes 1 byte, so the rest 3 bytes is unused 
	ppt->data[14] = QDAC_READER;;
	ppt->data[15] = tag->qdac_tag_type;
	if (protocol == PROTRAC_PROTOCOL) {
		ppt->data[16] = 0;
		ppt->data[17] = 0;
		ppt->data[18] = tag->qdac_hub_id;
		ppt->data[19] = tag->qdac_tag_model;
	/* and the tag serial number will start from 21 instead of 20 actually */
		qdac_bigE_encode3Bytes(ppt->data + 21, tag->qdac_tag_serial);
		if(tag->qdac_tag_type == PRESSURE_TAG) {		// insert the data of the pressure into the rssi, battery & temperature column
			seq_p = 24;
		} else {
			ppt->data[24] = (unsigned char)(tag->qdac_rssi & 0xFF);
			ppt->data[25] = tag->qdac_tag_battery;
			seq_p = 26;
		}	
	} else if (protocol == QDAC_PROTOCOL) {
		qdac_bigE_encode2Bytes(ppt->data + 16, tag->qdac_tag_model);
		qdac_bigE_encode4Bytes(ppt->data + 18, tag->qdac_tag_serial);
		ppt->data[22] = tag->qdac_rssi & 0xFF;
		ppt->data[23] = tag->qdac_tag_battery;
		seq_p = 24;
	}
	
//	if(tag->qdac_tag_type == LOW_TEMP_TAG || tag->qdac_tag_type == HIGH_TEMP_TAG) {
//		ppt->data[seq_p] = 0;
//		seq_p += 1;
//	}
	if(tag->qdac_tag_type == GFORCE_SENSOR_TAG) {
		qdac_bigE_encode2Bytes(ppt->data + seq_p, tag->qdac_gforce);
		seq_p += 2;
	} else if (tag->qdac_tag_type == CURRENT_SENSOR_TAG) {
//		if (qdac_debug) {
//			printf("!!Rec timer payload: ");		
//			for(i = 0; i < TIMER_LENGTH; i++) printf("%x ", tag->qdac_timer_payload[i]);
//			printf("\n");
//		}

//		for(i = 0; i < TIMER_LENGTH; i++) {
//			*(ppt->data+6+i) = tag->qdac_timer_payload[i];
//		}
		if (protocol == PROTRAC_PROTOCOL) {
			qdac_bigE_encode4Bytes(ppt->data + 6, tag->qdac_current_4_3);
       		qdac_bigE_encode4Bytes(ppt->data + 10, tag->qdac_current_2_1);
		} else if (protocol == QDAC_PROTOCOL) {
			qdac_bigE_encode4Bytes(ppt->data + seq_p, tag->qdac_current_4_3);
			seq_p += 4;
       		qdac_bigE_encode4Bytes(ppt->data + seq_p, tag->qdac_current_2_1);
			seq_p += 4;
		}
//		if (qdac_debug) {
//			printf("!!Encode timer payload: ");
//			for(i = 0; i < TIMER_LENGTH; i++) printf("%x ", *(ppt->data+6+i));
//			printf("\n");
//		}
	} else if (tag->qdac_tag_type == SWITCH_TAG) {
		qdac_bigE_encode2Bytes(ppt->data + seq_p, tag->qdac_switch);
		seq_p += 2;
	} else if(tag->qdac_tag_type == PRESSURE_TAG) {		// insert the data of the pressure into the rssi, battery & temperature column
			qdac_bigE_encode4Bytes(ppt->data + seq_p, tag->qdac_pressure);
			seq_p += 4;
	} else if (tag->qdac_tag_type == 0 || tag->qdac_tag_type > KNOWN_TAG_TYPE){		// for unknown type of tag or temeperary tag, upload the payload directly
			if (tag->raw_data_length) {
				printf("raw data length %d\n", tag->raw_data_length);
				memcpy(ppt->data + seq_p, tag->raw_data, (tag->raw_data_length)*sizeof(unsigned char));
				seq_p += tag->raw_data_length;
//				free(tag->raw_data);
			}
	}
	
	ppt->priority = PRIORITY_NORMAL;
	if (protocol == PROTRAC_PROTOCOL) 
		ppt->hdrType = PKT_CLIENT_DMTSP_FORMAT_1;
	else if (protocol == QDAC_PROTOCOL)
		ppt->hdrType = PKT_CLIENT_DMTSP_FORMAT_6;
	ppt->dataLen = (seq_p + 1) & 0xFF;  
	ppt->seqLen   = 1;   
	ppt->seqPos   = seq_p & 0xFF;
	
	evAddEncodedPacket(ppt);
}

static void make_qdac_temperature_event(struct qdac_tag_control *tag_c, GPSPoint_t *coord, 
	struct qdac_temp_spec *temp_spec, int alarm_type)
{	
	int seq_p;
	time_t now;
//	struct tm tm1;
//	struct tm *ptm; 
	Packet_t *pkt = &qdac_event_packet;
	static bool trig_alarm_pin = false; 	// indicate that the PIN regarding to the alarm is being trigured.
									// no necessary to tougle the PIN repeatly even though the state of the alarm is not being changed. 
	
	now = time(NULL);
	memset(pkt->data, 0, sizeof(pkt->data));
	qdac_bigE_encode4Bytes(pkt->data + 2, (uint32_t)now);
	gpsPointEncode8(pkt->data + 6, coord);
	
	if (protocol == PROTRAC_PROTOCOL) {
		qdac_bigE_encode2Bytes(pkt->data + 14, temp_spec->qdac_tag_type);
		qdac_bigE_encode3Bytes(pkt->data + 16, temp_spec->qdac_tag_serial);
//		pkt->data[19] = temp_spec->zone_id;
		if(tag_c->primary_id) 
			pkt->data[19] = (unsigned char)(temp_spec->zone_id & 0xFF);
		else
			pkt->data[19] = 0;
		seq_p = 20;
	} else if (protocol == QDAC_PROTOCOL) {
		pkt->data[14] = QDAC_READER;
		pkt->data[15] = temp_spec->qdac_tag_type;
		qdac_bigE_encode2Bytes(pkt->data + 16, temp_spec->qdac_tag_model);
		qdac_bigE_encode4Bytes(pkt->data + 18, temp_spec->qdac_tag_serial);	
		pkt->data[22] = temp_spec->zone_id;
		seq_p = 23;
	}	
	qdac_encode_temperature(temp_spec->cur_high, temp_spec->cur_low, temp_spec->cur_avg, (pkt->data + seq_p));
	seq_p += 6;
	
	if (alarm_type == QDAC_ALARM_NORM) {
		qdac_bigE_encode2Bytes(pkt->data, STATUS_QDAC_TEMPERATURE);
		pkt->priority = PRIORITY_NORMAL;
		print_time("Temperature", temp_spec->qdac_tag_serial);
		printf("Normal\n");
		// toggle the pin 
		if(trig_alarm_pin) {
			system("/usr/local/bin/disalarm");
			trig_alarm_pin = false;
		}
	} else if (alarm_type == QDAC_ALARM_HIGH) {
		qdac_bigE_encode2Bytes(pkt->data, STATUS_QDAC_TEMPERATURE_HIGH);
		pkt->priority = PRIORITY_HIGH;
		print_time("Temperature", temp_spec->qdac_tag_serial);
		printf("High\n");
		// toggle the pin 
		if (!trig_alarm_pin) {
			system("/usr/local/bin/trig_alarm");
			trig_alarm_pin = true;
		}
	} else if (alarm_type == QDAC_ALARM_LOW) {
		//read_temperature_alarm();
		qdac_bigE_encode2Bytes(pkt->data, STATUS_QDAC_TEMPERATURE_LOW);
		pkt->priority = PRIORITY_HIGH;
		print_time("Temperature", temp_spec->qdac_tag_serial);
		printf("Low\n");
		// toggle the pin 
		if (!trig_alarm_pin) {
			system("/usr/local/bin/trig_alarm");
			trig_alarm_pin = true;
		}
	}
	if (protocol == PROTRAC_PROTOCOL) 
		pkt->hdrType = PKT_CLIENT_DMTSP_FORMAT_0;
	else if (protocol == QDAC_PROTOCOL)
		pkt->hdrType = PKT_CLIENT_DMTSP_FORMAT_5;
	pkt->dataLen = (seq_p + 1); 
	pkt->seqLen = 1;    
//	pkt->seqPos   = seq_p & 0xFF;
	pkt->seqPos   = seq_p;
	evAddEncodedPacket(pkt);
}


static void qdac_encode_temperature(short high, short low, short avg, unsigned char *temp)
{
	qdac_bigE_encode2Bytes(temp, high);
	qdac_bigE_encode2Bytes(temp + 2, avg);
	qdac_bigE_encode2Bytes(temp + 4, low);
}

/*
static int qdac_read_temperature_alarm(struct freezer_control *freezer, unsigned char *pzone, int alarm_type)
{
	int i, l, m, mid, hi, lo;
	int n_zones = 0;
	uint32_t zone_mask = 0; 
	int compare;
	
	freezer->alarm_summary = 0;	
	for (i = 0; i < freezer->divisor; i++) {
		zone_mask = 1 << i;
		if (freezer->t_zone[i].tnum > 0) {
			if (!temp_sorted[i].sorted) {
				memcpy(&temp_sorted[i], &freezer->t_zone[i], sizeof(struct temp_record));
				l = freezer->t_zone[i].count - 1;
				quicksort(temp_sorted[i].t, 0, l);
				temp_sorted[i].sorted = true;
			} else
				l = freezer->t_zone[i].count - 1;
			m = (l + 1) >> 1;
			mid = (temp_sorted[i].t[m] << 1) - 500;
			if (mid >= freezer->t_zone[i].high)
				compare = 1;
			else if (mid <= freezer->t_zone[i].low)
				compare = -1;
			else
				compare = 0;
			if ((compare > 0 && alarm_type == FREEZER_ALARM_HIGH) ||
					(compare < 0 && alarm_type == FREEZER_ALARM_LOW)) {
				hi = (temp_sorted[i].t[l] << 1) - 500;	
				lo = (temp_sorted[i].t[0] << 1) - 500;	
				*pzone = (unsigned char)(i & 0xFF);
				EncodeInt16(pzone + 1, hi);
				EncodeInt16(pzone + 3, mid);
				EncodeInt16(pzone + 5, lo);
				pzone += 7;
				reset_freezer_zone(freezer, i);
				n_zones++;
			} 
			if (compare != 0)
				freezer->alarm_summary |= zone_mask;	
		}
	}

	return n_zones;
}
*/

static int gps_trail_next = 0;
static bool gps_trail_length = 0;
static bool gps_trail_valid = false;
static void update_gps_trail(void)
{
	GPS_t * gps_latest;
	gps_latest = gpsGetLastGPS(&gpsFleeting, USHRT_MAX);
	if (gps_latest == (GPS_t *)0) {
		if (gps_trail_valid) {
			gps_trail_next = 0;
			gps_trail_length = 0;
			gps_trail_valid = false;
		}
	} else {
		gpsCopy(&gpsFresh, &gpsFleeting);
		gps_trail_valid = true;
		if (gps_trail_length < GPS_TRAIL_LENGTH)
			gps_trail_length++;
		gps_trail[gps_trail_next++] = gpsFleeting.point;
		if (gps_trail_next == GPS_TRAIL_LENGTH)
			gps_trail_next = 0;
	}
}
static GPSPoint_t * backtrack_gps_trail(int backward_time)
{
	int back_index, index;

	if (!gps_trail_valid)
		return (&gpsFresh.point);
	back_index = backward_time / QDAC_MAIN_PERIOD;
	if (back_index <= 0)
		return (&gpsFresh.point);
	if (back_index > gps_trail_length) {
		if (gps_trail_length < GPS_TRAIL_LENGTH)
			return (&gps_trail[0]);
		else
			return (&gps_trail[gps_trail_next]);
	}
	else { 
		if (gps_trail_next >= back_index)
			index = gps_trail_next - back_index;
		else
			index = gps_trail_next + GPS_TRAIL_LENGTH - back_index;
		return &gps_trail[index];
	}
}

/*
bool am_i_moving(uint32_t speed)
{
	return (gps_trail_valid && (gpsFleeting.speedKPH > speed));
}
static uint32_t rssi_sort[NUM_RSSI];
uint32_t find_median_rssi(struct freezer_control *freezer)
{
	int goal;
	if (freezer->rssi_samples == 0)
		return 0;
	else if (freezer->rssi_samples < NUM_RSSI)
		return freezer->rssi_sample[freezer->rssi_samples - 1];
	else {
		memcpy(rssi_sort, freezer->rssi_sample, NUM_RSSI * sizeof(uint32_t));
		goal = quick_select(rssi_sort, 0, (NUM_RSSI - 1), (NUM_RSSI >> 1));
		return rssi_sort[goal];
	}
}
*/
/*
void sample_rssi(struct freezer_control *freezer, uint32_t rssi) 
{
	freezer->rssi_sample[freezer->rssi_point++] = rssi;
	if (freezer->rssi_point == NUM_RSSI)
		freezer->rssi_point = 0;
	if (freezer->rssi_samples < NUM_RSSI)
		++freezer->rssi_samples;
}
*/
/* indicate thread should stop */
void qdac_stop(void)
{
	qdac_main_running = 0;
	qdac_reader_running = 0;
}
void qdac_port_close(void)
{
	close(QDACCom.read_fd);
}

static int qdac_init_tag_queue(struct qdac_tag_control *tag_c, struct QDAC_Tag *tag_pool1, struct QDAC_Tag *tag_pool2)
{
	int i;

	TAILQ_INIT(&tag_c->qdac_lowtemp_queue);
	TAILQ_INIT(&tag_c->qdac_hightemp_queue);
	TAILQ_INIT(&tag_c->qdac_unknown_queue);
	TAILQ_INIT(&tag_c->qdac_gforcesensor_queue);
	TAILQ_INIT(&tag_c->qdac_currentsensor_queue);
	TAILQ_INIT(&tag_c->qdac_switchsensor_queue);
	TAILQ_INIT(&tag_c->qdac_pressuresensor_queue);
	TAILQ_INIT(&tag_c->qdac_tmp_queue);
	
	for (i = 0; i < NUM_TAGS_PER_POOL; i++) {
		tag_pool1[i].qdac_tag_serial = i;
		TAILQ_INSERT_HEAD(&tag_c->recycle_queue, &tag_pool1[i], link);
	}
	for (i = 0; i < NUM_TAGS_PER_POOL; i++) {
		tag_pool2[i].qdac_tag_serial = i;
		TAILQ_INSERT_HEAD(&tag_c->recycle_queue, &tag_pool2[i], link);
	}
	return 0;
}

static void qdac_init_tag_parameter(struct qdac_tag_control *tag_c)
{
//	uint32_t u0, u1, u0_2, u1_2;
	
//	tag_c->qdac_reader_type = propGetUInt32AtIndex(PROP_RFID_READER_TYPE, 0, 0);
	tag_c->primary_id = propGetUInt32(PROP_QDAC_PRIMARY_ID, 0);
	/* init all the tag type for the future usage */
/***********************************************************/
	if(tag_c->primary_id)
		init_temp_parameter();
	memset(&qdac_low_temp, 0, sizeof(qdac_low_temp));
	init_qdac_lowtemp_parameter(&qdac_low_temp);
	memset(&qdac_high_temp, 0, sizeof(qdac_high_temp));
	init_qdac_hightemp_parameter(&qdac_high_temp);
	if(tag_c->primary_id) {
		qdac_low_temp.report_cycle = qdac_zone.report_cycle;
		qdac_high_temp.report_cycle = qdac_zone.report_cycle;
	}
	
	memset(&qdac_unknown, 0, sizeof(qdac_unknown));
	init_qdac_unknown_parameter(&qdac_unknown);

	memset(&qdac_gforce_sensor, 0, sizeof(qdac_gforce_sensor));
	init_qdac_gforcesensor_parameter(&qdac_gforce_sensor);

	memset(&qdac_current_sensor, 0, sizeof(qdac_current_sensor));
	init_qdac_currentsensor_parameter(&qdac_current_sensor);	

	memset(&qdac_switch_sensor, 0, sizeof(qdac_switch_sensor));
	init_qdac_switchsensor_parameter(&qdac_switch_sensor);	

	memset(&qdac_pressure_sensor, 0, sizeof(qdac_pressure_sensor));
	init_qdac_pressuresensor_parameter(&qdac_pressure_sensor);	
	
	memset(&qdac_tmp, 0, sizeof(struct qdac_tmp_tag));
	init_qdac_tmp_parameter(&qdac_tmp);
}

static void init_temp_parameter(void) // either low or high temperature tag
{
	UInt16 i;
	uint32_t u0, u1;
	uint32_t serial; 
	Key_t tag, temp;
	
	memset(&qdac_zone, 0, sizeof(struct qdac_temp_zone));
	qdac_zone.primary_id = propGetUInt32(PROP_QDAC_PRIMARY_ID, 0);
	qdac_zone.primary_tag_type= propGetUInt32(PROP_QDAC_PRIMARY_ID, 1);
	u0 = propGetUInt32AtIndex(PROP_TEMP_REPORT_INTRVL, 0, 60); 
	if (u0 > 0)
//		qdac_zone.report_cycle = (u0 + QDAC_MAIN_PERIOD - 1) / QDAC_MAIN_PERIOD;
		qdac_zone.report_cycle = u0; 
	else
		qdac_zone.report_cycle = ONE_HOUR;
	qdac_zone.alarm_times = propGetUInt32AtIndex(PROP_TEMP_REPORT_INTRVL, 3, 0);	
	if (qdac_zone.alarm_times > 0) {
		u1 = propGetUInt32AtIndex(PROP_TEMP_REPORT_INTRVL, 1, 60); 
		if (u1 > 0)
			qdac_zone.alarm_cycle = u1; 
//			qdac_zone.alarm_cycle = (u1 + QDAC_MAIN_PERIOD - 1) / QDAC_MAIN_PERIOD;
		else
			qdac_zone.alarm_cycle = ONE_HOUR;
		u1 = propGetUInt32AtIndex(PROP_TEMP_REPORT_INTRVL, 2, 60); 		
//		qdac_zone.alarm_delay = (u1 + QDAC_MAIN_PERIOD) / QDAC_MAIN_PERIOD;
		qdac_zone.alarm_delay = u1;
		if (qdac_zone.alarm_delay > qdac_zone.report_cycle)
			qdac_zone.alarm_delay -= qdac_zone.report_cycle;
		else
			qdac_zone.alarm_delay = 1;
	} else {
			qdac_zone.alarm_cycle = LONG_MAX;
			qdac_zone.alarm_delay = LONG_MAX;
	}
	qdac_zone.low_temp_tag = &qdac_low_temp;
	qdac_zone.high_temp_tag = &qdac_high_temp;
	tag = PROP_QDAC_ZONE_0;
	temp = PROP_TEMP_RANGE_0;
	for (i = 0; i < QDAC_NUM_ZONES; i++) {
		memset(&qdac_zone.qdac_zone_tag[i], 0, sizeof(struct qdac_temp_zone_tag));
		serial = propGetUInt32AtIndex(tag+i, 0, 0); 	
		if (serial) {
			qdac_zone.qdac_zone_tag[i].zone_id = i;
			qdac_zone.qdac_zone_tag[i].qdac_serial = serial;											 // serial
			qdac_zone.qdac_zone_tag[i].qdac_tag_type = propGetUInt32AtIndex(tag+i, 1, 0);				// type
			qdac_zone.qdac_zone_tag[i].qdac_zone_temp_min = propGetUInt32AtIndex(temp+i, 0, 0); 		//min temperature
			qdac_zone.qdac_zone_tag[i].qdac_zone_temp_max = propGetUInt32AtIndex(temp+i, 1, 0); 	//max temperature
			qdac_zone.qdac_zone_tag[i].zone_own_spec = NULL;
//printf("!!%s: tag: %d - zone: %d - type: %d\n", __FUNCTION__, serial, i, qdac_zone.qdac_zone_tag[i].qdac_tag_type );
		}
		else
			continue;
	}
}

void init_qdac_lowtemp_parameter(struct qdac_lowtemp_tag *low_tag) 
{
	uint32_t cycle, t1;

//	low_tag->id_lower = propGetUInt32AtIndex(PROP_RFID_CARGO_ID_RANGE, 0, 1);
//	low_tag->id_upper = propGetUInt32AtIndex(PROP_RFID_CARGO_ID_RANGE, 1, 99);
	cycle = propGetUInt32AtIndex(PROP_QDAC_LOWTEMP_TAG, 0, 10);
	//low_tag->beacon_cycle = (cycle + QDAC_MAIN_PERIOD - 1) / QDAC_MAIN_PERIOD;
	low_tag->beacon_cycle = cycle;
	low_tag->tag_in_time = propGetUInt32AtIndex(PROP_QDAC_LOWTEMP_TAG, 1, 15);
	if (low_tag->tag_in_time < cycle)
		low_tag->tag_in_time = cycle;
	low_tag->tag_out_time = propGetUInt32AtIndex(PROP_QDAC_LOWTEMP_TAG, 2, 15);
	if (low_tag->tag_out_time < cycle * 2)
		low_tag->tag_out_time = cycle * 2;
	t1 = propGetUInt32(PROP_QDAC_LOWTEMP_REPORT_INTRVL, 60);
	if (t1 > 0)
		//low_tag->report_cycle = (t1 + QDAC_MAIN_PERIOD - 1) / QDAC_MAIN_PERIOD;
		low_tag->report_cycle = t1;
	else
		low_tag->report_cycle = LONG_MAX;
	low_tag->qdac_lowtemp_tag_spec = NULL;
//	low_tag->min_rssi = propGetUInt32(PROP_RFID_CARGO_MIN_RSSI, 0);
}

static void init_qdac_hightemp_parameter(struct qdac_hightemp_tag *high_temp) 
{
	uint32_t cycle, t1;

//	high_temp->id_lower = propGetUInt32AtIndex(PROP_RFID_HIGHTEMP_ID_RANGE, 0, 1);
//	high_temp->id_upper = propGetUInt32AtIndex(PROP_RFID_HIGHTEMP_ID_RANGE, 1, 99);
	cycle = propGetUInt32AtIndex(PROP_RFID_HIGHTEMP_ID_RANGE, 2, 10);
//	high_temp->beacon_cycle = (cycle + QDAC_MAIN_PERIOD - 1) / QDAC_MAIN_PERIOD;
	high_temp->beacon_cycle = cycle;
	high_temp->tag_in_time = propGetUInt32AtIndex(PROP_RFID_HIGHTEMP_ID_RANGE, 3, 15);
	if (high_temp->tag_in_time < cycle)
		high_temp->tag_in_time = cycle;
	high_temp->tag_out_time = propGetUInt32AtIndex(PROP_RFID_HIGHTEMP_ID_RANGE, 4, 15);
	if (high_temp->tag_out_time < cycle * 2)
		high_temp->tag_out_time = cycle * 2;
	t1 = propGetUInt32(PROP_RFID_HIGHTEMP_REPORT_INTRVL, 60);
//	high_temp->report_cycle = t1;
	if (t1 > 0)
		high_temp->report_cycle = t1;
//		high_temp->report_cycle = (t1 + QDAC_MAIN_PERIOD - 1) / QDAC_MAIN_PERIOD;
	else
		high_temp->report_cycle = LONG_MAX;
	high_temp->qdac_hightemp_tag_spec = NULL;
}


static void init_qdac_unknown_parameter(struct qdac_unknown_tag *unknown) 
{
	uint32_t t1, cycle;

	cycle = propGetUInt32AtIndex(PROP_QDAC_UNKNOWN_TAG, 0, 10);
	//unknown->beacon_cycle = (cycle + QDAC_MAIN_PERIOD - 1) / QDAC_MAIN_PERIOD;
	unknown->beacon_cycle = cycle; 
	unknown->tag_in_time = propGetUInt32AtIndex(PROP_QDAC_UNKNOWN_TAG, 1, 15);
	if (unknown->tag_in_time < cycle)
		unknown->tag_in_time = cycle;
	unknown->tag_out_time = propGetUInt32AtIndex(PROP_QDAC_UNKNOWN_TAG, 2, 15);
	if (unknown->tag_out_time < cycle * 2)
		unknown->tag_out_time = cycle * 2;

	t1 = propGetUInt32(PROP_QDAC_UNKNOWN_REPORT_INTRVL, 60);
	if (t1 > 0)
		//unknown->report_cycle = (t1 + QDAC_MAIN_PERIOD - 1) / QDAC_MAIN_PERIOD;
		unknown->report_cycle = t1; 
	else
		unknown->report_cycle = LONG_MAX;
}

static void init_qdac_tmp_parameter(struct qdac_tmp_tag *tmp_tag) 
{
	uint32_t t1, cycle;
	
//	for(i=0; i < TMP_TAG_NUMBER; i++) {
	tmp_tag->tmp_tag_type1 = propGetUInt32(PROP_QDAC_TMP_TYPE1, 0);
	tmp_tag->tmp_tag_type2 = propGetUInt32(PROP_QDAC_TMP_TYPE2, 0);
	if (tmp_tag->tmp_tag_type1 || tmp_tag->tmp_tag_type2) {
		cycle = propGetUInt32AtIndex(PROP_QDAC_TMP_TAG, 0, 30);
		tmp_tag->beacon_cycle = cycle;
		tmp_tag->tag_in_time = propGetUInt32AtIndex(PROP_QDAC_TMP_TAG, 1, 45);
		if (tmp_tag->tag_in_time < cycle)
			tmp_tag->tag_in_time = cycle;
		tmp_tag->tag_out_time = propGetUInt32AtIndex(PROP_QDAC_TMP_TAG, 2, 60);
		if (tmp_tag->tag_out_time < cycle * 2)
			tmp_tag->tag_out_time = cycle * 2;

		t1 = propGetUInt32(PROP_QDAC_TMP_REPORT_INTRVL, 30);
		if (t1 > 0)
			//tmp_tag[i].report_cycle = (t1 + QDAC_MAIN_PERIOD - 1) / QDAC_MAIN_PERIOD;
			tmp_tag->report_cycle = t1;
		else
			tmp_tag->report_cycle = LONG_MAX;
//		if (qdac_debug)
//			printf("tmp: type1-%d; type2-%d; beacon %d; tag_in %d; tag_out %d; report_cycle %d\n",
//			tmp_tag->tmp_tag_type1, tmp_tag->tmp_tag_type2, tmp_tag->beacon_cycle, 
//			tmp_tag->tag_in_time, tmp_tag->tag_out_time, tmp_tag->report_cycle);
	}
}

static void init_qdac_gforcesensor_parameter(struct qdac_gforcesensor_tag *gforcesensor_tag) 
{
	uint32_t cycle;

//	low_tag->id_lower = propGetUInt32AtIndex(PROP_RFID_CARGO_ID_RANGE, 0, 1);
//	low_tag->id_upper = propGetUInt32AtIndex(PROP_RFID_CARGO_ID_RANGE, 1, 99);
	cycle = propGetUInt32AtIndex(PROP_QDAC_GFORCESENSOR_TAG, 0, 10);
//	gforcesensor_tag->beacon_cycle = (cycle + QDAC_MAIN_PERIOD - 1) / QDAC_MAIN_PERIOD;
	gforcesensor_tag->tag_in_time = propGetUInt32AtIndex(PROP_QDAC_GFORCESENSOR_TAG, 1, 15);
	if (gforcesensor_tag->tag_in_time < cycle)
		gforcesensor_tag->tag_in_time = cycle;
	gforcesensor_tag->tag_out_time = propGetUInt32AtIndex(PROP_QDAC_GFORCESENSOR_TAG, 2, 15);
	if (gforcesensor_tag->tag_out_time < cycle * 2)
		gforcesensor_tag->tag_out_time = cycle * 2;
	gforcesensor_tag->report_threshold = propGetUInt32(PROP_QDAC_GFORCESENSOR_RPT_THRESHOLD, 11);
}

static void init_qdac_currentsensor_parameter(struct qdac_currentsensor_tag *currentsensor_tag) 
{
	uint32_t cycle, t1;

	cycle = propGetUInt32AtIndex(PROP_QDAC_CURRENTSENSOR_TAG, 0, 10);
	//currentsensor_tag->beacon_cycle = (cycle + QDAC_MAIN_PERIOD - 1) / QDAC_MAIN_PERIOD;
	currentsensor_tag->beacon_cycle = cycle;
	currentsensor_tag->tag_in_time = propGetUInt32AtIndex(PROP_QDAC_CURRENTSENSOR_TAG, 1, 15);
	if (currentsensor_tag->tag_in_time < cycle)
		currentsensor_tag->tag_in_time = cycle;
	currentsensor_tag->tag_out_time = propGetUInt32AtIndex(PROP_QDAC_CURRENTSENSOR_TAG, 2, 15);
	if (currentsensor_tag->tag_out_time < cycle * 2)
		currentsensor_tag->tag_out_time = cycle * 2;
	t1 = propGetUInt32(PROP_QDAC_CURRENTSENSOR_REPORT_INTRVL, 60);
	if (t1 > 0)
//		currentsensor_tag->report_cycle = (t1 + QDAC_MAIN_PERIOD - 1) / QDAC_MAIN_PERIOD;
		currentsensor_tag->report_cycle = t1;
	else
		currentsensor_tag->report_cycle = LONG_MAX;
}

static void init_qdac_switchsensor_parameter(struct qdac_switch_tag *switch_tag) 
{
	uint32_t cycle, t1;

	cycle = propGetUInt32AtIndex(PROP_QDAC_SWITCH_TAG, 0, 10);
	//currentsensor_tag->beacon_cycle = (cycle + QDAC_MAIN_PERIOD - 1) / QDAC_MAIN_PERIOD;
	switch_tag->beacon_cycle = cycle;
	switch_tag->tag_in_time = propGetUInt32AtIndex(PROP_QDAC_SWITCH_TAG, 1, 15);
	if (switch_tag->tag_in_time < cycle)
		switch_tag->tag_in_time = cycle;
	switch_tag->tag_out_time = propGetUInt32AtIndex(PROP_QDAC_SWITCH_TAG, 2, 15);
	if (switch_tag->tag_out_time < cycle * 2)
		switch_tag->tag_out_time = cycle * 2;
	t1 = propGetUInt32(PROP_QDAC_SWITCH_REPORT_INTRVL, 60);
	if (t1 > 0)
//		currentsensor_tag->report_cycle = (t1 + QDAC_MAIN_PERIOD - 1) / QDAC_MAIN_PERIOD;
		switch_tag->report_cycle = t1;
	else
		switch_tag->report_cycle = LONG_MAX;
}

static void init_qdac_pressuresensor_parameter(struct qdac_pressure_tag *pressure_tag) 
{
	uint32_t cycle, t1;

	cycle = propGetUInt32AtIndex(PROP_QDAC_PRESSURE_TAG, 0, 10);
	//currentsensor_tag->beacon_cycle = (cycle + QDAC_MAIN_PERIOD - 1) / QDAC_MAIN_PERIOD;
	pressure_tag->beacon_cycle = cycle;
	pressure_tag->tag_in_time = propGetUInt32AtIndex(PROP_QDAC_PRESSURE_TAG, 1, 15);
	if (pressure_tag->tag_in_time < cycle)
		pressure_tag->tag_in_time = cycle;
	pressure_tag->tag_out_time = propGetUInt32AtIndex(PROP_QDAC_PRESSURE_TAG, 2, 15);
	if (pressure_tag->tag_out_time < cycle * 2)
		pressure_tag->tag_out_time = cycle * 2;
	t1 = propGetUInt32(PROP_QDAC_PRESSURE_REPORT_INTRVL, 60);
	if (t1 > 0)
//		currentsensor_tag->report_cycle = (t1 + QDAC_MAIN_PERIOD - 1) / QDAC_MAIN_PERIOD;
		pressure_tag->report_cycle = t1;
	else
		pressure_tag->report_cycle = LONG_MAX;
}
/*
int quickpartition(uint32_t *A, int b, int e)
{
	uint32_t X;
	int i, j, m;
	if (b >= e)
		return b;
	m = A[e];
	i = b;
	for (j = b; j < e; j++) {
		if (A[j] <= m) {
			if (j > i) {
				X = A[i];
				A[i] = A[j];
				A[j] = X;
			}
			++i;
		}
	}
	if (i < e) {
		X = A[e];
		A[e] = A[i];
		A[i] = X;
	}
	return i;
}
void quicksort(uint32_t *A, int b, int e)
{
	int pivot;
	if (b < e) {
		pivot = quickpartition(A, b, e);
		if (pivot > b)
			quicksort(A, b,  (pivot - 1));
		if (pivot < e)
			quicksort(A, (pivot + 1), e);
	}
}
int quick_select(uint32_t *A, int b, int e, int rank)
{
	int pivot;
	if (b == e)
		return b; 
	pivot = quickpartition(A, b, e);
	if (pivot == rank)
		return pivot;
	else if (pivot > rank)
		return quick_select(A, b, (pivot - 1), rank);
	else
		return quick_select(A, (pivot + 1), e, rank);
}
*/
static int power_off_qdac(void) {
	if(qdac_power == POWER_ON) {
		if(system("echo high > /sys/aatsi_devs_ctrl/qdac") < 0) {
			printf("Power off QDAC hub failed\n");
			return -1;
		} else
		qdac_power = POWER_OFF;
	}
	sleep(2);
	return 1;
}
static int power_on_qdac(void) {	
	if(qdac_power == POWER_OFF) {
		if(system("echo low > /sys/aatsi_devs_ctrl/qdac") < 0) {
			printf("Power up QDAC hub failed\n");
			return -1;
		} else
			qdac_power = POWER_ON;
	}
	return 1;
}



static void print_response(unsigned char *resp, int pkt_length) {
	int i = 0;
	int print_length = pkt_length;
	
	if (qdac_debug) {
		printf("raw data: ");
		//if(resp[2]) print_length = 3 + resp[2] + 2;
		// use the reading length instead of containing length in the pkt
		if(print_length) {
			while (i < print_length)
				printf("%02x ",(char)resp[i++]);
			printf("\n");
		}
	}
}

static void print_command(unsigned char *cmd, int cmd_length) {
	int i = 0;
	
	printf("Command to hub: ");
	while(i < cmd_length)
		printf("%02x ", cmd[i++]);
	printf("\n");
}
static unsigned short qdac_crc16 (unsigned char *data, unsigned int n)
{
	unsigned char hi = 0xFF; /* High byte of CRC */
	unsigned char lo = 0xFF; /* Low byte of CRC */
	unsigned int idx; /* Index into CRC table */
	unsigned int i; /* Index into data */
	for (i = 0; i < n; ++i) {
		idx = lo ^ data[i];
		lo = hi ^ CRC_hi[idx];
		hi = CRC_low[idx];	
	}
	return (hi << 8 | lo);
}

static void qdac_littleE_encode2Bytes(unsigned char *pdest, unsigned short val) {
		*pdest = val;			/* encode low 8 bits */
		*(pdest + 1) = val>>8; 	/* encode high 8 bits */
}

static void qdac_littleE_encode4Bytes(unsigned char *pdest, unsigned int val) {
		*pdest = val;			/* encode least 8 bits */
		*(pdest + 1) = val>>8; 	/* encode the next least 8 bits */
		*(pdest + 2) = val>>16; 	/* encode the next least 8 bits */
		*(pdest + 3) = val>>24; 	/* encode the next least 8 bits */
}

static void qdac_bigE_encode2Bytes(unsigned char *pdest, unsigned short val) {
		*(pdest + 1)= val;			/* encode low 8 bits */
		*pdest = val>>8; 	/* encode high 8 bits */
}

static void qdac_bigE_encode4Bytes(unsigned char *pdest, unsigned int val) {
		*(pdest + 3) = val;			/* encode least 8 bits */
		*(pdest + 2) = val>>8; 	/* encode the next least 8 bits */
		*(pdest + 1) = val>>16; 	/* encode the next least 8 bits */
		*pdest = val>>24; 	/* encode the next least 8 bits */
}

static void qdac_bigE_encode3Bytes(unsigned char *pdest, unsigned int val) {
		*(pdest + 2) = val;			/* encode least 8 bits */
		*(pdest + 1) = val>>8; 	/* encode the next least 8 bits */
		*pdest = val>>16; 	/* encode the next least 8 bits */
//		*pdest = val>>24; 	/* encode the next least 8 bits */
}

static int tx_cmd_to_hub(unsigned char *cmd) {
	unsigned int crc_length = 0;
	unsigned int command_length = 0;
	int index = 0;
	unsigned short crc_val = 0;
	int n = -1;
	int write_fd;
	fd_set qdac_write_fd_set;
	struct timeval timeout;  /* Timeout for select */
	
	crc_length = cmd[2] + 2;
	index = crc_length + 1;
	crc_val = qdac_crc16(&(cmd[1]), crc_length);
	command_length = crc_length + 3;
	qdac_littleE_encode2Bytes(&(cmd[index]), crc_val);

	FD_ZERO(&qdac_write_fd_set);
	FD_SET(QDACCom.read_fd,&qdac_write_fd_set);

	timeout.tv_sec = 1;
	timeout.tv_usec = 0;
	
	write_fd = select(FD_SETSIZE , (fd_set *) 0,  &qdac_write_fd_set, 
	  (fd_set *) 0, &timeout);
	if (write_fd < 0) {
		perror("select");
		exit(EXIT_FAILURE);
	} else if (write_fd > 0) {
//	print_command(cmd, command_length);
		n = write(QDACCom.read_fd, cmd, command_length);
		return n;
	} else {
		printf("Write to QDAC Port Timeout\n");
		return n;
	}
}

static unsigned char *read_hub_buff(int *actur_rd_len, unsigned int expect_rd_length) {
	fd_set qdac_read_fd_set;
	struct timeval timeout;  /* Timeout for select */
	int read_fd;
	unsigned char *response = NULL; 

	/* FD_ZERO() clears out the fd_set called "qdac_read_fd_set", so that
	it doesn't contain any file descriptors. */
	FD_ZERO(&qdac_read_fd_set);
	/* FD_SET() adds the file descriptor "read_fd" to the fd_set,
	so that select() will return if data available on the i/o,
	which means you can read it, etc. */
	FD_SET(QDACCom.read_fd,&qdac_read_fd_set);

	timeout.tv_sec = 10;
	timeout.tv_usec = 0;
	
	read_fd = select(FD_SETSIZE , &qdac_read_fd_set, (fd_set *) 0, 
			  (fd_set *) 0, &timeout);
		
	/* select() returns the number of sockets that had
	things going on with them -- i.e. they're readable. */
			
	/* Once select() returns, the original fd_set has been
	modified so it now reflects the state of why select()
	woke up. i.e. If file descriptor 4 was originally in
	the fd_set, and then it became readable, the fd_set
	contains file descriptor 4 in it. */
		
	if (read_fd < 0) {
		perror("select");
		exit(EXIT_FAILURE);
	} else if (read_fd > 0) {
		if(FD_ISSET(QDACCom.read_fd, &qdac_read_fd_set)) {
			response = (unsigned char *)malloc(expect_rd_length*sizeof(unsigned char));
			if (response == NULL) {
				perror("rx_resp_from_hub() malloc");
				exit(-1);
			}
			else {
				memset(response, 0, expect_rd_length*sizeof(unsigned char));
				if((*actur_rd_len = read(QDACCom.read_fd, response, expect_rd_length))< 0) {
					free(response);
					response = NULL;
					*actur_rd_len = 0;
				}
/* for test: inputting qdac packet from the debug port */ 
				if(qdac_test == DEBUG_PORT) {
					unsigned char *resp = NULL;
					if (*(response+*actur_rd_len) == 13)
						*(response+*actur_rd_len)  = 0;
					resp = char_to_hex(response, actur_rd_len);
					free(response);
					response = NULL;
					return resp;
				}	
/*------*/				
			}
		}
	} 
	else {
		*actur_rd_len = 0;
//		printf("Nothing Read From QDAC Port\n");
	}
	return response;
}

static unsigned char *rx_resp_from_hub(int *read_length, int expect_rd_length,  int fun_code) {
	unsigned char *response = NULL;

	response = read_hub_buff(read_length, RESPONSE_MAX_SIZE);

	return response;
}

static int set_hub_mode(int mode) {
	unsigned char *response = NULL;
	int read_length = 0;
	int res = -1;
	memset(command, 0, sizeof(command));	
	command[0] = 0xAA; 
	command[1] = WRITE_HUB_CONFIGUATION;
	command[2] = 7;

	switch(mode) {
		case PROMISCUOUS_MODE:	{
			command[3] = WRITE_HUB_CONFIGUATION;
			qdac_littleE_encode2Bytes(&(command[4]), 0);
			qdac_littleE_encode2Bytes(&(command[6]), 0);
			qdac_littleE_encode2Bytes(&(command[8]), 0);
			break;
			}
		default:
			break;
		}

	if(tx_cmd_to_hub(command) < 0)
		return -1;
	response = rx_resp_from_hub(&read_length, HUB_MODE_SET_RESP_LENGTH*sizeof(unsigned char), WRITE_HUB_CONFIGUATION);
	if (response == NULL) {
		return -1;
	}
//	print_response(response, read_length);
	if(qdac_verify_tag_packet(&response, read_length, WRITE_HUB_CONFIGUATION) == 0) {
		if (response[1] == 0x82)
			printf("Set Mode Error with Code %d\n", response[3]);
		else if (response[1] == 0x2 && response[2] == 0x0) {
				printf("Set Mode Success\n");
				res = 1;
		} 
	}
	
	free(response);
	return res;
}

static int read_hub_version(void) {
	unsigned char *rsp = NULL;
	int read_length;
	int qdac_version = 0;
	memset(command, 0, sizeof(command));
	command[0] = 0xAA;
	command[1] = GET_HUB_VERSION;
	command[2] = 0;

	tx_cmd_to_hub(command);
	rsp = rx_resp_from_hub(&read_length, READ_HUB_VERSION_RESP_LENGTH*sizeof(unsigned char),GET_HUB_VERSION);
	if (rsp == NULL) {
		return -1;
	}
//	print_response(rsp, read_length);
	if(qdac_verify_tag_packet(&rsp, read_length, GET_HUB_VERSION) == 0)  {	
		qdac_version = rsp[11] << 8 | rsp[10];
		free(rsp);
	}
		
	return qdac_version;
}

static int stream_mode(unsigned int mode_switch) {
	unsigned char *response = NULL;
	int read_length;
	int res;
	memset(command, 0, sizeof(command));
	command[0] = 0xAA;
	command[1] = ENABLE_HUB_STREAM;
	command[2] = 1;
	command[3] = mode_switch;

	if(tx_cmd_to_hub(command) < 0)
		return -1;
	response = rx_resp_from_hub(&read_length, STREAM_MODE_RESP_LENGTH*sizeof(unsigned char), ENABLE_HUB_STREAM);
	if (response == NULL)
		return -1;
//	print_response(response, read_length);
	if(qdac_verify_tag_packet(&response, read_length, ENABLE_HUB_STREAM) == 0) {	
		if (response[1] == 0xb && response[2] == 0x1) {
			printf("Stream Mode %s\n", response[3] ? "ON":"OFF");
			res = 1;
		}
	} else 
		res = -1;
	
	free(response);
	return res;
}

static void qdac_hub_init(void) {
	int qdac_waiting = propGetUInt32AtIndex(PROP_QDAC_INIT_WAIT_SEC, 0, 6);;
	bool qdac_init = false;
	int qdac_version;
	char *hub_version = (char *)malloc(QDAC_VERSION_LENGTH*sizeof(char));
	int reset_wait = propGetUInt32AtIndex(PROP_QDAC_RESET_MIN, 0, 3); 

	if (hub_version == NULL) {
		perror("malloc hub_version");
		exit(-1);
	}
	if(qdac_waiting < 6);
			qdac_waiting = 6;
	if (!reset_wait)
		reset_wait = 3;
	
	do {
		power_off_qdac();
		power_on_qdac();
		sleep(qdac_waiting);
		/* open tty port */	
		if (open_serial_binary(&QDACCom) < 0) {
			printf("Cannot Open Serial Port for QDAC Reader! Retry...\n");
			goto init_fail;
		}

		qdac_args.tty_fd = QDACCom.read_fd;
	
		//	read_hub_version();
		qdac_version = read_hub_version();
		if (qdac_version <= 0) {	
			printf("Read Hub Version failed! Retry...\n");
			goto init_fail;
		} else {
			sprintf(hub_version, "QDAC Hub Version %d\n", qdac_version);
			printf("%s", hub_version);
			propInitFromString(PROP_QDAC_FIRMWARE, hub_version);
		}
		/* Set the working mode for the hub */
		if(set_hub_mode(PROMISCUOUS_MODE) < 0) {
			printf("Set Hub Mode failed! Retry...\n");
			goto init_fail;
		}
		/* Enable the streaming model of the hub*/
		if(stream_mode(1) < 0) {
			printf("Enable stream mode failed!Retry...\n");
			goto init_fail;
		}
		
		qdac_init = true;
		make_qdac_status_event(STATUS_QDAC_INIT_SUCCESS, qdac_version);
		break;
		
init_fail:
		make_qdac_status_event(STATUS_QDAC_INIT_FAIL, qdac_version);
		power_off_qdac(); 
		sleep(reset_wait*MINUTES);		// wait for 3 minutes
	}	while(qdac_init == false);
	
		free(hub_version);
		
		qdac_main_running = 1;
		qdac_reader_running = 1;
}

static void qdac_init_gps(void) {
	int n;
	
	gpsClear(&gpsFresh);
	gpsClear(&gpsFleeting);
	for (n = 0; n < GPS_TRAIL_LENGTH; n++)
		gpsPointClear(&gps_trail[n]);
}

static void remove_tag(struct qdac_tag_control *tag_c, struct tq_head *tqueue, struct QDAC_Tag *tag) {
	TAILQ_REMOVE(tqueue, tag, link);
	memset(tag, 0, sizeof(*tag));
	pthread_mutex_lock(&tag_c->mutex_recycle);
	TAILQ_INSERT_TAIL(&tag_c->recycle_queue, tag, link);
	pthread_mutex_unlock(&tag_c->mutex_recycle);
}		

static void print_time(char *des, int serial) {
	time_t now;
	struct tm tm1, *ptm;

	now = time(NULL);
	ptm = localtime_r(&now, &tm1);
	print_debug("%02d/%02d/%4d %02d:%02d:%02d %s %u: ", 
				ptm->tm_mon+1, ptm->tm_mday, ptm->tm_year+1900, 
				ptm->tm_hour, ptm->tm_min, ptm->tm_sec, 
				des, serial); 
}

static void qdac_sample_temperature(struct qdac_temp_spec *temp_spec, struct QDAC_Tag *tag)
{
	int index;

	if (tag->qdac_tag_type != HIGH_TEMP_TAG && tag->qdac_tag_type != LOW_TEMP_TAG)
		return;
	index = temp_spec->temp_index;

	*(temp_spec->qdac_temp+index) = tag->cur_temp;
	if (index == QDAC_NUM_TEMPERATURES - 1)
		temp_spec->temp_index = 0; 
	else
		temp_spec->temp_index += 1;
	if(temp_spec->temp_count < QDAC_NUM_TEMPERATURES)
		temp_spec->temp_count += 1;
}

static struct qdac_temp_spec *search_temp_spec(struct qdac_temp_spec **temp_spec, int tag_serial, int tag_type) {
	struct qdac_temp_spec *tmp = *temp_spec; 
	struct qdac_temp_spec *last = NULL;

	if(tmp == NULL) {
		tmp = create_temp_spec();
		tmp->qdac_tag_serial = tag_serial;
		tmp->qdac_tag_type = tag_type;
		*temp_spec = tmp;
	} 
	else {
		for(;tmp->qdac_temp_spec_next != NULL; tmp=tmp->qdac_temp_spec_next) {
			if(tmp->qdac_tag_serial == tag_serial && tmp->qdac_tag_type == tag_type)  
				return tmp;
		}
		last = tmp;
		tmp = create_temp_spec();
		tmp->qdac_tag_serial = tag_serial;
		tmp->qdac_tag_type = tag_type;
		last->qdac_temp_spec_next = tmp;
		tmp = last;
	}
	
	return tmp;
}

static struct qdac_temp_spec *create_temp_spec(void) {
	struct qdac_temp_spec *tmp = (struct qdac_temp_spec *)malloc(sizeof(struct qdac_temp_spec));
	tmp->qdac_tag_serial = 0;
	tmp->qdac_tag_model = 0;
	tmp->qdac_tag_type = 0;
	tmp->temp_index = 0;
	tmp->temp_count = 0;
	tmp->zone_id = 0xFF;
	tmp->cur_avg = 0;
	tmp->cur_high = 0;
	tmp->cur_low = 0;
	tmp->set_high = 0;
	tmp->set_low = 0;
	tmp->alarm_type = QDAC_ALARM_NORM;
	tmp->qdac_temp = (short *)malloc(QDAC_NUM_TEMPERATURES*sizeof(short));
	tmp->qdac_temp_spec_next = NULL;
//	tmp->qdac_zone_general = NULL;
	if (tmp == NULL) {
		perror("malloc tmp");
		exit(-1);
	}
	if (tmp->qdac_temp == NULL) {
		perror("malloc tmp->qdac_temp");
		exit(-1);
	}
	
	return tmp;
}

static void qdac_temp_cal(struct qdac_temp_spec *temp_spec)
{
	int i;
	
	temp_spec->cur_high = *(temp_spec->qdac_temp);
	temp_spec->cur_low = *(temp_spec->qdac_temp);
	temp_spec->cur_avg = *(temp_spec->qdac_temp);

	if (temp_spec->temp_count <= 1)
		return;
	else {
//	printf("Tag %d Stored: ", temp_spec->qdac_serial);
//	for (i=0; i<temp_spec->temp_count; i++) 
//		printf("%d ", *(temp_spec->qdac_temp+i));
//	printf("\n");
		for (i=1; i<temp_spec->temp_count; i++) {
			if(*(temp_spec->qdac_temp+i) > temp_spec->cur_high) 
				temp_spec->cur_high= *(temp_spec->qdac_temp+i);
			else if(*(temp_spec->qdac_temp+i) < temp_spec->cur_low) 
				temp_spec->cur_low = *(temp_spec->qdac_temp+i);
			temp_spec->cur_avg += *(temp_spec->qdac_temp+i);
		}	
		temp_spec->cur_avg /= temp_spec->temp_count;
	}	
//printf("!!Serial %d: high: %d; low: %d; avg: %d\n", temp_spec->qdac_tag_serial, temp_spec->cur_high,
//		temp_spec->cur_avg, temp_spec->cur_low);
}

static int qdac_cmp_temp(struct qdac_temp_spec *temp_spec) {
	short avg = temp_spec->cur_avg;

	if (avg > temp_spec->set_high) 
		return QDAC_ALARM_HIGH;
	else if (avg < temp_spec->set_low)
		return QDAC_ALARM_LOW;
	else
		return QDAC_ALARM_NORM;
}

static unsigned char *char_to_hex(unsigned char *response, int *actur_rd_len) {
	int i = 0;
	unsigned char *resp = (unsigned char *)malloc(*actur_rd_len*sizeof(unsigned char));
	
	while(i < *actur_rd_len) {
		if (*(response+i) >= '0' && *(response+i) <= '9' )
			*(response+i) -= '0';
		if (*(response+i) >= 'a' && *(response+i) <= 'f' )
			*(response+i) -= 87;
		if (*(response+i) >= 'A' && *(response+i) <= 'F' )
			*(response+i) -= 55;
//			printf("%x ", *(response+i));
		i++;
	}
//	printf("\n");
	*actur_rd_len /= 2;
	for(i=0; i < *actur_rd_len; i++) {
		*(response+i*2) <<= 4;		
		*(resp+i) = (*(response+i*2) | *(response+i*2+1));
//		printf("%x ", *(resp+i));	
	}
//	printf("\n");
	
	return resp;
}

// returns NULL: 	if error, verify_status < 0;
//				if need more data, verify_status > 0; 
// returns the pointer if verify a full pkt success, and verify_status = 0

static unsigned char *verify_recv_queue(struct qdac_recv_pkts *pkts_queue, int *verify_status)
{
	int error = 0;
	int payload_length=0,	 pkt_length=0;
	int verify_res = 0;
	unsigned char *pkt_start = NULL; //pkts_queue->qdac_pkts_queue+pkts_queue->parse_index; 
	unsigned char *pkt_cur = NULL; 
	unsigned char *tag_data = NULL;
	int parse_index = pkts_queue->parse_index;
	int unparse_length =pkts_queue->queue_unparse_length;
	int pos_dif = 0;
	int pkts_que_rest = 0;
	int pkt_exced = 0;
// start
	pkt_cur = pkts_queue->qdac_pkts_queue+parse_index;

//	printf("%s ", __FUNCTION__);
//	printf("parse_index: %d\n", parse_index);
	pkts_que_rest = PKTS_QUEUE_LENGTH - parse_index;
	if(pkts_que_rest >= unparse_length)
		pkt_start = find_out_start_byte (pkt_cur, &error, unparse_length);
	else {
		pkt_start = find_out_start_byte (pkt_cur, &error, pkts_que_rest);
		if (!pkt_start) {
			move_parse_index(pkts_queue, pkts_que_rest);
			parse_index = pkts_queue->parse_index;
//			printf("parse_index: %d\n", parse_index);
			pkt_cur = pkts_queue->qdac_pkts_queue+parse_index;
			unparse_length =pkts_queue->queue_unparse_length;
			pkt_start = find_out_start_byte (pkt_cur, &error, unparse_length);
		}
	}	
	if (pkt_start) {
		pos_dif = pkt_start-pkt_cur;
		if (pos_dif) {							// move n bytes to the 0xAA
			move_parse_index(pkts_queue, pos_dif);
			unparse_length -= pos_dif;
		}
	} 
	else if (pkt_start == NULL) {
		move_parse_index(pkts_queue, unparse_length);
//		unparse_length = 0;
		*verify_status = error;
		return NULL;
	}
	
	payload_length = *(pkt_start+2); 							// found out a expected complete pkt length
	pkt_length = payload_length + 5;

	if (unparse_length >= pkt_length) {
		tag_data = (unsigned char *)malloc(sizeof(unsigned char)*pkt_length);
		if(tag_data == NULL)
			perror("malloc tag_data");
		memset(tag_data, 0, pkt_length);
// cpy the expected complete pkt to be ready to handle it		
		parse_index = pkts_queue->parse_index;
		pkt_exced = parse_index + pkt_length - PKTS_QUEUE_LENGTH;
		if(pkt_exced <= 0)
			memcpy(tag_data, pkt_start, pkt_length);
		else {
			memcpy(tag_data, pkt_start, pkt_length - pkt_exced);
			memcpy(tag_data+pkt_length-pkt_exced, pkts_queue->qdac_pkts_queue, pkt_exced);	
		}
		
		verify_res = qdac_verify_tag_packet(&tag_data, pkt_length, READ_TAG_DATA); 
	} else 
		verify_res = pkt_length - unparse_length;
	
//printf("verify_res %d\n", verify_res);

	if (verify_res == VERIFY_SUCCESS)  {
		error = 0;	
		move_parse_index(pkts_queue, pkt_length);
		*verify_status = verify_res;
	} else if (verify_res < 0){					// cause verify_res is a minus value, so to count the errors, we have to minus it.
		error += verify_res;		 
		if(tag_data != NULL) free(tag_data);
		move_parse_index(pkts_queue, 1);
		*verify_status = error;
		tag_data = NULL;
	} else if (verify_res > 0) {					// if returns >0, it means more bytes need to be read to assemble as a full pkt.
//		printf("waiting for more data\n");
		*verify_status = verify_res;
		if(tag_data != NULL) free(tag_data);
		tag_data = NULL;
	}

	return tag_data;
}

static unsigned char *find_out_start_byte(unsigned char *pkt, int *parse_error, int queue_length) {
	int index = 0;
	
	if(*pkt != 0xAA)	 {				//  1. no AA
		*parse_error = -1;
		printf("!!0xAA not as starter error\n");
	}
	// searching for 0xAA
	
	while(index < queue_length) {
		 if (*(pkt+index) == 0xAA) {
			return pkt+index;
		 }
		 index++;	
	}
	printf("!!No 0xAA error\n");
	return NULL;
}

static void move_parse_index(struct qdac_recv_pkts *pkts_queue, int pkt_length) {
	int parse_index = pkts_queue->parse_index;
	int pkt_rest;
	unsigned char *queue = pkts_queue->qdac_pkts_queue;

//	pkt_rest = PKTS_QUEUE_LENGTH - pkt_length;
	pkt_rest = PKTS_QUEUE_LENGTH - parse_index;
	if (pkt_rest < pkt_length) {
		memset(queue+parse_index, 0, pkt_rest*sizeof(unsigned char));
		memset(queue, 0, (pkt_length-pkt_rest)*sizeof(unsigned char));
		pkts_queue->parse_index = pkt_length-pkt_rest;
	} else {
		memset(queue+parse_index, 0, pkt_length*sizeof(unsigned char));
		pkts_queue->parse_index += pkt_length;
		if (pkts_queue->parse_index == PKTS_QUEUE_LENGTH)
		pkts_queue->parse_index = 0;
	}
	pkts_queue->queue_unparse_length -= pkt_length;
}

