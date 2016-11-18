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
#include "rfid.h"
#include "reader_type.h"
#define RFID_MAIN_PERIOD 10 
#define RFID_IF_TIMEOUT 3600
#define ONE_HOUR 360
#define RFID_FUNCTION_ENABLE 0x80000000
#define NUM_ZONES 8
#define NUM_TEMPERATURES 10
#define NUM_RSSI 8
#define RAW_READ_SIZE 32
#define MAX_READ_LEN 24
#define TAG_PACKET_SIZE_MAX 24
#define TAG_PACKET_SIZE_19 19
#define TAG_PACKET_SIZE_20 20
#define TAG_PACKET_SIZE_HALF 10
#define TAG_FRESH 1
#define TAG_SENIOR 2
#define TAG_BURSTING 0x40
#define TAG_BURSTED 0x80
#define TAG_SWITCH_CLOSED 0x10
#define TAG_MOTION_OPEN 0x10
#define TAG_STATE_BURST 0xC0
#define LOCK_CONTACT_A 0x80
#define LOCK_CONTACT_B 0x20
#define LOCK_FLAG_ALARM LOCK_CONTACT_A
#define LOCK_FLAG_ARM (LOCK_CONTACT_A | LOCK_CONTACT_B)
#define NUM_TAGS_PER_POOL 128
#define MAX_TEMPERATURE_RECORDS 8
#define STATUS_RFID_TAG_IN 0xF500
#define STATUS_RFID_TAG_OUT 0xF501
#define STATUS_RFID_BATTERY_ALERT 0xF503
#define STATUS_RFID_PRIMARY_IN 0xF504
#define STATUS_RFID_PRIMARY_OUT 0xF505
#define STATUS_RFID_LOCK_DISABLED 0xF506
#define STATUS_RFID_LOCK_PREARMED 0xF507
#define STATUS_RFID_LOCK_ARMED 0xF508
#define STATUS_RFID_LOCK_OUT 0xF509
#define STATUS_RFID_LOCK_ALARM 0xF502
#define STATUS_RFID_SWITCH_CLOSED 0xF50A
#define STATUS_RFID_SWITCH_OPEN 0xF50B
#define STATUS_RFID_SWITCH_OUT 0xF50C
#define STATUS_RFID_HIGHTEMP 0xF50D
#define STATUS_RFID_MOTION_CLOSED 0xF50E
#define STATUS_RFID_MOTION_OPEN 0xF50F
#define STATUS_RFID_SENSOR_STATUS 0xF510

#define CARGO_SAMPLE_IN_MOTION 1
#define FREEZER_ALARM_LOW 1
#define FREEZER_ALARM_HIGH 2
#define ROLE_FREEZER 1
#define ROLE_LOCKER 2
#define ROLE_CARGO 4
#define ROLE_SWITCH 8
#define ROLE_HIGHTEMP 16
#define ROLE_MOTION 32
#define ROLE_SENSOR 64
#define ROLE_HUMIDITY 128
#define	GPS_TRAIL_LENGTH 32 
#define	RSSI_BITS 5 
#define	RSSI_TOTAL (1 << RSSI_BITS)
#define	RSSI_INDEX_M (RSSI_TOTAL - 1)
#define	RSSI_WORDS  (RSSI_TOTAL >> 2)

pthread_t thread_rfid_main;
pthread_t thread_rfid_reader;
extern bool isInMotion2;
bool rfid_queue_time_adjust = false;

struct Tag {
	TAILQ_ENTRY(Tag) link;
	uint32_t tnum;
	time_t recent;
	long nsec;
	uint32_t status;
	uint32_t rssi;
	uint16_t temp;
	uint8_t battery;
	uint8_t flag;
	uint8_t humidity;
};

TAILQ_HEAD(tq_head, Tag) tag_queue_1;

struct tag_control {
	uint32_t tnum_min;
	uint32_t tnum_max;
	uint32_t role;
	pthread_mutex_t mutex_freezer;
	pthread_mutex_t mutex_locker;
	pthread_mutex_t mutex_cargo;
	pthread_mutex_t mutex_switch;
	pthread_mutex_t mutex_recycle;
	pthread_mutex_t mutex_hightemp;
	pthread_mutex_t mutex_motion;
	pthread_mutex_t mutex_sensor;
	pthread_mutex_t mutex_humidity;
	struct tq_head freezer_queue;
	struct tq_head locker_queue;
	struct tq_head cargo_queue;
	struct tq_head switch_queue;
	struct tq_head recycle_queue;
	struct tq_head hightemp_queue;
	struct tq_head motion_queue;
	struct tq_head sensor_queue;
	struct tq_head humidity_queue;
	uint32_t reader_type;
	uint32_t battery_maximum;
	uint32_t battery_alarm_cycle;
	uint16_t reader_id;
	uint16_t customer_id;
};
struct rfid_thread_args {
	int tty_fd;
	struct tag_control * tag_c;
};
struct temp_record {
	uint32_t tnum;
	uint32_t count;
	uint32_t next;
	int high;
	int low;
	bool sorted;
	uint32_t t[NUM_TEMPERATURES];
};
struct freezer_control {
	uint32_t id_lower;
	uint32_t id_upper;
	int tag_in_time;
	int tag_out_time;
	uint32_t beacon_cycle;
	uint32_t primary_id;
	uint32_t min_rssi;
	uint32_t divisor;
	uint32_t fading_duration;
	bool	is_zone0;
	bool	permanently;
	uint32_t alarm_summary;
	uint32_t alarm_count;
	uint32_t report_cycle;
	uint32_t alarm_cycle;
	uint32_t alarm_delay;
	uint32_t alarm_times;
	bool	latched;
	uint32_t rssi_point;
	uint32_t rssi_delta;
	uint32_t rssi_samples;
	uint32_t fading_count;
	uint32_t rssi_sample[NUM_RSSI];
	struct temp_record t_zone[NUM_ZONES];
};
struct lock_control {
	uint32_t id_lower;
	uint32_t id_upper;
	int tag_in_time;
	int tag_out_time;
	uint32_t beacon_cycle;
	uint32_t armed_cycle;
	uint32_t alarm_cycle;
	bool non_exclusive;
};
struct cargo_control {
	uint32_t id_lower;
	uint32_t id_upper;
	int tag_in_time;
	int tag_out_time;
	uint32_t min_rssi;
	uint32_t beacon_cycle;
	uint32_t report_cycle;
	uint32_t sample_mode;
	uint32_t min_speed;
	uint32_t motion_duration;
};
struct switch_control {
	uint32_t id_lower;
	uint32_t id_upper;
	int tag_in_time;
	int tag_out_time;
	uint32_t beacon_cycle;
	uint32_t open_report_cycle;
	uint32_t closed_report_cycle;
};

struct motion_control {
	uint32_t id_lower;
	uint32_t id_upper;
	int tag_in_time;
	int tag_out_time;
	uint32_t beacon_cycle;
	uint32_t alarm_report_rate;
//	uint32_t closed_report_cycle;
};

struct high_temp_control {
	uint32_t id_lower;
	uint32_t id_upper;
	uint32_t id_lower_2;
	uint32_t id_upper_2;
	int tag_in_time;
	int tag_out_time;
	uint32_t min_rssi;
	uint32_t beacon_cycle;
	uint32_t report_cycle;
};

struct sensor_control {
	uint32_t id_lower;
	uint32_t id_upper;
	int tag_in_time;
	int tag_out_time;
	uint32_t beacon_cycle;
	uint32_t report_cycle;
//	uint32_t closed_report_cycle;
};

struct humidity_control {
	uint32_t id_lower;
	uint32_t id_upper;
	int tag_in_time;
	int tag_out_time;
	uint32_t beacon_cycle;
	uint32_t report_cycle;
//	uint32_t closed_report_cycle;
};

static struct tag_control tag_control_1 = {.tnum_max = 0, .role = 0,
			.reader_id = 0, 
			.customer_id = 0,
			.mutex_freezer = PTHREAD_MUTEX_INITIALIZER, 
			.mutex_locker = PTHREAD_MUTEX_INITIALIZER, 
			.mutex_cargo = PTHREAD_MUTEX_INITIALIZER, 
			.mutex_switch = PTHREAD_MUTEX_INITIALIZER, 
			.mutex_hightemp = PTHREAD_MUTEX_INITIALIZER, 
			.mutex_motion = PTHREAD_MUTEX_INITIALIZER,
			.mutex_sensor = PTHREAD_MUTEX_INITIALIZER,
			.mutex_recycle = PTHREAD_MUTEX_INITIALIZER,
			.mutex_humidity = PTHREAD_MUTEX_INITIALIZER,};

static struct rfid_thread_args rfid_args =  {0, &tag_control_1};
static struct freezer_control freezer1;
static struct lock_control locker1;
static struct cargo_control truck1;
static struct switch_control switch1;
static struct high_temp_control high_temp;
static struct motion_control motion;
static struct sensor_control sensor;
static struct humidity_control humidity;

static struct temp_record temp_sorted[NUM_ZONES];
static ComPort_t rfidCom = {.name = "ttyS2", .bps = 115200, .read_len = TAG_PACKET_SIZE_19,};
static GPS_t gpsFresh;
static GPS_t gpsFleeting;
static GPSPoint_t gps_trail[GPS_TRAIL_LENGTH];
static Packet_t rfid_event_packet;
static struct Tag * tag_pool1;
static struct Tag * tag_pool2;
static int rfid_reader_running = 1;
static int rfid_main_running = 1;
static unsigned char raw_buf[RAW_READ_SIZE * 2];

static int HighTemp_tag_processing(struct Tag *high_temp_tag);
void * rfid_thread_main(void * thread_args);
void * rfid_thread_reader(void * thread_args);
static int verify_tag_packet(unsigned char * pkt, int len, int * pkt_len);
static void parse_tag(struct tag_control *tag_c, unsigned char *pkt);
static bool admit_tag(struct Tag *tag, time_t now, int period);
static bool tag_not_bursting(struct Tag *tag, struct timespec ts1);
static int calculate_crc8(unsigned char * pkt, int len);
static int init_tag_queue(struct tag_control *tag_c, struct Tag *tag_pool1, struct Tag *tag_pool2);
static void freezer_queue_processing(struct tag_control *tag_c, struct freezer_control *freezer);
static int locker_queue_processing(struct tag_control *tag_c, struct lock_control *locker);
static void cargo_queue_processing(struct tag_control *tag_c, struct cargo_control *truck);
static void hightemp_queue_processing(struct tag_control *tag_c, struct high_temp_control *high_temp);
static void switch_queue_processing(struct tag_control *tag_c, struct switch_control *pswitch);
static int motion_queue_processing(struct tag_control *tag_c, struct motion_control *pmotion);
static int sensor_queue_processing(struct tag_control *tag_c, struct sensor_control *psensor);
static int freezer_processing(struct tag_control *tag_c, struct freezer_control *freezer);
static void humidity_queue_processing(struct tag_control *tag_c, struct humidity_control *phumidity);
static void battery_time_checking(struct tag_control *tag_c);
static struct Tag *search_tag(struct tq_head *tqueue, uint32_t tnum);
static void sample_temperature(struct freezer_control *freezer, struct Tag *tag);
static void update_gps_trail(void);
static bool am_i_moving(uint32_t speed);
static void sample_rssi(struct freezer_control *freezer, uint32_t rssi);
static uint32_t find_median_rssi(struct freezer_control *freezer);
static void reset_freezer_rssi_record(struct freezer_control *freezer);
static GPSPoint_t * backtrack_gps_trail(int backward_time);
static void init_tag_parameter(struct tag_control *tag_c);
static void init_freezer_parameter(struct freezer_control *freezer);
static void init_lock_parameter(struct lock_control *locker); 
static void init_cargo_parameter(struct cargo_control *truck);
static void init_hightemp_parameter(struct high_temp_control * high_temp);
static void init_switch_parameter(struct switch_control *p_switch);
static void init_motion_parameter(struct motion_control *p_motion);
static void init_sensor_parameter(struct sensor_control *p_sensor);
static void init_humidity_parameter(struct humidity_control *p_humidity);
static void init_temperature_parameter(struct freezer_control *freezer);
static int make_temperature_event(struct tag_control *tag_c, struct freezer_control *freezer, int alarm_type);
static void make_rfid_event(struct tag_control *tag_c, struct Tag * tag, GPSPoint_t *coord, uint32_t ev_status, time_t timestamp);
static void reset_freezer_zone(struct freezer_control *freezer, int zn);
static void reset_freezer_zones(struct freezer_control *freezer);
static void quicksort(uint32_t *A, int b, int e);
static int quick_select(uint32_t *A, int b, int e, int rank);

/*static int print_uint32(uint32_t *T, int l)
{
	int i;
	for (i = 0; i < l; i++)
		printf("%u ", T[i]);
	printf("\n");
	return i;
} */
int rfid_initialize(void)
{
	const char *rfid_port = propGetString(PROP_RFID_READER_PORT, ""); 
	strncpy(rfidCom.name, rfid_port, PORT_NAME_SIZE);
	rfidCom.bps = propGetUInt32(PROP_RFID_READER_BPS, 115200); 

	if (pthread_create(&thread_rfid_main, NULL, rfid_thread_main, NULL) != 0) {
		printf("Creating Thread Error\n");
		return -1;
	}
	usleep(10000);
	/* open tty port */
	if (open_serial_binary(&rfidCom) < 0) {
		printf("Cannot Open Serial Port for RFID Reader!\n");
		return -1;
	}
	rfid_args.tty_fd = rfidCom.read_fd;
	rfid_main_running = 1;
	rfid_reader_running = 1;

	if (pthread_create(&thread_rfid_reader, NULL, rfid_thread_reader, &rfid_args) != 0) {
		printf("Creating Thread Error\n");
		return -1;
	}
	return 0;
}
void * rfid_thread_main(void *args)
{
	int n, n_alarms;
	
	tag_pool1 = malloc(NUM_TAGS_PER_POOL * sizeof(struct Tag));
	if (tag_pool1 == NULL) {
		perror("Outof Memory");
		return NULL;
	}
	tag_pool2 = malloc(NUM_TAGS_PER_POOL * sizeof(struct Tag));
	if (tag_pool2 == NULL) {
		perror("Outof Memory");
		return NULL;
	}
	memset(tag_pool1, 0, NUM_TAGS_PER_POOL * sizeof(struct Tag));
	memset(tag_pool2, 0, NUM_TAGS_PER_POOL * sizeof(struct Tag));
	init_tag_queue(&tag_control_1, tag_pool1, tag_pool2);
	init_tag_parameter(&tag_control_1);
	gpsClear(&gpsFresh);
	gpsClear(&gpsFleeting);
	for (n = 0; n < GPS_TRAIL_LENGTH; n++)
		gpsPointClear(&gps_trail[n]);
	if (tag_control_1.role & ROLE_FREEZER) {
		memset(&freezer1, 0, sizeof(freezer1));
		init_freezer_parameter(&freezer1);
		init_temperature_parameter(&freezer1);
	}
	if (tag_control_1.role & ROLE_LOCKER) {
		memset(&locker1, 0, sizeof(locker1));
		init_lock_parameter(&locker1);
	}
	if (tag_control_1.role & ROLE_CARGO) {
		memset(&truck1, 0, sizeof(truck1));
		init_cargo_parameter(&truck1);
	}
	if (tag_control_1.role & ROLE_HIGHTEMP) {
		memset(&high_temp, 0, sizeof(high_temp));
		init_hightemp_parameter(&high_temp);
	}
	if (tag_control_1.role & ROLE_SWITCH) {
		memset(&switch1, 0, sizeof(switch1));
		init_switch_parameter(&switch1);
	}
	if (tag_control_1.role & ROLE_MOTION) {
		memset(&motion, 0, sizeof(motion));
		init_motion_parameter(&motion);
	}
	if (tag_control_1.role & ROLE_SENSOR) {
		memset(&sensor, 0, sizeof(sensor));
		init_sensor_parameter(&sensor);
	}
	if (tag_control_1.role & ROLE_HUMIDITY) {
		memset(&humidity, 0, sizeof(humidity));
		init_humidity_parameter(&humidity); 
	}
	printf("The ROLE is %x\n", tag_control_1.role);

	while (rfid_main_running != 0) { 
		update_gps_trail();
		battery_time_checking(&tag_control_1);
		n_alarms = 0;
		if (tag_control_1.role & ROLE_FREEZER) 
			n_alarms = freezer_processing(&tag_control_1, &freezer1);
		if (rfid_queue_time_adjust)
			goto recession;
		if (tag_control_1.role & ROLE_LOCKER)
			n_alarms += locker_queue_processing(&tag_control_1, &locker1);
		if (tag_control_1.role & ROLE_CARGO) 
			cargo_queue_processing(&tag_control_1, &truck1);
		if (tag_control_1.role & ROLE_HIGHTEMP) { 
			hightemp_queue_processing(&tag_control_1, &high_temp);
		}
		if (tag_control_1.role & ROLE_SWITCH)
			switch_queue_processing(&tag_control_1, &switch1);
		if (tag_control_1.role & ROLE_MOTION)
			n_alarms += motion_queue_processing(&tag_control_1, &motion);
		if (tag_control_1.role & ROLE_SENSOR)
			n_alarms += sensor_queue_processing(&tag_control_1, &sensor);
		if (tag_control_1.role & ROLE_HUMIDITY)
			humidity_queue_processing(&tag_control_1, &humidity);
		if (n_alarms > 0)
			protocolStartSession();
/*recession*/
recession:
		sleep(RFID_MAIN_PERIOD);
		if (property_refresh_flag & PROP_REFRESH_TEMPERATURE)
			init_temperature_parameter(&freezer1);	
	}
	free(tag_pool1);
	free(tag_pool2);
		
	return NULL;
}
void * rfid_thread_reader(void * thread_args)
{
	struct rfid_thread_args * args = (struct rfid_thread_args *)thread_args;	
	struct tag_control *tag_c = args->tag_c;
	struct sigevent sev1;
	struct itimerspec it1;
	timer_t timer1;
	int rfid_fd = args->tty_fd;
	int i, len, ilen, rlen, offset;
	unsigned char *pread, *pkt;

	while (tag_c->role == 0 && rfid_reader_running != 0)
		sleep(RFID_MAIN_PERIOD);
	if (rfid_reader_running == 0)
		goto exit_r;
	/*setup timer */
	sev1.sigev_notify = SIGEV_SIGNAL;
	sev1.sigev_signo = SIG_TIMEOUT;
	sev1.sigev_value.sival_int = TIMER_RFID_1;
	i = timer_create(CLOCK_REALTIME, &sev1, &timer1);
	it1.it_interval.tv_sec = RFID_IF_TIMEOUT;
	it1.it_interval.tv_nsec = 0;
	it1.it_value.tv_sec = RFID_IF_TIMEOUT;
	it1.it_value.tv_nsec = 0;
	pkt = raw_buf;
	pread = pkt; 
	len = 0;
	ilen = 0;
	i = 0;
//	rlen = TAG_PACKET_SIZE_19;
	rlen = TAG_PACKET_SIZE_20;
	
	while (rfid_reader_running != 0) { 
		timer_settime(timer1, 0, &it1, NULL);
		if ((len = read(rfid_fd, pread, rlen)) < 0) {
		//	if (errno == EINTR)
			perror("Read RFID reader");
				continue;
		}
		print_tag_raw(pread);
		ilen += len;
		if ((offset = verify_tag_packet(pkt, ilen, &rlen)) >= 0) 
			parse_tag(tag_c, pkt + offset);
		if (offset >= 0 || ilen + rlen >= RAW_READ_SIZE) {
			i ^= 1;
			pkt = i ? raw_buf : (raw_buf + RAW_READ_SIZE);
			pread = pkt;
			ilen = 0;
		}
		else {
			pread += len;
		}
	}
	rfid_reader_running = 0;
exit_r:
	comPortClose(&rfidCom);
	return NULL;
}
int calculate_crc8(unsigned char * pkt, int len)
{
	int i, j; 
	int crc = 0xFF;
	for (i = 0; i < len; i++) {
		crc ^= pkt[i];
		for (j = 0; j < 8; j++) {
			if (crc & 1)
				crc = (crc >> 1) ^ 0x8C;
			else
				crc >>= 1;
		}
	}
	return crc;
}
int verify_tag_packet(unsigned char * pkt, int len, int * rlen)
{
	unsigned char *sop, *cstart;
	int offset, l0, l1, l2, crc;

	cstart = pkt;
	l0 = len;
	l1 = 0;
	while ((sop = memchr(cstart, 0xAA, l0)) != NULL) {
		if (sop[1] >= TAG_PACKET_SIZE_HALF && sop[1] <= TAG_PACKET_SIZE_MAX) {
			l1 = sop[1];
			break;
		}
		else {
			cstart = sop + 1;	
			l0 -= (sop - pkt) + 1; 
			if (l0 <= 0) {
				sop = NULL;
				break;
			}
		}
	}
	if (sop == NULL) {
		*rlen = TAG_PACKET_SIZE_HALF;
		return -1;
	}
	offset = sop - pkt;
	l2 = len - (offset + l1 + 2); 
	if (l2 == 0)
		*rlen = l1 + 2;
	else if (l2 < 0) {
		*rlen = -l2; 
		return -1;
	}
	else
		*rlen = l1 + 2 - l2;
	if (sop[l1 + 1] != 0x44) {
		sop[0] = 0;
		*rlen = TAG_PACKET_SIZE_HALF;
		return -1;
	}
	if ((crc = calculate_crc8(sop, l1 + 1)) == 0)
		return offset;
	else {
		sop[0] = 0;
		return -1;
	}
}

uint32_t monitor_tag = 0;
void parse_tag(struct tag_control *tag_c, unsigned char *pkt) 
{
	uint32_t tnum, tag_type; 
	int max_period;
	struct Tag *tag = NULL; 
	struct tq_head *tqueue;
	pthread_mutex_t *pmutex;
	struct timespec tsnow;
	uint16_t temp = 0xFFFF;
#define TAG_TYPE_FREEZER 1
#define TAG_TYPE_LOCK 2
#define TAG_TYPE_CARGO 4
#define TAG_TYPE_SWITCH 8
#define TAG_TYPE_HIGHTEMP 16
#define TAG_TYPE_MOTION 32
#define TAG_TYPE_SENSOR 64
#define TAG_TYPE_HUMIDITY 128

	tnum = (pkt[7] << 16) + (pkt[8] << 8) + pkt[9];

	if (monitor_tag == tnum)
		printf("Tag %d is read\n", monitor_tag);
	if (tnum > tag_c->tnum_max || tnum < tag_c->tnum_min)
		return;
	if ((tag_c->role & ROLE_FREEZER) && tnum >= freezer1.id_lower && tnum <= freezer1.id_upper) {
		tag_type = TAG_TYPE_FREEZER;
		max_period = freezer1.tag_in_time;
		tqueue = &tag_c->freezer_queue;
		pmutex = &tag_c->mutex_freezer;
	} else if ((tag_c->role & ROLE_LOCKER) && tnum >= locker1.id_lower && tnum <= locker1.id_upper) {
		tag_type = TAG_TYPE_LOCK;
		max_period = locker1.tag_in_time;
		tqueue = &tag_c->locker_queue;
		pmutex = &tag_c->mutex_locker;
	} else if ((tag_c->role & ROLE_CARGO) && tnum >= truck1.id_lower && tnum <= truck1.id_upper) {
		tag_type = TAG_TYPE_CARGO;
		max_period = truck1.tag_in_time;
		tqueue = &tag_c->cargo_queue;
		pmutex = &tag_c->mutex_cargo;
	} else if ((tag_c->role & ROLE_SWITCH) && tnum >= switch1.id_lower && tnum <= switch1.id_upper) {
		tag_type = TAG_TYPE_SWITCH;
		max_period = switch1.tag_in_time;
		tqueue = &tag_c->switch_queue;
		pmutex = &tag_c->mutex_switch;
	} else if ((tag_c->role & ROLE_HIGHTEMP) && 
	((tnum >= high_temp.id_lower && tnum <= high_temp.id_upper) || (tnum >= high_temp.id_lower_2 && tnum <= high_temp.id_upper_2))) {
			tag_type = TAG_TYPE_HIGHTEMP;
			max_period = high_temp.tag_in_time;
			tqueue = &tag_c->hightemp_queue;
			pmutex = &tag_c->mutex_hightemp;
	} else if ((tag_c->role & ROLE_MOTION) && tnum >= motion.id_lower && tnum <= motion.id_upper) {
		tag_type = TAG_TYPE_MOTION;
		max_period = motion.tag_in_time;
		tqueue = &tag_c->motion_queue;
		pmutex = &tag_c->mutex_motion;
	} else if ((tag_c->role & ROLE_SENSOR) && tnum >= sensor.id_lower && tnum <= sensor.id_upper) {
		tag_type = TAG_TYPE_SENSOR;
		max_period = sensor.tag_in_time;
		tqueue = &tag_c->sensor_queue;
		pmutex = &tag_c->mutex_sensor;
	} else if ((tag_c->role & ROLE_HUMIDITY) && tnum >= humidity.id_lower && tnum <= humidity.id_upper) {
		tag_type = TAG_TYPE_HUMIDITY;
		max_period = humidity.tag_in_time;
		tqueue = &tag_c->humidity_queue;
		pmutex = &tag_c->mutex_humidity;
	} 
	else
		return;

	if (tag_c->customer_id == 0) {
		tag_c->customer_id = (pkt[5] << 8) + pkt[6];
		tag_c->reader_id = pkt[2];
	}
	if (clock_gettime(CLOCK_REALTIME, &tsnow) < 0) {
		perror("Realtime clock");
		return;
	}
	pthread_mutex_lock(pmutex);
	tag = search_tag(tqueue, tnum);
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
		tag->tnum = tnum;
		tag->rssi = pkt[4];
		tag->status = 0;
	} else if (!(tag->status & (TAG_FRESH | TAG_SENIOR))) {
		if (admit_tag(tag, tsnow.tv_sec, max_period))
			tag->status |= TAG_FRESH;
	}
	pthread_mutex_unlock(pmutex);
	tag->rssi = pkt[4];
	tag->battery = pkt[12];
	if (pkt[1] == 0x11 || pkt[1] == 0x12)
		temp = (pkt[15] << 8) + pkt[16]; 
	tag->temp = temp;
	tag->flag = pkt[13];
	tag->humidity = pkt[17];
	
//if(tag->tnum == 45000) printf("tag %d flag %x\n", tag->tnum, tag->flag);

	if (tag_type == TAG_TYPE_LOCK) {
		if (tag_not_bursting(tag, tsnow))
			tag->status &= ~TAG_BURSTING;
		else
			tag->status |= (TAG_BURSTING | TAG_BURSTED);
		tag->nsec = tsnow.tv_nsec;
	}
	
/*	if (tag_c->role & ROLE_HIGHTEMP) {
		pthread_mutex_lock(pmutex);
		if (tag->recent < (tsnow.tv_sec - high_temp.report_cycle))
			tag->recent = tsnow.tv_sec;
		pthread_mutex_unlock(pmutex);
	}
	else
*/		tag->recent = tsnow.tv_sec;
}
int init_tag_queue(struct tag_control *tag_c, struct Tag *tag_pool1, struct Tag *tag_pool2)
{
	int i;
	TAILQ_INIT(&tag_c->locker_queue);
	TAILQ_INIT(&tag_c->cargo_queue);
	TAILQ_INIT(&tag_c->switch_queue);
	TAILQ_INIT(&tag_c->recycle_queue);
	TAILQ_INIT(&tag_c->freezer_queue);
	TAILQ_INIT(&tag_c->hightemp_queue);
	TAILQ_INIT(&tag_c->motion_queue);
	TAILQ_INIT(&tag_c->sensor_queue);
	TAILQ_INIT(&tag_c->humidity_queue);

	for (i = 0; i < NUM_TAGS_PER_POOL; i++) {
		tag_pool1[i].tnum = i;
		TAILQ_INSERT_HEAD(&tag_c->recycle_queue, &tag_pool1[i], link);
	}
	for (i = 0; i < NUM_TAGS_PER_POOL; i++) {
		tag_pool2[i].tnum = i;
		TAILQ_INSERT_HEAD(&tag_c->recycle_queue, &tag_pool2[i], link);
	}
	return 0;
}
struct Tag *search_tag(struct tq_head *tqueue, uint32_t tnum)
{
	struct Tag *tag = NULL;
	if (TAILQ_EMPTY(tqueue))
		return tag;
	TAILQ_FOREACH(tag, tqueue, link) {
		if (tag->tnum == tnum)
			break;
	}
	return tag;
}
bool admit_tag(struct Tag *tag, time_t now, int period)
{
	return (now - tag->recent <= period);
}
bool tag_not_bursting(struct Tag *tag, struct timespec ts1)
{
	return ((ts1.tv_sec > tag->recent + 1) || 
			(ts1.tv_sec == tag->recent + 1 && ts1.tv_nsec >= tag->nsec));
}
static uint32_t freezer_beat = 0;

/* freezer high temperature processing algorithm 
The temperature table:
*/
int PT1000_Table [70][2] =
{
	{ 400, -570 },
	{ 410, -521 },
	{ 420, -472 },
	{ 430, -423 },
	{ 440, -374 },
	{ 450, -325 },
	{ 460, -276 },
	{ 470, -227 },
	{ 480, -178 },
	{ 490, -128 },
	{ 500, -79 },
	{ 510, -29 },
	{ 520,  20 },
	{ 530,  70 },
	{ 540, 120 },
	{ 550, 169 },
	{ 560, 219 },
	{ 570, 269 },
	{ 580, 319 },
	{ 590, 369 },
	{ 600, 419 },
	{ 610, 470 },
	{ 620, 520 },
	{ 630, 570 },
	{ 640, 621 },
	{ 650, 671 },
	{ 660, 722 },
	{ 670, 773 },
	{ 680, 823 },
	{ 690, 874 },
	{ 700, 925 },
	{ 710, 976 },
	{ 720, 1027 },
	{ 730, 1079 },
	{ 740, 1130 },
	{ 750, 1181 },
	{ 760, 1232 },
	{ 770, 1284 },
	{ 780, 1336 },
	{ 790, 1387 },
	{ 800, 1439 },
	{ 810, 1491 },
	{ 820, 1543 },
	{ 830, 1595 },
	{ 840, 1647 },
	{ 850, 1699 },
	{ 860, 1751 },
	{ 870, 1803 },
	{ 880, 1856 },
	{ 890, 1908 },
	{ 900, 1961 },
	{ 910, 2014 },
	{ 920, 2066 },
	{ 930, 2119 },
	{ 940, 2172 },
	{ 950, 2225 },
	{ 960, 2278 },
	{ 970, 2332 },
	{ 980, 2385 },
	{ 990, 2438 },
	{ 1000, 2492 },
	{ 1010, 2545 },
	{ 1020, 2599 },
	{ 1030, 2653 },
	{ 0, 0}
};


int HighTemp_tag_processing(struct Tag *high_temp_tag)
{

		int Temperature = high_temp_tag->temp;
		int hex, remain;
		int Temp_low, Temp_high, TempDiff, TempOffset, HighTemp;

        // get the RAW Data from the RX Packet
        if (( Temperature >= 400 ) && ( Temperature <= 1023 ))
        	Temperature -= 400;                  // first entry into the table is ADC of 400
        else
            Temperature = 0;

            hex = Temperature / 10;                             // index into the table
            remain = Temperature % 10;
       	    Temp_low = PT1000_Table[hex][1];		      // load the lower temperature
       	    hex++;
   	        Temp_high = PT1000_Table[hex][1];            // load the upper temperature
            TempDiff = Temp_high - Temp_low;
            TempOffset = TempDiff / 10;
            TempOffset = TempOffset * remain;
            HighTemp = Temp_low + TempOffset;			// Use HighTemp as the Displayed Temperature

			return HighTemp;
}

/** 	put the handling of the high temperature queue processing freezer_queue_high into the freezer_queue_processing()
	to deal with the detected high temperature **/
void freezer_queue_processing(struct tag_control *tag_c, struct freezer_control *freezer)
{
	struct Tag *tag, *tag1;
	struct tq_head *tqueue;
	time_t now, past;
	int zone;
	struct tm tm1; 
	struct tm *ptm;
	struct Tag tag_y;
	uint32_t primary_id = 0, primary_candidate = 0;
	uint32_t first_rssi = 0, second_rssi = 0, mid_rssi = 0;
	bool latched = false;
	bool detached = false;
	bool sample_pulse = false;

	tqueue = &tag_c->freezer_queue;
	if (TAILQ_EMPTY(tqueue))
		return;
	now = time(NULL);
	past = now - freezer->tag_out_time;
	ptm = localtime_r(&now, &tm1);
	if (++freezer_beat == freezer->beacon_cycle) {
		sample_pulse = true; 
		freezer_beat = 0;
	}
	if (freezer->latched)
		primary_id = freezer->primary_id;
	pthread_mutex_lock(&tag_c->mutex_freezer);
	for (tag = tqueue->tqh_first; tag != NULL; tag = tag->link.tqe_next) {
		if (tag->recent < past) {
			if (tag->tnum == primary_id) {
				freezer->latched = false;
				detached = true;
				tag_y = *tag;
			} else if (freezer->latched && (tag->tnum > primary_id && tag->tnum < primary_id + freezer->divisor)) {
				zone = (freezer->is_zone0)? (tag->tnum - primary_id) : (tag->tnum - primary_id - 1);
				reset_freezer_zone(freezer, zone);
			}
			tag1 = tag->link.tqe_next;
			TAILQ_REMOVE(tqueue, tag, link);
			memset(tag, 0, sizeof(*tag));
			pthread_mutex_lock(&tag_c->mutex_recycle);
			TAILQ_INSERT_TAIL(&tag_c->recycle_queue, tag, link);
			pthread_mutex_unlock(&tag_c->mutex_recycle);
			if (tag1 == NULL)
				break;
			tag = tag1;
		} else { /*IN processing */
			if (tag->status & TAG_FRESH) {
				tag->status &= ~TAG_FRESH;
				tag->status |= TAG_SENIOR;
				if (!freezer->latched && freezer->permanently &&
					(tag->tnum == freezer->primary_id) && (tag->rssi >= freezer->min_rssi)) {
					freezer->latched = true;
					latched = true;
					primary_id = tag->tnum;
				}
			} else if (sample_pulse && (tag->status & TAG_SENIOR)) { /*sample pulse*/
				if (!freezer->permanently) {
					if (freezer->latched && tag->tnum == primary_id) {
						mid_rssi = find_median_rssi(freezer);
						if (mid_rssi > freezer->rssi_delta && tag->rssi < mid_rssi - freezer->rssi_delta) {
							if (++freezer->fading_count >= freezer->fading_duration) { 
								freezer->latched = false;
								detached = true;
								tag_y = *tag;
							}
						} else {
							freezer->fading_count = 0;
							sample_rssi(freezer, tag->rssi);
						}
					} else if (!freezer->latched && (tag->tnum % freezer->divisor == 0) &&
															(tag->rssi >= freezer->min_rssi)) {
						if (tag->rssi >= first_rssi) {
							second_rssi = first_rssi; 
							primary_candidate = tag->tnum;
							first_rssi = tag->rssi;
						} else if (tag->rssi >= second_rssi)
							second_rssi = tag->rssi;
					}
				}
				if (freezer->latched && tag->tnum >= primary_id && tag->tnum < primary_id + freezer->divisor)
					sample_temperature(freezer, tag);
			} /*sample pulse*/
		} /*IN processing*/
	} /*for: traverse queue*/
	pthread_mutex_unlock(&tag_c->mutex_freezer);

	if (detached) {
		make_rfid_event(tag_c, &tag_y, backtrack_gps_trail(now - tag_y.recent), STATUS_RFID_PRIMARY_OUT, now);
		print_debug("%02d/%02d/%4d %02d:%02d:%02d Target Unlatched! Former Primary Tag %u\n", 
					ptm->tm_mon+1, ptm->tm_mday, ptm->tm_year+1900, 
					ptm->tm_hour, ptm->tm_min, ptm->tm_sec, 
					primary_id); 
		reset_freezer_zones(freezer);
		if (!freezer->permanently) {
			reset_freezer_rssi_record(freezer);
			freezer->primary_id = 0;
		}
	} 
	else if (!freezer->permanently && !freezer->latched && (primary_candidate != 0)) {
		if (second_rssi == 0 || (first_rssi - second_rssi > freezer->rssi_delta)) {
			freezer->latched = true;
			latched = true;
			primary_id = primary_candidate;
			freezer->primary_id = primary_candidate;
		}
	}
	if (latched) {
		if ((tag = search_tag(&tag_c->freezer_queue, primary_id)) != NULL) {
			make_rfid_event(tag_c, tag, &gpsFresh.point, STATUS_RFID_PRIMARY_IN, now);
			print_debug("%02d/%02d/%4d %02d:%02d:%02d Target latched! Primary Tag %u\n", 
					ptm->tm_mon+1, ptm->tm_mday, ptm->tm_year+1900, 
					ptm->tm_hour, ptm->tm_min, ptm->tm_sec, 
					primary_id);
		}
	}		
}
static uint32_t locker_armed_beat = 0;
static uint32_t locker_alarm_beat = 0;
int locker_queue_processing(struct tag_control *tag_c, struct lock_control *locker)
{
	struct Tag *tag, *tag1;
	struct tq_head *tqueue;
	time_t now, past;
	int n_alarms = 0;
	struct tm tm1; 
	struct tm *ptm;
	bool armed_pulse = false;
	bool alarm_pulse = false;
	bool freshly;

	tqueue = &tag_c->locker_queue;
	if (TAILQ_EMPTY(tqueue))
		return 0;
	now = time(NULL);
	past = now - locker->tag_out_time;

	ptm = localtime_r(&now, &tm1);
	if (++locker_armed_beat == locker->armed_cycle) {
		armed_pulse = true; 
		locker_armed_beat = 0;
	}
	if (++locker_alarm_beat == locker->alarm_cycle) {
		alarm_pulse = true; 
		locker_alarm_beat = 0;
	}

	pthread_mutex_lock(&tag_c->mutex_locker);
	for (tag = tqueue->tqh_first; tag != NULL; tag = tag->link.tqe_next) {
		freshly = false;
		if (tag->recent < past) {
			if ((tag->status & TAG_BURSTED) && !(tag->flag & LOCK_CONTACT_A)) {
				make_rfid_event(tag_c, tag, backtrack_gps_trail(now - tag->recent), STATUS_RFID_LOCK_DISABLED, now);
				print_debug("%02d/%02d/%4d %02d:%02d:%02d Lock Tag Disabled: %u\n", 
						ptm->tm_mon+1, ptm->tm_mday, ptm->tm_year+1900, 
						ptm->tm_hour, ptm->tm_min, ptm->tm_sec, 
						tag->tnum); 
			} else { 
				make_rfid_event(tag_c, tag, backtrack_gps_trail(now - tag->recent), STATUS_RFID_LOCK_OUT, now);
				print_debug("%02d/%02d/%4d %02d:%02d:%02d Lock Tag OUT: %u\n", 
						ptm->tm_mon+1, ptm->tm_mday, ptm->tm_year+1900, 
						ptm->tm_hour, ptm->tm_min, ptm->tm_sec, 
						tag->tnum); 
				n_alarms++;
			}
			tag1 = tag->link.tqe_next;
			TAILQ_REMOVE(tqueue, tag, link);
			memset(tag, 0, sizeof(*tag));
			pthread_mutex_lock(&tag_c->mutex_recycle);
			TAILQ_INSERT_TAIL(&tag_c->recycle_queue, tag, link);
			pthread_mutex_unlock(&tag_c->mutex_recycle);
			if (tag1 == NULL)
				break;
			tag = tag1;
		} else { /*IN processing */
			if (tag->status & TAG_FRESH) {
				if (((tag->status & TAG_BURSTING) && (tag->flag == LOCK_FLAG_ALARM)) ||
				(((tag->status & TAG_STATE_BURST) == TAG_BURSTED) && (tag->flag == LOCK_FLAG_ARM))) {
					make_rfid_event(tag_c, tag, &gpsFresh.point, STATUS_RFID_LOCK_PREARMED, now);
					print_debug("%02d/%02d/%4d %02d:%02d:%02d Lock Tag PREARMED: %u\n", 
							ptm->tm_mon+1, ptm->tm_mday, ptm->tm_year+1900, 
							ptm->tm_hour, ptm->tm_min, ptm->tm_sec, 
							tag->tnum); 
					tag->status &= ~TAG_FRESH;
					tag->status |= TAG_SENIOR;
				} else if (locker->non_exclusive) {
					tag->status &= ~TAG_FRESH;
					tag->status |= TAG_SENIOR;
					freshly = true;
				}
			}
			if (tag->status & TAG_SENIOR) { /*senior*/
				if (tag->status & TAG_BURSTED) {
					if (((tag->status & TAG_BURSTING) && (tag->flag == LOCK_FLAG_ARM)) ||
								(!(tag->status & TAG_BURSTING) && (tag->flag == LOCK_FLAG_ALARM))) {
						make_rfid_event(tag_c, tag, &gpsFresh.point, STATUS_RFID_LOCK_ALARM, now);
						print_debug("%02d/%02d/%4d %02d:%02d:%02d Lock Tag ALARM: %u\n", 
								ptm->tm_mon+1, ptm->tm_mday, ptm->tm_year+1900, 
								ptm->tm_hour, ptm->tm_min, ptm->tm_sec, 
								tag->tnum); 
						n_alarms++;
					} else if (((tag->status & TAG_BURSTING) && (tag->flag == LOCK_FLAG_ALARM)) ||
								(!(tag->status & TAG_BURSTING) && (tag->flag == LOCK_FLAG_ARM))) {
						make_rfid_event(tag_c, tag, &gpsFresh.point, STATUS_RFID_LOCK_PREARMED, now);
						print_debug("%02d/%02d/%4d %02d:%02d:%02d Lock Tag PREARMED: %u\n", 
								ptm->tm_mon+1, ptm->tm_mday, ptm->tm_year+1900, 
								ptm->tm_hour, ptm->tm_min, ptm->tm_sec, 
								tag->tnum); 
					}
				} else { /*non bursting*/
					if ((tag->flag == LOCK_FLAG_ARM) && (armed_pulse | freshly)) {
						make_rfid_event(tag_c, tag, &gpsFresh.point, STATUS_RFID_LOCK_ARMED, now);
						print_debug("%02d/%02d/%4d %02d:%02d:%02d Lock Tag ARMED: %u\n", 
								ptm->tm_mon+1, ptm->tm_mday, ptm->tm_year+1900, 
								ptm->tm_hour, ptm->tm_min, ptm->tm_sec, 
								tag->tnum); 
					} else if ((tag->flag == LOCK_FLAG_ALARM) && (alarm_pulse | freshly)) {
						make_rfid_event(tag_c, tag, &gpsFresh.point, STATUS_RFID_LOCK_ALARM, now);
						print_debug("%02d/%02d/%4d %02d:%02d:%02d Lock Tag ALARM: %u\n", 
								ptm->tm_mon+1, ptm->tm_mday, ptm->tm_year+1900, 
								ptm->tm_hour, ptm->tm_min, ptm->tm_sec, 
								tag->tnum); 
						n_alarms++;
					} 
				} /*non bursting*/
			} /*senior*/
			if ((tag->status & TAG_BURSTED) && (tag->flag & LOCK_CONTACT_A))
				tag->status &= ~TAG_BURSTED;
		} /*IN processing*/
	} /*for: traverse queue*/
	pthread_mutex_unlock(&tag_c->mutex_locker);
	return n_alarms;
}
static uint32_t cargo_report_beat = 0;
static uint32_t cargo_moving_beat = 0;
void cargo_queue_processing(struct tag_control *tag_c, struct cargo_control *truck)
{
	struct Tag *tag, *tag1;
	struct tq_head *tqueue;
	time_t now, past;
	struct tm tm1; 
	struct tm *ptm;
	bool report_pulse = false;
	bool ready_to_sample = true;

	tqueue = &tag_c->cargo_queue;
	if (TAILQ_EMPTY(tqueue))
		return;
	now = time(NULL);
	past = now - truck->tag_out_time;
	ptm = localtime_r(&now, &tm1);
	if (truck->sample_mode == CARGO_SAMPLE_IN_MOTION) {
		if (am_i_moving(truck->min_speed)) {
			if (++cargo_moving_beat < truck->motion_duration)
				ready_to_sample = false;
		} else {
			cargo_moving_beat = 0;
			ready_to_sample = false;
		}
	}
	if (ready_to_sample) {
		if (++cargo_report_beat == truck->report_cycle) {
			report_pulse = true; 
			cargo_report_beat = 0;
		}
	}

	pthread_mutex_lock(&tag_c->mutex_cargo);
	for (tag = tqueue->tqh_first; tag != NULL; tag = tag->link.tqe_next) {
		if (tag->recent < past) {
			if (tag->status & TAG_SENIOR) {
				make_rfid_event(tag_c, tag, backtrack_gps_trail(now - tag->recent), STATUS_RFID_TAG_OUT, now);
				print_debug("%02d/%02d/%4d %02d:%02d:%02d Cargo Tag OUT: %u\n", 
						ptm->tm_mon+1, ptm->tm_mday, ptm->tm_year+1900, 
						ptm->tm_hour, ptm->tm_min, ptm->tm_sec, 
						tag->tnum); 
			} 
			tag1 = tag->link.tqe_next;
			TAILQ_REMOVE(tqueue, tag, link);
			memset(tag, 0, sizeof(*tag));
			pthread_mutex_lock(&tag_c->mutex_recycle);
			TAILQ_INSERT_TAIL(&tag_c->recycle_queue, tag, link);
			pthread_mutex_unlock(&tag_c->mutex_recycle);
			if (tag1 == NULL)
				break;
			tag = tag1;
		} else if (ready_to_sample) { /*ready_to_sample*/
			if ((tag->status & TAG_FRESH) && (tag->rssi >= truck->min_rssi)) {
				make_rfid_event(tag_c, tag, &gpsFresh.point, STATUS_RFID_TAG_IN, now);
				print_debug("%02d/%02d/%4d %02d:%02d:%02d Cargo Tag IN: %u\n", 
						ptm->tm_mon+1, ptm->tm_mday, ptm->tm_year+1900, 
						ptm->tm_hour, ptm->tm_min, ptm->tm_sec, 
						tag->tnum); 
				tag->status &= ~TAG_FRESH;
				tag->status |= TAG_SENIOR;
			} else if (report_pulse && (tag->status & TAG_SENIOR)) { /*senior*/
				make_rfid_event(tag_c, tag, &gpsFresh.point, STATUS_RFID_TAG_IN, now);
				print_debug("%02d/%02d/%4d %02d:%02d:%02d Cargo Tag IN: %u\n", 
						ptm->tm_mon+1, ptm->tm_mday, ptm->tm_year+1900, 
						ptm->tm_hour, ptm->tm_min, ptm->tm_sec, 
						tag->tnum); 
			} /*senior*/
		} /*ready_to_sample*/
	} /*for: traverse queue*/
	pthread_mutex_unlock(&tag_c->mutex_cargo);
}

static uint32_t hightemp_beat = 0;
void hightemp_queue_processing(struct tag_control *tag_c, struct high_temp_control *high_temp)
{
	struct Tag *tag, *tag1;
	struct tq_head *tqueue;
	time_t now, past;
	struct tm tm1; 
	struct tm *ptm;
	bool ready_to_sample = false;

	tqueue = &tag_c->hightemp_queue;
	if (TAILQ_EMPTY(tqueue))
		return;
	now = time(NULL);
	past = now - high_temp->tag_out_time;
//	past = now - high_temp->report_cycle;

	ptm = localtime_r(&now, &tm1);

	if (++hightemp_beat == high_temp->report_cycle) {
		ready_to_sample = true;
	}
	pthread_mutex_lock(&tag_c->mutex_hightemp);

	for (tag = tqueue->tqh_first; tag != NULL; tag = tag->link.tqe_next) {
//		now = time(NULL);
		if (tag->recent < past) {
			if (tag->status & TAG_SENIOR) {
				make_rfid_event(tag_c, tag, backtrack_gps_trail(now - tag->recent), STATUS_RFID_TAG_OUT, now);
				print_debug("%02d/%02d/%4d %02d:%02d:%02d High Temperature Tag OUT: %u\n", 
						ptm->tm_mon+1, ptm->tm_mday, ptm->tm_year+1900, 
						ptm->tm_hour, ptm->tm_min, ptm->tm_sec, 
						tag->tnum); 
			} 
			tag1 = tag->link.tqe_next;
			TAILQ_REMOVE(tqueue, tag, link);
			memset(tag, 0, sizeof(*tag));
			pthread_mutex_lock(&tag_c->mutex_recycle);
			TAILQ_INSERT_TAIL(&tag_c->recycle_queue, tag, link);
			pthread_mutex_unlock(&tag_c->mutex_recycle);
			if (tag1 == NULL)
				break;
			tag = tag1;
//		} else if (ready_to_sample) { /*ready_to_sample*/
//			if ((tag->status & TAG_FRESH) && (tag->rssi >= truck->min_rssi)) {			
		} else if ((tag->status & TAG_FRESH)) {
				make_rfid_event(tag_c, tag, &gpsFresh.point, STATUS_RFID_TAG_IN, now);
				print_debug("%02d/%02d/%4d %02d:%02d:%02d High Temperature Tag IN: %u\n", 
						ptm->tm_mon+1, ptm->tm_mday, ptm->tm_year+1900, 
						ptm->tm_hour, ptm->tm_min, ptm->tm_sec, 
						tag->tnum); 
				tag->status &= ~TAG_FRESH;
				tag->status |= TAG_SENIOR;
		} else if (ready_to_sample && (tag->status & TAG_SENIOR)) { /*senior*/
				make_rfid_event(tag_c, tag, &gpsFresh.point, STATUS_RFID_TAG_IN, now);
				print_debug("%02d/%02d/%4d %02d:%02d:%02d High Temperature Tag IN: %u\n", 
						ptm->tm_mon+1, ptm->tm_mday, ptm->tm_year+1900, 
						ptm->tm_hour, ptm->tm_min, ptm->tm_sec, 
						tag->tnum); 
		} /*senior*/
//		} /*ready_to_sample*/
	}
	pthread_mutex_unlock(&tag_c->mutex_hightemp);	
	if (ready_to_sample) 
		hightemp_beat = 0;
}

static uint32_t switch_open_beat = 0;
static uint32_t switch_closed_beat = 0;
void switch_queue_processing(struct tag_control *tag_c, struct switch_control *pswitch)
{
	struct Tag *tag, *tag1;
	struct tq_head *tqueue;
	int n_alarms = 0;
	time_t now, past;
	struct tm tm1; 
	struct tm *ptm;
	bool open_pulse = false;
	bool closed_pulse= false;

	tqueue = &tag_c->switch_queue;
	if (TAILQ_EMPTY(tqueue))
		return;
	now = time(NULL);
	past = now - pswitch->tag_out_time;
	ptm = localtime_r(&now, &tm1);
	if (++switch_closed_beat == pswitch->closed_report_cycle) {
		closed_pulse = true; 
		switch_closed_beat = 0;
	}
	if (++switch_open_beat == pswitch->open_report_cycle) {
		open_pulse = true; 
		switch_open_beat = 0;
	}
	pthread_mutex_lock(&tag_c->mutex_switch);
	for (tag = tqueue->tqh_first; tag != NULL; tag = tag->link.tqe_next) {
		if (tag->recent < past) {
			if (tag->status & TAG_SENIOR) {
				make_rfid_event(tag_c, tag, backtrack_gps_trail(now - tag->recent), STATUS_RFID_SWITCH_OUT, now);
				print_debug("%02d/%02d/%4d %02d:%02d:%02d Switch Tag OUT: %u\n", 
						ptm->tm_mon+1, ptm->tm_mday, ptm->tm_year+1900, 
						ptm->tm_hour, ptm->tm_min, ptm->tm_sec, 
						tag->tnum); 
				++n_alarms;
			} 
			tag1 = tag->link.tqe_next;
			TAILQ_REMOVE(tqueue, tag, link);
			memset(tag, 0, sizeof(*tag));
			pthread_mutex_lock(&tag_c->mutex_recycle);
			TAILQ_INSERT_TAIL(&tag_c->recycle_queue, tag, link);
			pthread_mutex_unlock(&tag_c->mutex_recycle);
			if (tag1 == NULL)
				break;
			tag = tag1;
		} else {
			if (tag->status & TAG_FRESH) {
				if (tag->flag != 0) {
					make_rfid_event(tag_c, tag, &gpsFresh.point, STATUS_RFID_SWITCH_CLOSED, now);
					print_debug("%02d/%02d/%4d %02d:%02d:%02d Switch Closed: %u\n", 
							ptm->tm_mon+1, ptm->tm_mday, ptm->tm_year+1900, 
							ptm->tm_hour, ptm->tm_min, ptm->tm_sec, 
							tag->tnum); 
					tag->status |= TAG_SWITCH_CLOSED;
				} else {
					make_rfid_event(tag_c, tag, &gpsFresh.point, STATUS_RFID_SWITCH_OPEN, now);
					print_debug("%02d/%02d/%4d %02d:%02d:%02d Switch Open: %u\n", 
							ptm->tm_mon+1, ptm->tm_mday, ptm->tm_year+1900, 
							ptm->tm_hour, ptm->tm_min, ptm->tm_sec, 
							tag->tnum); 
					tag->status &= ~TAG_SWITCH_CLOSED;
				}
				tag->status &= ~TAG_FRESH;
				tag->status |= TAG_SENIOR;
			/*fresh*/
			} else if (tag->status & TAG_SENIOR) {
				if ((tag->flag != 0) && (!(tag->status & TAG_SWITCH_CLOSED) || closed_pulse)) {
					make_rfid_event(tag_c, tag, &gpsFresh.point, STATUS_RFID_SWITCH_CLOSED, now);
					print_debug("%02d/%02d/%4d %02d:%02d:%02d Switch Closed: %u\n", 
							ptm->tm_mon+1, ptm->tm_mday, ptm->tm_year+1900, 
							ptm->tm_hour, ptm->tm_min, ptm->tm_sec, 
							tag->tnum); 
					tag->status |= TAG_SWITCH_CLOSED;
				} else if ((tag->flag == 0) && ((tag->status & TAG_SWITCH_CLOSED) || open_pulse)) {
					make_rfid_event(tag_c, tag, &gpsFresh.point, STATUS_RFID_SWITCH_OPEN, now);
					print_debug("%02d/%02d/%4d %02d:%02d:%02d Switch Open: %u\n", 
							ptm->tm_mon+1, ptm->tm_mday, ptm->tm_year+1900, 
							ptm->tm_hour, ptm->tm_min, ptm->tm_sec, 
							tag->tnum); 
					tag->status &= ~TAG_SWITCH_CLOSED;
				}
			} /*senior*/
		}
	} /*for: traverse queue*/
	pthread_mutex_unlock(&tag_c->mutex_switch);
}

static uint32_t motion_open_beat = 0;
//static uint32_t motion_closed_beat = 0;
static int motion_queue_processing(struct tag_control *tag_c, struct motion_control *pmotion)
{
	struct Tag *tag, *tag1;
	struct tq_head *tqueue;
	int n_alarms = 0;
	time_t now, past;
	struct tm tm1; 
	struct tm *ptm;
	bool alarm_pulse = false;
//	bool closed_pulse= false;

	tqueue = &tag_c->motion_queue;
	if (TAILQ_EMPTY(tqueue))
		return 0;
	now = time(NULL);
	past = now - pmotion->tag_out_time;
	ptm = localtime_r(&now, &tm1);
/*	if (++motion_closed_beat == pmotion->closed_report_cycle) {
		closed_pulse = true; 
		motion_closed_beat = 0;
	}
*/
	if (++motion_open_beat == pmotion->alarm_report_rate) {
		alarm_pulse = true; 
		motion_open_beat = 0;
	}
	pthread_mutex_lock(&tag_c->mutex_switch);
	for (tag = tqueue->tqh_first; tag != NULL; tag = tag->link.tqe_next) {
		if (tag->recent < past) {
			if (tag->status & TAG_SENIOR) {
				make_rfid_event(tag_c, tag, backtrack_gps_trail(now - tag->recent), STATUS_RFID_TAG_OUT, now);
				print_debug("%02d/%02d/%4d %02d:%02d:%02d Motion Tag OUT: %u\n", 
						ptm->tm_mon+1, ptm->tm_mday, ptm->tm_year+1900, 
						ptm->tm_hour, ptm->tm_min, ptm->tm_sec, 
						tag->tnum); 
			} 
			tag1 = tag->link.tqe_next;
			TAILQ_REMOVE(tqueue, tag, link);
			memset(tag, 0, sizeof(*tag));
			pthread_mutex_lock(&tag_c->mutex_recycle);
			TAILQ_INSERT_TAIL(&tag_c->recycle_queue, tag, link);
			pthread_mutex_unlock(&tag_c->mutex_recycle);
			if (tag1 == NULL)
				break;
			tag = tag1;
		} else {
			if (tag->status & TAG_FRESH) {
				if (tag->flag == 0x20) {
					make_rfid_event(tag_c, tag, &gpsFresh.point, STATUS_RFID_MOTION_OPEN, now);
//					make_rfid_event(tag_c, tag, &gpsFresh.point, STATUS_RFID_SWITCH_OPEN, now);
					print_debug("%02d/%02d/%4d %02d:%02d:%02d Motion Open: %u\n", 
							ptm->tm_mon+1, ptm->tm_mday, ptm->tm_year+1900, 
							ptm->tm_hour, ptm->tm_min, ptm->tm_sec, 
							tag->tnum); 
					tag->status |= TAG_MOTION_OPEN;
					n_alarms++;
				} else {
					make_rfid_event(tag_c, tag, &gpsFresh.point, STATUS_RFID_MOTION_CLOSED, now);
//					make_rfid_event(tag_c, tag, &gpsFresh.point, STATUS_RFID_SWITCH_CLOSED, now);
					print_debug("%02d/%02d/%4d %02d:%02d:%02d Motion Closed: %u\n", 
							ptm->tm_mon+1, ptm->tm_mday, ptm->tm_year+1900, 
							ptm->tm_hour, ptm->tm_min, ptm->tm_sec, 
							tag->tnum); 
					tag->status &= ~TAG_MOTION_OPEN;
				}
				tag->status &= ~TAG_FRESH;
				tag->status |= TAG_SENIOR;
			/*fresh*/
			} else if (tag->status & TAG_SENIOR) {
				if ((tag->flag == 0) && (tag->status & TAG_MOTION_OPEN) ) {
					make_rfid_event(tag_c, tag, &gpsFresh.point, STATUS_RFID_MOTION_CLOSED, now);
//					make_rfid_event(tag_c, tag, &gpsFresh.point, STATUS_RFID_SWITCH_CLOSED, now);
					print_debug("%02d/%02d/%4d %02d:%02d:%02d Motion Closed: %u\n", 
							ptm->tm_mon+1, ptm->tm_mday, ptm->tm_year+1900, 
							ptm->tm_hour, ptm->tm_min, ptm->tm_sec, 
							tag->tnum); 
					tag->status &= ~TAG_MOTION_OPEN;
					n_alarms++;
				} else if (tag->flag == 0x20 && (!(tag->status & TAG_MOTION_OPEN) || alarm_pulse)) {
					make_rfid_event(tag_c, tag, &gpsFresh.point, STATUS_RFID_MOTION_OPEN, now);
//					make_rfid_event(tag_c, tag, &gpsFresh.point, STATUS_RFID_SWITCH_OPEN, now);
					print_debug("%02d/%02d/%4d %02d:%02d:%02d Motion Open: %u\n", 
							ptm->tm_mon+1, ptm->tm_mday, ptm->tm_year+1900, 
							ptm->tm_hour, ptm->tm_min, ptm->tm_sec, 
							tag->tnum); 
					tag->status |= TAG_MOTION_OPEN;
					n_alarms++;
				}
			} /*senior*/
		}
	} /*for: traverse queue*/
	pthread_mutex_unlock(&tag_c->mutex_switch);
	return n_alarms;
}

static uint32_t sensor_rpt_beat = 0;
//static uint32_t motion_closed_beat = 0;
static int sensor_queue_processing(struct tag_control *tag_c, struct sensor_control *psensor)
{
	struct Tag *tag, *tag1;
	struct tq_head *tqueue;
	int n_alarms = 0;
	time_t now, past;
	struct tm tm1; 
	struct tm *ptm;
	bool rpt_ready = false;
	static int pre_flag = 0;
//	bool closed_pulse= false;

	tqueue = &tag_c->sensor_queue;
	if (TAILQ_EMPTY(tqueue))
		return 0;
	now = time(NULL);
	past = now - psensor->tag_out_time;
	ptm = localtime_r(&now, &tm1);
/*	if (++motion_closed_beat == pmotion->closed_report_cycle) {
		closed_pulse = true; 
		motion_closed_beat = 0;
	}
*/
	if (++sensor_rpt_beat == psensor->report_cycle) {
		rpt_ready = true; 
		sensor_rpt_beat = 0;
	}
	pthread_mutex_lock(&tag_c->mutex_sensor);
	for (tag = tqueue->tqh_first; tag != NULL; tag = tag->link.tqe_next) {
		if (tag->recent < past) {
			if (tag->status & TAG_SENIOR) {
				make_rfid_event(tag_c, tag, backtrack_gps_trail(now - tag->recent), STATUS_RFID_TAG_OUT, now);
				print_debug("%02d/%02d/%4d %02d:%02d:%02d Sensor Tag OUT: %u\n", 
						ptm->tm_mon+1, ptm->tm_mday, ptm->tm_year+1900, 
						ptm->tm_hour, ptm->tm_min, ptm->tm_sec, 
						tag->tnum); 
			} 
			tag1 = tag->link.tqe_next;
			TAILQ_REMOVE(tqueue, tag, link);
			memset(tag, 0, sizeof(*tag));
			pthread_mutex_lock(&tag_c->mutex_recycle);
			TAILQ_INSERT_TAIL(&tag_c->recycle_queue, tag, link);
			pthread_mutex_unlock(&tag_c->mutex_recycle);
			if (tag1 == NULL)
				break;
			tag = tag1;
		} else {
			if (tag->status & TAG_FRESH) {
				if (tag->flag) {
					make_rfid_event(tag_c, tag, &gpsFresh.point, STATUS_RFID_SENSOR_STATUS, now);
//					make_rfid_event(tag_c, tag, &gpsFresh.point, STATUS_RFID_SWITCH_OPEN, now);
					print_debug("%02d/%02d/%4d %02d:%02d:%02d Sensor: %u with flag %x\n", 
							ptm->tm_mon+1, ptm->tm_mday, ptm->tm_year+1900, 
							ptm->tm_hour, ptm->tm_min, ptm->tm_sec, 
							tag->tnum, tag->flag); 
					pre_flag = tag->flag;
				}
				tag->status &= ~TAG_FRESH;
				tag->status |= TAG_SENIOR;
			} else if (tag->status & TAG_SENIOR) {
				/* if the tag->flag being changed from the previous value */
				/* report the new value of the flag immediately			 */
				if (tag->flag != pre_flag){
					make_rfid_event(tag_c, tag, &gpsFresh.point, STATUS_RFID_SENSOR_STATUS, now);
//					make_rfid_event(tag_c, tag, &gpsFresh.point, STATUS_RFID_SWITCH_CLOSED, now);
					print_debug("%02d/%02d/%4d %02d:%02d:%02d Sensor: %u with flag %x\n", 
							ptm->tm_mon+1, ptm->tm_mday, ptm->tm_year+1900, 
							ptm->tm_hour, ptm->tm_min, ptm->tm_sec, 
							tag->tnum, tag->flag); 
					n_alarms++;
					pre_flag = tag->flag;
				}  else if (rpt_ready) { 
				/* if the tag->flag are the same, 				 */
				/* just report the event after the certain interval */
					make_rfid_event(tag_c, tag, &gpsFresh.point, STATUS_RFID_SENSOR_STATUS, now);
//					make_rfid_event(tag_c, tag, &gpsFresh.point, STATUS_RFID_SWITCH_CLOSED, now);
					print_debug("%02d/%02d/%4d %02d:%02d:%02d Sensor: %u with flag %x\n", 
							ptm->tm_mon+1, ptm->tm_mday, ptm->tm_year+1900, 
							ptm->tm_hour, ptm->tm_min, ptm->tm_sec, 
							tag->tnum, tag->flag); 
				}
			} 
		}
	} /*for: traverse queue*/
	pthread_mutex_unlock(&tag_c->mutex_sensor);
	if (rpt_ready) 
		rpt_ready = false;
	return n_alarms;
}

static uint32_t battery_check_beat = 0;
void battery_time_checking(struct tag_control *tag_c)
{
	struct Tag *tag;
	struct tq_head *tqueue;
	time_t now;
	struct tm tm1; 
	struct tm *ptm;

	if (rfid_queue_time_adjust)
		goto start_checking;
	if (tag_c->battery_maximum == 255)
		return;
	if (++battery_check_beat == tag_c->battery_alarm_cycle)
		battery_check_beat = 0;
	else
		return;
start_checking:
	now = time(NULL);
	ptm = localtime_r(&now, &tm1);
	tqueue = &tag_c->freezer_queue;
	if (!TAILQ_EMPTY(tqueue)) {
		pthread_mutex_lock(&tag_c->mutex_freezer);
		for (tag = tqueue->tqh_first; tag != NULL; tag = tag->link.tqe_next) {
			if (tag->battery > tag_c->battery_maximum) {
				make_rfid_event(tag_c, tag, &gpsFresh.point, STATUS_RFID_BATTERY_ALERT, now);
				print_debug("%02d/%02d/%4d %02d:%02d:%02d  Battery Alert: %u\n", 
						ptm->tm_mon+1, ptm->tm_mday, ptm->tm_year+1900, 
						ptm->tm_hour, ptm->tm_min, ptm->tm_sec, 
						tag->tnum); 
			}
			if (rfid_queue_time_adjust)
				tag->recent = now;
		}
		pthread_mutex_unlock(&tag_c->mutex_freezer);
	}
	tqueue = &tag_c->locker_queue;
	if (!TAILQ_EMPTY(tqueue)) {
		pthread_mutex_lock(&tag_c->mutex_locker);
		for (tag = tqueue->tqh_first; tag != NULL; tag = tag->link.tqe_next) {
			if (tag->battery > tag_c->battery_maximum) {
				make_rfid_event(tag_c, tag, &gpsFresh.point, STATUS_RFID_BATTERY_ALERT, now);
				print_debug("%02d/%02d/%4d %02d:%02d:%02d  Battery Alert: %u\n", 
						ptm->tm_mon+1, ptm->tm_mday, ptm->tm_year+1900, 
						ptm->tm_hour, ptm->tm_min, ptm->tm_sec, 
						tag->tnum); 
			}
			if (rfid_queue_time_adjust)
				tag->recent = now - 2;
		}
		pthread_mutex_unlock(&tag_c->mutex_locker);
	}
	tqueue = &tag_c->cargo_queue;
	if (!TAILQ_EMPTY(tqueue)) {
		pthread_mutex_lock(&tag_c->mutex_cargo);
		for (tag = tqueue->tqh_first; tag != NULL; tag = tag->link.tqe_next) {
			if (tag->battery > tag_c->battery_maximum) {
				make_rfid_event(tag_c, tag, &gpsFresh.point, STATUS_RFID_BATTERY_ALERT, now);
				print_debug("%02d/%02d/%4d %02d:%02d:%02d  Battery Alert: %u\n", 
						ptm->tm_mon+1, ptm->tm_mday, ptm->tm_year+1900, 
						ptm->tm_hour, ptm->tm_min, ptm->tm_sec, 
						tag->tnum); 
			}
			if (rfid_queue_time_adjust)
				tag->recent = now - 2;
		}
		pthread_mutex_unlock(&tag_c->mutex_cargo);
	}
	tqueue = &tag_c->switch_queue;
	if (!TAILQ_EMPTY(tqueue)) {
		pthread_mutex_lock(&tag_c->mutex_switch);
		for (tag = tqueue->tqh_first; tag != NULL; tag = tag->link.tqe_next) {
			if (tag->battery > tag_c->battery_maximum) {
				make_rfid_event(tag_c, tag, &gpsFresh.point, STATUS_RFID_BATTERY_ALERT, now);
				print_debug("%02d/%02d/%4d %02d:%02d:%02d  Battery Alert: %u\n", 
						ptm->tm_mon+1, ptm->tm_mday, ptm->tm_year+1900, 
						ptm->tm_hour, ptm->tm_min, ptm->tm_sec, 
						tag->tnum); 
			}
			if (rfid_queue_time_adjust)
				tag->recent = now - 2;
		}
		pthread_mutex_unlock(&tag_c->mutex_switch);
	}

	tqueue = &tag_c->hightemp_queue;
	if (!TAILQ_EMPTY(tqueue)) {
		pthread_mutex_lock(&tag_c->mutex_hightemp);
		for (tag = tqueue->tqh_first; tag != NULL; tag = tag->link.tqe_next) {
			if (tag->battery > tag_c->battery_maximum) {
				make_rfid_event(tag_c, tag, &gpsFresh.point, STATUS_RFID_BATTERY_ALERT, now);
				print_debug("%02d/%02d/%4d %02d:%02d:%02d  Battery Alert: %u\n", 
						ptm->tm_mon+1, ptm->tm_mday, ptm->tm_year+1900, 
						ptm->tm_hour, ptm->tm_min, ptm->tm_sec, 
						tag->tnum); 
			}
			if (rfid_queue_time_adjust)
				tag->recent = now - 2;
		}
		pthread_mutex_unlock(&tag_c->mutex_hightemp);
	}

	tqueue = &tag_c->motion_queue;
	if (!TAILQ_EMPTY(tqueue)) {
		pthread_mutex_lock(&tag_c->mutex_motion);
		for (tag = tqueue->tqh_first; tag != NULL; tag = tag->link.tqe_next) {
			if (tag->battery > tag_c->battery_maximum) {
				make_rfid_event(tag_c, tag, &gpsFresh.point, STATUS_RFID_BATTERY_ALERT, now);
				print_debug("%02d/%02d/%4d %02d:%02d:%02d  Battery Alert: %u\n", 
						ptm->tm_mon+1, ptm->tm_mday, ptm->tm_year+1900, 
						ptm->tm_hour, ptm->tm_min, ptm->tm_sec, 
						tag->tnum); 
			}
			if (rfid_queue_time_adjust)
				tag->recent = now - 2;
		}
		pthread_mutex_unlock(&tag_c->mutex_motion);
	}
	
	tqueue = &tag_c->sensor_queue;
	if (!TAILQ_EMPTY(tqueue)) {
		pthread_mutex_lock(&tag_c->mutex_sensor);
		for (tag = tqueue->tqh_first; tag != NULL; tag = tag->link.tqe_next) {
			if (tag->battery > tag_c->battery_maximum) {
				make_rfid_event(tag_c, tag, &gpsFresh.point, STATUS_RFID_BATTERY_ALERT, now);
				print_debug("%02d/%02d/%4d %02d:%02d:%02d  Battery Alert: %u\n", 
						ptm->tm_mon+1, ptm->tm_mday, ptm->tm_year+1900, 
						ptm->tm_hour, ptm->tm_min, ptm->tm_sec, 
						tag->tnum); 
			}
			if (rfid_queue_time_adjust)
				tag->recent = now - 2;
		}
		pthread_mutex_unlock(&tag_c->mutex_sensor);
	}

	rfid_queue_time_adjust = false;
}
/*int print_raw(unsigned char *cbuf, int len)
{
	int i, num_shorts;
	unsigned char *c = cbuf;

	num_shorts= (len + 1) >> 1  ;
	for (i = 0; i < num_shorts; i++) {
		printf("%02X%02X", c[0], c[1]);
		c += 2;
	}
	printf("\n");
	return num_shorts;
} */
static int freezer_report_beat = 0;
static int freezer_alarm_beat = 0;
static uint32_t freezer_alarm_flag = 0;
int freezer_processing(struct tag_control *tag_c, struct freezer_control *freezer)
{
	int i;
	int upcoming;
	int n = 0, m = 0;
	freezer_queue_processing(tag_c, freezer);
	if (freezer->latched) {
		for (i = 0; i < NUM_ZONES; i++) 
			temp_sorted[i].sorted = false;
		if (++freezer_report_beat == freezer->report_cycle) {
			m = make_temperature_event(tag_c, freezer, 0);
			freezer_report_beat = 0;
			if (freezer->alarm_summary == 0) {
				freezer->alarm_count = 0;
				freezer_alarm_beat = 0;
			}
		}
		if (freezer->alarm_summary != 0) {
			if (freezer->alarm_count == 0)
				upcoming = freezer->alarm_delay;
			else if (freezer->alarm_count < freezer->alarm_times)
				upcoming = freezer->alarm_cycle;
			else {
				if ((freezer->alarm_summary & ~freezer_alarm_flag) != 0) {
					freezer->alarm_count = 0;
					upcoming = 1;
				} else
					return 0;
			}
			if (++freezer_alarm_beat >= upcoming) {
				freezer_alarm_flag = freezer->alarm_summary;
				n = make_temperature_event(tag_c, freezer, FREEZER_ALARM_HIGH);
				if (freezer->alarm_summary != 0) {
					if ((freezer->alarm_summary & ~freezer_alarm_flag) != 0)
						freezer->alarm_count = 0;
				} else
					return 0;
				freezer_alarm_flag = freezer->alarm_summary;
				n += make_temperature_event(tag_c, freezer, FREEZER_ALARM_LOW);
				if (n > 0)
					freezer->alarm_count++;
				freezer_alarm_beat = 0;
				freezer->alarm_summary = freezer_alarm_flag;
			}
		} 
	}
	return n;
}

static uint32_t humidity_beat = 0;
static void humidity_queue_processing(struct tag_control *tag_c, struct humidity_control *phumidity)
{
	struct Tag *tag, *tag1;
	struct tq_head *tqueue;
	time_t now, past;
	struct tm tm1; 
	struct tm *ptm;
	bool ready_to_sample = false;

	tqueue = &tag_c->humidity_queue;
	if (TAILQ_EMPTY(tqueue))
		return;
	now = time(NULL);
	past = now - phumidity->tag_out_time;
//	past = now - high_temp->report_cycle;

	ptm = localtime_r(&now, &tm1);

	if (++humidity_beat == phumidity->report_cycle) {
		ready_to_sample = true;
	}
	pthread_mutex_lock(&tag_c->mutex_humidity);

	for (tag = tqueue->tqh_first; tag != NULL; tag = tag->link.tqe_next) {
//		now = time(NULL);
		if (tag->recent < past) {
			if (tag->status & TAG_SENIOR) {
				make_rfid_event(tag_c, tag, backtrack_gps_trail(now - tag->recent), STATUS_RFID_TAG_OUT, now);
				print_debug("%02d/%02d/%4d %02d:%02d:%02d Humidity Tag OUT: %u\n", 
						ptm->tm_mon+1, ptm->tm_mday, ptm->tm_year+1900, 
						ptm->tm_hour, ptm->tm_min, ptm->tm_sec, 
						tag->tnum); 
			} 
			tag1 = tag->link.tqe_next;
			TAILQ_REMOVE(tqueue, tag, link);
			memset(tag, 0, sizeof(*tag));
			pthread_mutex_lock(&tag_c->mutex_recycle);
			TAILQ_INSERT_TAIL(&tag_c->recycle_queue, tag, link);
			pthread_mutex_unlock(&tag_c->mutex_recycle);
			if (tag1 == NULL)
				break;
			tag = tag1;
//		} else if (ready_to_sample) { /*ready_to_sample*/
//			if ((tag->status & TAG_FRESH) && (tag->rssi >= truck->min_rssi)) {			
		} else if ((tag->status & TAG_FRESH)) {
				make_rfid_event(tag_c, tag, &gpsFresh.point, STATUS_RFID_TAG_IN, now);
				print_debug("%02d/%02d/%4d %02d:%02d:%02d Humidity Tag IN: %u\n", 
						ptm->tm_mon+1, ptm->tm_mday, ptm->tm_year+1900, 
						ptm->tm_hour, ptm->tm_min, ptm->tm_sec, 
						tag->tnum); 
				tag->status &= ~TAG_FRESH;
				tag->status |= TAG_SENIOR;
		} else if (ready_to_sample && (tag->status & TAG_SENIOR)) { /*senior*/
				make_rfid_event(tag_c, tag, &gpsFresh.point, STATUS_RFID_TAG_IN, now);
				print_debug("%02d/%02d/%4d %02d:%02d:%02d Humidity Tag IN: %u\n", 
						ptm->tm_mon+1, ptm->tm_mday, ptm->tm_year+1900, 
						ptm->tm_hour, ptm->tm_min, ptm->tm_sec, 
						tag->tnum); 
		} /*senior*/
//		} /*ready_to_sample*/
	}
	pthread_mutex_unlock(&tag_c->mutex_humidity);	
	if (ready_to_sample) 
		humidity_beat = 0;
}
static void EncodeUInt32(unsigned char *outbuf, uint32_t value)
{
	int i;
	if (value == 0) {
		memset(outbuf, 0, 4);
		return;
	}
	for (i = 3; i >= 0; i--) {
		outbuf[i] = (unsigned char)(value & 0xFF);
		value >>= 8;
	}
}
static void EncodeUInt16(unsigned char *outbuf, uint16_t value)
{
	int i;

	if (value == 0) {
		memset(outbuf, 0, 2);
		return;
	}
	for (i = 1; i >= 0; i--) {
		outbuf[i] = (unsigned char)(value & 0xFF);
		value >>= 8;
	}
}
static void EncodeInt16(unsigned char *outbuf, int value)
{
	union trans {
		int i1;
		unsigned char uc[4];
	} tr; 

	if (value == 0) {
		memset(outbuf, 0, 2);
		return;
	}
	tr.i1 = value;
	outbuf[1] = tr.uc[0];
	outbuf[0] = tr.uc[1];
}
int read_temperature(struct freezer_control *freezer, unsigned char *pzone)
{
	int i, l, m, mid, hi, lo;
	int n_zones = 0;
	uint32_t zone_mask = 0; 
	freezer->alarm_summary = 0;	
	for (i = 0; i < freezer->divisor; i++) {
		zone_mask = 1 << i;
		if (freezer->t_zone[i].tnum > 0) {
			memcpy(&temp_sorted[i], &freezer->t_zone[i], sizeof(struct temp_record));
			l = freezer->t_zone[i].count - 1;
			quicksort(temp_sorted[i].t, 0, l);
			temp_sorted[i].sorted = true;
			m = (l + 1) >> 1;
			mid = (temp_sorted[i].t[m] << 1) - 500;
			if (mid < freezer->t_zone[i].high && mid > freezer->t_zone[i].low) {
				hi = (temp_sorted[i].t[l] << 1) - 500;	
				lo = (temp_sorted[i].t[0] << 1) - 500;	
				*pzone = (unsigned char)(i & 0xFF);
				EncodeInt16(pzone + 1, hi);
				EncodeInt16(pzone + 3, mid);
				EncodeInt16(pzone + 5, lo);
				pzone += 7;
				reset_freezer_zone(freezer, i);
				n_zones++;
			} else 
				freezer->alarm_summary |= zone_mask;	
		}
	}
	return n_zones;
}
int read_temperature_alarm(struct freezer_control *freezer, unsigned char *pzone, int alarm_type)
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
int make_temperature_event(struct tag_control *tag_c, struct freezer_control *freezer, int alarm_type)
{
	int n_zones = 0;
	time_t now;
	struct tm tm1, *ptm; 
	unsigned char * pzone;
	Packet_t *pkt = &rfid_event_packet;
	static bool trig_alarm_pin = false; /* indicate that the PIN regarding to the alarm is being trigured. no necessary to tougle the PIN repeatly even though the state of the alarm is not being changed. */
	
	now = time(NULL);
	memset(pkt->data, 0, sizeof(pkt->data));
	EncodeUInt32(pkt->data + 2, (uint32_t)now);
	gpsPointEncode8(pkt->data + 6, &gpsFresh.point);
/*	EncodeUInt32(pkt->data + 19, freezer->primary_id);
	EncodeUInt16(pkt->data + 18, tag_c->customer_id);
	EncodeUInt32(pkt->data + 14, tag_c->reader_type);
	pkt->hdrType = PKT_CLIENT_DMTSP_FORMAT_0;
	pzone = pkt->data + 23;
*/
	EncodeUInt32(pkt->data + 15, freezer->primary_id);
	EncodeUInt16(pkt->data + 14, tag_c->customer_id);
	pkt->hdrType = PKT_CLIENT_DMTSP_FORMAT_0;
	pzone = pkt->data + 19;
	
	if (alarm_type == 0) { 
		if ((n_zones = read_temperature(freezer, pzone)) == 0)
			return 0;
	} else if (alarm_type == FREEZER_ALARM_HIGH) {
		if ((n_zones = read_temperature_alarm(freezer, pzone, FREEZER_ALARM_HIGH)) == 0)
			return 0;
	} else if (alarm_type == FREEZER_ALARM_LOW) {
		if ((n_zones = read_temperature_alarm(freezer, pzone, FREEZER_ALARM_LOW)) == 0)
			return 0;
	}
	ptm = localtime_r(&now, &tm1);
	if (alarm_type == 0) {
		EncodeUInt16(pkt->data, STATUS_TEMPERATURE);
		pkt->priority = PRIORITY_NORMAL;
		print_debug("%02d/%02d/%4d %02d:%02d:%02d Temperature Normal\n",
				ptm->tm_mon+1, ptm->tm_mday, ptm->tm_year+1900, 
				ptm->tm_hour, ptm->tm_min, ptm->tm_sec); 
		/* toggle the pin */
		if(trig_alarm_pin) {
			system("/usr/local/bin/disalarm");
			trig_alarm_pin = false;
		}
	} else if (alarm_type == FREEZER_ALARM_HIGH) {
		EncodeUInt16(pkt->data, STATUS_TEMPERATURE_RANGE_0);
		pkt->priority = PRIORITY_HIGH;
		print_debug("%02d/%02d/%4d %02d:%02d:%02d Temperature High!\n",
				ptm->tm_mon+1, ptm->tm_mday, ptm->tm_year+1900, 
				ptm->tm_hour, ptm->tm_min, ptm->tm_sec); 
		/* toggle the pin */
		if (!trig_alarm_pin) {
			system("/usr/local/bin/trig_alarm");
			trig_alarm_pin = true;
		}
	} else if (alarm_type == FREEZER_ALARM_LOW) {
		EncodeUInt16(pkt->data, STATUS_TEMPERATURE_RANGE_1);
		pkt->priority = PRIORITY_HIGH;
		print_debug("%02d/%02d/%4d %02d:%02d:%02d Temperature Low!\n",
				ptm->tm_mon+1, ptm->tm_mday, ptm->tm_year+1900, 
				ptm->tm_hour, ptm->tm_min, ptm->tm_sec); 
		/* toggle the pin */
		if (!trig_alarm_pin) {
			system("/usr/local/bin/trig_alarm");
			trig_alarm_pin = true;
		}
	}
//	pkt->dataLen = n_zones * 7 + 23 + 1;
	pkt->dataLen = n_zones * 7 + 19 + 1;
	pzone += n_zones * 7;
	pkt->seqLen = 1;   
   	pkt->seqPos = pzone - pkt->data;   
	evAddEncodedPacket(pkt);
	return n_zones;
}
void make_rfid_event(struct tag_control *tag_c, struct Tag * tag, GPSPoint_t *coord, uint32_t ev_status, time_t timestamp)
{
	int t, seq_p;
	Packet_t *ppt = &rfid_event_packet;
	uint32_t humidity_max, humidity_min;
	
	memset(ppt->data, 0, sizeof(ppt->data));
	EncodeUInt16(ppt->data, ev_status);

	EncodeUInt32(ppt->data + 2, (uint32_t)timestamp);
	gpsPointEncode8(ppt->data + 6, coord);

	EncodeUInt32(ppt->data + 14, tag_c->reader_type);
	EncodeUInt32(ppt->data + 20, tag->tnum);

	EncodeUInt16(ppt->data + 19, tag_c->customer_id);

	ppt->data[18] = tag_c->reader_id;
	ppt->data[24] = (unsigned char)(tag->rssi & 0xFF);
	humidity_min = propGetUInt32AtIndex(PROP_RFID_HUMIDITY_ID_RANGE, 0, 1);
       humidity_max = propGetUInt32AtIndex(PROP_RFID_HUMIDITY_ID_RANGE, 1, 99);
	if (tag->tnum >= humidity_min && tag->tnum <= humidity_max)
		ppt->data[25] = tag->humidity;
	else
		ppt->data[25] = tag->battery;

//	EncodeUInt16(ppt->data, ev_status);
	seq_p = 26;

	if ((ev_status >= STATUS_RFID_LOCK_DISABLED && ev_status <= STATUS_RFID_SWITCH_OUT) || 
		(ev_status == STATUS_RFID_LOCK_ALARM ) || (ev_status == STATUS_RFID_SENSOR_STATUS)) {
		ppt->data[26] = tag->flag;
		++seq_p;
	}
	
	if (tag->temp != 0xFFFF) {
 		uint32_t hightemp_max, hightemp_min, hightemp_max_2, hightemp_min_2;
        	hightemp_min = propGetUInt32AtIndex(PROP_RFID_HIGHTEMP_ID_RANGE, 0, 1);
       	hightemp_max = propGetUInt32AtIndex(PROP_RFID_HIGHTEMP_ID_RANGE, 1, 99);
		hightemp_min_2 = propGetUInt32AtIndex(PROP_RFID_HIGHTEMP_ID_RANGE_2, 0, 1);
       	hightemp_max_2 = propGetUInt32AtIndex(PROP_RFID_HIGHTEMP_ID_RANGE_2, 1, 99);
		if ((tag->tnum >= hightemp_min && tag->tnum <= hightemp_max) ||(tag->tnum >= hightemp_min_2 && tag->tnum <= hightemp_max_2))
			t = HighTemp_tag_processing(tag);
// if it's the normal temperature tag 
		else
			t = (tag->temp << 1) - 500;	
		
		EncodeInt16(ppt->data + seq_p, t);
	seq_p += 2;
	//	printf("!make a rfid event, tnum = %u, temp = %d\n", tag->tnum, t);
	}

	ppt->priority = PRIORITY_NORMAL;
	ppt->hdrType = PKT_CLIENT_DMTSP_FORMAT_1;
	ppt->dataLen = (seq_p + 1) & 0xFF;  
	ppt->seqLen   = 1;   
	ppt->seqPos   = seq_p & 0xFF;

	evAddEncodedPacket(ppt);
}
void sample_temperature(struct freezer_control *freezer, struct Tag *tag)
{
	int zone, i;

	if (!freezer->is_zone0 && tag->tnum == freezer->primary_id)
		return;
	else
		zone = tag->tnum - freezer->primary_id;
	if (freezer->t_zone[zone].tnum == 0)
		freezer->t_zone[zone].tnum = tag->tnum;
	i = freezer->t_zone[zone].next;
	freezer->t_zone[zone].t[i] = tag->temp;
	if (i == NUM_TEMPERATURES - 1)
		freezer->t_zone[zone].next = 0; 
	else
		++freezer->t_zone[zone].next;
	if (freezer->t_zone[zone].count < NUM_TEMPERATURES)
		++freezer->t_zone[zone].count;
}
static int gps_trail_next = 0;
static bool gps_trail_length = 0;
static bool gps_trail_valid = false;
void update_gps_trail(void)
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
GPSPoint_t * backtrack_gps_trail(int backward_time)
{
	int back_index, index;

	if (!gps_trail_valid)
		return (&gpsFresh.point);
	back_index = backward_time / RFID_MAIN_PERIOD;
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
void sample_rssi(struct freezer_control *freezer, uint32_t rssi) 
{
	freezer->rssi_sample[freezer->rssi_point++] = rssi;
	if (freezer->rssi_point == NUM_RSSI)
		freezer->rssi_point = 0;
	if (freezer->rssi_samples < NUM_RSSI)
		++freezer->rssi_samples;
}
/* indicate thread should stop */
void rfid_stop(void)
{
	rfid_main_running = 0;
	rfid_reader_running = 0;
}
void rfid_port_close(void)
{
	close(rfidCom.read_fd);
}
void reset_freezer_rssi_record(struct freezer_control *freezer) 
{
	int i;
	for (i = 0; i < NUM_RSSI; i++)
		freezer->rssi_sample[i] = 0;
	freezer->rssi_point = 0;
	freezer->rssi_samples = 0;
	freezer->fading_count = 0;
}
void reset_freezer_zone(struct freezer_control *freezer, int zn) 
{
	Key_t r = PROP_TEMP_RANGE_0 + zn;
	memset(&freezer->t_zone[zn], 0, sizeof(struct temp_record));
	freezer->t_zone[zn].low = propGetUInt32AtIndex(r, 0, 0);
	freezer->t_zone[zn].high = propGetUInt32AtIndex(r, 1, 99);
}
void reset_freezer_zones(struct freezer_control *freezer) 
{
	int i;
	Key_t r = PROP_TEMP_RANGE_0;
	memset(freezer->t_zone, 0, sizeof(freezer->t_zone));
	for (i = 0; i < NUM_ZONES; i++, r++) {
		freezer->t_zone[i].low = propGetUInt32AtIndex(r, 0, 0);
		freezer->t_zone[i].high = propGetUInt32AtIndex(r, 1, 99);
	}
	freezer->alarm_summary = 0;
	freezer->alarm_count = 0;
}
void init_freezer_parameter(struct freezer_control *freezer) 
{
	uint32_t cycle, u1, n;
	
	freezer->id_lower = propGetUInt32AtIndex(PROP_RFID_PRIMARY_ID_RANGE, 0, 1);
	freezer->id_upper = propGetUInt32AtIndex(PROP_RFID_PRIMARY_ID_RANGE, 1, 99);
	cycle = propGetUInt32AtIndex(PROP_RFID_PRIMARY_ID_RANGE, 2, 10);
	freezer->beacon_cycle = (cycle + RFID_MAIN_PERIOD - 1) / RFID_MAIN_PERIOD;
	freezer->tag_in_time = propGetUInt32AtIndex(PROP_RFID_PRIMARY_ID_RANGE, 3, 15);
	if (freezer->tag_in_time < cycle)
		freezer->tag_in_time = cycle;
	freezer->tag_out_time = propGetUInt32AtIndex(PROP_RFID_PRIMARY_ID_RANGE, 4, 15);
	if (freezer->tag_out_time < cycle * 2)
		freezer->tag_out_time = cycle * 2;
	freezer->divisor = propGetUInt32(PROP_RFID_PRIMARY_ID_DIVISOR, 0); 
	if (freezer->divisor == 0)
		freezer->divisor = 8;
	freezer->min_rssi = propGetUInt32AtIndex(PROP_RFID_PRIMARY_RSSI, 0, 1);
	freezer->is_zone0 = propGetUInt32AtIndex(PROP_RFID_PRIMARY_ID_DIVISOR, 1, 0)? false : true;
	n = propGetUInt32(PROP_RFID_PRIMARY_ID, 0);
	if (n == 0) {
		freezer->permanently = false;
		freezer->rssi_delta = propGetUInt32AtIndex(PROP_RFID_PRIMARY_RSSI, 1, 1);
		u1 = propGetUInt32(PROP_RFID_PRIMARY_RSSI_TIMER, 100); 
		if (u1 > 0 && u1 < (cycle * 4))
			freezer->fading_duration = u1 / cycle; 
		else
			freezer->fading_duration = 4;
	} else {
		freezer->permanently = true;
		freezer->primary_id = n;
	}
	memset(freezer->t_zone, 0, sizeof(freezer->t_zone));
	freezer->alarm_summary = 0;
}
void init_tag_parameter(struct tag_control *tag_c)
{
	uint32_t u0, u1, u0_2, u1_2;
	
//	tag_c->reader_type = propGetUInt32AtIndex(PROP_RFID_READER_TYPE, 0, 0);
	tag_c->reader_type = PROTRAC_READER;
	tag_c->battery_maximum = propGetUInt32(PROP_RFID_BATTERY_LIFE_MAX, 255);
	if (tag_c->battery_maximum < 255) {
		u0 = propGetUInt32(PROP_RFID_BATTERY_ALARM_INTRVL, 0);
		tag_c->battery_alarm_cycle = (u0 + RFID_MAIN_PERIOD - 1) / RFID_MAIN_PERIOD;
	} else
		tag_c->battery_alarm_cycle = LONG_MAX;
	tag_c->tnum_min = propGetUInt32AtIndex(PROP_RFID_COMPANY_ID_RANGE, 0, 1);
	tag_c->tnum_max = propGetUInt32AtIndex(PROP_RFID_COMPANY_ID_RANGE, 1, 99);
	if ((tag_c->tnum_max >= tag_c->tnum_min) && (tag_c->tnum_max > 0)) {
		tag_c->role |= RFID_FUNCTION_ENABLE;
		u0 = propGetUInt32AtIndex(PROP_RFID_PRIMARY_ID_RANGE, 0, 1);
		u1 = propGetUInt32AtIndex(PROP_RFID_PRIMARY_ID_RANGE, 1, 99);
		if (u1 > 0 && u0 <= u1)
			tag_c->role |= ROLE_FREEZER;
		u0 = propGetUInt32AtIndex(PROP_RFID_LOCK_ID_RANGE, 0, 1);
		u1 = propGetUInt32AtIndex(PROP_RFID_LOCK_ID_RANGE, 1, 99);
		if (u1 > 0 && u0 <= u1)
			tag_c->role |= ROLE_LOCKER;
		u0 = propGetUInt32AtIndex(PROP_RFID_CARGO_ID_RANGE, 0, 1);
		u1 = propGetUInt32AtIndex(PROP_RFID_CARGO_ID_RANGE, 1, 99);		
		if (u1 > 0 && u0 <= u1)
			tag_c->role |= ROLE_CARGO;
		u0 = propGetUInt32AtIndex(PROP_RFID_HIGHTEMP_ID_RANGE, 0, 1);
		u1 = propGetUInt32AtIndex(PROP_RFID_HIGHTEMP_ID_RANGE, 1, 99);
		u0_2 = propGetUInt32AtIndex(PROP_RFID_HIGHTEMP_ID_RANGE_2, 0, 1);
		u1_2 = propGetUInt32AtIndex(PROP_RFID_HIGHTEMP_ID_RANGE_2, 1, 99);
		if ((u1 > 0 && u0 <= u1) || (u1_2 > 0 && u0_2 <= u1_2))
			tag_c->role |= ROLE_HIGHTEMP;
		u0 = propGetUInt32AtIndex(PROP_RFID_SWITCH_ID_RANGE, 0, 1);
		u1 = propGetUInt32AtIndex(PROP_RFID_SWITCH_ID_RANGE, 1, 99);
		if (u1 > 0 && u0 <= u1)
			tag_c->role |= ROLE_SWITCH;
		u0 = propGetUInt32AtIndex(PROP_RFID_MOTION_ID_RANGE, 0, 1);
		u1 = propGetUInt32AtIndex(PROP_RFID_MOTION_ID_RANGE, 1, 99);
		if (u1 > 0 && u0 <= u1)
			tag_c->role |= ROLE_MOTION;
		u0 = propGetUInt32AtIndex(PROP_RFID_SENSOR_ID_RANGE, 0, 1);
		u1 = propGetUInt32AtIndex(PROP_RFID_SENSOR_ID_RANGE, 1, 99);
		if (u1 > 0 && u0 <= u1)
			tag_c->role |= ROLE_SENSOR;
		u0 = propGetUInt32AtIndex(PROP_RFID_HUMIDITY_ID_RANGE, 0, 1);
		u1 = propGetUInt32AtIndex(PROP_RFID_HUMIDITY_ID_RANGE, 1, 99);
		if (u1 > 0 && u0 <= u1)
			tag_c->role |= ROLE_HUMIDITY;
	}
}
void init_temperature_parameter(struct freezer_control *freezer)
{
	int i;
	uint32_t u0, u1;
	Key_t r;

	r = PROP_TEMP_RANGE_0;
	for (i = 0; i < NUM_ZONES; i++, r++) {
		freezer->t_zone[i].low = propGetUInt32AtIndex(r, 0, 0);
		freezer->t_zone[i].high = propGetUInt32AtIndex(r, 1, 99);
	}
	u0 = propGetUInt32AtIndex(PROP_TEMP_REPORT_INTRVL, 0, 60); 
	if (u0 > 0)
		freezer->report_cycle = (u0 + RFID_MAIN_PERIOD - 1) / RFID_MAIN_PERIOD;
	else
		freezer->report_cycle = ONE_HOUR;
	freezer->alarm_times = propGetUInt32AtIndex(PROP_TEMP_REPORT_INTRVL, 3, 0); 
	if (freezer->alarm_times > 0) {
		u1 = propGetUInt32AtIndex(PROP_TEMP_REPORT_INTRVL, 1, 60); 
		if (u1 > 0)
			freezer->alarm_cycle = (u1 + RFID_MAIN_PERIOD - 1) / RFID_MAIN_PERIOD;
		else
			freezer->alarm_cycle = ONE_HOUR;
		u1 = propGetUInt32AtIndex(PROP_TEMP_REPORT_INTRVL, 2, 60); 
		freezer->alarm_delay = (u1 + RFID_MAIN_PERIOD) / RFID_MAIN_PERIOD;
		if (freezer->alarm_delay > freezer->report_cycle)
			freezer->alarm_delay -= freezer->report_cycle;
		else
			freezer->alarm_delay = 1;
	} else {
		freezer->alarm_cycle = LONG_MAX;
		freezer->alarm_delay = LONG_MAX;
	}
}
void init_lock_parameter(struct lock_control *locker) 
{
	uint32_t cycle, t1;

	locker->id_lower = propGetUInt32AtIndex(PROP_RFID_LOCK_ID_RANGE, 0, 1);
	locker->id_upper = propGetUInt32AtIndex(PROP_RFID_LOCK_ID_RANGE, 1, 99);
	cycle = propGetUInt32AtIndex(PROP_RFID_LOCK_ID_RANGE, 2, 10);
	locker->beacon_cycle = (cycle + RFID_MAIN_PERIOD - 1) / RFID_MAIN_PERIOD;
	locker->tag_in_time = propGetUInt32AtIndex(PROP_RFID_LOCK_ID_RANGE, 3, 15);
	if (locker->tag_in_time < cycle)
		locker->tag_in_time = cycle;
	locker->tag_out_time = propGetUInt32AtIndex(PROP_RFID_LOCK_ID_RANGE, 4, 15);
	if (locker->tag_out_time < cycle * 2)
		locker->tag_out_time = cycle * 2;
	t1 = propGetUInt32AtIndex(PROP_RFID_LOCK_REPORT_INTRVL, 0, 60);
	if (t1 > 0)
		locker->armed_cycle = (t1 + RFID_MAIN_PERIOD - 1) / RFID_MAIN_PERIOD;
	else
		locker->armed_cycle = LONG_MAX;
	t1 = propGetUInt32AtIndex(PROP_RFID_LOCK_REPORT_INTRVL, 1, 60);
	if (t1 > 0)
		locker->alarm_cycle = (t1 + RFID_MAIN_PERIOD - 1) / RFID_MAIN_PERIOD;
	else
		locker->alarm_cycle = LONG_MAX;
	locker->non_exclusive = propGetUInt32AtIndex(PROP_RFID_LOCK_REPORT_INTRVL, 2, 1);
}
void init_cargo_parameter(struct cargo_control *truck) 
{
	uint32_t cycle, t1;

	truck->id_lower = propGetUInt32AtIndex(PROP_RFID_CARGO_ID_RANGE, 0, 1);
	truck->id_upper = propGetUInt32AtIndex(PROP_RFID_CARGO_ID_RANGE, 1, 99);
	cycle = propGetUInt32AtIndex(PROP_RFID_CARGO_ID_RANGE, 2, 10);
	truck->beacon_cycle = (cycle + RFID_MAIN_PERIOD - 1) / RFID_MAIN_PERIOD;
	truck->tag_in_time = propGetUInt32AtIndex(PROP_RFID_CARGO_ID_RANGE, 3, 15);
	if (truck->tag_in_time < cycle)
		truck->tag_in_time = cycle;
	truck->tag_out_time = propGetUInt32AtIndex(PROP_RFID_CARGO_ID_RANGE, 4, 15);
	if (truck->tag_out_time < cycle * 2)
		truck->tag_out_time = cycle * 2;
	t1 = propGetUInt32(PROP_RFID_CARGO_REPORT_INTRVL, 60);
	if (t1 > 0)
		truck->report_cycle = (t1 + RFID_MAIN_PERIOD - 1) / RFID_MAIN_PERIOD;
	else
		truck->report_cycle = LONG_MAX;
	truck->min_rssi = propGetUInt32(PROP_RFID_CARGO_MIN_RSSI, 0);
	truck->sample_mode = propGetUInt32(PROP_RFID_CARGO_SAMPLE_MODE, 0);
	if (truck->sample_mode == CARGO_SAMPLE_IN_MOTION) {
		truck->min_speed = propGetUInt32AtIndex(PROP_RFID_IN_MOTION, 0, 6);
		t1 = propGetUInt32AtIndex(PROP_RFID_IN_MOTION, 1, 60);
		truck->motion_duration = (t1 + RFID_MAIN_PERIOD - 1) / RFID_MAIN_PERIOD;
	}
}

static void init_hightemp_parameter(struct high_temp_control *high_temp) 
{
	uint32_t cycle, t1;

	high_temp->id_lower = propGetUInt32AtIndex(PROP_RFID_HIGHTEMP_ID_RANGE, 0, 1);
	high_temp->id_upper = propGetUInt32AtIndex(PROP_RFID_HIGHTEMP_ID_RANGE, 1, 99);
	high_temp->id_lower_2 = propGetUInt32AtIndex(PROP_RFID_HIGHTEMP_ID_RANGE_2, 0, 1);
	high_temp->id_upper_2 = propGetUInt32AtIndex(PROP_RFID_HIGHTEMP_ID_RANGE_2, 1, 99);
	cycle = propGetUInt32AtIndex(PROP_RFID_HIGHTEMP_ID_RANGE, 2, 10);
	high_temp->beacon_cycle = (cycle + RFID_MAIN_PERIOD - 1) / RFID_MAIN_PERIOD;
	high_temp->tag_in_time = propGetUInt32AtIndex(PROP_RFID_HIGHTEMP_ID_RANGE, 3, 15);
	if (high_temp->tag_in_time < cycle)
		high_temp->tag_in_time = cycle;
	high_temp->tag_out_time = propGetUInt32AtIndex(PROP_RFID_HIGHTEMP_ID_RANGE, 4, 15);
	if (high_temp->tag_out_time < cycle * 2)
		high_temp->tag_out_time = cycle * 2;
	t1 = propGetUInt32(PROP_RFID_HIGHTEMP_REPORT_INTRVL, 60);
//	high_temp->report_cycle = t1;
	if (t1 > 0)
		high_temp->report_cycle = (t1 + RFID_MAIN_PERIOD - 1) / RFID_MAIN_PERIOD;
	else
		high_temp->report_cycle = LONG_MAX;
}

void init_motion_parameter(struct motion_control *p_motion) 
{
	uint32_t cycle, t1;

	p_motion->id_lower = propGetUInt32AtIndex(PROP_RFID_MOTION_ID_RANGE, 0, 1);
	p_motion->id_upper = propGetUInt32AtIndex(PROP_RFID_MOTION_ID_RANGE, 1, 99);
	cycle = propGetUInt32AtIndex(PROP_RFID_MOTION_ID_RANGE, 2, 10);
	p_motion->beacon_cycle = (cycle + RFID_MAIN_PERIOD - 1) / RFID_MAIN_PERIOD;
	p_motion->tag_in_time = propGetUInt32AtIndex(PROP_RFID_MOTION_ID_RANGE, 3, 15);
	if (p_motion->tag_in_time < cycle)
		p_motion->tag_in_time = cycle;
	p_motion->tag_out_time = propGetUInt32AtIndex(PROP_RFID_MOTION_ID_RANGE, 4, 15);
	if (p_motion->tag_out_time < cycle * 2)
		p_motion->tag_out_time = cycle * 2;
	t1 = propGetUInt32AtIndex(PROP_RFID_MOTION_REPORT_RATE, 0, 60);
	if (t1 > 0)
/*		p_motion->closed_report_cycle = (t1 + RFID_MAIN_PERIOD - 1) / RFID_MAIN_PERIOD;
	else
		p_motion->closed_report_cycle = LONG_MAX;
	t1 = propGetUInt32AtIndex(PROP_RFID_MOTION_REPORT_INTRVL, 1, 60);
	if (t1 > 0)
*/		p_motion->alarm_report_rate = (t1 + RFID_MAIN_PERIOD - 1) / RFID_MAIN_PERIOD;
	else
		p_motion->alarm_report_rate = LONG_MAX;
}

void init_switch_parameter(struct switch_control *p_switch) 
{
	uint32_t cycle, t1;

	p_switch->id_lower = propGetUInt32AtIndex(PROP_RFID_SWITCH_ID_RANGE, 0, 1);
	p_switch->id_upper = propGetUInt32AtIndex(PROP_RFID_SWITCH_ID_RANGE, 1, 99);
	cycle = propGetUInt32AtIndex(PROP_RFID_SWITCH_ID_RANGE, 2, 10);
	p_switch->beacon_cycle = (cycle + RFID_MAIN_PERIOD - 1) / RFID_MAIN_PERIOD;
	p_switch->tag_in_time = propGetUInt32AtIndex(PROP_RFID_SWITCH_ID_RANGE, 3, 15);
	if (p_switch->tag_in_time < cycle)
		p_switch->tag_in_time = cycle;
	p_switch->tag_out_time = propGetUInt32AtIndex(PROP_RFID_SWITCH_ID_RANGE, 4, 15);
	if (p_switch->tag_out_time < cycle * 2)
		p_switch->tag_out_time = cycle * 2;
	t1 = propGetUInt32AtIndex(PROP_RFID_SWITCH_REPORT_INTRVL, 0, 60);
	if (t1 > 0)
		p_switch->closed_report_cycle = (t1 + RFID_MAIN_PERIOD - 1) / RFID_MAIN_PERIOD;
	else
		p_switch->closed_report_cycle = LONG_MAX;
	t1 = propGetUInt32AtIndex(PROP_RFID_SWITCH_REPORT_INTRVL, 1, 60);
	if (t1 > 0)
		p_switch->open_report_cycle = (t1 + RFID_MAIN_PERIOD - 1) / RFID_MAIN_PERIOD;
	else
		p_switch->open_report_cycle = LONG_MAX;
}

void init_sensor_parameter(struct sensor_control *p_sensor) 
{
	uint32_t cycle, t1;

	p_sensor->id_lower = propGetUInt32AtIndex(PROP_RFID_SENSOR_ID_RANGE, 0, 1);
	p_sensor->id_upper = propGetUInt32AtIndex(PROP_RFID_SENSOR_ID_RANGE, 1, 99);
	cycle = propGetUInt32AtIndex(PROP_RFID_SENSOR_ID_RANGE, 2, 10);
	p_sensor->beacon_cycle = (cycle + RFID_MAIN_PERIOD - 1) / RFID_MAIN_PERIOD;
	p_sensor->tag_in_time = propGetUInt32AtIndex(PROP_RFID_SENSOR_ID_RANGE, 3, 15);
	if (p_sensor->tag_in_time < cycle)
		p_sensor->tag_in_time = cycle;
	p_sensor->tag_out_time = propGetUInt32AtIndex(PROP_RFID_SENSOR_ID_RANGE, 4, 15);
	if (p_sensor->tag_out_time < cycle * 2)
		p_sensor->tag_out_time = cycle * 2;
	t1 = propGetUInt32AtIndex(PROP_RFID_SENSOR_REPORT_INTRVL, 0, 60);
	if (t1 > 0)
/*		p_motion->closed_report_cycle = (t1 + RFID_MAIN_PERIOD - 1) / RFID_MAIN_PERIOD;
	else
		p_motion->closed_report_cycle = LONG_MAX;
	t1 = propGetUInt32AtIndex(PROP_RFID_MOTION_REPORT_INTRVL, 1, 60);
	if (t1 > 0)
*/		p_sensor->report_cycle = (t1 + RFID_MAIN_PERIOD - 1) / RFID_MAIN_PERIOD;
	else
		p_sensor->report_cycle = LONG_MAX;
}

static void init_humidity_parameter(struct humidity_control *p_humidity) 
{
	uint32_t cycle, t1;

	p_humidity->id_lower = propGetUInt32AtIndex(PROP_RFID_HUMIDITY_ID_RANGE , 0, 1);
	p_humidity->id_upper = propGetUInt32AtIndex(PROP_RFID_HUMIDITY_ID_RANGE , 1, 99);
	cycle = propGetUInt32AtIndex(PROP_RFID_HUMIDITY_ID_RANGE, 2, 10);
	p_humidity->beacon_cycle = (cycle + RFID_MAIN_PERIOD - 1) / RFID_MAIN_PERIOD;
	p_humidity->tag_in_time = propGetUInt32AtIndex(PROP_RFID_HUMIDITY_ID_RANGE, 3, 15);
	if (p_humidity->tag_in_time < cycle)
		p_humidity->tag_in_time = cycle;
	p_humidity->tag_out_time = propGetUInt32AtIndex(PROP_RFID_HUMIDITY_ID_RANGE, 4, 15);
	if (p_humidity->tag_out_time < cycle * 2)
		p_humidity->tag_out_time = cycle * 2;
	t1 = propGetUInt32(PROP_RFID_HUMIDITY_REPORT_INTRVL, 60);
//	high_temp->report_cycle = t1;
	if (t1 > 0)
		p_humidity->report_cycle = (t1 + RFID_MAIN_PERIOD - 1) / RFID_MAIN_PERIOD;
	else
		p_humidity->report_cycle = LONG_MAX;
}

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
