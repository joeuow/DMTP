#include "float_point_handle.h"

double str_to_d(char *string) {
        int high = 0, low = 0;
        int low_length = 0;
        double low_fin = 0.0;
        double res = 0.0;
        int i = 0;

if((strlen(string) > 1) && (strchr(string, '.'))) {
        high = atoi(string);
       	while(string[i++] != '.');
       	low = atoi(&string[i]);
        while(string[i++])
                low_length++;
        low_fin = (double)low;
        while(low_length--) {
                low_fin /= 10.0;
        }
        res = (double)high + low_fin;
}
        return res;
}

float str_to_f(char *string) {
        int high = 0, low = 0;
        int high_length = 0, low_length = 0;
        float low_fin = 0.0;
        float res = 0.0;
        int i = 0;

if((strlen(string) > 1) && (strchr(string, '.'))) {
        high = atoi(string);
        while(string[i++] != '.');
        low = atoi(&string[i]);
        while(string[i++])
                low_length++;
        low_fin = (float)low;
        while(low_length--) {
                low_fin /= 10.0;
        }
        res = (float)high + low_fin;
}

        return res;
}

int fp_to_str(char *dest, float float_org, int dcm_num) {
//	char *str_res = (char *)malloc(sizeof(float)*sizeof(char));
	float dcm_plc;
	int dcm_plc_int;
	int integer;
	int i = 0;
	int length;
	
	if (sizeof(float) == sizeof(int))
		integer = (int)float_org;
	dcm_plc = (integer > float_org) ? (float_org+1-integer) : (float_org - integer);
	while (i++<dcm_num) dcm_plc *= 10.0f;
	dcm_plc_int = ((int)dcm_plc >= dcm_plc) ? (int)dcm_plc : (int)dcm_plc+1;

//	snprintf(str_res, 64, "%d.%d", integer, dcm_plc_int);
	length = snprintf(dest, 64, "%d.%d", integer, dcm_plc_int);
	return length;
}

int dbl_to_str(char *dest, double double_org, int dcm_num, int rang_pos) {
//	char *str_res = (char *)malloc(sizeof(float)*sizeof(char));
	float dcm_plc;
	int dcm_plc_int;
	int integer;
	int i = 0;
	int length;
	
	if (sizeof(float) == sizeof(int))
		integer = (int)double_org;
	dcm_plc = (integer > double_org) ? (double_org+1-integer) : (double_org - integer);
	while (i++<dcm_num) dcm_plc *= 10.0f;
	dcm_plc_int = ((int)dcm_plc >= dcm_plc) ? (int)dcm_plc : (int)dcm_plc+1;

//	snprintf(str_res, 64, "%d.%d", integer, dcm_plc_int);
	if(rang_pos == RANG_START)
		length = snprintf(dest, 64, "%d.%d,", integer, dcm_plc_int);
	else if(rang_pos == RANG_END)
		length = snprintf(dest, 64, "%d.%d", integer, dcm_plc_int);
	
	return length;
}