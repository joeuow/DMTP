#include <string.h>
#include <stdlib.h>

#define RANG_START 1
#define RANG_END 0

extern double str_to_d(char *string);
extern float str_to_f(char *string);
extern int fp_to_str(char *dest, float float_org, int dcm_num);
extern int dbl_to_str(char *dest, double double_org, int dcm_num, int rang_pos);