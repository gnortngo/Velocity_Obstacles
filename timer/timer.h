#include <time.h>
#include <stdint.h>
#include <sys/time.h>

long long get_time_usec();

long long get_time_msec();

long long sec2msec(uint32_t sec);

void delay(long long msec);
