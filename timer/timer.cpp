#include "timer.h"
#include <chrono>

long long get_time_usec()
{
    auto currentTime = std::chrono::high_resolution_clock::now();
    auto microseconds = std::chrono::time_point_cast<std::chrono::microseconds>(currentTime);
    return microseconds.time_since_epoch().count();
}

long long get_time_msec()
{
    static struct timeval _time_stamp;
    gettimeofday(&_time_stamp, NULL);
    return _time_stamp.tv_sec * 1000 + _time_stamp.tv_usec / 1000;
}

long long sec2msec(uint32_t sec)
{
    return sec * 1000;
}

void delay(long long msec)
{
    long long start = get_time_msec();
    while (get_time_msec() - start <= msec)
    {
        /* code */
    }
}
