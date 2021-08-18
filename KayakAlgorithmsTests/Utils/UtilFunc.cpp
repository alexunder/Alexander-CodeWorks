#include "UtilFunc.h"
#include <sys/time.h>
#include <time.h>
#include <stdio.h>

void GetCurDateTime(char* szDateTime)
{
    struct timespec       timeCur = { 0, 0 };
    struct tm             tmNow;
    unsigned int          uMS = 0;
    clock_gettime(CLOCK_REALTIME, &timeCur);
    localtime_r(&timeCur.tv_sec, &tmNow);
    sprintf(szDateTime,
            "%04d%02d%02d%02d%02d%02d%03d",
            tmNow.tm_year + 1900,
            tmNow.tm_mon + 1,
            tmNow.tm_mday,
            tmNow.tm_hour,
            tmNow.tm_min,
            tmNow.tm_sec,
            (int)(timeCur.tv_nsec / (unsigned long long)1000000));
}

void Second2Time(unsigned long long ullTimeSec, char* szDateTime)
{
    time_t        tvSec     = (ullTimeSec) / ((unsigned long long)1000000000) ;
    unsigned int  tvMSec    = ((ullTimeSec) % ((unsigned long long)1000000000)) / 1000000 ;
    struct        tm        tmNow ;
    localtime_r(&tvSec, &tmNow) ;
    sprintf(szDateTime,
            "%04d%02d%02d%02d%02d%02d%03d",
            tmNow.tm_year + 1900,
            tmNow.tm_mon + 1,
            tmNow.tm_mday,
            tmNow.tm_hour,
            tmNow.tm_min,
            tmNow.tm_sec,
            tvMSec) ;
}