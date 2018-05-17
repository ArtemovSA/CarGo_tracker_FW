
#ifndef CLOCK_H
#define CLOCK_H

#include <time.h>
#include "stdint.h"

extern time_t globalUTC_time;
extern struct tm globalTime;

void CL_setUTC_Time(time_t UTC_Time); //Set UTC time
time_t CL_getUTC_Time(); //Get UTC time
void CL_set_Time(struct tm struct_Time); //Set time struct
void CL_setDateTime(float date, float time); //Set for time and date format
void CL_getDateTime(float *date, float *time); //Get for time and date format
struct tm CL_get_Time(); //Get struct data
void CL_incUTC(uint16_t sec); //Increment UTC

#endif