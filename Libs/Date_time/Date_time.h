
#ifndef DATE_TIME
#define DATE_TIME

#include "stdint.h"

void DT_init(); //Date time init
void DT_inc_time(uint32_t value); //Inc data time
void DC_get_date_time(float* date, float* time); //Get data time
void DC_set_date_time(float date, float time); //Set data time

#endif