
#include "Clock.h"
#include "stdint.h"
#include "em_int.h"
#include "string.h"
#include <time.h>

time_t globalUTC_time;
struct tm globalTime;

//--------------------------------------------------------------------------------------------------
//Set UTC time
void CL_setUTC_Time(time_t UTC_Time)
{
  //INT_Disable();
  globalUTC_time = UTC_Time;
  //INT_Enable();
}
//--------------------------------------------------------------------------------------------------
//Get UTC time
time_t CL_getUTC_Time()
{
  return time( &globalUTC_time );
}
//--------------------------------------------------------------------------------------------------
//Set time struct
void CL_set_Time(struct tm struct_Time)
{
  INT_Disable();
  globalUTC_time = mktime( &struct_Time );
  //time( &globalUTC_time );
  INT_Enable();
}
//--------------------------------------------------------------------------------------------------
//Set for time and date format
void CL_setDateTime(float date, float time)
{
  struct tm struct_Time;
  
  struct_Time.tm_hour = (uint8_t)(time/10000);
  struct_Time.tm_min = (uint8_t)((time-struct_Time.tm_hour*10000)/100);
  struct_Time.tm_sec = (uint8_t)(time-struct_Time.tm_hour*10000-struct_Time.tm_min*100);
  struct_Time.tm_mday = (uint8_t)(date/10000);
  struct_Time.tm_mon = (uint8_t)((date-struct_Time.tm_mday*10000)/100);
  struct_Time.tm_year = (uint8_t)(date-struct_Time.tm_mday*10000-struct_Time.tm_mon*100);
  
  CL_set_Time(struct_Time);
}
//--------------------------------------------------------------------------------------------------
//Get for time and date format
void CL_getDateTime(float *date, float *time)
{
  *date = globalTime.tm_mday * 10000 + globalTime.tm_mon * 100 + globalTime.tm_year;
  *time = globalTime.tm_hour * 10000 + globalTime.tm_min * 100 + globalTime.tm_sec;
}
//--------------------------------------------------------------------------------------------------
//Get struct data
struct tm CL_get_Time()
{
  struct tm tm_buf;
  memcpy(&tm_buf, localtime( &globalUTC_time ), sizeof(tm_buf));
  return tm_buf; 
}
//--------------------------------------------------------------------------------------------------
//Increment UTC
void CL_incUTC(uint16_t sec)
{
  INT_Disable();
  globalUTC_time += sec;
  INT_Enable();
}
