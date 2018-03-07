
#include "Date_time.h"
#include "Device_ctrl.h"
#include "em_int.h"

int days_in_month[]={0,31,28,31,30,31,30,31,31,30,31,30,31};

//--------------------------------------------------------------------------------------------------
//Init date time
void DT_init()
{
  int year = (int)(DC_params.date/10000);
  
  if(((year%4==0) && (year%100!=0)) || (year%400==0))
  {
    days_in_month[2] = 29;
  }
  else
  {
    days_in_month[2] = 28;
  }
}
//--------------------------------------------------------------------------------------------------
//Inc data time
void DT_inc_time(uint32_t value)
{
  uint8_t hour = (uint8_t)(DC_params.time/10000);
  uint8_t min = (uint8_t)((DC_params.time-hour*10000)/100);
  uint8_t sec = (uint8_t)(DC_params.time-hour*10000-min*100);
  uint8_t day = (uint8_t)(DC_params.date/10000);
  uint8_t mount = (uint8_t)((DC_params.date-day*10000)/100);
  uint8_t year = (uint8_t)(DC_params.date-day*10000-mount*100);
  
  if (sec+value > 60)
  {
    if (min == 59)
    {
      if (hour == 23)
      {
        if (day == days_in_month[mount]-1)
        {
          if (mount == 12)
          {
            year++;
            mount = 1;
          }else{
            mount++;
          }
          day = 1;
        }else{
          day++;
        }
        hour = 0;
      }else{
        hour++;
      }
      min = 0;
    }else{
      min++;
    }
    sec = 0;
  }else{
    sec = sec + value;
  }
  
  DC_params.time = hour*10000+min*100+sec;
  DC_params.date = day*10000+mount*100+year; 
}
//--------------------------------------------------------------------------------------------------
//Get data time
void DC_get_date_time(float* date, float* time)
{
  *date = DC_params.date;
  *time = DC_params.time;
}
//--------------------------------------------------------------------------------------------------
//Set data time
void DC_set_date_time(float date, float time)
{
  INT_Disable();
  DC_params.date = date;
  DC_params.time = time;
  INT_Enable();
}