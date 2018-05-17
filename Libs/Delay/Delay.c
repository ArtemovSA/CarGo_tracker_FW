
#include "Delay.h"

#ifndef BOOTLOADER

#include "FreeRTOS.h"
#include "task.h"
#include "em_cmu.h"
#include "em_timer.h"

#define TIMER_FREQ 48000000

//--------------------------------------------------------------------------------------------------
//Milleseconds delay
void _delay_ms( uint16_t ms)
{
  vTaskDelay(ms);
}
//--------------------------------------------------------------------------------------------------
//Microsecond delay
void _delay_us (uint16_t delay)
{
  uint32_t endValue = delay * (TIMER_FREQ / 1000000);
  
  TIMER1->CNT = 0;
  
  TIMER1->CMD = TIMER_CMD_START;
  
  while ( TIMER1->CNT < endValue );
  
  TIMER1->CMD = TIMER_CMD_STOP;
}

//--------------------------------------------------------------------------------------------------
//Milleseconds delay
void _delay_timer_ms (uint16_t delay)
{  
  while (delay--)
    _delay_us (1000);
}

#else

#include "em_timer.h"

//--------------------------------------------------------------------------------------------------
//Milleseconds delay
void _delay_ms (uint16_t delay)
{  
  while (delay--)
    _delay_us (1000);
}

#endif