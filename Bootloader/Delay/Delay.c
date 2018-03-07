
#include "Delay.h"

#ifndef BOOTLOADER

#include "FreeRTOS.h"
#include "task.h"

//Milleseconds delay
void _delay_ms( uint16_t ms)
{
  vTaskDelay(ms);
}

#else

#include "em_timer.h"

void _delay_us (uint16_t delay)
{
  uint16_t cycle_delay = delay * 16 - 28;

  /* Reset Timer */
  TIMER_CounterSet(TIMER0, 0);

  /* Start TIMER0 */
  TIMER0->CMD = TIMER_CMD_START;

  /* Wait until counter value is over top */
  while(TIMER0->CNT < cycle_delay){ };
  TIMER0->CMD = TIMER_CMD_STOP;
}

void _delay_ms (uint16_t delay)
{  
  while (delay--)
    _delay_us (1000);
}
#endif