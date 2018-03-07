
#ifndef DELAY_H
#define DELAY_H

#include "stdint.h"

void _delay_ms(uint16_t ms); //Milleseconds delay
void _delay_timer_ms (uint16_t delay); //Milleseconds delay
void _delay_us (uint16_t delay); //Us delay

#endif