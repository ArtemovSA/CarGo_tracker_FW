
#ifndef DELAY_H
#define DELAY_H

#include "stdint.h"

void _delay_ms(uint16_t ms); //Milleseconds delay

#ifndef BOOTLOADER
void _delay_us (uint16_t delay); //Us dekay
#endif

#endif