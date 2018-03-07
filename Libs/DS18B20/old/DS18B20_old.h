#ifndef __DS18B20_H__
#define __DS18B20_H__

#include "stdint.h"

typedef enum {
  DS18B20_Resolution_9_bit 	= 0x1F,
  DS18B20_Resolution_10_bit 	= 0x3F,
  DS18B20_Resolution_11_bit 	= 0x5F,
  DS18B20_Resolution_12_bit 	= 0x7F
} DS18B20_Resolution;

void DS18B20_init(DS18B20_Resolution resolution);
void DS18B20_setResolution(DS18B20_Resolution resolution);
uint16_t DS18B20_getTemperature();

#endif