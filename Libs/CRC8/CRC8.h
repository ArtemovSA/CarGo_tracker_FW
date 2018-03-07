
#ifndef CRC8_H
#define CRC8_H

#include "stdint.h"

uint8_t crc8(const uint8_t *addr, uint8_t len, uint8_t crc_init); 

#endif