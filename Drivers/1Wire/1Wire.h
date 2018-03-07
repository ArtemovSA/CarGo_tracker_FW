#ifndef _1WIRE_H_
#define _1WIRE_H_

#include "stdint.h"
#include "InitDevice.h"

typedef struct {
	GPIO_TypeDef* GPIOx;           /*!< GPIOx port to be used for I/O functions */
	uint16_t GPIO_Pin;             /*!< GPIO Pin to be used for I/O functions */
	uint8_t LastDiscrepancy;       /*!< Search private */
	uint8_t LastFamilyDiscrepancy; /*!< Search private */
	uint8_t LastDeviceFlag;        /*!< Search private */
	uint8_t ROM_NO[8];             /*!< 8-bytes address of last search device */
} OneWire_t;

#define ONEWIRE_PORT    _1_WIRE_EFM_PORT
#define ONEWIRE_PIN     _1_WIRE_EFM_PIN

void ONEW_init(); //init
void ONEW_reset(); //reset
void ONEW_writeByte(uint8_t data); //Write Byte
uint8_t ONEW_readByte(); //Read byte
uint16_t ONEW_readWord(); //Read word

#endif 