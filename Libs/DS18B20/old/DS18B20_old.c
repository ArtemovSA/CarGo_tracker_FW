
#include "DS18B20.h"
#include "1Wire.h"
#include "Delay.h"

#define DELAY_T_CONVERT       760000            //Conver time

typedef enum {
  SKIP_ROM = 0xCC,
  CONVERT_T = 0x44,
  READ_SCRATCHPAD = 0xBE,
  WRITE_SCRATCHPAD = 0x4E,
  TH_REGISTER = 0x4B,
  TL_REGISTER = 0x46,
} COMMANDS;

uint32_t DELAY_WAIT_CONVERT = DELAY_T_CONVERT;

//--------------------------------------------------------------------------------------------------
//init
void DS18B20_init(DS18B20_Resolution resolution) 
{
  ONEW_init();
  DS18B20_setResolution(resolution);
}
//--------------------------------------------------------------------------------------------------
//get resolution devider
uint8_t DS18B20_getDevider(DS18B20_Resolution resolution) {
  uint8_t devider;
  switch (resolution) {
  case DS18B20_Resolution_9_bit:
    devider = 8;
    break;
  case DS18B20_Resolution_10_bit:
    devider = 4;
    break;
  case DS18B20_Resolution_11_bit:
    devider = 2;
    break;
  case DS18B20_Resolution_12_bit:
  default:
    devider = 1;
    break;
  }
  
  return devider;
}
//--------------------------------------------------------------------------------------------------
//read temperature
uint16_t DS18B20_readTemperature() 
{
  return (uint16_t)(((float) ONEW_readWord() / 16.0) * 10.0);
}
//--------------------------------------------------------------------------------------------------
//get temperature
uint16_t DS18B20_getTemperature() {
  ONEW_reset();
  ONEW_writeByte(SKIP_ROM);
  ONEW_writeByte(CONVERT_T);
  _delay_ms(DELAY_WAIT_CONVERT);
  
  ONEW_reset();
  ONEW_writeByte(SKIP_ROM);
  ONEW_writeByte(READ_SCRATCHPAD);
  
  return DS18B20_readTemperature();
}
//--------------------------------------------------------------------------------------------------
//Set resolution
void DS18B20_setResolution(DS18B20_Resolution resolution) {
  ONEW_reset();
  ONEW_writeByte(SKIP_ROM);
  ONEW_writeByte(WRITE_SCRATCHPAD);
  ONEW_writeByte(TH_REGISTER);
  ONEW_writeByte(TL_REGISTER);
  ONEW_writeByte(resolution);
  DELAY_WAIT_CONVERT = DELAY_T_CONVERT / DS18B20_getDevider(resolution);
}