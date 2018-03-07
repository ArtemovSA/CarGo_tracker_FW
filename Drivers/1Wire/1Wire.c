
#include "1Wire.h"
#include "Delay.h"
#include "em_gpio.h"
#include "em_cmu.h"

#define DELAY_RESET           500
#define DELAY_WRITE_0         60
#define DELAY_WRITE_0_PAUSE   10
#define DELAY_WRITE_1         10
#define DELAY_WRITE_1_PAUSE   60
#define DELAY_READ_SLOT       10
#define DELAY_BUS_RELAX       10
#define DELAY_READ_PAUSE      50
#define DELAT_PROTECTION      5

//--------------------------------------------------------------------------------------------------
//init
void ONEW_init() {
  
  GPIO_PinModeSet(ONEWIRE_PORT, ONEWIRE_PIN, gpioModeWiredAndPullUp, 0);
  GPIO_PinOutClear(ONEWIRE_PORT, ONEWIRE_PIN);
  
  // 50MHz output open-drain
//  GPIOB->CRL |= GPIO_CRL_MODE5;
//  GPIOB->CRL |= GPIO_CRL_CNF5_0;
//  GPIOB->CRL &= ~GPIO_CRL_CNF5_1;
  
}
//--------------------------------------------------------------------------------------------------
//reset
void ONEW_reset() 
{
  GPIO_PinOutClear(ONEWIRE_PORT, ONEWIRE_PIN);
  //GPIOB->ODR &= ~GPIO_ODR_ODR5;
  _delay_timer_ms(DELAY_RESET);
  GPIO_PinOutSet(ONEWIRE_PORT, ONEWIRE_PIN);
  //GPIOB->ODR |= GPIO_ODR_ODR5;
  _delay_timer_ms(DELAY_RESET);
}
//--------------------------------------------------------------------------------------------------
//Write bit
void ONEW_writeBit(uint8_t bit) 
{
  GPIO_PinOutClear(ONEWIRE_PORT, ONEWIRE_PIN);
  //GPIOB->ODR &= ~GPIO_ODR_ODR5; //pull down
  _delay_timer_ms(bit ? DELAY_WRITE_1 : DELAY_WRITE_0);
  GPIO_PinOutSet(ONEWIRE_PORT, ONEWIRE_PIN);
  //GPIOB->ODR |= GPIO_ODR_ODR5; //pull up
  _delay_timer_ms(bit ? DELAY_WRITE_1_PAUSE : DELAY_WRITE_0_PAUSE);
}
//--------------------------------------------------------------------------------------------------
//Write Byte
void ONEW_writeByte(uint8_t data) {
  
  for (uint8_t i = 0; i < 8; i++) {
    ONEW_writeBit(data >> i & 1);
    _delay_timer_ms(DELAT_PROTECTION);
  }
  
}
//--------------------------------------------------------------------------------------------------
//Read bit
uint8_t ONEW_readBit() {
  uint8_t bit = 0;
  GPIO_PinOutClear(ONEWIRE_PORT, ONEWIRE_PIN);
  //GPIOB->ODR &= ~GPIO_ODR_ODR5;
  _delay_timer_ms(DELAY_READ_SLOT);
  GPIO_PinOutSet(ONEWIRE_PORT, ONEWIRE_PIN);
  //GPIOB->ODR |= GPIO_ODR_ODR5;
  // ... switch to INPUT
  GPIO_PinModeSet(ONEWIRE_PORT, ONEWIRE_PIN, gpioModeInputPull, 0);
  _delay_timer_ms(DELAY_BUS_RELAX);
  bit = (GPIO_PinInGet(ONEWIRE_PORT, ONEWIRE_PIN) ? 1 : 0);
  _delay_timer_ms(DELAY_READ_PAUSE);
  GPIO_PinModeSet(ONEWIRE_PORT, ONEWIRE_PIN, gpioModeWiredAndPullUp, 0);
  // ... switch to OUTPUT
  return bit;
}
//--------------------------------------------------------------------------------------------------
//Read byte
uint8_t ONEW_readByte()
{
  uint8_t data = 0;
  for (uint8_t i = 0; i < 16; i++)
    data += (uint8_t) ONEW_readBit() << i;
  
  return data;
}
//--------------------------------------------------------------------------------------------------
//Read word
uint16_t ONEW_readWord()
{
  uint16_t data = 0;
  for (uint8_t i = 0; i < 16; i++)
    data += (uint16_t) ONEW_readBit() << i;
  
  return data;
}
//--------------------------------------------------------------------------------------------------
//Search
uint8_t ONEW_Search(OneWire_t* OneWireStruct, uint8_t command) {
  
  uint8_t id_bit_number;
  uint8_t last_zero, rom_byte_number, search_result;
  uint8_t id_bit, cmp_id_bit;
  uint8_t rom_byte_mask, search_direction;
  
  /* Initialize for search */
  id_bit_number = 1;
  last_zero = 0;
  rom_byte_number = 0;
  rom_byte_mask = 1;
  search_result = 0;
  
  // if the last call was not the last one
  if (!OneWireStruct->LastDeviceFlag)
  {
    // 1-Wire reset
    if (OneWire_Reset(OneWireStruct)) 
    {
      /* Reset the search */
      OneWireStruct->LastDiscrepancy = 0;
      OneWireStruct->LastDeviceFlag = 0;
      OneWireStruct->LastFamilyDiscrepancy = 0;
      return 0;
    }
    
    // issue the search command 
    OneWire_WriteByte(OneWireStruct, command);  
    
    // loop to do the search
    do {
      // read a bit and its complement
      id_bit = OneWire_ReadBit(OneWireStruct);
      cmp_id_bit = OneWire_ReadBit(OneWireStruct);
      
      // check for no devices on 1-wire
      if ((id_bit == 1) && (cmp_id_bit == 1)) {
        break;
      } else {
        // all devices coupled have 0 or 1
        if (id_bit != cmp_id_bit) {
          search_direction = id_bit;  // bit write value for search
        } else {
          // if this discrepancy if before the Last Discrepancy
          // on a previous next then pick the same as last time
          if (id_bit_number < OneWireStruct->LastDiscrepancy) {
            search_direction = ((OneWireStruct->ROM_NO[rom_byte_number] & rom_byte_mask) > 0);
          } else {
            // if equal to last pick 1, if not then pick 0
            search_direction = (id_bit_number == OneWireStruct->LastDiscrepancy);
          }
          
          // if 0 was picked then record its position in LastZero
          if (search_direction == 0) {
            last_zero = id_bit_number;
            
            // check for Last discrepancy in family
            if (last_zero < 9) {
              OneWireStruct->LastFamilyDiscrepancy = last_zero;
            }
          }
        }
        
        // set or clear the bit in the ROM byte rom_byte_number
        // with mask rom_byte_mask
        if (search_direction == 1) {
          OneWireStruct->ROM_NO[rom_byte_number] |= rom_byte_mask;
        } else {
          OneWireStruct->ROM_NO[rom_byte_number] &= ~rom_byte_mask;
        }
        
        // serial number search direction write bit
        OneWire_WriteBit(OneWireStruct, search_direction);
        
        // increment the byte counter id_bit_number
        // and shift the mask rom_byte_mask
        id_bit_number++;
        rom_byte_mask <<= 1;
        
        // if the mask is 0 then go to new SerialNum byte rom_byte_number and reset mask
        if (rom_byte_mask == 0) {
          //docrc8(ROM_NO[rom_byte_number]);  // accumulate the CRC
          rom_byte_number++;
          rom_byte_mask = 1;
        }
      }
    } while (rom_byte_number < 8);  // loop until through all ROM bytes 0-7
    
    // if the search was successful then
    if (!(id_bit_number < 65)) {
      // search successful so set LastDiscrepancy,LastDeviceFlag,search_result
      OneWireStruct->LastDiscrepancy = last_zero;
      
      // check for last device
      if (OneWireStruct->LastDiscrepancy == 0) {
        OneWireStruct->LastDeviceFlag = 1;
      }
      
      search_result = 1;
    }
  }
  
  // if no device found then reset counters so next 'search' will be like a first
  if (!search_result || !OneWireStruct->ROM_NO[0]) {
    OneWireStruct->LastDiscrepancy = 0;
    OneWireStruct->LastDeviceFlag = 0;
    OneWireStruct->LastFamilyDiscrepancy = 0;
    search_result = 0;
  }
  
  return search_result;
}
//--------------------------------------------------------------------------------------------------
//Reset search
void OneWire_ResetSearch(OneWire_t* OneWireStruct) {
	/* Reset the search state */
	OneWireStruct->LastDiscrepancy = 0;
	OneWireStruct->LastDeviceFlag = 0;
	OneWireStruct->LastFamilyDiscrepancy = 0;
}