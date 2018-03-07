
#include "stdint.h"
#include "InitDevice.h"
#include "EXT_Flash.h"
#include "InitDevice.h"
#include <intrinsics.h>
#include "em_int.h"
#include "em_gpio.h"
#include "CRC8.h"
#include "em_msc.h"
#include "Delay.h"

#define PROGRAMM_IMAGE_START_ADDRESS    0x00003000L

uint32_t ApplicationAddress = PROGRAMM_IMAGE_START_ADDRESS;
uint32_t JumpAddress;
typedef void ( *pFunction )( void );
pFunction Jump_To_Application;
EXT_FLASH_image_t booloader;
uint8_t id = 0;
uint8_t flash_buffer[ EXT_FLASH_IO_SIZE ];
uint8_t crc_val = 0;

int main()
{
  SystemInit(); //Init chip with errata
  
  enter_DefaultMode_from_RESET();
  
  id = EXT_Flash_get_DevIDReg();
  
  if ( id != 0x15 )
    goto jump_to_application;
  
  EXT_Flash_readData(EXT_FLASH_CONTROL_STRUCT, (uint8_t*)&booloader, sizeof( EXT_FLASH_image_t ));
  
  //booloader.statusCode = COMMAND_TO_REPROGRAMM;
  
  //Check reprogram status
  if ( booloader.statusCode != COMMAND_TO_REPROGRAMM )
    goto jump_to_application;
  
  //Calculate CRC value
  for ( uint32_t address = 0; address < booloader.imageSize; address += EXT_FLASH_IO_SIZE )
  {
    int block_len = booloader.imageSize - address;
    if ( block_len > EXT_FLASH_IO_SIZE )
      block_len = EXT_FLASH_IO_SIZE;
    EXT_Flash_readData( EXT_FLASH_PROGRAMM_IMAGE + address, flash_buffer, block_len );
    crc_val = crc8( flash_buffer, block_len, crc_val);
  }
  
  //Check CRC
  if ( crc_val != booloader.imageCRC )
    goto jump_to_application;
  
  //Writing firmware to internal flash
  MSC_Init();
  
  for ( uint32_t address = 0; address < booloader.imageSize; address += EXT_FLASH_IO_SIZE )
  {
    int block_len = booloader.imageSize - address;
    if ( block_len > EXT_FLASH_IO_SIZE )
      block_len = EXT_FLASH_IO_SIZE;
    EXT_Flash_readData( EXT_FLASH_PROGRAMM_IMAGE + address, flash_buffer, block_len );
    
    MSC_ErasePage( (uint32_t*)(PROGRAMM_IMAGE_START_ADDRESS + address) );
    for ( int i = 0; i < block_len ; i += 4 )
      MSC_WriteWord( (uint32_t*)(PROGRAMM_IMAGE_START_ADDRESS + address + i), &flash_buffer[ i ], 1);
  }  
  
  MSC_Deinit();
  
  //Writing status
  EXT_Flash_readData( EXT_FLASH_CONTROL_STRUCT, flash_buffer, EXT_FLASH_IO_SIZE );
  ((EXT_FLASH_image_t*)flash_buffer)->statusCode = 0x00000002L;
  EXT_Flash_writeData( EXT_FLASH_CONTROL_STRUCT, flash_buffer, EXT_FLASH_IO_SIZE );
  
//Jump to main program
jump_to_application:
  JumpAddress = *( uint32_t* )( ApplicationAddress + 4 );
  Jump_To_Application = ( pFunction )JumpAddress;
  uint32_t Stack = *( uint32_t* ) ApplicationAddress;
  INT_Disable();
  __set_MSP( Stack );

  Jump_To_Application();
  
  return 0;
  
}
