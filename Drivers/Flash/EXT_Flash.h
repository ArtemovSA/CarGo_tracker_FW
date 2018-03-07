
#ifndef EXT_FLASH_H
#define EXT_FLASH_H

#include "stdint.h"
#include "InitDevice.h"

#define EXT_FLASH_CS_PORT       USART0_CS_PORT
#define EXT_FLASH_CS_PIN        USART0_CS_PIN
#define EXT_FLASH_SCK_PORT      USART0_CLK_PORT
#define EXT_FLASH_SCK_PIN       USART0_CLK_PIN
#define EXT_FLASH_MOSI_PORT     USART0_TX_PORT
#define EXT_FLASH_MOSI_PIN      USART0_TX_PIN
#define EXT_FLASH_MISO_PORT     USART0_RX_PORT
#define EXT_FLASH_MISO_PIN      USART0_RX_PIN

#define EXT_FLASH_SPI_CS_SET GPIO_PinOutSet(EXT_FLASH_CS_PORT, EXT_FLASH_CS_PIN)
#define EXT_FLASH_SPI_CS_RESET GPIO_PinOutClear(EXT_FLASH_CS_PORT, EXT_FLASH_CS_PIN)

#define COMMAND_TO_REPROGRAMM           0x12345678

//Параметры
#define EXT_FLASH_SIZE                  0x1FFFFF        //Размер памяти
#define EXT_FLASH_IO_SIZE               0x1000          //Буфера работы с памятью
#define EXT_FLASH_SECTOR_SIZE           0x1000          //Размер сектора 4Кб
#define EXT_FLASH_BLOCK_SIZE            0x10000         //Размер блока 64Кб

//Program part
#define EXT_FLASH_PROGRAMM_IMAGE        0x1F0000 //Блок для хранения прошивки
#define EXT_FLASH_TEMP_SECTOR           0x1E0000 //Сектор для хранения буфера  
#define EXT_FLASH_CONTROL_STRUCT        0x1E2000 //Контрольная структура
#define EXT_FLASH_PARAMS                0x1E3000 //Параметры
#define EXT_FLASH_SETTINGS              0x1E4000 //Настройки системы
#define EXT_FLASH_LOG_DATA              0x1E5000 //Лог

#define EXT_FLASH_LOG_DATA_SIZE         3        //Размера лога в блоках

//Регистры
#define EXT_FLASH_STATUS_GET 0x05
#define EXT_FLASH_STATUS_SET 0x01
#define EXT_FLASH_DEV_ID_GET 0xAB

//Статусный регистр
#define EXT_FLASH_STATUS_WIP 0
#define EXT_FLASH_STATUS_WEL 1

//Комманды
#define EXT_FLASH_READ_DATA 0x03
#define EXT_FLASH_FAST_READ_DATA 0x0B
#define EXT_FLASH_WRITE_DATA 0x02
#define EXT_FLASH_WRITE_ENABLE 0x06
#define EXT_FLASH_WRITE_DISABLE 0x04
#define EXT_FLASH_ERASE_BLOCK 0xD8
#define EXT_FLASH_ERASE_SECTOR 0x20
#define EXT_FLASH_ERASE_CHIP 0xC7

//Bootloader struct
typedef struct{
  int statusCode;
  int imageVersion;
  int imageVersion_new;
  int imageSize;
  int imageLoaded;
  int imageCRC;
}EXT_FLASH_image_t;

extern uint8_t EXT_Flash_init(); //Инициализация Flash
extern uint8_t EXT_Flash_get_DevIDReg(); //Читать DEV ID
extern uint8_t EXT_Flash_get_StatusReg(); //Читать статусный регистр
extern void EXT_Flash_set_StatusReg(uint8_t value); //Писать статусный регистр
extern void EXT_Flash_writeData(uint32_t address, uint8_t *data, uint16_t count); //Писать массив данных
extern void EXT_Flash_readData(uint32_t address, uint8_t *data, uint16_t count); //Читать массив данных
extern void EXT_Flash_erace_all();//Стереть память
extern void EXT_Flash_erace_sector(uint32_t address);//Стереть сектор 4 кб
extern void EXT_Flash_erace_block(uint32_t address);//Стереть блок 64 кб
extern void EXT_Flash_ReWriteData(uint32_t address, uint8_t *data, uint16_t count); //Переписать массив данных


#endif