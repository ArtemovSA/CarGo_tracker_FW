
#include "EXT_Flash.h"
#include "stdint.h"
#include "string.h"
#include "em_device.h"
#include "em_gpio.h"
#include "Delay.h"

uint8_t* SPI_RxBuffer;
int SPI_RxBufferSize;
volatile int SPI_RxBufferIndex;

uint8_t buf_flash[EXT_FLASH_IO_SIZE];

//Прототипы
void SPI_sendBuffer(uint8_t* txBuffer, uint16_t bytesToSend); //Send data
void SPI_reciveBuffer(uint8_t* rxBuffer, uint16_t bytesToRecive); //Recive data
void EXT_Flash_write_enable(); //Разрешить запись
uint8_t EXT_Flash_waitStatusByte(uint8_t bit_num, uint8_t val, uint16_t wait_ms); //Ждать выставления статусного бита

//-------------------------------------------------------------------------------------------------------
//Инициализация Flash
uint8_t EXT_Flash_init() {

  EXT_FLASH_SPI_CS_SET;
  uint8_t EXT_flash_id = EXT_Flash_get_DevIDReg();   //Читать DEV ID
  
  if (EXT_flash_id == 0x15)
    return 1;
  
  return 0;
}
//-------------------------------------------------------------------------------------------------------
//Писать статусный регистр
void EXT_Flash_set_StatusReg(uint8_t value) {
  
  uint8_t address = EXT_FLASH_STATUS_SET;
  
  EXT_FLASH_SPI_CS_RESET;
  
  SPI_sendBuffer(&address,1);
  SPI_sendBuffer(&value, 1);
  
  EXT_FLASH_SPI_CS_SET;
}
//-------------------------------------------------------------------------------------------------------
//Стереть память
void EXT_Flash_erace_all() {
  
  uint8_t command = EXT_FLASH_ERASE_CHIP;

  EXT_FLASH_SPI_CS_RESET;
  
  SPI_sendBuffer(&command,1);
  
  EXT_FLASH_SPI_CS_SET;
}
//-------------------------------------------------------------------------------------------------------
//Стереть сектор 4 кб
void EXT_Flash_erace_sector(uint32_t address) {
  
  uint8_t addr[3];
  uint8_t command = EXT_FLASH_ERASE_SECTOR;
  
  EXT_Flash_write_enable(); //Разрешить запись
  EXT_Flash_waitStatusByte(EXT_FLASH_STATUS_WEL, 1, 200); //Ждем сигнала приема данных
  
  addr[0] = (address & 0xFF0000) >> 16;
  addr[1] = (address & 0x00FF00) >> 8;
  addr[2] = (address & 0x0000FF);

  EXT_FLASH_SPI_CS_RESET;
  SPI_sendBuffer(&command,1);
  SPI_sendBuffer(addr,3);
  EXT_FLASH_SPI_CS_SET;
  _delay_ms(300);
  
  EXT_Flash_waitStatusByte(EXT_FLASH_STATUS_WIP, 0, 200); //Ждем завершения стирания
}
//-------------------------------------------------------------------------------------------------------
//Стереть блок 64 кб
void EXT_Flash_erace_block(uint32_t address) {
  
  uint8_t addr[3];
  uint8_t command = EXT_FLASH_ERASE_BLOCK;
  
  EXT_Flash_write_enable(); //Разрешить запись
  EXT_Flash_waitStatusByte(EXT_FLASH_STATUS_WEL, 1, 200); //Ждем сигнала приема данных
  
  addr[0] = (address & 0xFF0000) >> 16;
  addr[1] = (address & 0x00FF00) >> 8;
  addr[2] = (address & 0x0000FF);

  EXT_FLASH_SPI_CS_RESET;
  SPI_sendBuffer(&command,1);
  SPI_sendBuffer(addr,3);
  EXT_FLASH_SPI_CS_SET;
  _delay_ms(1000);
  
  EXT_Flash_waitStatusByte(EXT_FLASH_STATUS_WIP, 0, 5000); //Ждем стирания
}
//-------------------------------------------------------------------------------------------------------
//Разрешить запись
void EXT_Flash_write_enable() {
  
  uint8_t address = EXT_FLASH_WRITE_ENABLE;

  EXT_FLASH_SPI_CS_RESET;
  SPI_sendBuffer(&address,1);
  EXT_FLASH_SPI_CS_SET;
}

//-------------------------------------------------------------------------------------------------------
//Читать статусный регистр
uint8_t EXT_Flash_get_StatusReg() {
  
  uint8_t value_flash;
  uint8_t address = EXT_FLASH_STATUS_GET;
  
  EXT_FLASH_SPI_CS_RESET;
  SPI_sendBuffer(&address,1);
  SPI_reciveBuffer(&value_flash, 1);
  EXT_FLASH_SPI_CS_SET;
  return value_flash;
}
//-------------------------------------------------------------------------------------------------------
//Читать DEV ID
uint8_t EXT_Flash_get_DevIDReg() {
  
  uint8_t value_flash;
  uint8_t address = EXT_FLASH_DEV_ID_GET;
  uint8_t dummy[3];
  
  dummy[0] = 0x00;
  dummy[1] = 0x00;
  dummy[2] = 0x00;
  
  EXT_FLASH_SPI_CS_RESET;
  
  SPI_sendBuffer(&address,1);
  SPI_sendBuffer(dummy,3);
  SPI_reciveBuffer(&value_flash, 1);
  
  EXT_FLASH_SPI_CS_SET;
  
  return value_flash;
}
//-------------------------------------------------------------------------------------------------------
//Переписать массив данных в секторе
void EXT_Flash_ReWriteData(uint32_t address, uint8_t *data, uint16_t count) {

  uint32_t sector_addr = (uint32_t)(address/EXT_FLASH_SECTOR_SIZE)*EXT_FLASH_SECTOR_SIZE; //Найти адрес сектора
  uint16_t len = 0;
  uint32_t part_addr = sector_addr;
   
  memset(buf_flash,0,sizeof(buf_flash)); //Очистить буфер
  
  EXT_Flash_erace_sector(EXT_FLASH_TEMP_SECTOR); //Стереть буферный сектор
  _delay_ms(10);
  
  //По частям прочитать данные из памяти, заменить и записать в буферный сектор
  for (uint16_t part_n = 0; part_n<EXT_FLASH_SECTOR_SIZE/EXT_FLASH_IO_SIZE; part_n++)
  {
    part_addr = sector_addr+part_n*EXT_FLASH_IO_SIZE;
    EXT_Flash_readData(part_addr, buf_flash,EXT_FLASH_IO_SIZE); //Читать сектор
    
    //Если начало фрагмента данных лежит в уже считанной части
    if ((address <= part_addr+EXT_FLASH_IO_SIZE)&(count > 0))
    {
      //Расчет длины записи
      if (count > EXT_FLASH_IO_SIZE)
        len = part_addr+EXT_FLASH_IO_SIZE-address;
      else
        len = count;
      
     //Защита буфера
     if (len > EXT_FLASH_IO_SIZE)
        len = EXT_FLASH_IO_SIZE;
      
      memcpy(&buf_flash[address-part_addr],data,len); //Переписать данные

      data += len;
      address += len;
      count -= len;
    }
    
    //Записать часть в буферный сектор
    EXT_Flash_writeData(EXT_FLASH_TEMP_SECTOR+part_n*EXT_FLASH_IO_SIZE, buf_flash,EXT_FLASH_IO_SIZE);
  }
  
  //Стереть целевой сектор
  EXT_Flash_erace_sector(sector_addr);
  _delay_ms(10);
  
  //Переписать буферный сектор обратно
  //По частям прочитать данные из памяти, заменить и записать в буферный сектор
  for (uint16_t part_n = 0; part_n<EXT_FLASH_SECTOR_SIZE/EXT_FLASH_IO_SIZE; part_n++)
  {
    EXT_Flash_readData(EXT_FLASH_TEMP_SECTOR+part_n*EXT_FLASH_IO_SIZE, buf_flash, EXT_FLASH_IO_SIZE);
    EXT_Flash_writeData(sector_addr+part_n*EXT_FLASH_IO_SIZE, buf_flash,EXT_FLASH_IO_SIZE);
  }
}
//-------------------------------------------------------------------------------------------------------
//Ждать выставления статусного бита
uint8_t EXT_Flash_waitStatusByte(uint8_t bit_num, uint8_t val, uint16_t wait_ms) {
  uint16_t try_counter = 0;
  while ((EXT_Flash_get_StatusReg() & (1<<bit_num)) != (val<<bit_num)) {
    if (try_counter == wait_ms)
      return 0;
   _delay_ms(1);
   try_counter++;
  }
  return 1;
}
//-------------------------------------------------------------------------------------------------------
//Писать массив данных
void EXT_Flash_writeData(uint32_t address, uint8_t *data, uint16_t count) {
  
  uint8_t addr[3];
  uint8_t command = EXT_FLASH_WRITE_DATA;
  uint8_t count_full = (uint32_t)(count/EXT_FLASH_IO_SIZE); //Найти количество страниц
  uint32_t buf_addr = address;
  uint16_t count_end;
  
  //Записываем целые страницы
  for (int i=0; i<count_full; i++) {
    
    EXT_Flash_write_enable(); //Разрешить запись
    EXT_Flash_waitStatusByte(EXT_FLASH_STATUS_WEL, 1, 200); //Ждем флага приема данных
    
    addr[0] = (buf_addr & 0xFF0000) >> 16;
    addr[1] = (buf_addr & 0x00FF00) >> 8;
    addr[2] = (buf_addr & 0x0000FF);
    
    memcpy(buf_flash,&data[EXT_FLASH_IO_SIZE*i],EXT_FLASH_IO_SIZE);
    
    EXT_FLASH_SPI_CS_RESET;
    SPI_sendBuffer(&command, 1);
    SPI_sendBuffer(addr, 3);
    SPI_sendBuffer(buf_flash, EXT_FLASH_IO_SIZE);
    EXT_FLASH_SPI_CS_SET;
    _delay_ms(5);
    
    EXT_Flash_waitStatusByte(EXT_FLASH_STATUS_WIP, 0, 200); //Ждем завершения записи
    
    buf_addr += EXT_FLASH_IO_SIZE;
  }
  
  //Считаем количество оставшися байт
  count_end = count-(count_full*EXT_FLASH_IO_SIZE);
  
  //Записываем остальные данные
  if (count_end > 0) {
    
    EXT_Flash_write_enable(); //Разрешить запись
    EXT_Flash_waitStatusByte(EXT_FLASH_STATUS_WEL, 1, 200); //Ждем флага приема данных
    
    addr[0] = (buf_addr & 0xFF0000) >> 16;
    addr[1] = (buf_addr & 0x00FF00) >> 8;
    addr[2] = (buf_addr & 0x0000FF);
    
    memcpy(buf_flash,&data[EXT_FLASH_IO_SIZE*count_full],count_end);
    
    EXT_FLASH_SPI_CS_RESET;
    SPI_sendBuffer(&command, 1);
    SPI_sendBuffer(addr, 3);
    SPI_sendBuffer(buf_flash, count_end);
    EXT_FLASH_SPI_CS_SET;
    _delay_ms(5);
    
    EXT_Flash_waitStatusByte(EXT_FLASH_STATUS_WIP, 0, 200); //Ждем завершения записи
  }  
  
}
//-------------------------------------------------------------------------------------------------------
//Читать массив данных
void EXT_Flash_readData(uint32_t address, uint8_t *data, uint16_t count) {
  
  uint8_t addr[3];
  uint8_t command = EXT_FLASH_READ_DATA;
  uint8_t count_full = (uint32_t)(count/EXT_FLASH_IO_SIZE); //Найти количество пакетов
  uint32_t buf_addr = address;
    
  //Читаем целые пакеты
  for (int i=0; i<count_full; i++) {
  
    addr[0] = (buf_addr & 0xFF0000) >> 16;
    addr[1] = (buf_addr & 0x00FF00) >> 8;
    addr[2] = (buf_addr & 0x0000FF);
    
    EXT_FLASH_SPI_CS_RESET;
    
    SPI_sendBuffer(&command, 1);
    SPI_sendBuffer(addr, 3);
    SPI_reciveBuffer(&data[EXT_FLASH_IO_SIZE*i], EXT_FLASH_IO_SIZE);
    
    EXT_FLASH_SPI_CS_SET;
    
    buf_addr += EXT_FLASH_IO_SIZE;
  }
  
  //Получаем остальные данные
  if (count-(count_full*EXT_FLASH_IO_SIZE) > 0) {
    
    addr[0] = (buf_addr & 0xFF0000) >> 16;
    addr[1] = (buf_addr & 0x00FF00) >> 8;
    addr[2] = (buf_addr & 0x0000FF);
    
    EXT_FLASH_SPI_CS_RESET;
    
    SPI_sendBuffer(&command, 1);
    SPI_sendBuffer(addr, 3);
    SPI_reciveBuffer(&data[EXT_FLASH_IO_SIZE*count_full], count-(count_full*EXT_FLASH_IO_SIZE));
    
    EXT_FLASH_SPI_CS_SET;
  }
}
//-------------------------------------------------------------------------------------------------------
//Recive data
void SPI_reciveBuffer(uint8_t* rxBuffer, uint16_t bytesToRecive)
{
  int ii;

  SPI_RxBuffer      = rxBuffer;
  SPI_RxBufferSize  = bytesToRecive;
  SPI_RxBufferIndex = 0;
  
  USART0->CMD = USART_CMD_CLEARRX;

  /* Enable interrupts */
  NVIC_ClearPendingIRQ(USART0_RX_IRQn);
  NVIC_EnableIRQ(USART0_RX_IRQn);
  
  USART0->IEN |= USART_IEN_RXDATAV;
  
  for (ii = 0; ii < bytesToRecive;  ii++)
  {
    while (!(USART0->STATUS & USART_STATUS_TXBL));
    USART0->TXDATA = 0;
  }

  while (!(USART0->STATUS & USART_STATUS_TXC));

}
//-------------------------------------------------------------------------------------------------------
//Send data
void SPI_sendBuffer(uint8_t* txBuffer, uint16_t bytesToSend)
{
  /* Sending the data */
  for (int ii = 0; ii < bytesToSend;  ii++)
  {
    /* Waiting for the usart to be ready */
    while (!(USART0->STATUS & USART_STATUS_TXBL)) ;

    if (txBuffer != 0)
    {
      /* Writing next byte to USART */
      USART0->TXDATA = *txBuffer;
      txBuffer++;
    }
    else
    {
      USART0->TXDATA = 0;
    }
  }

  /*Waiting for transmission of last byte */
  while (!(USART0->STATUS & USART_STATUS_TXC)) ;
}
//-------------------------------------------------------------------------------------------------------
//RX IRQ
void USART0_RX_IRQHandler(void)
{
  uint8_t rxdata;

  if (USART0->STATUS & USART_STATUS_RXDATAV)
  {
    /* Reading out data */
    rxdata = USART0->RXDATA;

    if (SPI_RxBufferIndex < SPI_RxBufferSize)
    {
      /* Store Data */
      SPI_RxBuffer[SPI_RxBufferIndex] = rxdata;
      SPI_RxBufferIndex++;
    }
  }
  NVIC_ClearPendingIRQ(USART0_RX_IRQn);
}