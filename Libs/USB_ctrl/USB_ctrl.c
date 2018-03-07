
#include "USB_ctrl.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

#include "ADD_func.h"
#include "cdc.h"
#include "crc16.h"
#include "EXT_Flash.h"

uint8_t USB_rx_buf[60];
uint8_t USB_tx_buf[60];
uint8_t USB_rx_count = 0;

uint8_t USB_command;
uint8_t USB_state = USB_STATE_WAIT_STOP1;
const char USB_STOP_1 = 0x54;
const char USB_STOP_2 = 0x55;

extern xSemaphoreHandle extFlash_mutex;

void USB_msg_process()
{
  uint16_t crc_val; //Значение CRC
  uint32_t addr; //Адрес
  uint16_t len; //Длина
  
  crc_val = crc16(USB_rx_buf,USB_rx_count-2);
  
  if (crc_val == 0x0000) {
    
    USB_command = USB_rx_buf[0];
    
     switch(USB_command) {
       
     case USB_CMD_FLASH_ARR_WRITE:
       addr = (USB_rx_buf[1]<<16) | (USB_rx_buf[2]<<8) | USB_rx_buf[3];
       len = (USB_rx_buf[4]<<8) | USB_rx_buf[5];  
       
       if ( xSemaphoreTake(extFlash_mutex, 100) == pdTRUE ) {
         EXT_Flash_ReWriteData(addr, &USB_rx_buf[6], len); //Переписать массив данных в секторе
         xSemaphoreGive(extFlash_mutex);
         
         //Формирование ответа
         memcpy(USB_tx_buf, USB_rx_buf, 5);
//         USB_tx_buf[0] = USB_rx_buf[0]; //Команда
//         USB_tx_buf[1] = USB_rx_buf[1]; //Адрес
//         USB_tx_buf[2] = USB_rx_buf[2]; //Адрес
//         USB_tx_buf[3] = USB_rx_buf[3]; //Адрес
//         USB_tx_buf[4] = USB_rx_buf[4]; //Длина
//         USB_tx_buf[5] = USB_rx_buf[5]; //Длина
         
         //Расчет CRC
         crc_val = crc16(USB_tx_buf,6);
         USB_tx_buf[6] = (crc_val & 0x00FF);//CRC16
         USB_tx_buf[7] = (crc_val & 0xFF00) >> 8; //CRC16
         
         USB_tx_buf[8] = USB_STOP_1; //Стоповый байт
         USB_tx_buf[9] = USB_STOP_2; //Стоповый байт        
         
         USB_Send(USB_tx_buf,10); //Отправить ответ
         
       }
       break;
       
     case USB_CMD_FLASH_ARR_READ:
       
      addr = (USB_rx_buf[1]<<16) | (USB_rx_buf[2]<<8) | USB_rx_buf[3];
      len = ADD(USB_rx_buf[5],USB_rx_buf[4]);
      
      memset(USB_tx_buf,0,sizeof(USB_tx_buf));
      
      if ( xSemaphoreTake(extFlash_mutex, 100) == pdTRUE ) {
        EXT_Flash_readData(addr, &USB_tx_buf[6], len); //Читать данные из флеш
        xSemaphoreGive(extFlash_mutex);
        
        //Формирование ответа
        memcpy(USB_tx_buf, USB_rx_buf, 5);
//        USB_tx_buf[0] = USB_rx_buf[0]; //Команда
//        USB_tx_buf[1] = USB_rx_buf[1]; //Адрес
//        USB_tx_buf[2] = USB_rx_buf[2]; //Адрес
//        USB_tx_buf[3] = USB_rx_buf[3]; //Адрес
//        USB_tx_buf[4] = USB_rx_buf[4]; //Длина
//        USB_tx_buf[5] = USB_rx_buf[5]; //Длина
        
        
        //Расчет CRC
        crc_val = crc16(USB_tx_buf,len+6);
        USB_tx_buf[len+6] = (crc_val & 0x00FF); //CRC16
        USB_tx_buf[len+7] = (crc_val & 0xFF00) >> 8; //CRC16
        
        USB_tx_buf[len+8] = USB_STOP_1; //Стоповый байт
        USB_tx_buf[len+9] = USB_STOP_2; //Стоповый байт        
      
        USB_Send(USB_tx_buf,len+10); //Отправить ответ

      }
      
      break;
     case USB_CMD_FLASH_BYTE_WRITE:
       
      addr = (USB_tx_buf[1]<<16) | (USB_tx_buf[2]<<8) | USB_tx_buf[3];

      if ( xSemaphoreTake(extFlash_mutex, 100) == pdTRUE ) {
        EXT_Flash_ReWriteData(addr,&USB_rx_buf[4], 1); //Записать данные   
        xSemaphoreGive(extFlash_mutex);
        
        //Формирование ответа
        memcpy(USB_tx_buf, USB_rx_buf, 3);
//        USB_tx_buf[0] = USB_rx_buf[0]; //Команда
//        USB_tx_buf[1] = USB_rx_buf[1]; //Адрес
//        USB_tx_buf[2] = USB_rx_buf[2]; //Адрес
//        USB_tx_buf[3] = USB_rx_buf[3]; //Адрес
        
        //Расчет CRC
        crc_val = crc16(USB_tx_buf,4);
        USB_tx_buf[4] = (crc_val & 0x00FF); //CRC16
        USB_tx_buf[5] = (crc_val & 0xFF00) >> 8; //CRC16
        
        USB_tx_buf[6] = USB_STOP_1; //Стоповый байт
        USB_tx_buf[7] = USB_STOP_2; //Стоповый байт        
        
        USB_Send(USB_tx_buf,8); //Отправить ответ
      }
      
       break;
     case USB_CMD_FLASH_BYTE_READ:
       
      addr = (USB_rx_buf[1]<<16) | (USB_rx_buf[2]<<8) | USB_rx_buf[3];

      if ( xSemaphoreTake(extFlash_mutex, 100) == pdTRUE ) {
        EXT_Flash_readData(addr, &USB_tx_buf[4], 1); //Читать данные из флеш
        xSemaphoreGive(extFlash_mutex);
        
        //Формирование ответа
        memcpy(USB_tx_buf, USB_rx_buf, 3);
//        USB_tx_buf[0] = USB_rx_buf[0]; //Команда
//        USB_tx_buf[1] = USB_rx_buf[1]; //Адрес
//        USB_tx_buf[2] = USB_rx_buf[2]; //Адрес
//        USB_tx_buf[3] = USB_rx_buf[3]; //Адрес
        
        //Расчет CRC
        crc_val = crc16(USB_tx_buf,5);
        USB_tx_buf[5] = (crc_val & 0x00FF); //CRC16
        USB_tx_buf[6] = (crc_val & 0xFF00) >> 8; //CRC16
        
        USB_tx_buf[7] = USB_STOP_1; //Стоповый байт
        USB_tx_buf[8] = USB_STOP_2; //Стоповый байт        
        
        USB_Send(USB_tx_buf,9); //Отправить ответ
      }
      
      break;
     }

  }else{
    
  }
}

void USBReceive_int(uint8_t *data, int byteCount)
{
  if (USB_rx_count <= sizeof(USB_rx_buf))
    USB_rx_count = 0;
  
  memcpy(&USB_rx_buf[USB_rx_count], data, byteCount);
  USB_rx_count += byteCount;
  
  for (int i=0; i<byteCount; i++)
  {
    if (USB_state == USB_STATE_WAIT_STOP2)
      if (data[i] == USB_STOP_2)
      {
        USB_state = USB_STATE_MSG_PROCESS;
      }else{
        USB_state = USB_STATE_WAIT_STOP1;
      }
    
    if (USB_state == USB_STATE_WAIT_STOP1)
      if (data[i] == USB_STOP_1)
      {
        USB_state = USB_STATE_WAIT_STOP2;
      }else{
        USB_state = USB_STATE_WAIT_STOP1;
      }
  }
}
