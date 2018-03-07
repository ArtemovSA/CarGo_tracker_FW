
#include "I2C.h"
#include "stdlib.h"
#include "em_i2c.h"
#include "em_emu.h"
#include "stdint.h"
#include "Delay.h"

I2C_TypeDef *i2c = I2C_PORT;  //I2C port

//--------------------------------------------------------------------------------------------------
//writeByte
uint8_t I2C_writeByte(uint8_t addr, uint8_t data)
{  
  return I2C_write(addr, &data, 1);
}
//--------------------------------------------------------------------------------------------------
//writeReg
uint8_t I2C_writeReg(uint8_t addr, uint8_t reg_addr, uint8_t value)
{
  uint8_t data[2];
  
  data[0] = reg_addr;
  data[1] = value;
  
  return I2C_write(addr, data,  2);
}
//--------------------------------------------------------------------------------------------------
//readReg
uint8_t I2C_readReg(uint8_t addr, uint8_t reg_addr, uint8_t *value)
{ 
  return I2C_readWrite(addr, &reg_addr, 1, value, 1);
}

//--------------------------------------------------------------------------------------------------
//write
uint8_t I2C_write(uint8_t addr, uint8_t *tx_data, uint16_t tx_len)
{
  I2C_TransferSeq_TypeDef seq;
  I2C_TransferReturn_TypeDef I2C_Status;
  //uint16_t try_count;
  
  seq.addr = addr;
  seq.flags = I2C_FLAG_WRITE;
  seq.buf[0].data = tx_data;
  seq.buf[0].len = tx_len;
  seq.buf[1].data = NULL;
  seq.buf[1].len = 0;
  I2C_Status = I2C_TransferInit(i2c, &seq);

  //I2C_Status = I2C_Transfer(i2c);
  while (I2C_Status == i2cTransferInProgress)
  {
    I2C_Status = I2C_Transfer(i2c);
//    if (try_count >= 500)
//    {
//      break;
//    }else{
//      try_count++;
//      _delay_ms(1);
//    }
  }

  if (I2C_Status == i2cTransferDone)
    return 1;
  
  return 0;  
}
//--------------------------------------------------------------------------------------------------
//readWrite
uint8_t I2C_readWrite(uint8_t addr, uint8_t *tx_data, uint16_t tx_len, uint8_t *rx_data, uint16_t rx_len) 
{
  I2C_TransferSeq_TypeDef seq;
  I2C_TransferReturn_TypeDef I2C_Status;
//  uint16_t try_count;
  
  seq.addr = addr;
  seq.flags = I2C_FLAG_WRITE_READ;
  seq.buf[0].data = tx_data;
  seq.buf[0].len = tx_len;
  
  seq.buf[1].data = rx_data;
  seq.buf[1].len = rx_len;
  
  I2C_Status = I2C_TransferInit(i2c, &seq);

  while (I2C_Status == i2cTransferInProgress)
  {
    I2C_Status = I2C_Transfer(i2c);
//    if (try_count >= 500)
//    {
//      break;
//    }else{
//      try_count++;
//      _delay_ms(1);
//    }
  }
  
  if (I2C_Status == i2cTransferDone)
    return 1;
  
  return 0;  
}
//--------------------------------------------------------------------------------------------------
//read
uint8_t I2C_read(uint8_t addr, uint8_t *rx_data, uint16_t rx_len) 
{
  I2C_TransferSeq_TypeDef seq;
  I2C_TransferReturn_TypeDef I2C_Status;
  //uint16_t try_count;
  
  seq.addr = addr;
  seq.flags = I2C_FLAG_READ;
  seq.buf[0].data = rx_data;
  seq.buf[0].len = rx_len;
  
  I2C_Status = I2C_TransferInit(i2c, &seq);

  while (I2C_Status == i2cTransferInProgress)
  {
    I2C_Status = I2C_Transfer(i2c);
//    if (try_count >= 500)
//    {
//      break;
//    }else{
//      try_count++;
//      _delay_ms(2);
//    }
  }
  
  if (I2C_Status == i2cTransferDone)
    return 1;
  
  return 0;  
}
