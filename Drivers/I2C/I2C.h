
#ifndef I2C_H
#define I2C_H

#include "stdint.h"

#define I2C_PORT I2C0 //I2c port

uint8_t I2C_write(uint8_t addr, uint8_t *tx_data, uint16_t tx_len); //write
uint8_t I2C_readWrite(uint8_t addr, uint8_t *tx_data, uint16_t tx_len, uint8_t *rx_data, uint16_t rx_len); //readWrite
uint8_t I2C_read(uint8_t addr, uint8_t *rx_data, uint16_t rx_len);  //read
uint8_t I2C_writeReg(uint8_t addr, uint8_t reg_addr, uint8_t value); //writeReg
uint8_t I2C_readReg(uint8_t addr, uint8_t reg_addr, uint8_t *value); //readReg
uint8_t I2C_writeByte(uint8_t addr, uint8_t data); //writeByte

#endif