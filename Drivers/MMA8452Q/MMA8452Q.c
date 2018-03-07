
#include "MMA8452Q.h"
#include "math.h"
#include "I2C.h"
#include "Task_transfer.h"

short MMA8452Q_delta_all;
short MMA8452Q_delta_val_x;
short MMA8452Q_delta_val_y;
short MMA8452Q_delta_val_z;
short middle_val_x;
short middle_val_y;
short middle_val_z;
short x_arr[ACC_WINDOW_SIZE];
short y_arr[ACC_WINDOW_SIZE];
short z_arr[ACC_WINDOW_SIZE];
int32_t sum_x;
int32_t sum_y;
int32_t sum_z;
uint8_t arr_start, arr_end;
uint16_t calc_counter;
uint8_t scale;
short x, y, z;

void MMA8452Q_Standby(); //Включение MMA8452Q
void MMA8452Q_setScale(MMA8452Q_Scale fsr); //Задание масштаба MMA8452Q
void MMA8452Q_setPolarity(); //Задание масштаба MMA8452Q
void MMA8452Q_active(); //Активирование чтения MMA8452Q
void MMA8452Q_setODR(MMA8452Q_ODR odr); //Установка сокрости вывода данных MMA8452Q

//------------------------------------------------------------------------------
//Инициализация MMA8452Q
void MMA8452Q_init(MMA8452_cnf_t *config, uint8_t *status) {
  uint8_t data_mm;

    
  I2C_readReg(MMA8452Q_ADRESS, WHO_AM_I, &data_mm);
  
  if (data_mm != 0x2A) {
    *status = EXEC_ERROR;
  }else{
  
    scale = (uint8_t)config->scale;
    MMA8452Q_Standby(); //Ожидание

    
    MMA8452Q_setScale(config->scale); //Настройка разрешения
    MMA8452Q_setODR(config->ODR); //Скрорость данных
    MMA8452Q_setPolarity(); //Задание масштаба MMA8452Q
    MMA8452Q_setupTap(config->acc_level, config->acc_level, config->acc_level);//Установка границ срабатывания
    MMA8452Q_active(); //Активирование чтения MMA8452Q
      
    *status =  EXEC_OK;
  }
}
//------------------------------------------------------------------------------
//Расчет среднего отклонения по осям
void MMA8452Q_calc_delta(MMA8452_delta_t delta ,uint8_t *status) 
{

  sum_x += x_arr[arr_end];
  sum_y += y_arr[arr_end];
  sum_z += z_arr[arr_end];
  
  if (calc_counter >= ACC_WINDOW_SIZE) { //Если окно заполнилось
    sum_x -= x_arr[arr_start];
    sum_y -= y_arr[arr_start];
    sum_z -= z_arr[arr_start];
    
    //Расчет среднего
    middle_val_x = (sum_x/ACC_WINDOW_SIZE);
    middle_val_y = (sum_y/ACC_WINDOW_SIZE);
    middle_val_z = (sum_z/ACC_WINDOW_SIZE);
  }else{
    calc_counter++;
  }
  
  //Вектор общего отклонения
  *delta.delta_sum = (int)sqrt(MMA8452Q_delta_val_x*MMA8452Q_delta_val_x+MMA8452Q_delta_val_y*MMA8452Q_delta_val_y+MMA8452Q_delta_val_z*MMA8452Q_delta_val_z);
    
  //Работа с кольцевым буфером
  arr_end = arr_start;
  if (arr_start == ACC_WINDOW_SIZE-1) {
    arr_start = 0;
  }else{
    arr_start++;
  } 

  *status = 1;
}  
//------------------------------------------------------------------------------
//Включение MMA8452Q
void MMA8452Q_Standby() {
  uint8_t data;
  
  I2C_readReg(MMA8452Q_ADRESS, CTRL_REG1, &data);
  data = data & ~(0x01);
  I2C_writeReg(MMA8452Q_ADRESS, CTRL_REG1, data);
}
//------------------------------------------------------------------------------
//Задание масштаба MMA8452Q
void MMA8452Q_setScale(MMA8452Q_Scale fsr) {
  uint8_t data;
  
  I2C_readReg(MMA8452Q_ADRESS, XYZ_DATA_CFG, &data);
  data &= 0xFC; // Mask out scale bits
  data |= (fsr >> 2);  // Neat trick, see page 22. 00 = 2G, 01 = 4G, 10 = 8G
  I2C_writeReg(MMA8452Q_ADRESS, XYZ_DATA_CFG, data);
}
//------------------------------------------------------------------------------
//Задание полярности MMA8452Q
void MMA8452Q_setPolarity() {
  uint8_t data;
  
  I2C_readReg(MMA8452Q_ADRESS, PL_CFG, &data);
  data = data | 0x40;
  I2C_writeReg(MMA8452Q_ADRESS, PL_CFG, data);
  data = 0x50;
  I2C_writeReg(MMA8452Q_ADRESS, PL_COUNT, data);
}
//------------------------------------------------------------------------------
//Активирование чтения MMA8452Q
void MMA8452Q_active() {
  uint8_t data;
  
  I2C_readReg(MMA8452Q_ADRESS, CTRL_REG1, &data);
  data = data | 0x01;
  I2C_writeReg(MMA8452Q_ADRESS, CTRL_REG1, data);
}
//------------------------------------------------------------------------------
// Конвертировать координаты MMA8452Q
float MMA8452Q_rawToCoord(uint8_t msb, uint8_t lsb)
{
  short raw_data = ((short)(msb<<8 | lsb)) >> 4;

  return (float) raw_data / (float)(1<<11) * (float)(scale);
}
//------------------------------------------------------------------------------
// Чтение MMA8452Q
void MMA8452Q_read(MMA8452_coord_t *coord, uint8_t *status) {
  uint8_t rawData[6];
  uint8_t tx_data;
    
  tx_data = OUT_X_MSB;
  I2C_readWrite(MMA8452Q_ADRESS, &tx_data, 1, &rawData[0], 2);
  tx_data = OUT_Y_MSB;
  I2C_readWrite(MMA8452Q_ADRESS, &tx_data, 1, &rawData[2], 2);
  tx_data = OUT_Z_MSB;
  I2C_readWrite(MMA8452Q_ADRESS, &tx_data, 1, &rawData[4], 2);

  coord->x = MMA8452Q_rawToCoord(rawData[0], rawData[1]);
  coord->y = MMA8452Q_rawToCoord(rawData[2], rawData[3]);
  coord->z = MMA8452Q_rawToCoord(rawData[4], rawData[5]);
  
  *status = EXEC_OK;
}
//------------------------------------------------------------------------------
//Установка сокрости вывода данных MMA8452Q
void MMA8452Q_setODR(MMA8452Q_ODR odr) {
  uint8_t data;
  
  I2C_readReg(MMA8452Q_ADRESS, CTRL_REG1, &data);
  data &= 0xC7;
  data |= (odr << 3);
  I2C_writeReg(MMA8452Q_ADRESS, CTRL_REG1, data);
}
//------------------------------------------------------------------------------
//Установка границ срабатывания
void MMA8452Q_setupTap(uint8_t xThs, uint8_t yThs, uint8_t zThs)
{
  // Set up single and double tap - 5 steps:
  // for more info check out this app note:
  // http://cache.freescale.com/files/sensors/doc/app_note/AN4072.pdf
  // Set the threshold - minimum required acceleration to cause a tap.
  uint8_t temp = 0;
  if (!(xThs & 0x80)) // If top bit ISN'T set
  {
          temp |= 0x3; // Enable taps on x
          I2C_writeReg(MMA8452Q_ADRESS, PULSE_THSX, xThs);
  }
  if (!(yThs & 0x80))
  {
          temp |= 0xC; // Enable taps on y
          I2C_writeReg(MMA8452Q_ADRESS, PULSE_THSY, yThs);
  }
  if (!(zThs & 0x80))
  {
          temp |= 0x30; // Enable taps on z
          I2C_writeReg(MMA8452Q_ADRESS, PULSE_THSZ, zThs);
  }
  // Set up single and/or double tap detection on each axis individually.
  I2C_writeReg(MMA8452Q_ADRESS, PULSE_CFG, temp | 0x00);
  // Set the time limit - the maximum time that a tap can be above the thresh
  I2C_writeReg(MMA8452Q_ADRESS, PULSE_TMLT, 0x10); // 30ms time limit at 800Hz odr
  // Set the pulse latency - the minimum required time between pulses
  I2C_writeReg(MMA8452Q_ADRESS, PULSE_LTCY, 0xCA); // 200ms (at 800Hz odr) between taps min
  // Set the second pulse window - maximum allowed time between end of
  //	latency and start of second pulse
  I2C_writeReg(MMA8452Q_ADRESS, PULSE_WIND, 0xAA); // 5. 318ms (max value) between taps max
  
//  I2C_writeReg(MMA8452Q_ADRESS, FF_MT_CFG, 0x58);// Set motion flag on x and y axes
//  I2C_writeReg(MMA8452Q_ADRESS, FF_MT_THS, 0x84);// Clear debounce counter when condition no longer obtains, set threshold to 0.25 g
//  I2C_writeReg(MMA8452Q_ADRESS, FF_MT_COUNT, 0x8);// Set debounce to 0.08 s at 100 Hz
  
  //Interrupt polarity
  I2C_writeReg(MMA8452Q_ADRESS, CTRL_REG3, 0x02);
  
  //Enable pulse interrupt
  I2C_writeReg(MMA8452Q_ADRESS, CTRL_REG4,  0x08); //0x1D 0x19
  
  //Enable pulse interrupt on INT1
  I2C_writeReg(MMA8452Q_ADRESS, CTRL_REG5, 0x00); //0x0C 0x08
  
}