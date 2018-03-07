
#ifndef I2C_GATE_TASK_H
#define I2C_GATE_TASK_H

#include "stdint.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "Task_transfer.h"
#include "MMA8452Q.h"

extern QueueHandle_t I2C_gate_con_Queue; //Queue
extern TaskHandle_t xHandle_I2CG;

//Task function
void vI2C_gate_Task(void *pvParameters); //I2C_gate_Task

////PCA9536 functions
//uint8_t I2C_gate_Port_ext_setGPIO(uint8_t pin, uint8_t val); //Set value from extender
//uint8_t I2C_gate_Port_ext_getGPIO(uint8_t pin, uint8_t *val); //Get value from extender

//MMA8452Q functions
uint8_t I2C_gate_MMA8452Q_init(MMA8452Q_Scale scale, MMA8452Q_ODR ODR, uint8_t acc_level); //MMA8452Q init
uint8_t I2C_gate_MMA8452Q_read(MMA8452_coord_t *coord); //MMA8452Q read


#endif