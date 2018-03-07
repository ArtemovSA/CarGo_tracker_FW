
#include "I2C_gate_task.h"
#include "Task_transfer.h"
#include "MMA8452Q.h"
#include "Device_ctrl.h"
#include "EXT_Flash.h"
#include "cdc.h"
#include "Delay.h"
#include "EXT_Flash.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "em_gpio.h"

QueueHandle_t I2C_gate_con_Queue; //Queue
TaskHandle_t xHandle_I2C;

////--------------------------------------------------------------------------------------------------
////ACC_init
//void I2C_ACC_init()
//{
//  uint8_t acel_level = (uint8_t)DC_settings.acel_level_int*SCALE_8G/127; //Convert to threshold
//  MMA8452_cnf_t MMA8452_cnf = {SCALE_8G, ODR_800, acel_level};
//  
//  MMA8452Q_init(&MMA8452_cnf, NULL); //MMA8452Q init
//}
//--------------------------------------------------------------------------------------------------
//I2C_gate_Task
void vI2C_gate_Task(void *pvParameters) { 
  
  TT_mes_type gate_msg; //Recive message
  TT_mes_type return_msg; //Return message
  uint16_t status = 0; //Return status

  //I2C_ACC_init();

  while (1) {
    
    //Wait message
    if ( xQueueReceive( I2C_gate_con_Queue, &gate_msg, 0 ) == pdTRUE )
    {
      if (gate_msg.type == CALL_MSG) //Call message
      {
        gate_msg.func( gate_msg.message, &status); //Make process and return data
      }

      //Prepare return message
      return_msg.type = RETURN_MSG;
      return_msg.task = TT_I2C_TASK;
      return_msg.status = (TT_status_type)status;
      return_msg.message = gate_msg.message;
      
      //Return queue
      if( xQueueSend( (QueueHandle_t*)(gate_msg.queue), &return_msg, ( TickType_t ) 500 ) != pdPASS )
      {
        vTaskDelay(500);
      }
      
    }
    
    vTaskDelay(10);
  }
}
////**************************************************************************************************
////                              PCA9536 gate
////**************************************************************************************************
////--------------------------------------------------------------------------------------------------
////Set value from extender
//uint8_t I2C_gate_Port_ext_setGPIO(uint8_t pin, uint8_t val) {
//  PCA9536_gpio_out_t gpio_out;
//  uint8_t currentTaskID = getCurrentTaskID()-1; //User function Get Current task ID 
//  
//  gpio_out.pin = pin;
//  gpio_out.val = val;
//  
//  if (TT_send_query((TT_taskID)currentTaskID, I2C_gate_con_Queue, tasksDesc[currentTaskID].conQueue, &gpio_out, &PCA9536_setGPIO)) //I2C_gate_Task send query
//    return 1;
//  
//  return 0;  
//}
////--------------------------------------------------------------------------------------------------
////Get value from extender
//uint8_t I2C_gate_Port_ext_getGPIO(uint8_t pin, uint8_t *val) {
//  PCA9536_gpio_in_t gpio_in;
//  uint8_t currentTaskID = getCurrentTaskID()-1; //User function Get Current task ID 
//
//  gpio_in.pin = pin;
//  gpio_in.val = val;
//  
//  if (TT_send_query((TT_taskID)currentTaskID, I2C_gate_con_Queue, tasksDesc[currentTaskID].conQueue, &gpio_in, &PCA9536_getGPIO))  //I2C_gate_Task send query
//    return 1;
//  
//  return 0;
//}
////**************************************************************************************************
////                              DS18B20 gate
////**************************************************************************************************
////--------------------------------------------------------------------------------------------------
////getAddress by index
//uint8_t I2C_gate_Temp_getAddress(DS18B20_DeviceAddress addr, uint8_t index)
//{
//  get_Address_t data;
//  uint8_t currentTaskID = getCurrentTaskID()-1; //User function Get Current task ID 
//  
//  data.deviceAddress = addr;
//  data.index = index;
//
//  if (TT_send_query((TT_taskID)currentTaskID, I2C_gate_con_Queue, tasksDesc[currentTaskID].conQueue, &data, &DS18B20_getAddress_task))  //I2C_gate_Task send query
//    return 1;
//  
//  return 0;
//}
////--------------------------------------------------------------------------------------------------
////Begin
//uint8_t I2C_gate_Temp_begin()
//{
//  uint8_t currentTaskID = getCurrentTaskID()-1; //User function Get Current task ID 
//  
//  if (TT_send_query((TT_taskID)currentTaskID, I2C_gate_con_Queue, tasksDesc[currentTaskID].conQueue, NULL, &DS18B20_begin_task))  //I2C_gate_Task send query
//    return 1;
//  
//  return 0;
//}
////--------------------------------------------------------------------------------------------------
////Get temp
//uint8_t I2C_gate_get_Temp(float* temper, uint8_t index)
//{
//  getTemp_t data;
//  uint8_t currentTaskID = getCurrentTaskID()-1; //User function Get Current task ID 
//  
//  data.deviceIndex = index;
//  data.temper = temper;
//  
//  if (TT_send_query((TT_taskID)currentTaskID, I2C_gate_con_Queue, tasksDesc[currentTaskID].conQueue, &data, &DS18B20_getTempCByIndex_task))  //I2C_gate_Task send query
//    return 1;
//  
//  return 0;
//}
////--------------------------------------------------------------------------------------------------
////Conversion
//uint8_t I2C_gate_conversion_Temp()
//{
//  uint8_t currentTaskID = getCurrentTaskID()-1; //User function Get Current task ID 
//  
//  if (TT_send_query((TT_taskID)currentTaskID, I2C_gate_con_Queue, tasksDesc[currentTaskID].conQueue, NULL, &DS18B20_requestTemperatures_task))  //I2C_gate_Task send query
//    return 1;
//  
//  return 0;
//}

//**************************************************************************************************
//                              MMA8452Q gate
//**************************************************************************************************
//--------------------------------------------------------------------------------------------------
//MMA8452Q init
uint8_t I2C_gate_MMA8452Q_init(MMA8452Q_Scale scale, MMA8452Q_ODR ODR, uint8_t acc_level)
{
  MMA8452_cnf_t data;
  uint8_t currentTaskID = getCurrentTaskID()-1; //User function Get Current task ID
  
  data.scale = scale;
  data.ODR = ODR;
  data.acc_level = acc_level;
  
  if (TT_send_query((TT_taskID)currentTaskID, I2C_gate_con_Queue, tasksDesc[currentTaskID].conQueue, &data, &MMA8452Q_init))  //I2C_gate_Task send query
    return 1;
  
  return 0;  
}
//--------------------------------------------------------------------------------------------------
//MMA8452Q read
uint8_t I2C_gate_MMA8452Q_read(MMA8452_coord_t *coord)
{
  uint8_t currentTaskID = getCurrentTaskID()-1; //User function Get Current task ID
  
  if (TT_send_query((TT_taskID)currentTaskID, I2C_gate_con_Queue, tasksDesc[currentTaskID].conQueue, coord, &MMA8452Q_read))  //I2C_gate_Task send query
    return 1;
  
  return 0;  
}