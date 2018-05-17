
//System
#include "stdint.h"
#include "stdlib.h"
#include "stdio.h"
#include "string.h"
#include "math.h"
#include "em_gpio.h"
#include "em_emu.h"
#include "em_chip.h"

//RTOS
#include "portmacro.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "portable.h"
#include "croutine.h"
#include "task.h"
#include "timers.h"

//Device
#include "system_efm32lg.h"
#include "InitDevice.h"
#include "MMA8452Q.h"
#include "ADC.h"

//Lib
#include "USB_ctrl.h"
#include "Add_func.h"
#include "Device_ctrl.h"
#include "Delay.h"

//Tasks
#include "I2C_gate_task.h"
#include "Modem_gate_task.h"
#include "Tracker_task.h"
#include "Monitor_task.h"
//#include "Main_task.h"

QueueHandle_t xBinarySemaphore;

//Queues
extern QueueHandle_t I2C_gate_con_Queue;
QueueHandle_t Debug_con_Queue;
extern QueueHandle_t MGT_con_Queue;
extern QueueHandle_t Monitor_con_Queue;
//extern QueueHandle_t Main_con_Queue;
extern QueueHandle_t Tracker_con_Queue;

//Tasks handles
extern TaskHandle_t xHandle_I2C;
TaskHandle_t xDebug_Task;
extern TaskHandle_t xHandle_Tracker;
extern TaskHandle_t xHandle_MGT;
extern TaskHandle_t xHandle_Monitor;

//Tasks
void vI2C_gate_Task (void *pvParameters);
void vDebug_Task(void *pvParameters);
void vMGT_Task (void *pvParameters);
void vMonitor_Task (void *pvParameters);
void vMain_Task (void *pvParameters);

//Mutex
xSemaphoreHandle extFlash_mutex;
xSemaphoreHandle ADC_mutex;
#include "em_cmu.h"

//--------------------------------------------------------------------------------------------------
//MAIN
int main()
{
  //Set vector
  SCB->VTOR = 0x00003000;

  CHIP_Init(); //Init chip with errata
  
  enter_DefaultMode_from_RESET(); //Inint perefery
  
  //Semaphores
  vSemaphoreCreateBinary( xBinarySemaphore );
  
  //Queue create
  I2C_gate_con_Queue = xQueueCreate( 5, sizeof( TT_mes_type ) );
  Debug_con_Queue = xQueueCreate( 5, sizeof( TT_mes_type ) );
  MGT_con_Queue = xQueueCreate( 5, sizeof( TT_mes_type ) );
  Tracker_con_Queue = xQueueCreate( 5, sizeof( TT_mes_type ) );
  Monitor_con_Queue = xQueueCreate( 5, sizeof( TT_mes_type ) );
  
  //Mutex
  extFlash_mutex = xSemaphoreCreateMutex();
  ADC_mutex = xSemaphoreCreateMutex();
  
  //Tasks create
  xTaskCreate(vI2C_gate_Task,(char*)"I2CG_Task", 100, NULL, tskIDLE_PRIORITY+2, &xHandle_I2C);
  xTaskCreate(vDebug_Task,(char*)"Debug_Task", 100, NULL, tskIDLE_PRIORITY, &xDebug_Task);
  xTaskCreate(vMGT_Task,(char*)"MGT_Task", 170, NULL, tskIDLE_PRIORITY+2, &xHandle_MGT);
  xTaskCreate(vTracker_Task,(char*)"Tracker_Task", 170, NULL, tskIDLE_PRIORITY+1, &xHandle_Tracker);
  xTaskCreate(vMonitor_Task,(char*)"Mon_Task", 165, NULL, tskIDLE_PRIORITY+1, &xHandle_Monitor);

  TT_init(); //Task transfer inint
  
  vTaskStartScheduler(); //Start
  
  return 0;
}

//**************************************************************************************************
void vApplicationIdleHook( void )
{
  
}

void vApplicationMallocFailedHook( void )
{
  DC_reset_system(); //System reset
}

void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName )
{
  DC_reset_system(); //System reset
}

void vApplicationTickHook( void )
{
  
}
//**************************************************************************************************
//Monitor gate task
uint16_t val;

void vDebug_Task(void *pvParameters)
{       
  while (1)
  {
//    GPIO_PinOutSet(STATUS_LED_PORT, STATUS_LED_PIN);
//    vTaskDelay(1000);
//    GPIO_PinOutClear(STATUS_LED_PORT, STATUS_LED_PIN);
    vTaskDelay(1000);
  }
}