
#include "Monitor_task.h"
#include "String.h"
#include "Task_transfer.h"
#include "Modem_gate_task.h"
#include "I2C_gate_task.h"
#include "Device_ctrl.h"
#include "MMA8452Q.h"

TaskHandle_t xHandle_Monitor;
extern TaskHandle_t xHandle_Tracker;
QueueHandle_t Monitor_con_Queue;
TT_mes_type task_msg;

//**************************************************************************************************
//Monitor task
void vMonitor_Task(void *pvParameters)
{
  DC_init(); //device init
  
  //Init
  MGT_init(&Monitor_con_Queue);
  vTaskResume( xHandle_Tracker );
  vTaskResume( xHandle_MGT );
  uint16_t BT_ErrorCode;
  
  uint8_t acel_level = (uint8_t)DC_settings.acel_level_int*127/SCALE_8G; //Convert to threshold
    
  if (I2C_gate_MMA8452Q_init(SCALE_8G, ODR_800, acel_level))
    DC_debugOut("ACC OK\r\n");
  
  while(1)
  {
    //Recive message
    if ( xQueueReceive( Monitor_con_Queue, &task_msg, 10 ) == pdTRUE )
    {
      //BT event
      if ((task_msg.task == TT_MGT_TASK) && (task_msg.type == EVENT_BT_EVENT))
      {
        switch(((MC60_BT_event_t*)task_msg.message)->type)
        {
        case BT_PAIR:
          if (DC_BT_connCount < DC_BT_CONN_MAX)
          {
            //Accept pairing
            if (MGT_acceptPairingBT((void*)DC_settings.BT_pass, &DC_BT_Status[DC_BT_connCount].connStatus, &DC_BT_Status[DC_BT_connCount].ID, DC_BT_Status[DC_BT_connCount].connName, &BT_ErrorCode) == MC60_STD_OK)
            {
              DC_debugOut("###BT pairing###\r\n");
            }
          }
          break;
        case BT_CONN:
          // Connect BT
          if (MGT_connectBT(DC_BT_Status[DC_BT_connCount-1].ID, BT_SPP_PROFILE, BT_MODE_AT, &BT_ErrorCode) == MC60_STD_OK)
          {
            //Check connection status
            if (DC_BT_Status[DC_BT_connCount].connStatus == true)
            {
              DC_BT_Status[DC_BT_connCount].addr = ((MC60_BT_event_t*)task_msg.message)->addr; //Copy address
              DC_BT_connCount++;
              DC_debugOut("###BT connect###\r\n");
            }
          }
          break;
        case BT_DISCONN:
          break;
        default:
          break;
        }
      }
      
      //Check disconnect event
      if ((task_msg.task == TT_MGT_TASK) && (task_msg.type == EVENT_TCP_CLOSE))
      {
        //Main Disconnect
        if ((*(uint8_t *)task_msg.message) == 1)
          DC_status.flags.flag_MAIN_TCP = false;
        //Service disconnect
        if ((*(uint8_t *)task_msg.message) == 0)
          DC_status.flags.flag_SERVICE_TCP = false;
      }
      
      //Check undervoltage event
      if ((task_msg.task == TT_MGT_TASK) && (task_msg.type == EVENT_UNDEVOLTAGE))
      {
        DC_debugOut("Undervoltage\r\n");
        //DC_reset_system(); //System reset
      }
      
      //Check undervoltage event
      if ((task_msg.task == TT_MGT_TASK) && (task_msg.type == EVENT_SMS_MES))
      {
        DC_debugOut("SMS\r\n");
      }
      
    }      
  }
}