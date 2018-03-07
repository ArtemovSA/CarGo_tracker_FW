
#include "Task_transfer.h"
#include "String.h"

#include "FreeRTOS.h"
#include "FreeRTOS.h"

TT_tasksDesc tasksDesc[TT_TASK_COUNT]; //Tasks description

//Queues connection
extern QueueHandle_t I2C_gate_con_Queue;
extern QueueHandle_t Debug_con_Queue;
extern QueueHandle_t MGT_con_Queue;
extern QueueHandle_t Monitor_con_Queue;
extern QueueHandle_t Main_con_Queue;
extern QueueHandle_t Tracker_con_Queue;

//--------------------------------------------------------------------------------------------------
//Send query
TT_status_type TT_send_query(TT_taskID taskID, QueueHandle_t* queue_send, QueueHandle_t* queue_ret, void *message, void (*func)()) {
  TT_mes_type query;
  TT_mes_type ansver;
  
  //Prepare query
  query.type = CALL_MSG;
  query.task = taskID;
  query.queue = *queue_ret;
  query.message = message;
  query.func = func;

  //Send queue
  if( xQueueSend( queue_send, &query, ( TickType_t ) 1000 ) != pdPASS )
  {
    return EXEC_ERROR;
  }
  
  //Wait message
  if ( xQueueReceive( (QueueHandle_t)(*queue_ret), &ansver, 50000 ) == pdTRUE )
  {
    return ansver.status;
  }
  
  return EXEC_ERROR;
}
//--------------------------------------------------------------------------------------------------
//Task transfer inint
void TT_init() {
  
  //Create task_id queues task_name struct
  tasksDesc[TT_DEBUG_TASK].conQueue = &Debug_con_Queue;
  
  tasksDesc[TT_I2C_TASK].conQueue = &I2C_gate_con_Queue;
  //strcpy(tasksDesc[TT_I2C_TASK].taskName, "I2C_gate_Task");
  
  tasksDesc[TT_MGT_TASK].conQueue = &MGT_con_Queue;
  //strcpy(tasksDesc[TT_MGT_TASK].taskName, "MGT_task");
  
  tasksDesc[TT_TRACKER_TASK].conQueue = &Tracker_con_Queue;
  //strcpy(tasksDesc[TT_MGT_TASK].taskName, "TRACKER_task");
  
  tasksDesc[TT_MONITOR_TASK].conQueue = &Monitor_con_Queue;
  //strcpy(tasksDesc[TT_MONITOR_TASK].taskName, "Monitor_task");
  
  //tasksDesc[TT_MAIN_TASK].conQueue = &Main_con_Queue;
  //strcpy(tasksDesc[TT_WIALON_TASK].taskName, "Wialon_task");
}