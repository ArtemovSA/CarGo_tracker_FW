
#ifndef TASK_TRANS_H
#define TASK_TRANS_H

#include "FreeRTOS.h"
#include "queue.h"

//Tasks
typedef enum {
  TT_I2C_TASK = 0,
  TT_DEBUG_TASK,
  TT_MGT_TASK,
  TT_TRACKER_TASK,
  TT_MONITOR_TASK,
  TT_MAIN_TASK,
  TT_TASK_COUNT
}TT_taskID;

//Messages types
typedef enum {
  CALL_MSG = 1,
  EVENT_TCP_MSG,
  EVENT_TCP_CLOSE,
  EVENT_UNDEVOLTAGE,
  EVENT_BT_EVENT,
  EVENT_SMS_MES,
  RETURN_MSG
}TT_type;

//Status types
typedef enum {
  EXEC_ERROR = 0,
  EXEC_WAIT_ER,
  EXEC_OK
}TT_status_type;

//Tasks discription struct
typedef struct {
  QueueHandle_t *conQueue;
}TT_tasksDesc;

//Struct for tasks
typedef struct {
  TT_type type;                 //Type of message
  TT_taskID task;               //Task id
  void* queue;                  //Return queue
  void* message;                //Message p
  void (*func)();               //Exec func
  TT_status_type status;        //Exec status
}TT_mes_type;

extern TT_tasksDesc tasksDesc[TT_TASK_COUNT]; //Tasks description

void TT_init(); //Task transfer inint
TT_status_type TT_send_query(TT_taskID taskID, QueueHandle_t* queue_send, QueueHandle_t* queue_ret, void *message, void (*func)()); //Send query

#endif