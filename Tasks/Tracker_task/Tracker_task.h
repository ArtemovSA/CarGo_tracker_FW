
#ifndef TRACKER_TASK
#define TRACKER_TASK

//RTOS
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

extern QueueHandle_t Tracker_con_Queue;
extern TaskHandle_t xHandle_Tracker;

void vTracker_Task(void *pvParameters); //Tracker task

#endif