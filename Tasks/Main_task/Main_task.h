
#ifndef MAIN_TASK
#define MAIN_TASK

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "timers.h"

//Defines
#define DEFAUL_APN              "internet"
#define DEFAUL_LOGIN            "gdata"
#define DEFAUL_PASS             "gdata"

extern TaskHandle_t xHandle_Main;
void Main_sleep(uint8_t sleep_mode, uint32_t *sec); //Sleep
void Main_Wakeup(uint8_t wakeup_mode); //Wakeup

#endif