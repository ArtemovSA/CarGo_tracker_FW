
#include "Modem_gate_task.h"
#include "Device_ctrl.h"
#include "stdio.h"
#include "em_gpio.h"

#include "Delay.h"
#include "UART.h"
#include "MC60_lib.h"
#include "cdc.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "timers.h"
#include "efm32lg330f128.h"

//CMD automat
typedef enum{
  MGT_CMD_ERROR = 0,    //Error exec cmd
  MGT_CMD_WAIT,         //Waiting cmd from tasks
  MGT_CMD_TIMEOUT,      //Timeout        
  MGT_CMD_EXEC          //Execution cmd
}MGT_cmd_auto_t;

//Event automat
typedef enum{
  MGT_EVENT_ERROR = 0,   //Error
  MGT_EVENT_WAIT,        //Waiting event
  MGT_EVENT_TIMEOUT,     //Timeout reciving event        
  MGT_EVENT_RECEIVE      //Receiving event 
}MGT_event_auto_t;

//RTOS variables
TaskHandle_t xHandle_MGT; //Task handle
QueueHandle_t MGT_con_Queue; //Connection queue
QueueHandle_t *MGT_event_queue; //Return queue point
extern QueueHandle_t Tracker_con_Queue; //TCP return queue
TimerHandle_t Timeout_Timer_cmd; //Timeout timer cmd
TimerHandle_t Timeout_Timer_event; //Timeout timer event

//Buffers
#define MGT_RECIVE_BUFF_SIZE 200
char MGT_recive[MGT_RECIVE_BUFF_SIZE]; //Recive data
char MGT_event_buf[MGT_RECIVE_BUFF_SIZE]; //Event buf
uint16_t data_count; //Counter

//Variables
MGT_cmd_auto_t MGT_cmd_auto = MGT_CMD_WAIT; //Automat for cmd execution
MGT_event_auto_t MGT_event_auto = MGT_EVENT_WAIT; //Automat for event reciving
MC60_event_t MC60_event = MC60_EVENT_NULL; //Current event
MC60_std_ans_t MGT_std_ans = MC60_STD_NULL; //Current standart ansver
TT_mes_type MGT_cmd_msg; //Connection gate message
TT_mes_type MGT_event_msg; //Event message
uint8_t status; //Status execute
uint8_t MGT_ansver_step = 0; //Step of ansver
uint8_t MGT_event_step = 0; //Step of event
uint16_t MGT_status; //Status byte
char str_end_char[2]; //Char for detect end of string
const MGT_cmd_event_t MGT_event[MC60_EVENT_COUNT]; //Event list
MGT_parce_t MGT_parce; //Parce event
MC60_TCP_recive_t MC60_TCP_recive; //TCP recive type
MC60_BT_event_t MC60_BT_event; //BT event
MC60_SMS_event_t MC60_SMS_event; //SMS event
uint16_t MGT_TCP_ch_event; //Disconnect ch

//Prototypes
MGT_parce_t MGT_event_parce_Recive(char *data);

//--------------------------------------------------------------------------------------------------
//Reset task
void MGT_reset()
{
  data_count = 0;
  memset(MGT_recive,0,MGT_RECIVE_BUFF_SIZE);
  NVIC_ClearPendingIRQ(USART0_RX_IRQn);
}
//--------------------------------------------------------------------------------------------------
//Return message
void DC_returnMsg(TT_type mes_type, QueueHandle_t *queue, void *message ,uint8_t status)
{
  TT_mes_type return_msg;
  
  return_msg.type = mes_type;
  return_msg.task = TT_MGT_TASK;
  return_msg.status = (TT_status_type)status;
  return_msg.message = message;

  //Return queue
  if( xQueueSend( queue, &return_msg, ( TickType_t ) 500 ) != pdPASS )
  {
    
  }
}
//--------------------------------------------------------------------------------------------------
//Process message
void MGT_process()
{
  char ch = 0;
  
  //Process Rx data
  while (UART_GetRxCount() > 0)
  {
    ch = UART_ReciveByte();
    
    MGT_recive[data_count] = ch;
    
    //Overflow protect
    if (data_count < MGT_RECIVE_BUFF_SIZE-2)
    {
      data_count++;
    }else{
      data_count = 0;
      memset(MGT_recive,0,MGT_RECIVE_BUFF_SIZE);
    }
    
    //***************************************
    //Parce data
    
    //message
    if (ch == str_end_char[0])
    {
      MGT_recive[data_count+1] = 0;
    }
    
    //CMD ansver
    if ((MGT_cmd_auto == MGT_CMD_EXEC) && ((ch == str_end_char[0]) || (ch == str_end_char[1])))
    {      
      //MGT_recive[data_count+1] = 0;
      
      //Check, Parce ansver
      if (MGT_cmd[MC60_current_cmd].parce_func(MGT_recive, MGT_cmd_msg.message) == MGT_PARCE_OK)
      {
        xTimerStop(Timeout_Timer_cmd, 0);
        MGT_cmd_auto = MGT_CMD_WAIT;
        DC_returnMsg(RETURN_MSG, (QueueHandle_t*)(MGT_cmd_msg.queue), MGT_cmd_msg.message, EXEC_OK);//Return message 
        memset(MGT_recive, 0, sizeof(MGT_recive));
        data_count = 0;
      } 
      //data_count = 0;
      
    }
    
    //Event message
    if (ch == str_end_char[0])
    {
      if (MGT_parce == MGT_PARCE_NULL)
      {
        for(int i=1; i<MC60_EVENT_COUNT; i++ )
        {
          MGT_parce = MGT_PARCE_NULL;
          MGT_event_step = 0;
          
          MGT_parce = MGT_event[i].parce_func(MGT_recive);
          
          if (MGT_parce == MGT_PARCE_OK)
          {
            MC60_event = (MC60_event_t)i; //Get current event
            break;
          }
          
          if (MGT_parce == MGT_PARCE_NEXT)
          {
            MC60_event = (MC60_event_t)i; //Get current event
            if (MGT_cmd[i].wait_ms>0) 
              xTimerChangePeriod(Timeout_Timer_event, MGT_event[i].wait_ms, 0); //Change timer
            else
              xTimerChangePeriod(Timeout_Timer_event, 1000, 0); //Change timer
            xTimerStart(Timeout_Timer_event, 0);
            break;
          }
        }
        
      }
      else if (MGT_parce == MGT_PARCE_NEXT)
      {
        MGT_parce = MGT_event[MC60_event].parce_func(MGT_recive);
      }
      
      //If parce ok
      if (MGT_parce == MGT_PARCE_OK)
      {
        xTimerStop(Timeout_Timer_event, 0);
        
        switch(MC60_event) {
        case MC60_EVENT_NULL: break;
        case MC60_EVENT_RDY: DC_debugOut("Modem ready\r\n"); break; 
        case MC60_EVENT_RECIVE: DC_returnMsg(EVENT_TCP_MSG, Tracker_con_Queue, &MC60_TCP_recive, EXEC_OK); break;
        case MC60_EVENT_TCP_CLOSE: DC_returnMsg(EVENT_TCP_CLOSE, MGT_event_queue, &MGT_TCP_ch_event, EXEC_OK); break;
        case MC60_EVENT_UNDER_VOLTAGE: DC_returnMsg(EVENT_UNDEVOLTAGE, MGT_event_queue, NULL, EXEC_OK); break;
        case MC60_EVENT_BT_EVENT: DC_returnMsg(EVENT_BT_EVENT, MGT_event_queue, &MC60_BT_event, EXEC_OK); break;
        case MC60_EVENT_SMS_RECIVE: DC_returnMsg(EVENT_SMS_MES, MGT_event_queue, &MC60_SMS_event, EXEC_OK); break;
        }
        
        MGT_parce = MGT_PARCE_NULL;
      }
      
      data_count = 0;
    }
    
    //Parce data
    //***************************************
     
  }
  
  //Process cmd timeout
  if (MGT_cmd_auto == MGT_CMD_TIMEOUT) {
    MGT_cmd_auto = MGT_CMD_WAIT;
    MGT_event_step = 0;
    DC_returnMsg(RETURN_MSG, (QueueHandle_t*)(MGT_cmd_msg.queue), MGT_cmd_msg.message, EXEC_WAIT_ER);//Return message 
    xTimerStop(Timeout_Timer_cmd, 0);
    data_count = 0;
    
    //Cancel cmd
    if ((((MC60_str_query_t*)(MGT_cmd_msg.message))->cmd_id == MC60_CMD_QISEND) || (((MC60_str_query_t*)(MGT_cmd_msg.message))->cmd_id == MC60_CMD_CMGS))
      MC60_sendCR(); //Send CR
  }
  
  //Process cmd timeout
  if (MGT_event_auto == MGT_EVENT_TIMEOUT) {
    MGT_event_auto = MGT_EVENT_WAIT;
    MGT_event_step = 0;
    data_count = 0;
  }
}
//--------------------------------------------------------------------------------------------------
//Timeout cmd
void vTimerTimeout_cmd( TimerHandle_t xTimer )
{
  if (MGT_cmd_auto == MGT_CMD_EXEC) {
    MGT_cmd_auto = MGT_CMD_TIMEOUT; //Reset cmd
  }
}
//--------------------------------------------------------------------------------------------------
//Timeout event
void vTimerTimeout_event( TimerHandle_t xTimer )
{
  if (MGT_event_auto == MGT_EVENT_RECEIVE) {
    MGT_event_auto = MGT_EVENT_TIMEOUT; //Reset event
  }
}
//--------------------------------------------------------------------------------------------------
//Switch off ECHO
void MGT_switch_off_echo()
{
  _delay_ms(1000);
  UART_SendData("ATE0\r\n",6);
  _delay_ms(500);
}
//--------------------------------------------------------------------------------------------------
//Switch on modem
void MGT_switch_on_modem()
{
  //DC_switch_power(DC_DEV_GSM, 1);//Power
  ///MC60_GSM_EN_ON; 
  MC60_on_seq();
  _delay_ms(2000);
}
//--------------------------------------------------------------------------------------------------
//Init
void MGT_init(QueueHandle_t *MGT_event_queue_in)
{
  MGT_event_queue = *MGT_event_queue_in; //Return queue point
  MC60_TCP_recive.data = MGT_event_buf;
  
  Timeout_Timer_cmd = xTimerCreate("Timeout1", 1000, pdTRUE, ( void * )0, vTimerTimeout_cmd);
  Timeout_Timer_event = xTimerCreate("Timeout2", 1000, pdTRUE, ( void * )0, vTimerTimeout_event);
  
  str_end_char[0] = '\n'; //End of recived string
  str_end_char[1] = 0; //End of recived string
}
//**************************************************************************************************
//Modem gate task
void vMGT_Task(void *pvParameters) {
  
  vTaskSuspend( xHandle_MGT );
  
  while (1) {
    
    //Wait execution previous cmd and wait recive event
    if ((MGT_cmd_auto == MGT_CMD_WAIT) && (MGT_event_auto != MGT_EVENT_RECEIVE))
    {
      //Wait message
      if ( xQueueReceive( MGT_con_Queue, &MGT_cmd_msg, MGT_SCAN_MS ) == pdTRUE )
      {
        if (MGT_cmd_msg.type == CALL_MSG) //Call message
        {
          MGT_ansver_step = 0; //Reset ansver step
          MGT_cmd_auto = MGT_CMD_EXEC; //Executing CMD
          
          //Switch str end
          if (((MC60_str_query_t*)(MGT_cmd_msg.message))->cmd_id == MC60_CMD_QISEND)
          {
            str_end_char[1] = '>';
          }
          
          MGT_cmd_msg.func( MGT_cmd_msg.message ); //Make process and return data
          if (MGT_cmd[MC60_current_cmd].wait_ms>0) 
            xTimerChangePeriod(Timeout_Timer_cmd, MGT_cmd[MC60_current_cmd].wait_ms, 0); //Change timer
          else
            xTimerChangePeriod(Timeout_Timer_cmd, 1000, 0); //Change timer
          xTimerStart(Timeout_Timer_cmd, 0); //Start timer
        }
      }
    }else{
      vTaskDelay(MGT_SCAN_MS);
    }
    
    MGT_process(); //Process flow data
    //vTaskDelay(5);
  }
  
}  
//**************************************************************************************************
//                                      Parce Events
//**************************************************************************************************
//--------------------------------------------------------------------------------------------------
//Parse NULL event
MGT_parce_t MGT_event_parce_NULL(char *data)
{
  int n = 0; 
  
  if ( sscanf( data, event_text_NULL, &n), n )
    return MGT_PARCE_OK;
  
  return MGT_PARCE_NULL;
}
//--------------------------------------------------------------------------------------------------
//Parse Recive event
MGT_parce_t MGT_event_parce_Recive(char *data)
{
  char c;
  int n = 0;
  int index = 0;
  int len = 0;
  char str_buf[40];
  
  //Parce message
  if (MGT_event_step == 1)
  {
    sprintf(str_buf, event_text_Recive_TCP_data, MC60_TCP_recive.len);
    
    if ( sscanf( data, str_buf, &len, MC60_TCP_recive.data, &n, &c), n )
    {
      return MGT_PARCE_OK;
    }
  }
  
  //Parce head
  if (MGT_event_step == 0)
  {
    //Parce TCP pack
    if ( sscanf( data, event_text_Recive_TCP, &index, &len, &n, &c), n )
    {
      MC60_TCP_recive.index = index;
      MC60_TCP_recive.len = len;
      MGT_event_step = 1;
      return MGT_PARCE_NEXT;
    }
  }
  
  return MGT_PARCE_NULL;
}
//--------------------------------------------------------------------------------------------------
//Parse text
uint8_t MGT_parce_text(char *str, char *data)
{
  int n = 0; 
  
  if ( sscanf( data, str, &n), n )
    return 1;
  
  return 0;
}
//--------------------------------------------------------------------------------------------------
//Parse RDY event
MGT_parce_t MGT_event_parce_RDY(char *data)
{
  if (MGT_parce_text((char*)event_text_RDY, data))
    return MGT_PARCE_OK;
  
  return MGT_PARCE_NULL;
}
//--------------------------------------------------------------------------------------------------
//Parse TCP closed event
MGT_parce_t MGT_event_parce_CLOSED_TCP(char *data)
{
  int n = 0; 
  
  if ((sscanf( data, event_text_CLOSED_TCP, &MGT_TCP_ch_event, &n), n ))
    return MGT_PARCE_OK;
  
  return MGT_PARCE_NULL;
}
//--------------------------------------------------------------------------------------------------
//Parse TCP closed event
MGT_parce_t MGT_event_parce_UNDER_VOLTAGE(char *data)
{
  int n = 0; 
  
  if ((sscanf( data, event_text_UNDER_VOLTAGE, &n), n ))
    return MGT_PARCE_OK;
  
  return MGT_PARCE_NULL;
}
//--------------------------------------------------------------------------------------------------
//BT pair event
MGT_parce_t MGT_event_parce_BT_event(char *data)
{
  int n = 0;
  char BT_event_type_str[20];
  char BT_event_param1_str[20];
  char BT_event_param2_str[20];
  char BT_event_param3_str[20];
  MC60_BT_event.type = BT_NONE_TYPE;
  
  if ((sscanf( data, event_text_Recive_BT_event, BT_event_type_str, BT_event_param1_str, BT_event_param2_str, BT_event_param3_str,&n), n ))
  {
    if (strcmp(BT_event_type_str, "pair") == 0)
    {
      MC60_BT_event.type = BT_PAIR;
      strcpy(MC60_BT_event.name, BT_event_param1_str);
      sscanf(BT_event_param2_str,"%llx", &MC60_BT_event.addr);
      sscanf(BT_event_param3_str,"%d", &MC60_BT_event.numericcompare);
    }
    else if (strcmp(BT_event_type_str, "disc") == 0)
    {
      MC60_BT_event.type = BT_DISCONN;
      int id;
      sscanf(BT_event_param1_str,"%d", &id);
      MC60_BT_event.id = id;
    }
    else if (strcmp(BT_event_type_str, "conn") == 0)
    {
      MC60_BT_event.type = BT_CONN;
      strcpy(MC60_BT_event.name, BT_event_param1_str);
      sscanf(BT_event_param2_str,"%lu", (long*)&MC60_BT_event.addr);
    }

    return MGT_PARCE_OK;
  }
  
  return MGT_PARCE_NULL;
}
//--------------------------------------------------------------------------------------------------
//Parse SMS event
MGT_parce_t MGT_event_parce_SMS_event(char *data)
{
  int n = 0; 
  
  if ((sscanf( data, event_text_Recive_SMS, MC60_SMS_event.SMS_event_place, &MC60_SMS_event.SMS_event_count, &n), n ))
  {
    return MGT_PARCE_OK;
  }

  return MGT_PARCE_NULL;
}
//**************************************************************************************************
//                                      Parce functions
//**************************************************************************************************
//Parce standart ansvers
MC60_std_ans_t MGT_parce_std_ans(char *data)
{
  char c;
  int n = 0;
  
  n=0; if ( sscanf( data, " OK%n%c", &n, &c), n) return MC60_STD_OK;
  n=0; if ( sscanf( data, " ERROR%n%c", &n, &c), n) return MC60_STD_ERROR;
 
 return MC60_STD_NULL;
}
//--------------------------------------------------------------------------------------------------
//Parse standart cmd
MGT_parce_t MGT_cmd_std_parce(char *data, MC60_str_query_t *query)
{
  MC60_std_ans_t std_ans = MGT_parce_std_ans(data);  
  char c;
  int n, dig = 0;
  
  if (std_ans != MC60_STD_NULL)
  {
    *(query->std_ans) = std_ans;
    return MGT_PARCE_OK;
  }else{
    //Check command error
    n = 0;
    if (sscanf( data, " +CME ERROR: %d%n%c", &dig ,&n, &c), n){
      if ((query->std_ans) > (void*)RAM_MEM_BASE)
        *(query->std_ans) = MC60_STD_ERROR;
      if ((query->ans1) > (void*)RAM_MEM_BASE)
        *(uint16_t*)(query->ans1) = dig;
      return MGT_PARCE_OK;
    }
  }
  
  return MGT_PARCE_NULL;  
}
//--------------------------------------------------------------------------------------------------
//CREG_Q cmd
MGT_parce_t MGT_cmd_parce_CREG_Q(char *data, MC60_str_query_t *query)
{
  char c;
  int n, dig = 0;

  //Recive std ansver
  if (MGT_ansver_step == 1)
  {
    MC60_std_ans_t std_ans = MGT_parce_std_ans(data);  
    
    if (std_ans == MC60_STD_OK) {
      *(query->std_ans) = std_ans;
      return MGT_PARCE_OK;
    }
  }
  
  //Recive main ansver
  if (MGT_ansver_step == 0)
  {
    n = 0;
    if (sscanf( data, " +CREG: %c,%d%n%c", &c, &dig, &n, &c),n) {
      *(MC60_CREG_Q_ans_t*)(query->ans1) = (MC60_CREG_Q_ans_t)dig;
      MGT_ansver_step = 1;
      return MGT_PARCE_NEXT;
    }
  }
  
  return MGT_PARCE_NULL;
}
//--------------------------------------------------------------------------------------------------
//CSQ_Q cmd
MGT_parce_t MGT_cmd_parce_CSQ_Q(char *data, MC60_str_query_t *query)
{
  char c;
  int n, dig = 0;

  //Recive std ansver
  if (MGT_ansver_step == 1)
  {
    MC60_std_ans_t std_ans = MGT_parce_std_ans(data);  
    
    if (std_ans == MC60_STD_OK) {
      *(query->std_ans) = std_ans;
      return MGT_PARCE_OK;
    }
  }
  
  //Recive main ansver
  if (MGT_ansver_step == 0)
  {
    n = 0;
    if ( sscanf( data, " +CSQ: %d,%n%c", &dig, &n, &c), n) {
      *(uint8_t*)(query->ans1) = (uint8_t)dig;
      MGT_ansver_step = 1;
      return MGT_PARCE_NEXT;
    }
  }
  
  return MGT_PARCE_NULL;
}
//--------------------------------------------------------------------------------------------------
//CGACT_Q cmd
MGT_parce_t MGT_cmd_parce_CGACT_Q(char *data, MC60_str_query_t *query)
{
  char c;
  int n, dig = 0;

  //Recive std ansver
  if (MGT_ansver_step == 1)
  {
    MC60_std_ans_t std_ans = MGT_parce_std_ans(data);  
    
    if (std_ans == MC60_STD_OK) {
      *(query->std_ans) = std_ans;
      return MGT_PARCE_OK;
    }
  }
  
  //Recive main ansver
  if (MGT_ansver_step == 0)
  {
    n = 0;
    if ( sscanf( data, " +CGACT: 1,%d%n%c", &dig, &n, &c), n) {
      *(uint8_t*)(query->ans1) = (uint8_t)dig;
      MGT_ansver_step = 1;
      return MGT_PARCE_NEXT;
    }
  }
  
  return MGT_PARCE_NULL;
}
//--------------------------------------------------------------------------------------------------
//CGPADDR cmd
MGT_parce_t MGT_cmd_parce_CGPADDR(char *data, MC60_str_query_t *query)
{
  char c;
  int n = 0;

  //Recive std ansver
  if (MGT_ansver_step == 1)
  {
    MC60_std_ans_t std_ans = MGT_parce_std_ans(data);  
    
    if (std_ans == MC60_STD_OK) {
      *(query->std_ans) = std_ans;
      return MGT_PARCE_OK;
    }
  }
  
  //Recive main ansver
  if (MGT_ansver_step == 0)
  {
    n = 0;
    if ( sscanf( data, " +CGPADDR: 1, \"%16[^\"]\"%n%c", (char*)(query->ans1), &n, &c), n) {
      MGT_ansver_step = 1;
      return MGT_PARCE_NEXT;
    }
  }
  
  return MGT_PARCE_NULL;
}
//--------------------------------------------------------------------------------------------------
//QIOPEN cmd
MGT_parce_t MGT_cmd_parce_QIOPEN(char *data, MC60_str_query_t *query)
{
  char c;
  int n, dig = 0;

  //Recive std ansver
  if (MGT_ansver_step ==0)
  {
    MC60_std_ans_t std_ans = MGT_parce_std_ans(data);  
    
    if (std_ans == MC60_STD_OK) {
      *(query->std_ans) = std_ans;
      MGT_ansver_step = 1;
      return MGT_PARCE_NEXT;
    }
  }
  
  //Recive main ansver
  if (MGT_ansver_step == 1)
  {
    n = 0;
    if ( sscanf( data, " %d, CONNECT OK%n%c", &dig, &n, &c), n) {
      *(MC60_con_type_t*)(query->ans1) = MC60_CON_OK;
      *(uint8_t*)(query->ans2) = (uint8_t)dig;
      return MGT_PARCE_OK;
    }
    n = 0;
    if ( sscanf( data, " %d, CONNECT FAIL%n%c", &dig, &n, &c), n) {
      *(MC60_con_type_t*)(query->ans1) = MC60_CON_FAIL;
      *(uint8_t*)(query->ans2) = (uint8_t)dig;
      return MGT_PARCE_OK;
    }
    n = 0;
    if ( sscanf( data, " %d, ALREADY CONNECT%n%c", &dig, &n, &c), n) {
      *(MC60_con_type_t*)(query->ans1) = MC60_CON_ALREADY;
      *(uint8_t*)(query->ans2) = (uint8_t)dig;
      return MGT_PARCE_OK;
    }
    
    MC60_std_ans_t std_ans = MGT_parce_std_ans(data);  
    
    if (std_ans == MC60_STD_OK) {
      *(query->std_ans) = std_ans;
      return MGT_PARCE_OK;
    }
  }
  
  return MGT_PARCE_NULL;
}
//--------------------------------------------------------------------------------------------------
//QISTAT_Q cmd
MGT_parce_t MGT_cmd_parce_QISTAT_Q(char *data, MC60_str_query_t *query)
{
  char c;
  int n = 0;
  char str_buf[30];

  //Recive std ansver
  if (MGT_ansver_step == 1)
  {
    MC60_std_ans_t std_ans = MGT_parce_std_ans(data);  
    
    if (std_ans == MC60_STD_OK) {
      *(query->std_ans) = std_ans;
      return MGT_PARCE_OK;
    }
  }
  
  //Recive main ansver
  if (MGT_ansver_step == 0)
  {
    //Prepare parse string
    sprintf(str_buf,"+QISTAT: %1s, \"\"%%n%%c", (char*)(query->str1));
    
    n = 0;
    if ( sscanf( data, str_buf, &n, &c), n) {
      strcpy((char*)query->ans1, "NONE");
      MGT_ansver_step = 1;
      return MGT_PARCE_NEXT;
    }else{
      sprintf(str_buf,"+QISTAT: %1s, \"%%[^\"]\"%%n%%c", (char*)(query->str1));
      n = 0;
      if (sscanf( data, str_buf, (char*)(query->ans1), &n, &c), n)
      {
        MGT_ansver_step = 1;
        return MGT_PARCE_NEXT;
      }
    }

  }
  
  return MGT_PARCE_NULL;
}
//--------------------------------------------------------------------------------------------------
//QISEND cmd
MGT_parce_t MGT_cmd_parce_QISEND(char *data, MC60_str_query_t *query)
{
  char c;
  int n = 0;

  //Recive std ansver
  if (MGT_ansver_step == 1)
  {
    //Parce SEND OK
    if (sscanf( data, " SEND OK%n%c", &n, &c), n) {
      *(query->std_ans) = MC60_STD_OK;
      *(MC60_TCP_send_t*)(query->ans1) = MC60_SEND_OK;
      return MGT_PARCE_OK;      
    }
    
    //Parce SEND FAIL
    if (sscanf( data, " SEND FAIL%n%c", &n, &c), n) {
      *(query->std_ans) = MC60_STD_OK;
      *(MC60_TCP_send_t*)(query->ans1) = MC60_SEND_FAIL;
      return MGT_PARCE_OK;      
    }
  }
  
  //Recive main ansver
  if (MGT_ansver_step == 0)
  {
    n = 0;
    //Parce >
    if (sscanf( data, " >%n%c", &n, &c), n) {
      int str_len = 0;
      str_end_char[1] = 0; //Switch off second character
      sscanf((char*)(query->str2),"%d", &str_len);
      MC60_sendStr(query->str3, str_len);//Send message
      MGT_ansver_step = 1;
      return MGT_PARCE_NEXT;
    }
    //Parce Error
    if (sscanf( data, " ERROR%n%c", &n, &c), n) {
      str_end_char[1] = 0; //Switch off second character
      *(query->std_ans) = MC60_STD_OK;
      *(MC60_TCP_send_t*)(query->ans1) = MC60_CONNECT_ERR;
      return MGT_PARCE_OK;      
    }
  }
  
  return MGT_PARCE_NULL;
}
//--------------------------------------------------------------------------------------------------
//GSN cmd
MGT_parce_t MGT_cmd_parce_GSN(char *data, MC60_str_query_t *query)
{
  int n = 0;

  //Recive std ansver
  if (MGT_ansver_step == 1)
  {
    MC60_std_ans_t std_ans = MGT_parce_std_ans(data);  
    
    if (std_ans == MC60_STD_OK) {
      *(query->std_ans) = std_ans;
      return MGT_PARCE_OK;
    }
  }
  
  //Recive main ansver
  if (MGT_ansver_step == 0)
  {
    n = 0;
    if ( sscanf( data, "%17[^\r]\r\n%n", (char*)(query->ans1), &n), n) {
      MGT_ansver_step = 1;
      return MGT_PARCE_NEXT;
    }
  }
  
  return MGT_PARCE_NULL;
}
//--------------------------------------------------------------------------------------------------
//QGNSSRD cmd
MGT_parce_t MGT_cmd_parce_QGNSSRD(char *data, MC60_str_query_t *query)
{
  int n = 0;

  //Recive std ansver
  if (MGT_ansver_step == 1)
  {
    MC60_std_ans_t std_ans = MGT_parce_std_ans(data);  
    
    if (std_ans == MC60_STD_OK) {
      *(query->std_ans) = std_ans;
      return MGT_PARCE_OK;
    }
  }
  
  //Recive main ansver
  if (MGT_ansver_step == 0)
  {
    n = 0;
    if (sscanf( data, " +QGNSSRD: %100s%n\n", (char*)(query->ans1),&n),n) {
      MGT_ansver_step = 1;
      return MGT_PARCE_NEXT;
    }
    
    n = 0;
    if (sscanf( data, " +CME ERROR%n", &n), n){
      *(query->std_ans) = MC60_STD_ERROR;
      return MGT_PARCE_OK;
    }
  }
  
  return MGT_PARCE_NULL;
}
//--------------------------------------------------------------------------------------------------
//QGNSSTS_Q cmd
MGT_parce_t MGT_cmd_parce_QGNSSTS_Q(char *data, MC60_str_query_t *query)
{
  int n = 0;
  int dig = 0;

  //Recive std ansver
  if (MGT_ansver_step == 1)
  {
    MC60_std_ans_t std_ans = MGT_parce_std_ans(data);  
    
    if (std_ans == MC60_STD_OK) {
      *(query->std_ans) = std_ans;
      return MGT_PARCE_OK;
    }
  }
  
  //Recive main ansver
  if (MGT_ansver_step == 0)
  {
    n = 0;
    if (sscanf( data, " +QGNSSTS: %d%n\n", &dig,&n),n) {
      *(uint8_t*)(query->ans1) = dig;
      MGT_ansver_step = 1;
      return MGT_PARCE_NEXT;
    }
  }
  
  return MGT_PARCE_NULL;
}
//--------------------------------------------------------------------------------------------------
//QGNSSC_Q cmd
MGT_parce_t MGT_cmd_parce_QGNSSC_Q(char *data, MC60_str_query_t *query)
{
  int n = 0;

  //Recive std ansver
  if (MGT_ansver_step == 1)
  {
    MC60_std_ans_t std_ans = MGT_parce_std_ans(data);  
    
    if (std_ans == MC60_STD_OK) {
      *(query->std_ans) = std_ans;
      return MGT_PARCE_OK;
    }
  }
  
  //Recive main ansver
  if (MGT_ansver_step == 0)
  {
    n = 0;
    if (sscanf( data, " +QGNSSC: %1s%n\n", (char*)(query->ans1),&n),n) {
      MGT_ansver_step = 1;
      return MGT_PARCE_NEXT;
    }
  }
  
  return MGT_PARCE_NULL;
}
//--------------------------------------------------------------------------------------------------
//QCELLLOC cmd
MGT_parce_t MGT_cmd_parce_QCELLLOC(char *data, MC60_str_query_t *query)
{
  int n = 0;

  //Recive std ansver
  if (MGT_ansver_step == 1)
  {
    MC60_std_ans_t std_ans = MGT_parce_std_ans(data);  
    
    if (std_ans == MC60_STD_OK) {
      *(query->std_ans) = std_ans;
      return MGT_PARCE_OK;
    }
  }
  
  //Recive main ansver
  if (MGT_ansver_step == 0)
  {
    n = 0;
    if (sscanf( data, " +QCELLLOC: %12[^,],%12s%n\n", (char*)(query->ans1),(char*)(query->ans2),&n),n) {
      MGT_ansver_step = 1;
      return MGT_PARCE_NEXT;
    }
    
    n = 0;
    if (sscanf( data, " +CME ERROR%n", &n), n){
      *(query->std_ans) = MC60_STD_ERROR;
      return MGT_PARCE_OK;
    }
  }
  
  return MGT_PARCE_NULL;
}
//--------------------------------------------------------------------------------------------------
//CCLK cmd
MGT_parce_t MGT_cmd_parce_CCLK(char *data, MC60_str_query_t *query)
{
  int n = 0;

  //Recive std ansver
  if (MGT_ansver_step == 1)
  {
    MC60_std_ans_t std_ans = MGT_parce_std_ans(data);  
    
    if (std_ans == MC60_STD_OK)
    {
      *(query->std_ans) = std_ans;
      return MGT_PARCE_OK;
    }
  }
  
  //Recive main ansver
  if (MGT_ansver_step == 0)
  {
    n = 0;
    if (sscanf( data, " +CCLK: \"%25[^+\"]+%n\n", (char*)(query->ans1),&n) , n )
    {
      MGT_ansver_step = 1;
      return MGT_PARCE_NEXT;
    }
  }
  
  return MGT_PARCE_NULL;
}
//--------------------------------------------------------------------------------------------------
//QICLOSE cmd
MGT_parce_t MGT_cmd_parce_QICLOSE(char *data, MC60_str_query_t *query)
{
  int n = 0;
  char c;

  if (sscanf( data, " %c, CLOSE OK%n\n", &c,&n) , n )
  {
    *(query->std_ans) = MC60_STD_OK;
    return MGT_PARCE_OK;
  }

  return MGT_PARCE_NULL;
}
//--------------------------------------------------------------------------------------------------
//QBTPWR_Q cmd
MGT_parce_t MGT_cmd_parce_QBTPWR_Q(char *data, MC60_str_query_t *query)
{
  int n = 0;

  //Recive std ansver
  if (MGT_ansver_step == 1)
  {
    MC60_std_ans_t std_ans = MGT_parce_std_ans(data);  
    
    if (std_ans == MC60_STD_OK) {
      *(query->std_ans) = std_ans;
      return MGT_PARCE_OK;
    }
  }
  
  //Recive main ansver
  if (MGT_ansver_step == 0)
  {
    n = 0;
    if (sscanf( data, " +QBTPWR: %1s%n\n", (char*)(query->ans1),&n),n) {
      MGT_ansver_step = 1;
      return MGT_PARCE_NEXT;
    }
  }
  
  return MGT_PARCE_NULL;
}
//--------------------------------------------------------------------------------------------------
//QBTPAIRCNF cmd
MGT_parce_t MGT_cmd_parce_QBTPAIRCNF(char *data, MC60_str_query_t *query)
{
  int n = 0;

  //Recive std ansver
  if (MGT_ansver_step == 1)
  {
    MC60_std_ans_t std_ans = MGT_parce_std_ans(data);  
    
    if (sscanf( data, " +QBTPAIRCNF: %1s,%3[^,],%*s,%20[^,],%*s%n\n", (char*)(query->ans1),(char*)(query->ans2),(char*)(query->ans3),&n),n) {
      *(query->std_ans) = std_ans;
      return MGT_PARCE_OK;
    }
  }
  
  //Recive main ansver
  if (MGT_ansver_step == 0)
  {
    MC60_std_ans_t std_ans = MGT_parce_std_ans(data);  
    
    if (std_ans == MC60_STD_OK) {
      *(query->std_ans) = std_ans;
      MGT_ansver_step = 1;
      
      return MGT_PARCE_NEXT;
    }
    
    n = 0;
    //Parce Error
    if (sscanf( data, " +CME ERROR: %s%n", (char*)(query->ans1), &n), n) {
      str_end_char[1] = 0; //Switch off second character
      *(query->std_ans) = MC60_STD_OK;
      return MGT_PARCE_OK;      
    }
  }
  
  return MGT_PARCE_NULL;
}
//--------------------------------------------------------------------------------------------------
//QSPPSEND cmd
MGT_parce_t MGT_cmd_parce_QSPPSEND(char *data, MC60_str_query_t *query)
{
  int n = 0;

  //Recive std ansver
  if (MGT_ansver_step == 1)
  {
    //Parce SEND OK
    if (sscanf( data, " OK%n", &n), n) {
      *(query->std_ans) = MC60_STD_OK;
      *(MC60_BT_send_t*)(query->ans1) = MC60_BT_SEND_OK;
      return MGT_PARCE_OK;      
    }
    
    //Parce Error
    if (sscanf( data, " +CME ERROR: %s%n", (char*)(query->ans2), &n), n) {
      str_end_char[1] = 0; //Switch off second character
      *(query->std_ans) = MC60_STD_OK;
      *(MC60_BT_send_t*)(query->ans1) = MC60_BT_CONNECT_ERR;
      return MGT_PARCE_OK;      
    }
  }
  
  //Recive main ansver
  if (MGT_ansver_step == 0)
  {
    n = 0;
    //Parce >
    if (sscanf( data, " >%n", &n), n) {
      int str_len = 0;
      str_end_char[1] = 0; //Switch off second character
      sscanf((char*)(query->str2),"%d", &str_len);
      MC60_sendStr(query->str3, str_len);//Send message
      MGT_ansver_step = 1;
      return MGT_PARCE_NEXT;
    }
  }
  
  return MGT_PARCE_NULL;
}
//--------------------------------------------------------------------------------------------------
//QBTPAIRCNF cmd
MGT_parce_t MGT_cmd_parce_QBTCONN(char *data, MC60_str_query_t *query)
{
  int n = 0;

  //Recive std ansver
  if (MGT_ansver_step == 1)
  {
    MC60_std_ans_t std_ans = MGT_parce_std_ans(data);  
    
    if (sscanf( data, " +QBTCONN: %s,%3[^,]%n\n", (char*)(query->ans1),(char*)(query->ans2),&n),n) {
      *(query->std_ans) = std_ans;
      return MGT_PARCE_OK;
    }
  }
  
  //Recive main ansver
  if (MGT_ansver_step == 0)
  {
    MC60_std_ans_t std_ans = MGT_parce_std_ans(data);  
    
    if (std_ans == MC60_STD_OK) {
      *(query->std_ans) = std_ans;
      MGT_ansver_step = 1;
      
      return MGT_PARCE_NEXT;
    }
    
    n = 0;
    //Parce Error
    if (sscanf( data, " +CME ERROR: %s%n", (char*)(query->ans1), &n), n) {
      str_end_char[1] = 0; //Switch off second character
      *(query->std_ans) = MC60_STD_OK;
      return MGT_PARCE_OK;      
    }
    
  }
  
  return MGT_PARCE_NULL;
}
//--------------------------------------------------------------------------------------------------
//CUSD cmd
MGT_parce_t MGT_cmd_parce_CUSD(char *data, MC60_str_query_t *query)
{
  int n = 0;

  //Recive std ansver
  if (MGT_ansver_step == 1)
  {
    MC60_std_ans_t std_ans = MGT_parce_std_ans(data);  
    
    if (sscanf( data, " +CUSD: %*s,\"%100[^\"]%n\n", (char*)(query->ans1),&n),n) {
      *(query->std_ans) = std_ans;
      return MGT_PARCE_OK;
    }
  }
  
  //Recive main ansver
  if (MGT_ansver_step == 0)
  {
    MC60_std_ans_t std_ans = MGT_parce_std_ans(data);  
    
    if (std_ans == MC60_STD_OK) {
      *(query->std_ans) = std_ans;
      MGT_ansver_step = 1;
      
      return MGT_PARCE_NEXT;
    }
    
    n = 0;
    //Parce Error
    if (sscanf( data, " +CME ERROR: %s%n", (char*)(query->ans1), &n), n) {
      str_end_char[1] = 0; //Switch off second character
      *(query->std_ans) = MC60_STD_OK;
      return MGT_PARCE_OK;      
    }
    
  }
  
  return MGT_PARCE_NULL;
}
//--------------------------------------------------------------------------------------------------
//CMGC cmd
MGT_parce_t MGT_cmd_parce_CMGS(char *data, MC60_str_query_t *query)
{
  char c;
  int n = 0;

  //Recive std ansver
  if (MGT_ansver_step == 2)
  {
    MC60_std_ans_t std_ans = MGT_parce_std_ans(data);  
    
    if (std_ans == MC60_STD_OK)
    {
      *(query->std_ans) = std_ans;
      return MGT_PARCE_OK;
    }
  }
  
  //Recive send ansver
  if (MGT_ansver_step == 1)
  {
    //Parce OK
    if (sscanf( data, " +CMGS%n%c", &n, &c), n) {
      *(query->std_ans) = MC60_STD_OK;
      *(MC60_TCP_send_t*)(query->ans1) = MC60_SEND_OK;
      MGT_ansver_step = 2;
      return MGT_PARCE_NEXT;      
    }
  }
  
  //Recive main ansver
  if (MGT_ansver_step == 0)
  {
    n = 0;
    //Parce >
    if (sscanf( data, " >%n%c", &n, &c), n) {
      int str_len = 0;
      str_end_char[1] = 0; //Switch off second character
      MC60_sendStr(query->str2, str_len);//Send message
      MGT_ansver_step = 1;
      return MGT_PARCE_NEXT;
    }
    //Parce Error
    if (sscanf( data, " ERROR%n%c", &n, &c), n) {
      str_end_char[1] = 0; //Switch off second character
      *(query->std_ans) = MC60_STD_OK;
      *(MC60_TCP_send_t*)(query->ans1) = MC60_CONNECT_ERR;
      return MGT_PARCE_OK;      
    }
  }
  
  return MGT_PARCE_NULL;
}
//--------------------------------------------------------------------------------------------------
//CMD list
const MGT_cmd_event_t MGT_cmd[MC60_CMD_COUNT] = {
  {(char*)cmd_text_NULL,                MGT_cmd_std_parce,              1000},
  {(char*)cmd_text_AT,                  MGT_cmd_std_parce,              1000},
  {(char*)cmd_text_ATI,                 MGT_cmd_std_parce,              1000},
  {(char*)cmd_text_CREG_Q,              MGT_cmd_parce_CREG_Q,           1500},
  {(char*)cmd_text_CSQ_Q,               MGT_cmd_parce_CSQ_Q,            1000},
  {(char*)cmd_text_CGDCONT,             MGT_cmd_std_parce,              1000},
  {(char*)cmd_text_CGACT,               MGT_cmd_std_parce,              2000},
  {(char*)cmd_text_CGACT_Q,             MGT_cmd_parce_CGACT_Q,          1000},
  {(char*)cmd_text_CGPADDR,             MGT_cmd_parce_CGPADDR,          2000},
  {(char*)cmd_text_QIDNSIP,             MGT_cmd_std_parce,              1000},
  {(char*)cmd_text_QIOPEN,              MGT_cmd_parce_QIOPEN,           12000},
  {(char*)cmd_text_QIMUX,               MGT_cmd_std_parce,              1000},
  {(char*)cmd_text_QISTAT_Q,            MGT_cmd_parce_QISTAT_Q,         2000},
  {(char*)cmd_text_QISEND,              MGT_cmd_parce_QISEND,           2000},
  {(char*)cmd_text_GSN,                 MGT_cmd_parce_GSN,              1000},
  {(char*)cmd_text_QIHEAD,              MGT_cmd_std_parce,              1000},
  {(char*)cmd_text_QGNSSRD_RMC,         MGT_cmd_parce_QGNSSRD,          1000},
  {(char*)cmd_text_QGNSSRD_GGA,         MGT_cmd_parce_QGNSSRD,          1000},
  {(char*)cmd_text_QGNSSC,              MGT_cmd_std_parce,              1000},
  {(char*)cmd_text_QGNSSC_Q,            MGT_cmd_parce_QGNSSC_Q,         1000},
  {(char*)cmd_text_QGNSSTS_Q,           MGT_cmd_parce_QGNSSTS_Q,        1000},
  {(char*)cmd_text_QGREFLOC,            MGT_cmd_std_parce,              1000},
  {(char*)cmd_text_QGNSSEPO,            MGT_cmd_std_parce,              1000},
  {(char*)cmd_text_QGEPOAID,            MGT_cmd_std_parce,              1000},
  {(char*)cmd_text_QSCLK,               MGT_cmd_std_parce,              1000},
  {(char*)cmd_text_CFUN,                MGT_cmd_std_parce,              1000},
  {(char*)cmd_text_QIREGAPP,            MGT_cmd_std_parce,              1000},
  {(char*)cmd_text_QCELLLOC,            MGT_cmd_parce_QCELLLOC,         12000},
  {(char*)cmd_text_CTZU,                MGT_cmd_std_parce,              1000},
  {(char*)cmd_text_CCLK,                MGT_cmd_parce_CCLK,             1000},
  {(char*)cmd_text_QNITZ,               MGT_cmd_std_parce,              1000},
  {(char*)cmd_text_QICLOSE,             MGT_cmd_parce_QICLOSE,          1000},
  {(char*)cmd_text_CUSD,                MGT_cmd_parce_CUSD,             8000},
  {(char*)cmd_text_CSCS,                MGT_cmd_std_parce,              1000},
  {(char*)cmd_text_MODECUSD,            MGT_cmd_std_parce,              1000},
  {(char*)cmd_text_CMGF,                MGT_cmd_std_parce,              1000},
  {(char*)cmd_text_CMGS,                MGT_cmd_parce_CMGS,             1000},

  {(char*)cmd_text_QBTPWR,              MGT_cmd_std_parce,              3000},
  {(char*)cmd_text_QBTPWR_Q,            MGT_cmd_parce_QBTPWR_Q,         1000},
  {(char*)cmd_text_QBTNAME,             MGT_cmd_std_parce,              1000},
  {(char*)cmd_text_QBTVISB,             MGT_cmd_std_parce,              1000},
  {(char*)cmd_text_QBTPAIRCNF,          MGT_cmd_parce_QBTPAIRCNF,       2000},
  {(char*)cmd_text_QSPPSEND,            MGT_cmd_parce_QSPPSEND,         1000},
  {(char*)cmd_text_QBTCONN,             MGT_cmd_parce_QBTCONN,          2000}

};
//--------------------------------------------------------------------------------------------------
//Event list
const MGT_cmd_event_t MGT_event[MC60_EVENT_COUNT] = {
  {(char*)event_text_NULL,              NULL,                           200},
  {(char*)event_text_Recive_TCP,        MGT_event_parce_Recive,         200},
  {(char*)event_text_RDY,               MGT_event_parce_RDY,            200},
  {(char*)event_text_CLOSED_TCP,        MGT_event_parce_CLOSED_TCP,     200},
  {(char*)event_text_UNDER_VOLTAGE,     MGT_event_parce_UNDER_VOLTAGE,  200},
  {(char*)event_text_Recive_BT_event,   MGT_event_parce_BT_event,       200},
  {(char*)event_text_Recive_SMS,        MGT_event_parce_SMS_event,      200}
};
//**************************************************************************************************
//                                      MGT functions
//**************************************************************************************************
//Send request
TT_status_type MGT_send_req(MC60_str_query_t *query)
{
  uint8_t currentTaskID = getCurrentTaskID()-1; //User function Get Current task ID
  
  return TT_send_query((TT_taskID)currentTaskID, MGT_con_Queue, tasksDesc[currentTaskID].conQueue, query, MC60_PrepareAndSendCmd);
}
//--------------------------------------------------------------------------------------------------
//Check modem
MC60_std_ans_t MGT_modem_check()
{
  MC60_str_query_t query;
  MC60_std_ans_t std_ans = MC60_STD_NULL;
  uint16_t ans;

  query.cmd_id = MC60_CMD_AT;
  query.std_ans = &std_ans;
  query.ans1 = &ans;
  
  //Send request
  MGT_send_req(&query);

  return std_ans;    
}
//--------------------------------------------------------------------------------------------------
//Get regmodem
MC60_std_ans_t MGT_getReg(MC60_CREG_Q_ans_t *reg)
{
  MC60_str_query_t query;
  MC60_std_ans_t std_ans = MC60_STD_NULL;
    
  query.cmd_id = MC60_CMD_CREG_Q;
  query.std_ans = &std_ans;
  query.ans1 = reg;
  
  //Send request
  MGT_send_req(&query);
    
  return std_ans;
}
//--------------------------------------------------------------------------------------------------
//Get quality
MC60_std_ans_t MGT_getQuality(uint8_t *level, TT_status_type* status)
{
  MC60_str_query_t query;
  MC60_std_ans_t std_ans = MC60_STD_NULL;
  
  query.cmd_id = MC60_CMD_CSQ_Q;
  query.std_ans = &std_ans;
  query.ans1 = level;
  
  //Send request
  *status = MGT_send_req(&query);
  
  return std_ans;
}
//--------------------------------------------------------------------------------------------------
//setAPN
MC60_std_ans_t MGT_setAPN(char *APN, uint16_t* error_n)
{
  MC60_str_query_t query;
  MC60_std_ans_t std_ans = MC60_STD_NULL;
  
  query.cmd_id = MC60_CMD_CGDCONT;
  query.str1 = APN;
  query.std_ans = &std_ans;
  query.ans1 = error_n;
  
  MGT_send_req(&query);
  
  return std_ans;
}
//--------------------------------------------------------------------------------------------------
//activate GPRS
MC60_std_ans_t MGT_setGPRS(uint8_t active)
{
  MC60_std_ans_t std_ans = MC60_STD_NULL;
  MC60_str_query_t query;

  query.cmd_id = MC60_CMD_CGACT;
  
  if (active)
    query.str1 = "1";
  else
    query.str1 = "0";
  query.std_ans = &std_ans;
  
  MGT_send_req(&query);
  
  return std_ans;
}
//--------------------------------------------------------------------------------------------------
//Get status GPRS
MC60_std_ans_t MGT_getGPRS(uint8_t *status)
{
  MC60_std_ans_t std_ans = MC60_STD_NULL;
  MC60_str_query_t query;

  query.cmd_id = MC60_CMD_CGACT_Q;
  query.std_ans = &std_ans;
  query.ans1 = status;
  
  MGT_send_req(&query);
  
  return std_ans;
}
//--------------------------------------------------------------------------------------------------
//get IP
MC60_std_ans_t MGT_getIP(char *ip)
{
  MC60_std_ans_t std_ans = MC60_STD_NULL;
  MC60_str_query_t query;
  
  query.cmd_id = MC60_CMD_CGPADDR;
  query.std_ans = &std_ans;
  query.ans1 = ip;
  
  MGT_send_req(&query);
  
  return std_ans;
}
//--------------------------------------------------------------------------------------------------
//activate GPRS
MC60_std_ans_t MGT_setAdrType(MC60_adr_type_t type)
{
  MC60_std_ans_t std_ans = MC60_STD_NULL;
  MC60_str_query_t query;

  query.cmd_id = MC60_CMD_QIDNSIP;
  
  if (type ==  MC60_ADR_TYPE_DOMEN)
    query.str1 = "1";
  else
    query.str1 = "0";
  query.std_ans = &std_ans;
  
  MGT_send_req(&query);
  
  return std_ans;
}
//--------------------------------------------------------------------------------------------------
//make GPRS connection
MC60_std_ans_t MGT_openConn(MC60_con_type_t *conn_ans, uint8_t index, MC60_ip_con_type_t con_type, char *domen_or_ip, uint16_t port)
{
  MC60_std_ans_t std_ans = MC60_STD_NULL;
  MC60_str_query_t query;
  uint8_t index_conn;
  char port_str[6];
  char index_str[3];
  
  query.cmd_id = MC60_CMD_QIOPEN;
  
  //Convert data
  sprintf(port_str,"%d", port);
  sprintf(index_str,"%d", index);
  
  query.str1 = index_str;

  if (con_type ==  MC60_CON_TCP)
    query.str2 = "TCP";
  else
    query.str2 = "UDP";
  
  query.str3 = domen_or_ip;
  query.str4 = port_str;
    
  query.ans1 = conn_ans;
  query.ans2 = &index_conn;
  query.std_ans = &std_ans;
  
  MGT_send_req(&query);
  
  return std_ans;
}
//--------------------------------------------------------------------------------------------------
//Setting multiple TCP/IP
MC60_std_ans_t MGT_setMUX_TCP(uint8_t mux)
{
  MC60_std_ans_t std_ans = MC60_STD_NULL;
  MC60_str_query_t query;

  query.cmd_id = MC60_CMD_QIMUX;
  
  if (mux ==  1)
    query.str1 = "1";
  else
    query.str1 = "0";
  
  query.std_ans = &std_ans;
  
  MGT_send_req(&query);
  
  return std_ans;
}
//--------------------------------------------------------------------------------------------------
//Get connection statuses
MC60_std_ans_t MGT_getConnectStat(uint8_t index, uint8_t *status)
{
  MC60_std_ans_t std_ans = MC60_STD_NULL;
  MC60_str_query_t query;
  char index_str[5];
  char status_str[5];

  sprintf(index_str,"%d", index);
  
  query.cmd_id = MC60_CMD_QISTAT_Q;
  query.std_ans = &std_ans;
  query.str1 = index_str;
  query.ans1 = status_str;
  
  MGT_send_req(&query);
  
  //Check return status
  if ((strcmp(status_str, "TCP") == 0) || (strcmp(status_str, "TCP") == 0))
  {
    *status = 1;
  }else{
    *status = 0;
  }
  
  return std_ans;
}
//--------------------------------------------------------------------------------------------------
// Send TCP/UDP package index,len
MC60_std_ans_t MGT_sendTCP(uint8_t index, char *str, uint16_t len, MC60_TCP_send_t *TCP_send_status)
{
  MC60_std_ans_t std_ans = MC60_STD_NULL;
  MC60_str_query_t query;
  char index_str[5];
  char len_str[5];
  *TCP_send_status = MC60_SEND_FAIL;

  sprintf(index_str,"%d", index);
  sprintf(len_str,"%d", len);
  
  query.cmd_id = MC60_CMD_QISEND;
  query.std_ans = &std_ans;
  query.str1 = index_str;
  query.str2 = len_str;
  query.str3 = str;
  query.ans1 = TCP_send_status;
  
  MGT_send_req(&query);

  return std_ans;
}
//--------------------------------------------------------------------------------------------------
//Request IMEI
MC60_std_ans_t MGT_getIMEI(char* IMEI)
{
  MC60_std_ans_t std_ans = MC60_STD_NULL;
  MC60_str_query_t query;

  query.cmd_id = MC60_CMD_GSN;
  query.std_ans = &std_ans;
  query.ans1 = IMEI;
  
  MGT_send_req(&query);
  
  return std_ans;
}
//--------------------------------------------------------------------------------------------------
//Set qihead
MC60_std_ans_t MGT_set_qihead()
{
  MC60_str_query_t query;
  MC60_std_ans_t std_ans = MC60_STD_NULL;
  uint16_t ans;

  query.cmd_id = MC60_CMD_QIHEAD;
  query.std_ans = &std_ans;
  query.ans1 = &ans;
  
  MGT_send_req(&query);

  return std_ans;    
}
//--------------------------------------------------------------------------------------------------
//Get GNSS data
MC60_std_ans_t MGT_get_GNSS(MC60_GNSS_data_type_t GNSS_data_type,char* GNSS_str)
{
  MC60_str_query_t query;
  MC60_std_ans_t std_ans = MC60_STD_NULL;

  if (GNSS_data_type == RMC)
    query.cmd_id = MC60_CMD_QGNSSRD_RMC;
  
  if (GNSS_data_type == GGA)
    query.cmd_id = MC60_CMD_QGNSSRD_GGA;
  
  query.std_ans = &std_ans;
  query.ans1 = GNSS_str;
  
  MGT_send_req(&query);

  return std_ans;    
}
//--------------------------------------------------------------------------------------------------
//Switch GNSS
MC60_std_ans_t MGT_switch_GNSS(uint8_t on, uint16_t *error_n)
{
  MC60_std_ans_t std_ans = MC60_STD_NULL;
  MC60_str_query_t query;

  query.cmd_id = MC60_CMD_QGNSSC;
  
  if (on == 1)
    query.str1 = "1";
  else
    query.str1 = "0";
  
  query.std_ans = &std_ans;
  query.ans1 = error_n;
  
  MGT_send_req(&query);
  
  return std_ans;
}
//--------------------------------------------------------------------------------------------------
//Read time synchronization
MC60_std_ans_t  MGT_getTimeSynch(uint8_t *synch_status)
{
  MC60_str_query_t query;
  MC60_std_ans_t std_ans = MC60_STD_NULL;
    
  query.cmd_id = MC60_CMD_QGNSSTS_Q;
  query.std_ans = &std_ans;
  query.ans1 = synch_status;
  
  MGT_send_req(&query);
  
  return std_ans;
}
//--------------------------------------------------------------------------------------------------
// Set reference location information for QuecFastFix Online
MC60_std_ans_t MGT_set_ref_coord_GNSS(float lat, float lon)
{
  MC60_std_ans_t std_ans = MC60_STD_NULL;
  MC60_str_query_t query;
  char lat_str[15];
  char lon_str[15];
  
  snprintf(lat_str,sizeof(lat_str),"%.6f",lat);
  snprintf(lon_str,sizeof(lon_str),"%.6f",lon);

  query.cmd_id = MC60_CMD_QGREFLOC;
  
  query.str1 = lat_str;
  query.str2 = lon_str;
  
  query.std_ans = &std_ans;
  
  MGT_send_req(&query);
  
  return std_ans;
}
//--------------------------------------------------------------------------------------------------
//Switch EPO
MC60_std_ans_t MGT_switch_EPO(uint8_t on , uint16_t *error_n)
{
  MC60_std_ans_t std_ans = MC60_STD_NULL;
  MC60_str_query_t query;

  query.cmd_id = MC60_CMD_QGNSSEPO;
  
  if (on == 1)
    query.str1 = "1";
  else
    query.str1 = "0";
  
  query.std_ans = &std_ans;
  query.ans1 = error_n;
  
  MGT_send_req(&query);
  
  return std_ans;
}
//--------------------------------------------------------------------------------------------------
//Switch SLEEP
MC60_std_ans_t MGT_switch_Sleep_mode(uint8_t mode)
{
  MC60_std_ans_t std_ans = MC60_STD_NULL;
  MC60_str_query_t query;

  query.cmd_id = MC60_CMD_QSCLK;
  
  if (mode == 0)
    query.str1 = "0";
  if (mode == 1)
    query.str1 = "1";
  if (mode == 2)
    query.str1 = "2";
  
  query.std_ans = &std_ans;
  
  MGT_send_req(&query);
  
  return std_ans;
}
//--------------------------------------------------------------------------------------------------
//Switch FUN mode
MC60_std_ans_t MGT_switch_FUN_mode(uint8_t mode)
{
  MC60_std_ans_t std_ans = MC60_STD_NULL;
  MC60_str_query_t query;

  query.cmd_id = MC60_CMD_CFUN;

  if (mode == 0)
    query.str1 = "0";
  if (mode == 1)
    query.str1 = "1";
  if (mode == 4)
    query.str1 = "4";
  
  query.std_ans = &std_ans;
  
  MGT_send_req(&query);
  
  return std_ans;
}
//--------------------------------------------------------------------------------------------------
//Get GNSS power
MC60_std_ans_t MGT_getPowerGNSS(uint8_t *on)
{
  MC60_std_ans_t std_ans = MC60_STD_NULL;
  MC60_str_query_t query;
  char on_str[5];
  int on_int;

  query.cmd_id = MC60_CMD_QGNSSC_Q;
  query.std_ans = &std_ans;
  query.ans1 = on_str;
  
  if (MGT_send_req(&query) == EXEC_OK)
  {
    sscanf(on_str,"%d",&on_int);
    *on = on_int;
  }
  
  return std_ans;
}
//--------------------------------------------------------------------------------------------------
//Trigger EPO
MC60_std_ans_t MGT_EPO_trig()
{
  MC60_str_query_t query;
  MC60_std_ans_t std_ans = MC60_STD_NULL;

  query.cmd_id = MC60_CMD_QGEPOAID;
  query.std_ans = &std_ans;
  
  MGT_send_req(&query);

  return std_ans;    
}
//--------------------------------------------------------------------------------------------------
// Set APN, login, pass
MC60_std_ans_t MGT_set_apn_login_pass(char* apn, char* login, char* pass)
{
  MC60_str_query_t query;
  MC60_std_ans_t std_ans = MC60_STD_NULL;

  query.cmd_id = MC60_CMD_QIREGAPP;
  query.std_ans = &std_ans;
  query.str1 = apn;
  query.str2 = login;
  query.str3 = pass;
  
  MGT_send_req(&query);

  return std_ans;    
}
//--------------------------------------------------------------------------------------------------
// Get cell lock
MC60_std_ans_t MGT_get_cell_loc(float *lat, float *lon)
{
  MC60_str_query_t query;
  MC60_std_ans_t std_ans = MC60_STD_NULL;
  char lat_str[12];
  char lon_str[12];

  query.cmd_id = MC60_CMD_QCELLLOC;
  query.std_ans = &std_ans;
  query.ans1 = lon_str;
  query.ans2 = lat_str;
  
  if (MGT_send_req(&query) == EXEC_OK)
    if (std_ans == MC60_STD_OK)
    {
      sscanf(lon_str,"%f",lon);
      sscanf(lat_str,"%f",lat);
    }
  
  return std_ans;    
}
//--------------------------------------------------------------------------------------------------
// Get time synch
MC60_std_ans_t MGT_get_time_network(float *time, float *date)
{
  MC60_str_query_t query;
  MC60_std_ans_t std_ans = MC60_STD_NULL;
  char time_str[25];
  int yy,MM,dd,hh,mm,ss;

  query.cmd_id = MC60_CMD_CCLK;
  query.std_ans = &std_ans;
  query.ans1 = time_str;
  
  if (MGT_send_req(&query) == EXEC_OK)
  {
    sscanf(time_str, "%d/%d/%d,%d:%d:%d",&yy,&MM,&dd,&hh,&mm,&ss);
    *date = dd*10000+MM*100+yy;
    *time = hh*10000+mm*100+ss;
  }

  return std_ans;    
}
//--------------------------------------------------------------------------------------------------
// Set GSM time
MC60_std_ans_t MGT_setGSM_time_synch()
{
  MC60_str_query_t query;
  MC60_std_ans_t std_ans = MC60_STD_NULL;

  query.cmd_id = MC60_CMD_QNITZ;
  query.std_ans = &std_ans;
  
  MGT_send_req(&query);

  return std_ans;    
}
//--------------------------------------------------------------------------------------------------
// Close TCP
MC60_std_ans_t MGT_close_TCP_by_index(uint8_t index)
{
  MC60_str_query_t query;
  MC60_std_ans_t std_ans = MC60_STD_NULL;
  char index_str[5];
  
  sprintf(index_str, "%d", index);

  query.cmd_id = MC60_CMD_QICLOSE;
  query.std_ans = &std_ans;
  query.str1 = index_str;
  
  MGT_send_req(&query);

  return std_ans;    
}
//**************************************************************************************************
//                                      Bluetooth functions
//**************************************************************************************************
//Power on bluetooth
MC60_std_ans_t MGT_powerBT(uint8_t on)
{
  MC60_std_ans_t std_ans = MC60_STD_NULL;
  MC60_str_query_t query;

  query.cmd_id = MC60_CMD_QBTPWR;
  
  if (on ==  1)
    query.str1 = "1";
  else
    query.str1 = "0";
  query.std_ans = &std_ans;
  
  MGT_send_req(&query);
  
  return std_ans;
}
//--------------------------------------------------------------------------------------------------
//Get bluetooth power on status 
MC60_std_ans_t MGT_getStatusBT(uint8_t *status)
{
  MC60_std_ans_t std_ans = MC60_STD_NULL;
  MC60_str_query_t query;

  query.cmd_id = MC60_CMD_QBTPWR_Q;
  query.std_ans = &std_ans;
  query.ans1 = status;
  
  MGT_send_req(&query);
  
  return std_ans;
}
//--------------------------------------------------------------------------------------------------
//Set BT name
MC60_std_ans_t MGT_setNameBT(char *name)
{
  MC60_std_ans_t std_ans = MC60_STD_NULL;
  MC60_str_query_t query;

  query.cmd_id = MC60_CMD_QBTNAME;
  query.str1 = name;
  query.std_ans = &std_ans;
  
  MGT_send_req(&query);
  
  return std_ans;
}
//--------------------------------------------------------------------------------------------------
//Set bluetooth visibility
MC60_std_ans_t MGT_setVisibilityBT(uint8_t on)
{
  MC60_std_ans_t std_ans = MC60_STD_NULL;
  MC60_str_query_t query;

  query.cmd_id = MC60_CMD_QBTVISB;
  
  if (on ==  1)
    query.str1 = "1";
  else
    query.str1 = "0";
  query.std_ans = &std_ans;
  
  MGT_send_req(&query);
  
  return std_ans;
}
//--------------------------------------------------------------------------------------------------
//Accept pairing
MC60_std_ans_t MGT_acceptPairingBT(char* password, bool *pairStatus, uint8_t *pairID, char* pairName, uint16_t *ErrorCode)
{
  MC60_str_query_t query;
  MC60_std_ans_t std_ans = MC60_STD_NULL;
  char pairStatus_str[3];
  char pairID_str[3];

  query.cmd_id = MC60_CMD_QBTPAIRCNF;
  query.std_ans = &std_ans;
  query.str1 = password;
  query.ans1 = pairStatus_str;
  query.ans2 = pairID_str;
  query.ans3 = pairName;
  query.ans4 = ErrorCode;
  
  if (strcmp(pairStatus_str, "1") > 0)
  {
    *pairStatus = true;
    sscanf(pairID_str, "%d", (int*)pairID);
  }else{
    *pairStatus = false;
    pairID = NULL;
    pairName = NULL;
  }
  
  MGT_send_req(&query);

  return std_ans;    
}
//--------------------------------------------------------------------------------------------------
// Send BT
MC60_std_ans_t MGT_sendBT(uint8_t index, char *str, uint16_t len, MC60_BT_send_t *BT_send_status, uint16_t *ErrorCode)
{
  MC60_std_ans_t std_ans = MC60_STD_NULL;
  MC60_str_query_t query;
  char index_str[5];
  char len_str[5];
  *BT_send_status = MC60_BT_SEND_FAIL;

  sprintf(index_str,"%d", index);
  sprintf(len_str,"%d", len);
  
  query.cmd_id = MC60_CMD_QISEND;
  query.std_ans = &std_ans;
  query.str1 = index_str;
  query.str2 = len_str;
  query.str3 = str;
  query.ans1 = BT_send_status;
  query.ans2 = ErrorCode;
  
  MGT_send_req(&query);

  return std_ans;
}
//--------------------------------------------------------------------------------------------------
// Connect BT
MC60_std_ans_t MGT_connectBT(uint8_t id, MC60_BT_profile_t profile, MC60_BT_mode_t mode, uint16_t *ErrorCode)
{
  MC60_std_ans_t std_ans = MC60_STD_NULL;
  MC60_str_query_t query;
  char id_str[5];
  char profile_str[5];
  char mode_str[5];

  sprintf(id_str,"%d", id);
  sprintf(profile_str,"%d", profile);
  sprintf(mode_str,"%d", mode);
  
  query.cmd_id = MC60_CMD_QBTCONN;
  query.std_ans = &std_ans;
  query.str1 = id_str;
  query.str2 = profile_str;
  query.str3 = mode_str;
  query.ans1 = ErrorCode;
  
  MGT_send_req(&query);

  return std_ans;
}
//--------------------------------------------------------------------------------------------------
//Network Time Synchronization and Update the RTC
MC60_std_ans_t MGT_setTimeSynchGSM_mode(uint8_t mode)
{
  MC60_std_ans_t std_ans = MC60_STD_NULL;
  MC60_str_query_t query;

  query.cmd_id = MC60_CMD_QBTVISB;
  
  switch(mode)
  {
  case 0: query.str1 = "0"; break;
  case 1: query.str1 = "1"; break;
  case 2: query.str1 = "2"; break;
  case 3: query.str1 = "3"; break;
  case 4: query.str1 = "4"; break;
  }
  
  query.std_ans = &std_ans;
  
  MGT_send_req(&query);
  
  return std_ans;
}
//--------------------------------------------------------------------------------------------------
//Set charecter set
MC60_std_ans_t MGT_setCharacterSet(char *characterSet)
{
  MC60_std_ans_t std_ans = MC60_STD_NULL;
  MC60_str_query_t query;
  uint16_t ErrorCode;

  query.cmd_id = MC60_CMD_CSCS;
  
  query.str1 = characterSet;
  query.ans1 = &ErrorCode;
  
  query.std_ans = &std_ans;
  
  MGT_send_req(&query);
  
  return std_ans;
}
//--------------------------------------------------------------------------------------------------
//Get USSD query
MC60_std_ans_t MGT_returnUSSD(char *USSD, char* returnStr)
{
  MC60_std_ans_t std_ans = MC60_STD_NULL;
  MC60_str_query_t query;

  query.cmd_id = MC60_CMD_CUSD;
  
  query.str1 = USSD;
  query.ans1 = returnStr;
  
  query.std_ans = &std_ans;
  
  MGT_send_req(&query);
  
  return std_ans;
}
//--------------------------------------------------------------------------------------------------
//Set USSD mode
MC60_std_ans_t MGT_setUSSD_mode(char *mode)
{
  MC60_std_ans_t std_ans = MC60_STD_NULL;
  MC60_str_query_t query;
  uint16_t ErrorCode;

  query.cmd_id = MC60_CMD_MODECUSD;
  
  query.str1 = mode;
  query.ans1 = &ErrorCode;
  
  query.std_ans = &std_ans;
  
  MGT_send_req(&query);
  
  return std_ans;
}
//--------------------------------------------------------------------------------------------------
//SMS Message Format 0 PDU mode 1 Text mode
MC60_std_ans_t MGT_setSMS_Format(char *format)
{
  MC60_std_ans_t std_ans = MC60_STD_NULL;
  MC60_str_query_t query;
  uint16_t ErrorCode;

  query.cmd_id = MC60_CMD_CMGF;
  
  query.str1 = format;
  query.ans1 = &ErrorCode;
  
  query.std_ans = &std_ans;
  
  MGT_send_req(&query);
  
  return std_ans;
}
//--------------------------------------------------------------------------------------------------
//Send SMS
MC60_std_ans_t MGT_sendSMS(char *phone, char* message)
{
  MC60_std_ans_t std_ans = MC60_STD_NULL;
  MC60_str_query_t query;
  uint16_t ErrorCode;

  query.cmd_id = MC60_CMD_CMGF;
  
  query.str1 = phone;
  query.str2 = message;
  query.ans1 = &ErrorCode;
  
  query.std_ans = &std_ans;
  
  MGT_send_req(&query);
  
  return std_ans;
}