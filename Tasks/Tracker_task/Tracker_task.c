
#include "Tracker_task.h"

//Std
#include "stdbool.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "em_wdog.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "em_dbg.h"

//RTOS
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

//Tasks
#include "Modem_gate_task.h"
#include "I2C_gate_task.h"

//Libs
#include "Device_ctrl.h"
#include "Delay.h"
#include "MC60_lib.h"
#include "GNSS.h"
#include "wialon_ips2.h"
#include "ADC.h"
#include "UART.h"
#include "Date_time.h"
#include "Device_ctrl.h"
#include "Clock.h"
#include "cdc.h"
#include "onewire.h"
#include "ds18b20.h"

//RTOS variables
extern xSemaphoreHandle ADC_mutex;
QueueHandle_t Tracker_con_Queue;
TaskHandle_t xHandle_Tracker;

USBReceiveHandler USB_Handler;

//Try counts
#define TRY_COUNT_ERROR_AGPS          3
#define TRY_COUNT_ERROR_GNSS          3
//#define TRY_COUNT_ERROR_SENDDATA      3

//Counters
uint8_t tryCounter_AGPS;
uint8_t tryCounter_GNSS;
uint8_t tryCounter_SendData;

//Buffers
char gStr_buf[100];

//Cell coordinates
float gCell_lat;
float gCell_lon;

//Current system params
char current_ip[16];
char current_IMEI[17];

//Times in sec from UTC 
time_t UTC_gnss_time_next;   //Next time for GNSS
time_t UTC_send_time_next;   //Next time for send data
time_t UTC_AGPS_time_next;   //Next time for synch AGPS
time_t UTC_repeat_time_next; //Next time for repeated subtask

//Times executet tasks
time_t UTC_gnss_time_before;   //Before time for GNSS
time_t UTC_send_time_before;   //Before time for send data
time_t UTC_AGPS_time_before;   //Before time for synch AGPS
time_t UTC_repeat_time_before; //Before time for repeated subtask

//--------------------------------------------------------------------------------------------------
//Calc wake up
time_t Tracker_cal_wakeUpTime(time_t wake_up_before, uint32_t period)
{
  int32_t delta = (wake_up_before - globalUTC_time);
  time_t time_return;
    
  if (delta <0)
  {
    if (abs(delta) < period)
      time_return = period + globalUTC_time + delta;
    else
      time_return = period + globalUTC_time;
  }
  
  return time_return;
}
//--------------------------------------------------------------------------------------------------
//Calculate next Period
//arg: mode - next mode
uint32_t Tracker_getNextPeriodAndMode(DC_mode_t *mode)
{
  UTC_repeat_time_next = UTC_repeat_time_before + 20;
  UTC_gnss_time_next = UTC_gnss_time_before + DC_settings.data_gnss_period;
  UTC_send_time_next = UTC_send_time_before + DC_settings.data_send_period;
  UTC_AGPS_time_next = UTC_AGPS_time_before + DC_settings.AGPS_synch_period;

  
  //If send data next mode
  if ((UTC_send_time_next <= UTC_gnss_time_next) && (UTC_send_time_next <= UTC_AGPS_time_next) && (UTC_send_time_next <= UTC_repeat_time_next))
  {
    //If not executed
    if (UTC_send_time_next < globalUTC_time)
    {
      *mode = DC_MODE_SEND_DATA;
      return 0;
    }
    
    *mode = DC_MODE_SEND_DATA;
    return UTC_send_time_next - globalUTC_time;
  }  
  
  //If gnss data next mode
  if ((UTC_gnss_time_next <= UTC_AGPS_time_next) && (UTC_gnss_time_next <= UTC_send_time_next) && (UTC_gnss_time_next <= UTC_repeat_time_next))
  {
    
    //If not executed
    if (UTC_gnss_time_next < globalUTC_time)
    {
      *mode = DC_MODE_GNSS_DATA;
      return 0;
    }
    
    *mode = DC_MODE_GNSS_DATA;
    return UTC_gnss_time_next - globalUTC_time;
  }
  
  //If repeated next mode
  if ((UTC_repeat_time_next <= UTC_gnss_time_next) && (UTC_repeat_time_next <= UTC_AGPS_time_next) && (UTC_repeat_time_next <= UTC_send_time_next))
  {
    //If not executed
    if (UTC_repeat_time_next < globalUTC_time)
    {
      *mode = DC_MODE_REPEATED;
      return 0;
    }
    
    *mode = DC_MODE_REPEATED;
    return UTC_repeat_time_next - globalUTC_time;
  }  
  
  //If AGPS sunch next mode
  if ((UTC_AGPS_time_next <= UTC_send_time_next) && (UTC_AGPS_time_next <= UTC_gnss_time_next) && (UTC_AGPS_time_next <= UTC_repeat_time_next))
  {
    //If not executed
    if (UTC_AGPS_time_next < globalUTC_time)
    {
      *mode = DC_MODE_AGPS_UPDATE;
      return 0;
    }
    
    *mode = DC_MODE_AGPS_UPDATE;
    return UTC_AGPS_time_next - globalUTC_time;
  }  
  
  return DC_MODE_IDLE;
}
//--------------------------------------------------------------------------------------------------
//On modem sequence
DC_return_t Tracker_seq_modem_on()
{
  uint8_t try_on_counter = 10;
  uint8_t try_set_counter = 3;

  NVIC_EnableIRQ(USART1_RX_IRQn);
  NVIC_EnableIRQ(USART1_TX_IRQn);
  UART_RxEnable();
  
  if (MGT_modem_check() != MC60_STD_OK) //Check AT
  {
    MGT_reset(); //Reset gate task
    UART_RxDisable(); //Disable RX data
    MGT_switch_on_modem(); //Switch on modem
    UART_RxEnable(); //Enable RX data
    MGT_switch_off_echo();//Switch off ECHO
  }else{
    return DC_OK;
  }
  
  while(try_on_counter--)
  {        
    while(try_set_counter--)
    {
      if (!DC_getStatus(DC_FLAG_AT_OK))
      {
        if (MGT_modem_check() == MC60_STD_OK) //Check AT
          if (MGT_switch_Sleep_mode(1) == MC60_STD_OK) //Switch SLEEP in
          {
            DC_debugOut("AT OK\r\n");
            DC_setStatus(DC_FLAG_AT_OK);
            
            // Set GSM time
            if(MGT_setGSM_time_synch() == MC60_STD_OK)
              DC_debugOut("GSM time synch setted\r\n");
            return DC_OK;
          }
          
      }else{
        return DC_OK;
      }
    }
    
    try_set_counter = 3;
    MGT_reset(); //Reset gate task
    UART_RxDisable(); //Disable RX data
    MGT_switch_on_modem(); //Switch on modem
    UART_RxEnable(); //Enable RX data
    MGT_switch_off_echo();//Switch off ECHO
    
  }
  
  return DC_TIMEOUT;
}
//--------------------------------------------------------------------------------------------------
//Registration sequence
DC_return_t Tracker_cellReg_seq()
{
  MC60_CREG_Q_ans_t reg;
  MC60_std_ans_t modem_ans;
  uint8_t reg_try_count = 15;
  
  while(reg_try_count--){
    
    modem_ans = MGT_getReg(&reg);
    
    if (modem_ans == MC60_STD_OK) //Check registration in net
    {
      switch(reg) {
      case MC60_CREG_Q_REGISTERED: DC_debugOut("Reg OK\r\n"); DC_setStatus(DC_FLAG_REG_OK); return DC_OK;
      case MC60_CREG_Q_ROAMING: DC_debugOut("Reg roaming\r\n"); DC_setStatus(DC_FLAG_REG_OK); return DC_OK;
      case MC60_CREG_Q_SEARCH: DC_debugOut("Reg Search\r\n"); break;
      case MC60_CREG_Q_NOT_REG: DC_debugOut("Reg NOT\r\n"); return DC_ERROR;
      case MC60_CREG_Q_UNKNOWN: DC_debugOut("Reg UKN\r\n"); return DC_ERROR;
      case MC60_CREG_Q_DENIED: DC_debugOut("Reg Denied\r\n"); return DC_ERROR;
      default: return DC_ERROR;
      };
    }
    
    vTaskDelay(1000);
  }
  
  DC_debugOut("Reg timeout\r\n");
  return DC_TIMEOUT;
}
//--------------------------------------------------------------------------------------------------
//Connect GPRS sequence
DC_return_t Tracker_GPRS_con_seq()
{
  uint16_t error_n;
  uint8_t GPRS_status = 0;
  MC60_std_ans_t modem_ans;
  DC_return_t DC_return;
  
  //Registration sequence
  if ((DC_return = Tracker_cellReg_seq()) != DC_OK)
  {
    return DC_return;
  }
  
  //Set APN if registration and AT OK
  if ((!DC_getStatus(DC_FLAG_APN_OK)) && (DC_getStatus(DC_FLAG_REG_OK)))
  {
    modem_ans = MGT_setAPN(MC60_DEFAUL_APN, &error_n);
    if (modem_ans == MC60_STD_OK)
    {
      //Check APN setted
      if (DC_params.APN_setted == false)
      {
        // Set APN, login, pass
        if (MGT_set_apn_login_pass(MC60_DEFAUL_APN, MC60_DEFAUL_LOGIN, MC60_DEFAUL_PASS) == MC60_STD_OK)
        {
          DC_params.APN_setted = true;
          DC_debugOut("Setted APN settings, wait reboot\r\n");
          DC_reset_system(); //Reset system
        }
      }else{
        if (MGT_setMUX_TCP(1)) //Setting multiple TCP/IP
        {
          DC_setStatus(DC_FLAG_APN_OK);
          DC_debugOut("Check APN: OK\r\n");
        }
      }
      
    }else if((modem_ans == MC60_STD_ERROR)&&(error_n > 0)){
      if (MGT_setMUX_TCP(1)) //Setting multiple TCP/IP
      {
        DC_setStatus(DC_FLAG_APN_OK);
        DC_debugOut("Check APN: OK\r\n");
      }
    }
  }
  
  //Make GPRS connection
  if (!DC_getStatus(DC_FLAG_GPRS_ACTIVE) && DC_getStatus(DC_FLAG_APN_OK) && DC_getStatus(DC_FLAG_REG_OK))
  {
    if (MGT_getGPRS(&GPRS_status) == MC60_STD_OK) //Get status
      if (GPRS_status == 0)
      {
        if (MGT_setGPRS(1) == MC60_STD_OK) //activate GPRS
          if (MGT_getIP(current_ip) == MC60_STD_OK) //get IP
            if (MGT_setAdrType(MC60_ADR_TYPE_IP) == MC60_STD_OK) //Set type addr
              if ( MGT_set_qihead() == MC60_STD_OK) //Set qihead
              {
                DC_setStatus(DC_FLAG_GPRS_ACTIVE);
                DC_debugOut("GPRS OK\r\n");
                sprintf(gStr_buf, "IP address: %s\r\n", current_ip);
                DC_debugOut(gStr_buf);
                return DC_OK;
              }
      }else{
        if (MGT_getIP(current_ip) == MC60_STD_OK) //get IP
          if (MGT_setAdrType(MC60_ADR_TYPE_IP) == MC60_STD_OK) //Set type addr
            if ( MGT_set_qihead() == MC60_STD_OK) //Set qihead
            {
              DC_setStatus(DC_FLAG_GPRS_ACTIVE);
              DC_debugOut("GPRS OK\r\n");
              sprintf(gStr_buf, "IP address: %s\r\n", current_ip);
              DC_debugOut(gStr_buf);
              return DC_OK;
            }
      }
  }else{
    return DC_OK;
  }

  return DC_ERROR;
}
//--------------------------------------------------------------------------------------------------
//Get celloc
DC_return_t Tracker_get_celloc()
{
  if (MGT_get_cell_loc(&gCell_lat, &gCell_lon) == MC60_STD_OK) // Get cell lock
  {
    //Convertion to HHMMSS
    globalGNSS_data.lat1 = GNSS_convertTo_HHMMSS(gCell_lat);
    globalGNSS_data.lon1 = GNSS_convertTo_HHMMSS(gCell_lon);
    return DC_OK;
  }
  
  return DC_ERROR;
}
//--------------------------------------------------------------------------------------------------
//Set referent coord 
DC_return_t Tracker_setRef_loc_seq()
{
  //Get celloc
  if (!DC_getStatus(DC_FLAG_COORD_PRESET_OK) && (!DC_getStatus(DC_FLAG_GNSS_RECEIVED)) && DC_getStatus(DC_FLAG_AT_OK) && DC_getStatus(DC_FLAG_GPRS_ACTIVE))
    if (Tracker_get_celloc() == DC_OK)//Get celloc
    {
      //Set ref coord
      if (!DC_getStatus(DC_FLAG_COORD_PRESET_OK))
        if (MGT_set_ref_coord_GNSS(gCell_lat, gCell_lon) == MC60_STD_OK) // Set reference location information for QuecFastFix Online
        {
          DC_debugOut("Coord preset OK\r\n");
          DC_setStatus(DC_FLAG_COORD_PRESET_OK);
        }else{
          DC_debugOut("Can't preset preset OK\r\n");
        }
    
      sprintf(gStr_buf, "Cell loc lat:%0.3f lon:%0.3f\r\n", globalGNSS_data.lat1, globalGNSS_data.lon1);
      DC_debugOut(gStr_buf);
      
      return DC_OK;
    }else{
      DC_debugOut("Cell loc error");
    }
  
  return DC_ERROR;
}
//--------------------------------------------------------------------------------------------------
//Time Synch sequence
DC_return_t Tracker_synchTime_seq()
{
  uint8_t status;
  uint8_t try_counter = 3;
  
  while (try_counter --)
  {
    if (!DC_getStatus(DC_FLAG_GNSS_TYME_SYNCH) && (DC_getStatus(DC_FLAG_GNSS_ON)))
      if (MGT_getTimeSynch(&status) == MC60_STD_OK)
        if (status == 1)
        {
          DC_debugOut("Time sync ok\r\n");
          DC_setStatus(DC_FLAG_GNSS_TYME_SYNCH);
          return DC_OK;
        }
    vTaskDelay(1000);
  }
  
  DC_debugOut("Time sync error\r\n");
  return DC_ERROR;
}
//--------------------------------------------------------------------------------------------------
//AGPS
DC_return_t Tracker_AGPS_seq()
{
  uint16_t error_n;

  //Time Synch sequence
  if (Tracker_synchTime_seq() != DC_OK)
    return DC_ERROR;
  
  //Set referent coord 
  Tracker_setRef_loc_seq();
  
  //AGPS
  if (!DC_getStatus(DC_FLAG_EPO_OK) && DC_getStatus(DC_FLAG_GNSS_TYME_SYNCH))
    if (MGT_switch_EPO(1, &error_n) == MC60_STD_OK) //Switch EPO
    {
      if (MGT_EPO_trig() == MC60_STD_OK)//Trigger EPO
      {
        DC_debugOut("AGPS OK\r\n");
        DC_setStatus(DC_FLAG_EPO_OK);
        return DC_OK;
      }
    }else if(error_n == 7103)
    {
      DC_debugOut("AGPS already ok\r\n");
      DC_setStatus(DC_FLAG_EPO_OK);
      return DC_OK;
    }
  
  return DC_ERROR;
}
//--------------------------------------------------------------------------------------------------
//Switch on GNSS
DC_return_t Tracker_switch_on_GNSS()
{
  uint8_t status = 0;
  uint8_t try_count = 3;
  uint16_t error_n = 0;
  MC60_std_ans_t ansver;
  
  while(try_count--)
  {
    status = 0;
    
    if (MGT_getPowerGNSS(&status) == MC60_STD_OK) //Get GNSS power
    {
      if (status == 1)
      {
        DC_debugOut("GNSS already on\r\n");
        return DC_OK;
      }
      if (status == 0)
      {
        ansver = MGT_switch_GNSS(1, &error_n);
        if (ansver == MC60_STD_OK) { //Switch GNSS
          DC_debugOut("GNSS switch on\r\n");
          return DC_OK;
        }else if(ansver == MC60_STD_ERROR)
        {
          if (error_n == 7101)
            MGT_switch_GNSS(0, &error_n);
        }
      }
    }
    
    vTaskDelay(500);
  }
  
  return DC_ERROR;
} 
//--------------------------------------------------------------------------------------------------
//Sleep on sec
DC_return_t Tracker_sleep_sec(uint32_t sleep_period)
{
  //UART_RxDisable(); // Disable IRQ
  //MGT_reset(); //Reset task

  LED_STATUS_ON;
  vTaskDelay(5);
  
  if (sleep_period > 0){
    sleepMode = 1; //Sleep flag mode
    DC_set_RTC_timer_s(sleep_period); //Set RTC timer
    if (DBG_Connected())
    {
      DC_debugOut("Sleep on: %d\r\n", sleep_period);
      
      switch(DC_taskCtrl.DC_modeCurrent){
      case DC_MODE_AGPS_UPDATE: DC_debugOut("Next Function: DC_MODE_AGPS_UPDATE\r\n"); break;
      case DC_MODE_SEND_DATA: DC_debugOut("Next Function: DC_MODE_SEND_DATA\r\n"); break;
      case DC_MODE_REPEATED: DC_debugOut("Next Function: DC_MODE_REPEATED\r\n"); break;
      case DC_MODE_GNSS_DATA: DC_debugOut("Next Function: DC_MODE_GNSS_DATA\r\n"); break;
      };

      vTaskDelay(sleep_period*1000);
      
    }else{
      EMU_EnterEM2(true);
      
      USB_Driver_Init(USB_Handler);
    }
  }
  
  sleepMode = 0; //Sleep flag mode
  LED_STATUS_OFF;
  
  return DC_OK;
}
//--------------------------------------------------------------------------------------------------
//Go to Sleep
DC_return_t Tracker_sleep(DC_mode_t next_mode, uint32_t sleep_period)
{
  DC_setModemMode(DC_MODEM_GSM_SAVE_POWER_ON); //Set Modem mode DC_MODEM_ALL_IN_ONE
  
  globalSleepPeriod = sleep_period;
  return Tracker_sleep_sec(sleep_period);
}
//--------------------------------------------------------------------------------------------------
//Read sensors
void Tracker_readSensors()
{
  //Read sensors
  if( xSemaphoreTake( ADC_mutex, 1000 ) == pdTRUE )
  {
    DC_log.MCU_temper = DC_get_chip_temper(); //Get chip temper
    DC_log.BAT_voltage = DC_get_bat_voltage();
    xSemaphoreGive( ADC_mutex );
  }
  
  sprintf(gStr_buf,"Tmcu=%.2f; Vbat=%.2f; mA/h=%.3lf\r\n", DC_log.MCU_temper, (float)DC_log.BAT_voltage/1000, DC_getPower());
  DC_debugOut(gStr_buf);
}
//--------------------------------------------------------------------------------------------------
//Get GNSS data
DC_return_t Tracker_get_GNSS()
{  
  uint8_t try_count = DC_settings.gnss_try_time;
  MC60_std_ans_t ansver;
  
  while(try_count--)
  {
    ansver = MGT_get_GNSS(RMC, gStr_buf); //Get GNSS data
    if (ansver == MC60_STD_OK)
    {
      GNSS_parce_RMC(gStr_buf, &globalGNSS_data);
      
      //Check coordinate
      if ((globalGNSS_data.lat1 > 0) && (globalGNSS_data.lon1 > 0))
      {
        ansver = MGT_get_GNSS(GGA, gStr_buf); //Get GNSS data
        if (ansver == MC60_STD_OK)
        {
          GNSS_parce_GGA(gStr_buf, &globalGNSS_data);
        }
        
        DC_setStatus(DC_FLAG_GNSS_RECEIVED);
        DC_debugOut("GNSS recived\r\n");
        return DC_OK;
      }else{
        DC_debugOut("Wait GNSS\r\n");
      }
      
    }
    vTaskDelay(1000);
  }
  
  return DC_TIMEOUT;
}
//--------------------------------------------------------------------------------------------------
//Get IMEI
DC_return_t Tracker_getIMEI()
{
  if (!DC_getStatus(DC_FLAG_IMEI_OK))
  {
    if (MC60_check_IMEI(DC_settings.IMEI))  //Check IMEI
    {
      DC_debugOut("IMEI: %s\r\n", DC_settings.IMEI);
      DC_setStatus(DC_FLAG_IMEI_OK);
      return DC_OK;
    }else{
      if ( MGT_getIMEI(DC_settings.IMEI) == MC60_STD_OK ) //Get IMEI
      {
        if (MC60_check_IMEI(DC_settings.IMEI))  //Check IMEI
        {
          //sprintf(gStr_buf, "IMEI: %s\r\n", DC_settings.IMEI);
          DC_debugOut("IMEI: %s\r\n", DC_settings.IMEI);
          DC_setStatus(DC_FLAG_IMEI_OK);
          return DC_OK;
        }
      }else{
        return DC_ERROR;
      }
    }
  }else{
    DC_debugOut("IMEI already OK\r\n");
    return DC_OK;
  }
  
  return DC_TIMEOUT;
}
//--------------------------------------------------------------------------------------------------
//Send TCP str
DC_return_t Tracker_sendTCP(uint8_t index, char *str)
{
  MC60_TCP_send_t TCP_send_status = MC60_SEND_FAIL;
  
  if ( MGT_sendTCP(index, str, strlen(str), &TCP_send_status) == MC60_STD_OK) // Send TCP/UDP package index,len
    if (TCP_send_status == MC60_SEND_OK)
    {
      DC_debugOut("TCP message sended\r\n");
      return DC_OK;
    }else{
      DC_resetStatus(DC_FLAG_MAIN_TCP);
      DC_debugOut("Send fail\r\n");
      return DC_ERROR;
    }
  
  return DC_ERROR;
}
//--------------------------------------------------------------------------------------------------
//Send TCP pack
DC_return_t Tracker_sendWialonPack(uint8_t index, char *pack, char* ansver, uint16_t wait_ms)
{
  TT_mes_type tcp_task_msg;    
  uint8_t try_count = 3;
  
  while(try_count--)
  {
    if (Tracker_sendTCP(index, pack))//Send TCP str
    {
      //Wait message
      if ( xQueuePeek( Tracker_con_Queue, &tcp_task_msg, wait_ms ) == pdTRUE )
      {
        //If from Modem gate task
        if (tcp_task_msg.task == TT_MGT_TASK)
          xQueueReceive( Tracker_con_Queue, &tcp_task_msg, 0 );
        if (tcp_task_msg.type == EVENT_TCP_MSG) //If event message from TCP
          if (((MC60_TCP_recive_t*)(tcp_task_msg.message))->index == index)
          {
            memcpy(ansver, ((MC60_TCP_recive_t*)(tcp_task_msg.message))->data, ((MC60_TCP_recive_t*)(tcp_task_msg.message))->len);
            DC_debugOut("Wialon recive pack\r\n");
            return DC_OK;
          }else if(((MC60_TCP_recive_t*)(tcp_task_msg.message))->index == 0)
          {
            //Recived other data
            DC_debugOut("Data send fail\r\n");
          }
      }
    }
    vTaskDelay(1000);
  }
  
  return DC_ERROR;
}
//--------------------------------------------------------------------------------------------------
//Wialon login
DC_return_t Tracker_wialonLogin()
{
  char ansver[20];
  WL_ansvers_t wialon_ansver = WL_ANS_UNKNOWN;
  
  if (!DC_getStatus(DC_WL_LOGIN_OK) && (DC_getStatus(DC_FLAG_IMEI_OK)))
  {
    WL_prepare_LOGIN(gStr_buf, DC_settings.IMEI, DC_settings.dataPass);
    if (Tracker_sendWialonPack(1, gStr_buf, ansver, 8000) == DC_OK)
    {
      WL_parce_LOGIN_ANS(ansver, &wialon_ansver);
      if (wialon_ansver == WL_ANS_SUCCESS)
      {
        DC_debugOut("Login OK\r\n");
        DC_setStatus(DC_WL_LOGIN_OK);
        return DC_OK;
      }else{
        DC_debugOut("Login fail\r\n");
        DC_resetStatus(DC_FLAG_MAIN_TCP);
      }
    }
  }else{
    return DC_OK;
  }
  
  DC_resetStatus(DC_FLAG_MAIN_TCP);
  return DC_ERROR;
}
//--------------------------------------------------------------------------------------------------
//Server TCP connection
DC_return_t Tracker_server_connect()
{
  uint8_t try_count = 3;
  MC60_con_type_t conn_ans;
  uint8_t status_connect = 0;
  
  if ( !DC_getStatus(DC_FLAG_GPRS_ACTIVE) )
      return DC_ERROR;
  
  while (try_count--){
    
    if (MGT_getConnectStat(1, &status_connect) == MC60_STD_OK) //Get connection statuses
    {
      if (status_connect == 0)
      {
        //Try open TCP connection from list
        for (int i=0; i<4; i++)
        {
          if (MGT_openConn(&conn_ans, 1, MC60_CON_TCP, DC_settings.ip_dataList[i], DC_settings.dataPort) == MC60_STD_OK) //make GPRS connection
          {
            if (conn_ans == MC60_CON_OK)
            {
              DC_params.current_data_IP = i;
              DC_resetStatus(DC_WL_LOGIN_OK);
              DC_setStatus(DC_FLAG_MAIN_TCP);
              sprintf(gStr_buf,"Data TCP connection IP: %s\r\n",DC_settings.ip_dataList[i]);
              DC_debugOut(gStr_buf);
              return DC_OK;
            }
            
            if (conn_ans == MC60_CON_ALREADY)
            {
              DC_params.current_data_IP = i;
              DC_setStatus(DC_FLAG_MAIN_TCP);
              sprintf(gStr_buf,"Data TCP connection IP: %s\r\n",DC_settings.ip_dataList[i]);
              DC_debugOut(gStr_buf);
              DC_debugOut("Data TCP connected\r\n");
              return DC_OK;
            }
            
            if (conn_ans == MC60_CON_FAIL)
            {
              if (MGT_setGPRS(0) == MC60_STD_OK) //deactivate GPRS
              {
                DC_resetStatus(DC_FLAG_GPRS_ACTIVE);
                DC_resetStatus(DC_WL_LOGIN_OK);
              }
              MGT_close_TCP_by_index(1); // Close TCP
              DC_debugOut("Data TCP connction fail\r\n");
              return DC_ERROR;
            }
          }
        }
        return DC_ERROR;
      }else{
        DC_setStatus(DC_FLAG_MAIN_TCP);
        DC_debugOut("Already startes\r\n");
        return DC_OK;
      }
    }else{
      DC_resetStatuses(DC_STATUS_MASK_ONLY_GNSS_ON); //Reset statuses
      DC_debugOut("TCP status error\r\n");
      //return DC_ERROR;
    }
    
    vTaskDelay(1000);
  }
  
  MGT_close_TCP_by_index(1); // Close TCP
  
  return DC_TIMEOUT;
}
//--------------------------------------------------------------------------------------------------
//Set execution status
void Tracker_setExeStatus(DC_flag_t DC_flag, DC_func_t DC_func, DC_error_t DC_error)
{
  DC_taskCtrl.DC_exec.DC_flag = DC_flag;
  DC_taskCtrl.DC_exec.DC_error = DC_error;
  DC_taskCtrl.DC_exec.DC_func = DC_func;
}
//--------------------------------------------------------------------------------------------------
//Synch time
DC_return_t Tracker_timeSynch()
{
  float date;
  float time; 
  
  if(!DC_getStatus(DC_FLAG_GNSS_TYME_SYNCH))//It not synch
  {
    //Try get cell time
    if(MGT_get_time_network(&time, &date) == MC60_STD_OK)
    {
      CL_setDateTime(date, time);
      return DC_OK;
    }
    return DC_ERROR;
  }else{
    CL_setDateTime(globalGNSS_data.date, globalGNSS_data.time);
    DC_ledStatus_flash(7, 800);
    DC_params.UTC_time = globalUTC_time;
    DC_debugOut("Time synch OK: %d\r\n", (uint64_t)DC_params.UTC_time);
    return DC_OK;
  }
}
//--------------------------------------------------------------------------------------------------
//Send data
DC_return_t Tracker_wialonSend()
{
  char ansver[20];
  DC_log_t logData;
  WL_ansvers_t wialon_ansver = WL_ANS_UNKNOWN;
  uint8_t sendCounter;
  
  //Check login
  if(DC_getStatus(DC_WL_LOGIN_OK))
  {    
    sendCounter = DC_params.notsended_log_len;
    
    while (sendCounter--)
    {
      DC_readLog(DC_params.log_len - DC_params.notsended_log_len, &logData);
      WL_prepare_DATA(gStr_buf, &logData.GNSS_data, logData.MCU_temper, logData.BAT_voltage, logData.power, logData.Cell_quality, logData.Status); //Create package DATA
      Tracker_sendWialonPack(1, gStr_buf, ansver, 8000);
      WL_parce_DATA_ANS(ansver, &wialon_ansver);
      
      if(wialon_ansver == WL_ANS_SUCCESS )//data
      {
        DC_debugOut("Pack log #%i sended\r\n", DC_params.log_len- DC_params.notsended_log_len);
      }else{
        DC_debugOut("Pack log #%i not sended\r\n", DC_params.log_len- DC_params.notsended_log_len);
        if (MGT_close_TCP_by_index(1) == MC60_STD_OK) // Close TCP
          DC_resetStatus(DC_FLAG_MAIN_TCP);
        DC_resetStatus(DC_WL_LOGIN_OK);
        DC_resetStatus(DC_FLAG_GNSS_NOT_RECEIVED);
        DC_resetStatus(DC_FLAG_CELL_LOC_OK);
        DC_resetStatus(DC_FLAG_GNSS_RECEIVED);
        DC_params.notsended_log_len -= (DC_params.notsended_log_len - sendCounter);
        
        return DC_ERROR;
      }
    }
    
    DC_setStatus(DC_WL_PACK_SENED);
    DC_resetStatus(DC_WL_LOGIN_OK);
    DC_resetStatus(DC_FLAG_GNSS_NOT_RECEIVED);
    DC_params.notsended_log_len = 0;
    
    return DC_OK;
  }
  
  return DC_ERROR;
}
//--------------------------------------------------------------------------------------------------
//BT switch on
DC_return_t Tracker_BT_switchOn()
{
  uint8_t try_counter = 3;
  uint8_t status;
  
  //Check status
  if (DC_getStatus(DC_FLAG_BT_POWER_ON))
    return DC_OK;

  //Check status
  //Try BT power
  while(try_counter--)
  {  
    if (MGT_getStatusBT(&status) == MC60_STD_OK) //Get bluetooth power on status 
    {
      //Switched on
      if (status == '1')
      {
        
        DC_debugOut("###BT already on###\r\n");
        DC_setStatus(DC_FLAG_BT_POWER_ON);
        return DC_OK;
      }
    }
    
    if (MGT_powerBT(1) == MC60_STD_OK) //Power on bluetooth
    {      
      if (MGT_getStatusBT(&status) == MC60_STD_OK) //Get bluetooth power on status 
      {
        //Switched on
        if (status == '1')
        {
          if (MGT_setNameBT("MicroGo") == MC60_STD_OK)
            if (MGT_setVisibilityBT(1) == MC60_STD_OK)
            {
              DC_debugOut("###BT on###\r\n");
              DC_setStatus(DC_FLAG_BT_POWER_ON);
              return DC_OK;
            }
        }
      }
    }
    vTaskDelay(1000);
  }
  
  DC_resetStatus(DC_FLAG_BT_POWER_ON);
  return DC_ERROR;
}
uint16_t temperature;
//--------------------------------------------------------------------------------------------------
//IDLE subTask
void Tracker_IDLE_subTask()
{ 
  if (DC_taskCtrl.DC_modeCurrent == DC_MODE_IDLE)
  {
    WDOG_Feed(); //Watch dog

    //If first start
    if (DC_taskCtrl.DC_modeBefore== DC_MODE_IDLE)
    {
      DC_debugOut("***First start***\r\n");
      
      SWITCH_1WIRE_ON;

      
      temperature = DS18B20_getTemperature();
      
      
      DC_setModemMode(DC_MODEM_ALL_IN_ONE); //Set mode all in one
      
      //Try on modem
      if(Tracker_seq_modem_on() == DC_OK) //On modem sequence
      {
        
        //Tracker_BT_switchOn(); //On modem sequence
        
        //Registration sequence
        Tracker_cellReg_seq();
        
        //if (MGT_setCharacterSet("IRA") == MC60_STD_OK) //Set charecter set
          //if (MGT_setUSSD_mode("1") == MC60_STD_OK) //Set USSD mode
            MGT_returnUSSD("*105#", gStr_buf); //Get USSD query
        
        MGT_setUSSD_mode("2"); //Set USSD mode
        
        
//        if (MGT_setGSM_time_synch() == MC60_STD_OK) // Set GSM time
//          if(MGT_get_time_network(&globalGNSS_data.time, &globalGNSS_data.date) == MC60_STD_OK) // Get time synch from cell
//          {
//            //CL_setDateTime(globalGNSS_data.date, globalGNSS_data.time); //Set cell time
//          }
      }
      
      DC_taskCtrl.DC_modeCurrent = DC_MODE_AGPS_UPDATE;
      
      //      DC_nextPeriod = Tracker_getNextPeriodAndMode(&DC_taskCtrl.DC_modeCurrent); //Calculate next Period
      //      Tracker_sleep(DC_taskCtrl.DC_modeCurrent, DC_nextPeriod); //Go to Sleep
    }
    
    //If was before repeated subtask
    if (DC_taskCtrl.DC_modeBefore== DC_MODE_REPEATED)
    {
      //Timeout
      if (DC_taskCtrl.DC_exec.DC_flag == DC_FLAG_EXEC_TIMEOUT)
      {
        UTC_repeat_time_next = globalUTC_time;
        DC_nextPeriod = Tracker_getNextPeriodAndMode(&DC_taskCtrl.DC_modeCurrent); //Calculate next Period
        Tracker_sleep(DC_taskCtrl.DC_modeCurrent, DC_nextPeriod); //Go to Sleep
      }
      
       //Error
      if (DC_taskCtrl.DC_exec.DC_flag == DC_FLAG_EXEC_ERROR)
      {
        UTC_repeat_time_next = globalUTC_time;
        DC_nextPeriod = Tracker_getNextPeriodAndMode(&DC_taskCtrl.DC_modeCurrent); //Calculate next Period
        Tracker_sleep(DC_taskCtrl.DC_modeCurrent, DC_nextPeriod); //Go to Sleep
      }
      
      //OK
      if (DC_taskCtrl.DC_exec.DC_flag == DC_FLAG_EXEC_OK)
      {
        UTC_repeat_time_next = globalUTC_time;
        DC_nextPeriod = Tracker_getNextPeriodAndMode(&DC_taskCtrl.DC_modeCurrent); //Calculate next Period
        Tracker_sleep(DC_taskCtrl.DC_modeCurrent, DC_nextPeriod); //Go to Sleep
      }
    }
    
    //If was before AGPS mode
    if (DC_taskCtrl.DC_modeBefore== DC_MODE_AGPS_UPDATE)
    {
      //Timeout
      if (DC_taskCtrl.DC_exec.DC_flag == DC_FLAG_EXEC_TIMEOUT)
      {
        if (tryCounter_AGPS < TRY_COUNT_ERROR_AGPS)
        {
          DC_taskCtrl.DC_modeCurrent = DC_MODE_AGPS_UPDATE;
          tryCounter_AGPS++;
        }else{
          tryCounter_AGPS = 0;
          UTC_AGPS_time_before = globalUTC_time;
          DC_nextPeriod = Tracker_getNextPeriodAndMode(&DC_taskCtrl.DC_modeCurrent); //Calculate next Period
          Tracker_sleep(DC_taskCtrl.DC_modeCurrent, DC_nextPeriod); //Go to Sleep
        }
      }
      //Error
      if (DC_taskCtrl.DC_exec.DC_flag == DC_FLAG_EXEC_ERROR)
      {
        if (tryCounter_AGPS < TRY_COUNT_ERROR_AGPS)
        {
          DC_taskCtrl.DC_modeCurrent = DC_MODE_AGPS_UPDATE;
          tryCounter_AGPS++;
        }else{
          tryCounter_AGPS = 0;
          DC_nextPeriod = Tracker_getNextPeriodAndMode(&DC_taskCtrl.DC_modeCurrent); //Calculate next Period
          Tracker_sleep(DC_taskCtrl.DC_modeCurrent, DC_nextPeriod); //Go to Sleep
        }
      }
      //OK
      if (DC_taskCtrl.DC_exec.DC_flag == DC_FLAG_EXEC_OK)
      {
        UTC_AGPS_time_before = globalUTC_time;
        DC_nextPeriod = Tracker_getNextPeriodAndMode(&DC_taskCtrl.DC_modeCurrent); //Calculate next Period
        Tracker_sleep(DC_taskCtrl.DC_modeCurrent, DC_nextPeriod); //Go to Sleep
      }
    }
    
    //If was before GNSS mode
    if (DC_taskCtrl.DC_modeBefore == DC_MODE_GNSS_DATA)
    {
      //Timeout
      if (DC_taskCtrl.DC_exec.DC_flag == DC_FLAG_EXEC_TIMEOUT)
      {
        if (tryCounter_GNSS < DC_settings.gnss_try_count-1)
        {
          DC_taskCtrl.DC_modeCurrent = DC_MODE_GNSS_DATA;
          tryCounter_GNSS++;
          Tracker_sleep_sec(DC_settings.gnss_try_sleep);
        }else{
          tryCounter_GNSS = 0;
          UTC_gnss_time_before = globalUTC_time;
          DC_nextPeriod = Tracker_getNextPeriodAndMode(&DC_taskCtrl.DC_modeCurrent); //Calculate next Period
          Tracker_sleep(DC_taskCtrl.DC_modeCurrent, DC_nextPeriod); //Go to Sleep
        }
      }
      //Error
      if (DC_taskCtrl.DC_exec.DC_flag == DC_FLAG_EXEC_ERROR)
      {
        if (tryCounter_GNSS < DC_settings.gnss_try_count-1)
        {
          DC_taskCtrl.DC_modeCurrent = DC_MODE_GNSS_DATA;
          tryCounter_GNSS++;
          Tracker_sleep_sec(DC_settings.gnss_try_sleep);
        }else{
          tryCounter_GNSS = 0;
          DC_nextPeriod = Tracker_getNextPeriodAndMode(&DC_taskCtrl.DC_modeCurrent); //Calculate next Period
          Tracker_sleep(DC_taskCtrl.DC_modeCurrent, DC_nextPeriod); //Go to Sleep
        } 
      }
      //OK
      if (DC_taskCtrl.DC_exec.DC_flag == DC_FLAG_EXEC_OK)
      {
        UTC_gnss_time_before = globalUTC_time;
        DC_nextPeriod = Tracker_getNextPeriodAndMode(&DC_taskCtrl.DC_modeCurrent); //Calculate next Period
        Tracker_sleep(DC_taskCtrl.DC_modeCurrent, DC_nextPeriod); //Go to Sleep
      }
    }
    
    //If was before Send data mode
    if (DC_taskCtrl.DC_modeBefore == DC_MODE_SEND_DATA)
    {
      //Timeout
      if (DC_taskCtrl.DC_exec.DC_flag == DC_FLAG_EXEC_TIMEOUT)
      {
        if (tryCounter_SendData < DC_settings.gsm_try_count-1)
        {
          DC_taskCtrl.DC_modeCurrent = DC_MODE_SEND_DATA;
          tryCounter_SendData++;
          Tracker_sleep(DC_taskCtrl.DC_modeCurrent, DC_settings.gsm_try_sleep); //Go to Sleep
        }else{
          tryCounter_SendData = 0;
          UTC_send_time_before = globalUTC_time;
          DC_nextPeriod = Tracker_getNextPeriodAndMode(&DC_taskCtrl.DC_modeCurrent); //Calculate next Period
          Tracker_sleep(DC_taskCtrl.DC_modeCurrent, DC_nextPeriod); //Go to Sleep
        }
      }
      //Error
      if (DC_taskCtrl.DC_exec.DC_flag == DC_FLAG_EXEC_ERROR)
      {
        //If before not found GNSS
        if (DC_taskCtrl.DC_exec.DC_error == DC_ERROR_GNSS_NOT_FOUND)
        {
          DC_taskCtrl.DC_modeCurrent = DC_MODE_GNSS_DATA; //Go to GNSS mode and try get GNSS
        }else{
          DC_resetStatus(DC_FLAG_GPRS_ACTIVE);
          if (tryCounter_SendData < DC_settings.gsm_try_count-1)
          {
            DC_taskCtrl.DC_modeCurrent = DC_MODE_SEND_DATA;
            tryCounter_SendData++;
            Tracker_sleep(DC_taskCtrl.DC_modeCurrent, DC_settings.gsm_try_sleep); //Go to Sleep
          }else{
            tryCounter_SendData = 0;
            UTC_send_time_before = globalUTC_time;
            DC_nextPeriod = Tracker_getNextPeriodAndMode(&DC_taskCtrl.DC_modeCurrent); //Calculate next Period
            Tracker_sleep(DC_taskCtrl.DC_modeCurrent, DC_nextPeriod); //Go to Sleep
          }
          
        }
      }
      //OK
      if (DC_taskCtrl.DC_exec.DC_flag == DC_FLAG_EXEC_OK)
      {
        UTC_send_time_before = globalUTC_time;
        DC_nextPeriod = Tracker_getNextPeriodAndMode(&DC_taskCtrl.DC_modeCurrent); //Calculate next Period
        Tracker_sleep(DC_taskCtrl.DC_modeCurrent, DC_nextPeriod); //Go to Sleep
      }
    }      
    
    DC_taskCtrl.DC_modeBefore = DC_MODE_IDLE;
    
  }

}
//**************************************************************************************************
//Tracker task
void vTracker_Task(void *pvParameters)
{   
  DC_return_t return_stat;
  
  USB_Driver_Init(USB_Handler);
  
  vTaskSuspend( xHandle_Tracker );
  
  vTaskDelay(200);
  
  //Start sample timer
  DC_startSampleTimer(1); //Sample timer 1ms

  DC_taskCtrl.DC_modeCurrent = DC_MODE_IDLE;
  DC_taskCtrl.DC_modeBefore = DC_MODE_IDLE;
  
  LED_STATUS_OFF;
  
  while(1)
  {    
    //*************************************************************************************
    //Events
    
    //Ring
    if (DC_taskCtrl.DC_mes_status & (1<<DC_MES_INT_RING))
    {
      DC_debugOut("Event RING\r\n");
    }
    
    //ACC
    if (DC_taskCtrl.DC_mes_status & (1<<DC_MES_INT_ACC))
    {
      DC_debugOut("Event ACC\r\n");
    }
    
    //*************************************************************************************
    //Repeated mode
    if (DC_taskCtrl.DC_modeCurrent == DC_MODE_REPEATED)
    {
      if (DC_taskCtrl.DC_modeBefore == DC_MODE_IDLE)
      {
        DC_debugOut("***MODE: REPEATED***\r\n");
        
        DC_taskCtrl.DC_modeBefore = DC_MODE_REPEATED;
        
        DC_setModemMode(DC_MODEM_ALL_IN_ONE); //Set mode all in one
        
        //Try on modem
        if(Tracker_seq_modem_on() == DC_OK) //On modem sequence
        {
          Tracker_BT_switchOn(); //On modem sequence
        }
        
        //Get quality
        if (MGT_getQuality(&DC_params.Cell_quality, NULL) == MC60_STD_OK)
        {
          DC_debugOut("Cell quality: %d\r\n", DC_params.Cell_quality);
        }
        
        //Get sensors
        if( xSemaphoreTake( ADC_mutex, portMAX_DELAY ) == pdTRUE )
        {
          DC_params.MCU_temper = DC_get_chip_temper();
          DC_params.BAT_voltage = DC_get_bat_voltage();
          xSemaphoreGive( ADC_mutex );
        }
        
        UTC_repeat_time_before = globalUTC_time;
        Tracker_setExeStatus(DC_FLAG_EXEC_OK, DC_FUNC_NONE, DC_ERROR_NO); 
        DC_taskCtrl.DC_modeCurrent = DC_MODE_IDLE;
        goto IDLE;
      }
    }
    
    //*************************************************************************************
    //Update AGPS
    if (DC_taskCtrl.DC_modeCurrent == DC_MODE_AGPS_UPDATE)
    {
      if (DC_taskCtrl.DC_modeBefore == DC_MODE_IDLE)
      {
        DC_debugOut("***MODE: AGPS UPDATE***\r\n");
        
        DC_taskCtrl.DC_modeBefore = DC_MODE_AGPS_UPDATE;
        DC_setModemMode(DC_MODEM_ALL_IN_ONE); //Set mode all in one
        
        //Try on modem
        if(Tracker_seq_modem_on() != DC_OK) //On modem sequence
        {
          DC_debugOut("Can't switch on modem: reset\r\n");
          
          Tracker_setExeStatus(DC_FLAG_EXEC_ERROR, DC_FUNC_MODEM_ON, DC_ERROR_SWITCH); 
          DC_taskCtrl.DC_modeCurrent = DC_MODE_IDLE;
          goto IDLE;
        }
        
        //Switch on GNSS
        if (Tracker_switch_on_GNSS() == DC_OK)
        {
          DC_setStatus(DC_FLAG_GNSS_ON);
        }else{
          DC_debugOut("Can't switch on GNSS: reset\r\n");
          
          Tracker_setExeStatus(DC_FLAG_EXEC_ERROR, DC_FUNC_GNSS_ON, DC_ERROR_SWITCH); 
          DC_taskCtrl.DC_modeCurrent = DC_MODE_IDLE;
          goto IDLE;
            
        }
        
        //Connect to GPRS
        if ((return_stat = Tracker_GPRS_con_seq()) != DC_OK)
        {
          //If error seq
          if (return_stat == DC_ERROR)
          {
            DC_debugOut("Error, wait reboot\r\n");
            
            Tracker_setExeStatus(DC_FLAG_EXEC_ERROR, DC_FUNC_GPRS_CONN, DC_ERROR_CONNECT); 
            DC_taskCtrl.DC_modeCurrent = DC_MODE_IDLE;
            goto IDLE;
          }
          //If timeout seq
          if (return_stat == DC_TIMEOUT)
          {
            DC_debugOut("Wait operator\r\n");
            Tracker_setExeStatus(DC_FLAG_EXEC_TIMEOUT, DC_FUNC_GPRS_CONN, DC_ERROR_CONNECT); 
            DC_taskCtrl.DC_modeCurrent = DC_MODE_IDLE;
            goto IDLE;
          }
        }
        
        //AGPS
        if (Tracker_AGPS_seq() != DC_OK)
        {
          DC_debugOut("Can't get AGPS\r\n");
          Tracker_setExeStatus(DC_FLAG_EXEC_TIMEOUT, DC_FUNC_AGPS_GET, DC_ERROR_MODEM_CMD); 
          DC_taskCtrl.DC_modeCurrent = DC_MODE_IDLE;
          goto IDLE;
        }   

        //Synch time
        if ((return_stat = Tracker_timeSynch()) != DC_OK)
        {
          DC_debugOut("Can't synch time\r\n");          
          Tracker_setExeStatus(DC_FLAG_EXEC_ERROR, DC_FUNC_TIME_SYNCH, DC_ERROR_TIME_SYNCH);           
          DC_taskCtrl.DC_modeCurrent = DC_MODE_IDLE;
          goto IDLE;
          
        }else if(return_stat == DC_OK)
        {
          Tracker_setExeStatus(DC_FLAG_EXEC_OK, DC_FUNC_TIME_SYNCH, DC_ERROR_NO);        
          DC_taskCtrl.DC_modeCurrent = DC_MODE_IDLE;
          goto IDLE;
        }

      }
    }
    
    //*************************************************************************************
    //Collect GNSS
    if (DC_taskCtrl.DC_modeCurrent == DC_MODE_GNSS_DATA)
    {
      
      DC_debugOut("***MODE: GNSS DATA***\r\n");
      
      DC_taskCtrl.DC_modeBefore = DC_MODE_GNSS_DATA;
      
      DC_setModemMode(DC_MODEM_ALL_IN_ONE); //Set mode all in one
      
      //Try on modem if not on
      if(Tracker_seq_modem_on() != DC_OK) //On modem sequence
      {
        DC_debugOut("Can't switch on modem: reset\r\n");
        
        Tracker_setExeStatus(DC_FLAG_EXEC_ERROR, DC_FUNC_MODEM_ON, DC_ERROR_SWITCH); 
        DC_taskCtrl.DC_modeCurrent = DC_MODE_IDLE;
        goto IDLE;
      }
      
      //Switch on GNSS if not on
      if (Tracker_switch_on_GNSS() == DC_OK)
      {
        DC_setStatus(DC_FLAG_GNSS_ON);
      }else{
        DC_debugOut("Can't switch on GNSS: reset\r\n");
        
        Tracker_setExeStatus(DC_FLAG_EXEC_ERROR, DC_FUNC_GNSS_ON, DC_ERROR_SWITCH); 
        DC_taskCtrl.DC_modeCurrent = DC_MODE_IDLE;
        goto IDLE;
        
      }
      
      //Get GNSS data
      if ((return_stat = Tracker_get_GNSS()) != DC_OK)
      {
        DC_debugOut("Can't get position: sleep on time\r\n");
        
        Tracker_setExeStatus(DC_FLAG_EXEC_TIMEOUT, DC_FUNC_GNSS_GET, DC_ERROR_GNSS_NOT_FOUND);           
        DC_taskCtrl.DC_modeCurrent = DC_MODE_IDLE;
        goto IDLE;
      }
      
      //Read sensors
      Tracker_readSensors();
      
      //Log data
      DC_log.Cell_quality = DC_params.Cell_quality;
      DC_log.GNSS_data = globalGNSS_data;
      DC_log.power = DC_getPower();
      DC_log.Status = DC_params.Status;
      
      DC_saveLog(DC_log); //Save log data
      DC_readLog(DC_params.log_len-1, &DC_log);
      
      DC_params.notsended_log_len++;
      
      Tracker_setExeStatus(DC_FLAG_EXEC_OK, DC_FUNC_TIME_SYNCH, DC_ERROR_NO);        
      DC_taskCtrl.DC_modeCurrent = DC_MODE_IDLE;
      goto IDLE;
      
      //        //Synch time
      //        if ((return_stat = Tracker_timeSynch()) != DC_OK)
      //        {
      //          DC_debugOut("Can't synch time\r\n");
      //          
      //          Tracker_setExeStatus(DC_FLAG_EXEC_ERROR, DC_FUNC_TIME_SYNCH, DC_ERROR_TIME_SYNCH);           
      //          DC_taskCtrl.DC_modeCurrent = DC_MODE_IDLE;
      //          goto IDLE;
      //          
      //        }else if(return_stat == DC_OK)
      //        {
      //          Tracker_setExeStatus(DC_FLAG_EXEC_OK, DC_FUNC_TIME_SYNCH, DC_ERROR_NO);        
      //          DC_taskCtrl.DC_modeCurrent = DC_MODE_IDLE;
      //          goto IDLE;
      //        }
      
    }
    
    //*************************************************************************************
    //Send data mode
    if (DC_taskCtrl.DC_modeCurrent == DC_MODE_SEND_DATA)
    {
      if (DC_taskCtrl.DC_modeBefore == DC_MODE_IDLE)
      {
        DC_debugOut("***MODE: SEND DATA***\r\n");
        
        DC_taskCtrl.DC_modeBefore = DC_MODE_SEND_DATA;
        
        DC_setModemMode(DC_MODEM_ALL_IN_ONE); //Set mode all in one

        //Try on modem
        if(Tracker_seq_modem_on() != DC_OK) //On modem sequence
        {
          DC_debugOut("Can't switch on modem: reset\r\n");
          
          Tracker_setExeStatus(DC_FLAG_EXEC_ERROR, DC_FUNC_MODEM_ON, DC_ERROR_SWITCH);
          DC_taskCtrl.DC_modeCurrent = DC_MODE_IDLE;
          goto IDLE;
          
        }
        
        //Get IMEI
        if(Tracker_getIMEI() != DC_OK) //On modem sequence
        {
          DC_debugOut("Can't get IMEI\r\n");
          
          Tracker_setExeStatus(DC_FLAG_EXEC_ERROR, DC_FUNC_IMEI_GET, DC_ERROR_MODEM_CMD);
          DC_taskCtrl.DC_modeCurrent = DC_MODE_IDLE;
          goto IDLE;
        }
        
        //Connect to GPRS
        if ((return_stat = Tracker_GPRS_con_seq()) != DC_OK)
        {
          //If error seq
          if (return_stat == DC_ERROR)
          {
            DC_debugOut("Error, wait reboot\r\n");
            
            Tracker_setExeStatus(DC_FLAG_EXEC_ERROR, DC_FUNC_GPRS_CONN, DC_ERROR_CONNECT);
            DC_taskCtrl.DC_modeCurrent = DC_MODE_IDLE;
            goto IDLE;
          }
          //If timeout seq
          if (return_stat == DC_TIMEOUT)
          {
            DC_debugOut("Wait operator\r\n");
            
            Tracker_setExeStatus(DC_FLAG_EXEC_TIMEOUT, DC_FUNC_GPRS_CONN, DC_ERROR_CONNECT);
            DC_taskCtrl.DC_modeCurrent = DC_MODE_IDLE;
            goto IDLE;
          }
        }
        
        //Traccar server connection
        if ((return_stat = Tracker_server_connect()) != DC_OK)
        {
          //If error seq
          if (return_stat == DC_ERROR)
          {
            DC_debugOut("Error: TCP connection\r\n");
            
            Tracker_setExeStatus(DC_FLAG_EXEC_ERROR, DC_FUNC_TCP_CONN, DC_ERROR_CONNECT);
            DC_taskCtrl.DC_modeCurrent = DC_MODE_IDLE;
            goto IDLE;
          }
          //If timeout seq
          if (return_stat == DC_TIMEOUT)
          {
            DC_debugOut("Timeout: TCP connection\r\n");
            
            Tracker_setExeStatus(DC_FLAG_EXEC_TIMEOUT, DC_FUNC_TCP_CONN, DC_ERROR_CONNECT);
            DC_taskCtrl.DC_modeCurrent = DC_MODE_IDLE;
            goto IDLE;
          }           
        }
        
        //Login to wialon
        if ((return_stat = Tracker_wialonLogin()) != DC_OK)
        {
          //If error seq
          if (return_stat == DC_ERROR)
          {
            Tracker_setExeStatus(DC_FLAG_EXEC_ERROR, DC_FUNC_LOGIN, DC_ERROR_LOGIN);
            DC_taskCtrl.DC_modeCurrent = DC_MODE_IDLE;
            goto IDLE;
          }
          //If timeout seq
          if (return_stat == DC_TIMEOUT)
          {
            Tracker_setExeStatus(DC_FLAG_EXEC_TIMEOUT, DC_FUNC_LOGIN, DC_ERROR_LOGIN);
            DC_taskCtrl.DC_modeCurrent = DC_MODE_IDLE;
            goto IDLE;
          }       
        }
        
        //Send data
        
        //Check log len
        if (DC_params.notsended_log_len == 0)
        {
          Tracker_setExeStatus(DC_FLAG_EXEC_ERROR, DC_FUNC_SEND_DATA, DC_ERROR_NOTHING);
          DC_debugOut("GNSS not found\r\n");
          DC_taskCtrl.DC_modeCurrent = DC_MODE_IDLE;
          goto IDLE;
        }
        
        if ((return_stat = Tracker_wialonSend()) != DC_OK)
        {
          //If error seq
          if (return_stat == DC_ERROR)
          {
            Tracker_setExeStatus(DC_FLAG_EXEC_ERROR, DC_FUNC_LOGIN, DC_ERROR_SEND_DATA);
            DC_taskCtrl.DC_modeCurrent = DC_MODE_IDLE;
            goto IDLE;
          }
          //If timeout seq
          if (return_stat == DC_TIMEOUT)
          {
            Tracker_setExeStatus(DC_FLAG_EXEC_TIMEOUT, DC_FUNC_LOGIN, DC_ERROR_SEND_DATA);
            DC_taskCtrl.DC_modeCurrent = DC_MODE_IDLE;
            goto IDLE;
          }       
        }   
        
        Tracker_setExeStatus(DC_FLAG_EXEC_OK, DC_FUNC_SEND_DATA, DC_ERROR_NO);
        DC_taskCtrl.DC_modeCurrent = DC_MODE_IDLE;
        goto IDLE;
        
      }
      DC_taskCtrl.DC_modeCurrent = DC_MODE_IDLE;
      goto IDLE;
    }
    
    //*************************************************************************************
    //IDLE mode
  IDLE:
    Tracker_IDLE_subTask(); //IDLE subTask
    
    
  }
}