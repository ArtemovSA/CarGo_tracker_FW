
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

//APIs
#include "ModemAPI.h"

//Libs
#include "Device_ctrl.h"
#include "Delay.h"
#include "Modem.h"
#include "GNSS.h"
#include "wialon_ips2.h"
#include "ADC.h"
#include "UART.h"
#include "Date_time.h"
#include "Device_ctrl.h"
#include "Clock.h"
#include "cdc.h"
#include "DS18B20.h"

//IDLE command
#define CONTINUE_CMD    0
#define SWITCH_CMD      1

//Cell coordinates
float gCell_lat;
float gCell_lon;

//RTOS variables
extern xSemaphoreHandle ADC_mutex;
QueueHandle_t Tracker_con_Queue;
TaskHandle_t xHandle_Tracker;

USBReceiveHandler USB_Handler;

//Counters
uint8_t tryCounter_AGPS;
uint8_t tryCounter_CollectData;
uint8_t tryCounter_SendData;
uint8_t tryCounter_ACC_EVENT;

//Current modem params
char current_ip[16];
char current_IMEI[17];

//Cell coordinates
float gCell_lat;
float gCell_lon;

//GNSS coordinates
GNSS_data_t globalGNSS_data;

//Times in sec from UTC 
time_t UTC_collect_time_next;   //Next time for GNSS
time_t UTC_send_time_next;      //Next time for send data
time_t UTC_AGPS_time_next;      //Next time for synch AGPS

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
//Sleep on sec
DC_return_t Tracker_sleep_sec(uint32_t sleep_period)
{
  LED_STATUS_OFF;
  UART_RxDisable(); // Disable IRQ
  
  if (sleep_period > 0){
    sleepMode = 1; //Sleep flag mode
    DC_set_RTC_timer_s(sleep_period); //Set RTC timer
    
    DC_debugOut("Sleep on: %d\r\n", sleep_period);
    
    switch(DC_taskCtrl.DC_modeCurrent){
    case DC_MODE_AGPS_UPDATE: DC_debugOut("Next Function: DC_MODE_AGPS_UPDATE\r\n"); break;
    case DC_MODE_SEND_DATA: DC_debugOut("Next Function: DC_MODE_SEND_DATA\r\n"); break;
    case DC_MODE_COLLECT_DATA: DC_debugOut("Next Function: DC_MODE_COLLECT_DATA\r\n"); break;
    };
    
    if (DBG_Connected())
    {
      
      while (sleep_period--)
      {
        if (DC_taskCtrl.DC_event.flags.event_ACC)
          break;
        
        vTaskDelay(1000);
      }
    }else{
      
      while (sleep_period--)
      {
        if (DC_taskCtrl.DC_event.flags.event_ACC)
          break;
        
        vTaskDelay(1000);
      }
      
      //      DC_debugOut("%%%Sleep%%%\r\n");
      //      
      //      EMU_EnterEM2(true);
      //      
      //      DC_debugOut("%%%Wake UP%%%%\r\n");
      
      //      USBD_Disconnect();
      //      vTaskDelay(500);
      //      USBD_Connect();
      
      //USB_Driver_Init(USB_Handler);
    }
  }
  
  UART_RxEnable(); // Disable IRQ
  sleepMode = 0; //Sleep flag mode
  LED_STATUS_ON;
  
  return DC_OK;
}
//--------------------------------------------------------------------------------------------------
//Switch mode with sleep period
DC_return_t Tracker_modeSwitch()
{
  DC_mode_t mode = DC_MODE_IDLE;
  uint32_t sleep_period = 0;
  
  //ACC
  if (DC_taskCtrl.DC_event.flags.event_ACC)
  {
    DC_taskCtrl.DC_modeCurrent = DC_MODE_ACC_EVENT;    
    return DC_OK;
  }
  
  //Calc next exec time for modes
  if (DC_taskCtrl.DC_modeBefore == DC_MODE_AGPS_UPDATE)
  {
    UTC_AGPS_time_next = globalUTC_time + DC_settings.AGPS_synch_period;
  }
  
  if (DC_taskCtrl.DC_modeBefore == DC_MODE_SEND_DATA)
  {
    UTC_send_time_next = globalUTC_time + DC_settings.data_send_period;
  }
  
  if (DC_taskCtrl.DC_modeBefore == DC_MODE_COLLECT_DATA)
  {
    UTC_collect_time_next = globalUTC_time + DC_settings.data_gnss_period;
  }
  
  //If send data next mode
  if ((UTC_send_time_next <= UTC_collect_time_next) && (UTC_send_time_next <= UTC_AGPS_time_next))
  {
    //If not executed
    if (UTC_send_time_next < globalUTC_time)
    {
      mode = DC_MODE_SEND_DATA;
      sleep_period = 0;
    }else{
      mode = DC_MODE_SEND_DATA;
      sleep_period = UTC_send_time_next - globalUTC_time;
    }
  }  
  
  //If gnss data next mode
  if ((UTC_collect_time_next <= UTC_AGPS_time_next) && (UTC_collect_time_next <= UTC_send_time_next))
  {
    //If not executed
    if (UTC_collect_time_next < globalUTC_time)
    {
      mode = DC_MODE_COLLECT_DATA;
      sleep_period = 0;
    }else{
      mode = DC_MODE_COLLECT_DATA;
      sleep_period = UTC_collect_time_next - globalUTC_time;
    }
  }
  
  //If AGPS sunch next mode
  if ((UTC_AGPS_time_next <= UTC_send_time_next) && (UTC_AGPS_time_next <= UTC_collect_time_next))
  {
    //If not executed
    if (UTC_AGPS_time_next < globalUTC_time)
    {
      mode = DC_MODE_AGPS_UPDATE;
      sleep_period = 0;
    }else{
      mode = DC_MODE_AGPS_UPDATE;
      sleep_period = UTC_AGPS_time_next - globalUTC_time;
    }
  }
  
  //Set modem mode
  if (sleep_period < NON_SWITCH_OFF_PERIOD)
  {
    DC_setModemMode(DC_MODEM_ALL_IN_ONE);
  }else{
    DC_setModemMode(DC_MODEM_ALL_OFF);
  }
  
  DC_taskCtrl.DC_modeCurrent = mode;
  Tracker_sleep_sec(sleep_period);
  
  return DC_OK;
}
//--------------------------------------------------------------------------------------------------
//Read sensors
void Tracker_readSensors()
{
  //Read sensors
  if( xSemaphoreTake( ADC_mutex, 2000 ) == pdTRUE )
  {
    DC_dataLog.MCU_temper = DC_get_chip_temper(); //Get chip temper
    DC_dataLog.BAT_voltage = DC_get_bat_voltage();
    xSemaphoreGive( ADC_mutex );
  }
  
  DC_debugOut("Tmcu=%.2f; Vbat=%.2f; mA/h=%.3lf\r\n", DC_dataLog.MCU_temper, (float)DC_dataLog.BAT_voltage, DC_getPower());

  //Temperature sensors
  DC_params.countTempSensors = DC_get_sensor_temper((float*)DC_dataLog.SensorTemp);
  DC_dataLog.countTempSensors = DC_params.countTempSensors;
    
  //Read temperatures
  for (int i=0; i<DC_params.countTempSensors; i++)
  {
    DC_debugOut("Temp sensor #%d = %.2f\r\n", i, DC_dataLog.SensorTemp[i]);
  }
  
  //Get cell quality
  if (MAPI_getQuality(&DC_params.Cell_quality) == DC_OK)
  {
    DC_debugOut("Cell quality: %d\r\n", DC_params.Cell_quality);
  }
  
}
//--------------------------------------------------------------------------------------------------
//IDLE subTask
uint8_t Tracker_IDLE_subTask()
{ 
  
  WDOG_Feed(); //Watch dog
  
  //If first start
  if (DC_taskCtrl.DC_modeBefore== DC_MODE_IDLE)
  {
    DC_debugOut("***First start***\r\n");     
    DC_taskCtrl.DC_modeCurrent = DC_MODE_AGPS_UPDATE;//DC_MODE_AGPS_UPDATE;
  }
  
  //If was before event
  if (DC_taskCtrl.DC_modeBefore == DC_MODE_ACC_EVENT)
  { 
    
    //Error
    if (DC_taskCtrl.DC_exec.DC_execFlag == DC_EXEC_FLAG_ERROR)
    {
      if (tryCounter_ACC_EVENT < TRY_COUNT_ACC_EVENT-1)
      {
        DC_taskCtrl.DC_modeCurrent = DC_MODE_ACC_EVENT;
        tryCounter_ACC_EVENT++;
      }else{
        tryCounter_ACC_EVENT = 0;
        Tracker_modeSwitch();
        DC_status.statusWord = 0;
        return SWITCH_CMD;
      }
      
      switch (DC_taskCtrl.DC_exec.DC_execFunc)
      {
      case DC_EXEC_FUNC_MODEM_ON:
        DC_setModemMode(DC_MODEM_ALL_OFF);
        DC_status.statusWord = 0;
        return SWITCH_CMD;
        break;
      case DC_EXEC_FUNC_GNSS_ON:
        DC_setModemMode(DC_MODEM_ALL_OFF);
        DC_status.statusWord = 0;
        return SWITCH_CMD;
        break;
      case DC_EXEC_FUNC_COORD_GNSS_GET:
        Tracker_sleep_sec(DC_settings.collect_try_sleep);
        DC_status.statusWord = 0;
        return CONTINUE_CMD;
        break;
      case DC_EXEC_FUNC_COORD_GSM_GET:
        DC_status.statusWord = 0;
        return SWITCH_CMD;
        break;
      case DC_EXEC_FUNC_IMEI_GET:
        DC_status.statusWord = 0;
        return SWITCH_CMD;
        break;
      case DC_EXEC_FUNC_GPRS_CONN:
        Tracker_sleep_sec(DC_settings.collect_try_sleep);
        DC_status.statusWord = 0;
        return SWITCH_CMD;
        break;
      case DC_EXEC_FUNC_TCP_CONN:
        Tracker_sleep_sec(DC_settings.tcp_try_sleep);
        DC_status.statusWord = 0;
        return SWITCH_CMD;
        break;
      case DC_EXEC_FUNC_WIALON_LOGIN:
        Tracker_sleep_sec(DC_settings.tcp_try_sleep);
        DC_status.statusWord = 0;
        return SWITCH_CMD;
        break;
      case DC_EXEC_FUNC_GET_LOG:
        tryCounter_SendData = 0;
        Tracker_modeSwitch();
        DC_status.statusWord = 0;
        return CONTINUE_CMD;
        break;
      case DC_EXEC_FUNC_WIALON_SEND_DATA:
        Tracker_sleep_sec(DC_settings.tcp_try_sleep);
        DC_status.statusWord = 0;
        return SWITCH_CMD;
        break;
      default:
        break;
      }
    }
    
    //OK
    if (DC_taskCtrl.DC_exec.DC_execFlag == DC_EXEC_FLAG_OK)
    {
      Tracker_modeSwitch();
      DC_status.statusWord = 0;
      return SWITCH_CMD;
    }
    
  }
  
  //If was before AGPS mode
  if (DC_taskCtrl.DC_modeBefore == DC_MODE_AGPS_UPDATE)
  {    
    
    //Error
    if (DC_taskCtrl.DC_exec.DC_execFlag == DC_EXEC_FLAG_ERROR)
    {
      if (tryCounter_AGPS < DC_settings.AGPSMode_try_count-1)
      {
        DC_taskCtrl.DC_modeCurrent = DC_MODE_AGPS_UPDATE;
        tryCounter_AGPS++;
      }else{
        tryCounter_AGPS = 0;
        Tracker_modeSwitch();
        DC_status.statusWord = 0;
        return SWITCH_CMD;
      }
      
      switch (DC_taskCtrl.DC_exec.DC_execFunc)
      {
      case DC_EXEC_FUNC_MODEM_ON:
        DC_setModemMode(DC_MODEM_ALL_OFF);
        DC_status.statusWord = 0;
        return SWITCH_CMD;
        break;
      case DC_EXEC_FUNC_GNSS_ON:
        DC_setModemMode(DC_MODEM_ALL_OFF);
        DC_status.statusWord = 0;
        return SWITCH_CMD;
        break;
      case DC_EXEC_FUNC_GPRS_CONN:
        DC_status.flags.flag_GPRS_SETTED = false;
        DC_status.flags.flag_GPRS_CONNECTED = false;
        DC_setModemMode(DC_MODEM_ALL_OFF);
        DC_status.statusWord = 0;
        return SWITCH_CMD;
        break;
      case DC_EXEC_FUNC_AGPS_EXEC:
        if (tryCounter_AGPS < DC_settings.AGPSMode_try_count-1)
        {
          Tracker_sleep_sec(WAIT_DATA_PERIOD);
          DC_status.statusWord = 0;
          return SWITCH_CMD;
        }
        break;
      case DC_EXEC_FUNC_MODULE_TIME_SYNCH:
        if (tryCounter_AGPS < DC_settings.AGPSMode_try_count-1)
        {
          Tracker_sleep_sec(WAIT_DATA_PERIOD);
          DC_status.statusWord = 0;
          return SWITCH_CMD;
        }
        break;
      default:
        break;
      }

    }
    
    //OK
    if (DC_taskCtrl.DC_exec.DC_execFlag == DC_EXEC_FLAG_OK)
    {
      Tracker_modeSwitch();
      DC_status.statusWord = 0;
      return SWITCH_CMD;
    }
  }
  
  //If was before Collect mode
  if (DC_taskCtrl.DC_modeBefore == DC_MODE_COLLECT_DATA)
  {
    //Error
    if (DC_taskCtrl.DC_exec.DC_execFlag == DC_EXEC_FLAG_ERROR)
    {
      if (tryCounter_CollectData < DC_settings.collectMode_try_count)
      {
        DC_taskCtrl.DC_modeCurrent = DC_MODE_COLLECT_DATA;
        tryCounter_CollectData++;
      }else{
        tryCounter_CollectData = 0;
        Tracker_modeSwitch();
        DC_status.statusWord = 0;
        return SWITCH_CMD;
      } 
      
      switch (DC_taskCtrl.DC_exec.DC_execFunc)
      {
      case DC_EXEC_FUNC_MODEM_ON:
        DC_setModemMode(DC_MODEM_ALL_OFF);
        DC_status.statusWord = 0;
        return SWITCH_CMD;
        break;
      case DC_EXEC_FUNC_GNSS_ON:
        DC_setModemMode(DC_MODEM_ALL_OFF);
        DC_status.statusWord = 0;
        return SWITCH_CMD;
        break;
      case DC_EXEC_FUNC_COORD_GNSS_GET:
        Tracker_sleep_sec(DC_settings.collect_try_sleep);
        DC_status.statusWord = 0;
        return CONTINUE_CMD;
        break;
      case DC_EXEC_FUNC_COORD_GSM_GET:
        DC_status.statusWord = 0;
        return SWITCH_CMD;
        break;
      default:
        break;
      }

    }
    
    //OK
    if (DC_taskCtrl.DC_exec.DC_execFlag == DC_EXEC_FLAG_OK)
    {
      Tracker_modeSwitch();
      DC_status.statusWord = 0;
    }
  }
  
  //If was before Send data mode
  if (DC_taskCtrl.DC_modeBefore == DC_MODE_SEND_DATA)
  {
    //Error
    if (DC_taskCtrl.DC_exec.DC_execFlag == DC_EXEC_FLAG_ERROR)
    {
      if (tryCounter_SendData < DC_settings.sendMode_try_count-1)
      {
        DC_taskCtrl.DC_modeCurrent = DC_MODE_SEND_DATA;
        tryCounter_SendData++;
      }else{
        tryCounter_SendData = 0;
        Tracker_modeSwitch();
        DC_status.statusWord = 0;
        return SWITCH_CMD;
      }
      
      switch (DC_taskCtrl.DC_exec.DC_execFunc)
      {
      case DC_EXEC_FUNC_MODEM_ON:
        DC_setModemMode(DC_MODEM_ALL_OFF);
        DC_status.statusWord = 0;
        return SWITCH_CMD;
        break;
      case DC_EXEC_FUNC_IMEI_GET:
        DC_status.statusWord = 0;
        return SWITCH_CMD;
        break;
      case DC_EXEC_FUNC_GPRS_CONN:
        Tracker_sleep_sec(DC_settings.send_try_sleep);
        DC_status.statusWord = 0;
        return SWITCH_CMD;
        break;
      case DC_EXEC_FUNC_TCP_CONN:
        Tracker_sleep_sec(DC_settings.tcp_try_sleep);
        DC_status.statusWord = 0;
        return SWITCH_CMD;
        break;
      case DC_EXEC_FUNC_WIALON_LOGIN:
        Tracker_sleep_sec(DC_settings.tcp_try_sleep);
        DC_status.statusWord = 0;
        return SWITCH_CMD;
        break;
      case DC_EXEC_FUNC_GET_LOG:
        tryCounter_SendData = 0;
        Tracker_modeSwitch();
        DC_status.statusWord = 0;
        return CONTINUE_CMD;
        break;
      case DC_EXEC_FUNC_WIALON_SEND_DATA:
        Tracker_sleep_sec(DC_settings.tcp_try_sleep);
        DC_status.statusWord = 0;
        return CONTINUE_CMD;
        break;
      default:
        break;
      }
      
    }
    
    //OK
    if (DC_taskCtrl.DC_exec.DC_execFlag == DC_EXEC_FLAG_OK)
    {
      Tracker_modeSwitch();
      DC_status.statusWord = 0;
    }     
    
    DC_taskCtrl.DC_modeBefore = DC_MODE_IDLE;
    
  }
  
  return CONTINUE_CMD;
}
//--------------------------------------------------------------------------------------------------
//Go to IDLE
uint8_t Tracker_goToIDLE(DC_execFlag_t DC_flag, DC_execFunc_t DC_func)
{
  DC_taskCtrl.DC_exec.DC_execFlag = DC_flag;
  DC_taskCtrl.DC_exec.DC_execFunc = DC_func;
  
  return Tracker_IDLE_subTask();
}
//--------------------------------------------------------------------------------------------------
//AGPS mode
void Tracker_exec_AGPS_mode()
{
  DC_ledStatus_flash(20, 50);
  
  DC_taskCtrl.DC_modeBefore = DC_MODE_AGPS_UPDATE;
  
  DC_debugOut("***MODE: AGPS UPDATE***\r\n");
  
  DC_setModemMode(DC_MODEM_ALL_IN_ONE); //Set mode all in one
  
  //Try on modem
  if(MAPI_switch_on_Modem() == DC_OK) //On modem sequence
  {
    DC_debugOut("Modem switch OK\r\n"); 
  }else{
    DC_debugOut("Modem switch ERROR\r\n");
    if (Tracker_goToIDLE(DC_EXEC_FLAG_ERROR, DC_EXEC_FUNC_MODEM_ON))
      return;
  }
  
  //Switch on GNSS
  if (MAPI_switch_on_GNSS() == DC_OK)
  {
    DC_debugOut("GNSS switch OK\r\n");        
  }else{
    DC_debugOut("GNSS switch ERROR\r\n");
    if (Tracker_goToIDLE(DC_EXEC_FLAG_ERROR, DC_EXEC_FUNC_GNSS_ON))
      return;
  }
  
  //Connect to GPRS
  if (MAPI_GPRS_connect(current_ip) == DC_OK)
  {
    DC_debugOut("GPRS connection OK\r\n");      
  }else{
    DC_debugOut("GPRS connection ERROR\r\n");
    if (Tracker_goToIDLE(DC_EXEC_FLAG_ERROR, DC_EXEC_FUNC_GPRS_CONN))
      return;
  }
  
  //AGPS
  if (MAPI_set_AGPS() == DC_OK)
  {
    DC_debugOut("AGPS execution OK\r\n");
  }else{
    DC_debugOut("AGPS execution ERROR\r\n");
    if (Tracker_goToIDLE(DC_EXEC_FLAG_ERROR, DC_EXEC_FUNC_AGPS_EXEC))
      return;
  }
  
  if (Tracker_goToIDLE(DC_EXEC_FLAG_OK, DC_EXEC_FUNC_AGPS_EXEC))
      return;

  //Module Synch Time
  if (MAPI_ModuleSynchTime() == DC_OK)
  {
    DC_debugOut("Module time synch OK\r\n");
  }else{
    DC_debugOut("Module time synch ERROR\r\n");
    if (Tracker_goToIDLE(DC_EXEC_FLAG_ERROR, DC_EXEC_FUNC_MODULE_TIME_SYNCH))
      return;
  }
}
//--------------------------------------------------------------------------------------------------
//Collect mode
void Tracker_exec_Collect_mode()
{
  uint8_t try_getGNSSdata = DC_settings.gnss_try_count;
  
  DC_ledStatus_flash(20, 50);
  
  DC_taskCtrl.DC_modeBefore = DC_MODE_COLLECT_DATA;
  
  DC_debugOut("***MODE: COLLECT DATA***\r\n");
  
  DC_setModemMode(DC_MODEM_ALL_IN_ONE); //Set mode all in one
  
  //Try on modem
  if(MAPI_switch_on_Modem() == DC_OK) //On modem sequence
  {
    DC_debugOut("Modem switch OK\r\n"); 
  }else{
    DC_debugOut("Modem switch ERROR\r\n");
    if (Tracker_goToIDLE(DC_EXEC_FLAG_ERROR, DC_EXEC_FUNC_MODEM_ON))
      return;
  }
  
  //Switch on GNSS
  if (MAPI_switch_on_GNSS() == DC_OK)
  {
    DC_debugOut("GNSS switch OK\r\n");        
  }else{
    DC_debugOut("GNSS switch ERROR\r\n");
    if (Tracker_goToIDLE(DC_EXEC_FLAG_ERROR, DC_EXEC_FUNC_GNSS_ON))
      return;
  }
  
  while (1)
  {
    //Check end try
    if (try_getGNSSdata--)
    {
      
      //Get GNSS data
      if (MAPI_get_GNSS(&globalGNSS_data) == DC_OK)
      {
        globalGNSS_data.coordSource = COORD_SOURCE_GNSS; //Set coordinate source
        DC_debugOut("GNSS getting OK\r\n");
        break;
      }else{
        if (Tracker_goToIDLE(DC_EXEC_FLAG_ERROR, DC_EXEC_FUNC_COORD_GNSS_GET))
          return;
      }
      
    }else{
      
      float cellLat;
      float cellLon;
      float date;
      float time;
      
      //Get celloc
      if (MAPI_getCelloc(&cellLat, &cellLon) == DC_OK)//Get celloc
      {
        //Convertion to HHMMSS
        globalGNSS_data.lat1 = GNSS_convertTo_HHMMSS(cellLat);
        globalGNSS_data.lon1 = GNSS_convertTo_HHMMSS(cellLon);
        DC_debugOut("Getting cell loc lat:%0.3f lon:%0.3f\r\n", cellLat, cellLon);
        
        globalGNSS_data.coordSource = COORD_SOURCE_GNSS; //Set coordinate source
        
        CL_getDateTime(&date, &time);
        globalGNSS_data.date = date;
        globalGNSS_data.time = time;
        globalGNSS_data.alt = 0;
        globalGNSS_data.cource = 0;
        globalGNSS_data.hdop = 0;
        globalGNSS_data.lock = 1;
        globalGNSS_data.sats = 0;
        globalGNSS_data.speed = 0;
        break;
        
      }else{
        DC_debugOut("Getting cell ERROR\r\n");
        if (Tracker_goToIDLE(DC_EXEC_FLAG_ERROR, DC_EXEC_FUNC_COORD_GSM_GET))
          return;
      }
      
    }
  }
  
  //Read sensors
  Tracker_readSensors();
  
  //Log data
  DC_dataLog.Cell_quality = DC_params.Cell_quality;
  DC_dataLog.GNSS_data = globalGNSS_data;
  DC_dataLog.power = DC_getPower();
  DC_dataLog.Status = DC_status.statusWord;
  DC_dataLog.Event = DC_taskCtrl.DC_event.eventWord;
  
  DC_addDataLog(DC_dataLog);
  
  if (Tracker_goToIDLE(DC_EXEC_FLAG_OK, DC_EXEC_FUNC_COORD_GNSS_GET))
    return;
}
//--------------------------------------------------------------------------------------------------
//Send mode
void Tracker_exec_Send_mode()
{
  DC_return_t return_stat;
  DC_taskCtrl.DC_modeBefore = DC_MODE_SEND_DATA;
  DC_ledStatus_flash(20, 50);
  DC_debugOut("***MODE: SEND DATA***\r\n");
  DC_setModemMode(DC_MODEM_ALL_IN_ONE); //Set mode all in one
  
  //Try on modem
  if(MAPI_switch_on_Modem() == DC_OK) //On modem sequence
  {
    DC_debugOut("Modem switch OK\r\n"); 
  }else{
    DC_debugOut("Modem switch ERROR\r\n");
    if (Tracker_goToIDLE(DC_EXEC_FLAG_ERROR, DC_EXEC_FUNC_MODEM_ON))
      return;
  }
  
  //Get IMEI
  if (!Modem_check_IMEI((char*)DC_settings.IMEI))
  {
    if (MAPI_getIMEI((char*)DC_settings.IMEI) != DC_OK)
    {
      if (Tracker_goToIDLE(DC_EXEC_FLAG_ERROR, DC_EXEC_FUNC_IMEI_GET))
        return;
    }
  }else{
    DC_save_settings(); //Save IMEI
  }
  
  //Connect to GPRS
  if (MAPI_GPRS_connect(current_ip) == DC_OK)
  {
    DC_debugOut("GPRS connection OK\r\n");      
  }else{
    DC_debugOut("GPRS connection ERROR\r\n");
    if (Tracker_goToIDLE(DC_EXEC_FLAG_ERROR, DC_EXEC_FUNC_GPRS_CONN))
      return;
  }
  
  //Server TCP connection
  if ((return_stat = MAPI_TCP_connect(TCP_CONN_ID_MAIN, DC_settings.ip_dataListLen, (char*)DC_settings.ip_dataList, DC_settings.dataPort)) == DC_OK)
  {
    DC_debugOut("MAIN TCP connected\r\n");
  }else if (return_stat != DC_OK){
    DC_debugOut("MAIN TCP ERROR\r\n");
    MGT_close_TCP_by_index(TCP_CONN_ID_MAIN); // Close TCP
    if (Tracker_goToIDLE(DC_EXEC_FLAG_ERROR, DC_EXEC_FUNC_TCP_CONN))
      return;
  }
  
  //Wialon login
  if (MAPI_wialonLogin((char*)DC_settings.IMEI, (char*)DC_settings.dataPass) == DC_OK)
  {
    DC_debugOut("Wialon login OK\r\n");    
  }else{
    DC_debugOut("Wialon login ERROR\r\n"); 
    if (Tracker_goToIDLE(DC_EXEC_FLAG_ERROR, DC_EXEC_FUNC_WIALON_LOGIN))
      return;
  }
  
  //Check log len
  if (DC_params.dataLog_len == 0)
  {
    DC_debugOut("Data log nothing\r\n");
    if (Tracker_goToIDLE(DC_EXEC_FLAG_ERROR, DC_EXEC_FUNC_GET_LOG))
      return;
  }
  
  DC_dataLog_t dataLog;
  uint8_t sendCounter = 0;
  uint8_t logDataLen = DC_params.dataLog_len;
  
  DC_debugOut("Count log data: %d\r\n", logDataLen);
  
  //Send data
  while (logDataLen--)
  {
    if (DC_readDataLog(&dataLog) == DC_ERROR)
    {
      DC_debugOut("Read log ERROR\r\n");
    }else{
      
      DC_debugOut("Pack log #%d sending\r\n", sendCounter+1);
      //Try send
      if (MAPI_wialonSend(&dataLog) == DC_OK)
      {
        DC_debugOut("Pack log sended\r\n");
        
        sendCounter++;
      }else{
        DC_debugOut("Pack log ERROR\r\n");
      }
      
    }
 
  }
  
  DC_params.dataLog_len = 0;
  DC_save_params();
  
  MGT_close_TCP_by_index(TCP_CONN_ID_MAIN); // Close TCP
  
  //End send data
  if (Tracker_goToIDLE(DC_EXEC_FLAG_OK, DC_EXEC_FUNC_WIALON_SEND_DATA))
    return;

}
//--------------------------------------------------------------------------------------------------
//ACC event
void Tracker_exec_ACC_event()
{
  uint8_t try_getGNSSdata = DC_settings.gnss_try_count;
  
  DC_ledStatus_flash(20, 100);
  
  DC_taskCtrl.DC_modeBefore = DC_MODE_ACC_EVENT;
  
  DC_debugOut("***EVENT: ACC***\r\n");
  
  DC_setModemMode(DC_MODEM_ALL_IN_ONE); //Set mode all in one
  
  //Try on modem
  if(MAPI_switch_on_Modem() == DC_OK) //On modem sequence
  {
    DC_debugOut("Modem switch OK\r\n"); 
  }else{
    DC_debugOut("Modem switch ERROR\r\n");
    if (Tracker_goToIDLE(DC_EXEC_FLAG_ERROR, DC_EXEC_FUNC_MODEM_ON))
      return;
  }
  
  //Switch on GNSS
  if (MAPI_switch_on_GNSS() == DC_OK)
  {
    DC_debugOut("GNSS switch OK\r\n");        
  }else{
    DC_debugOut("GNSS switch ERROR\r\n");
    if (Tracker_goToIDLE(DC_EXEC_FLAG_ERROR, DC_EXEC_FUNC_GNSS_ON))
      return;
  }
  
  while (1)
  {
    //Check end try
    if (try_getGNSSdata--)
    {
      
      //Get GNSS data
      if (MAPI_get_GNSS(&globalGNSS_data) == DC_OK)
      {
        globalGNSS_data.coordSource = COORD_SOURCE_GNSS; //Set coordinate source
        DC_debugOut("GNSS getting OK\r\n");
        break;
      }else{
        if (Tracker_goToIDLE(DC_EXEC_FLAG_ERROR, DC_EXEC_FUNC_COORD_GNSS_GET))
          return;
      }
      
    }else{
      
      float cellLat;
      float cellLon;
      float date;
      float time;
      
      //Get celloc
      if (MAPI_getCelloc(&cellLat, &cellLon) == DC_OK)//Get celloc
      {
        //Convertion to HHMMSS
        globalGNSS_data.lat1 = GNSS_convertTo_HHMMSS(cellLat);
        globalGNSS_data.lon1 = GNSS_convertTo_HHMMSS(cellLon);
        DC_debugOut("Getting cell loc lat:%0.3f lon:%0.3f\r\n", cellLat, cellLon);
        
        globalGNSS_data.coordSource = COORD_SOURCE_GNSS; //Set coordinate source
        
        CL_getDateTime(&date, &time);
        globalGNSS_data.date = date;
        globalGNSS_data.time = time;
        globalGNSS_data.alt = 0;
        globalGNSS_data.cource = 0;
        globalGNSS_data.hdop = 0;
        globalGNSS_data.lock = 1;
        globalGNSS_data.sats = 0;
        globalGNSS_data.speed = 0;
        break;
        
      }else{
        DC_debugOut("Getting cell ERROR\r\n");
        if (Tracker_goToIDLE(DC_EXEC_FLAG_ERROR, DC_EXEC_FUNC_COORD_GSM_GET))
          return;
      }
      
    }
  }
  
  //Read sensors
  Tracker_readSensors();
  
  //Log data
  DC_dataLog.Cell_quality = DC_params.Cell_quality;
  DC_dataLog.GNSS_data = globalGNSS_data;
  DC_dataLog.power = DC_getPower();
  DC_dataLog.Status = DC_status.statusWord;
  DC_dataLog.Event = DC_taskCtrl.DC_event.eventWord;
  DC_dataLog.Event |= (1 << DC_WIALON_STAT_ACC);
  
  //Get IMEI
  if (!Modem_check_IMEI((char*)DC_settings.IMEI))
  {
    if (MAPI_getIMEI((char*)DC_settings.IMEI) != DC_OK)
    {
      if (Tracker_goToIDLE(DC_EXEC_FLAG_ERROR, DC_EXEC_FUNC_IMEI_GET))
        return;
    }
  }else{
    DC_save_settings(); //Save IMEI
  }
  
  //Connect to GPRS
  if (MAPI_GPRS_connect(current_ip) == DC_OK)
  {
    DC_debugOut("GPRS connection OK\r\n");      
  }else{
    DC_debugOut("GPRS connection ERROR\r\n");
    if (Tracker_goToIDLE(DC_EXEC_FLAG_ERROR, DC_EXEC_FUNC_GPRS_CONN))
      return;
  }
  
  //Server TCP connection
  if (MAPI_TCP_connect(TCP_CONN_ID_MAIN, DC_settings.ip_dataListLen, (char*)DC_settings.ip_dataList, DC_settings.dataPort))
  {
    DC_debugOut("MAIN TCP connected\r\n");
  }else{
    DC_debugOut("MAIN TCP ERROR\r\n");
    MGT_close_TCP_by_index(TCP_CONN_ID_MAIN); // Close TCP
    if (Tracker_goToIDLE(DC_EXEC_FLAG_ERROR, DC_EXEC_FUNC_TCP_CONN))
      return;
  }
  
  //Wialon login
  if (MAPI_wialonLogin((char*)DC_settings.IMEI, (char*)DC_settings.dataPass) == DC_OK)
  {
    DC_debugOut("Wialon login OK\r\n");    
  }else{
    DC_debugOut("Wialon login ERROR\r\n"); 
    if (Tracker_goToIDLE(DC_EXEC_FLAG_ERROR, DC_EXEC_FUNC_WIALON_LOGIN))
      return;
  }

  //Try send
  if (MAPI_wialonSend(&DC_dataLog) == DC_OK)
  {
    DC_debugOut("Pack log sended\r\n");
  }else{
    DC_debugOut("Pack log ERROR\r\n");
    
    if (Tracker_goToIDLE(DC_EXEC_FLAG_ERROR, DC_EXEC_FUNC_WIALON_SEND_DATA))
      return;  
  }
  
  MGT_close_TCP_by_index(TCP_CONN_ID_MAIN); // Close TCP
  
  //End send data
  if (Tracker_goToIDLE(DC_EXEC_FLAG_OK, DC_EXEC_FUNC_WIALON_SEND_DATA))
    return;  

}
//**************************************************************************************************
//Tracker task
void vTracker_Task(void *pvParameters)
{   
  USB_Driver_Init(USB_Handler);
  
  MAPI_init(Tracker_con_Queue);
  vTaskSuspend( xHandle_Tracker );
  vTaskDelay(200);
  
  //Start sample timer
  DC_startSampleTimer(1); //Sample timer 1ms
  DC_startMonitorTimer(1000); //Start monitor timer

  LED_STATUS_ON;
  
  //Set start modes
  DC_taskCtrl.DC_modeCurrent = DC_MODE_IDLE;
  DC_taskCtrl.DC_modeBefore = DC_MODE_IDLE;

  while(1)
  {    
      
    DC_debugOut("Work!\r\n");
    
    //*************************************************************************************
    //Events
    
    //Ring
    if (DC_taskCtrl.DC_event.flags.event_RING)
    {
      DC_taskCtrl.DC_event.flags.event_RING = false;
      DC_debugOut("Event RING\r\n");
    }
    
    //ACC
    if (DC_taskCtrl.DC_event.flags.event_ACC)
    {
      
      DC_debugOut("Event ACC\r\n");
      DC_taskCtrl.DC_event.flags.event_ACC = false;
      DC_taskCtrl.DC_modeCurrent = DC_MODE_ACC_EVENT;     
      
    }
    //*************************************************************************************
    //ACC event
    if (DC_taskCtrl.DC_modeCurrent == DC_MODE_ACC_EVENT)
    {
      Tracker_exec_ACC_event();
    }  
    
    //*************************************************************************************
    //IDLE
    if (DC_taskCtrl.DC_modeCurrent == DC_MODE_IDLE)
    {
      Tracker_IDLE_subTask();// IDLE subTask
    }
    
    //*************************************************************************************
    
    //Update AGPS
    if (DC_taskCtrl.DC_modeCurrent == DC_MODE_AGPS_UPDATE)
    {
      //ACC
      if (DC_taskCtrl.DC_event.flags.event_ACC)
      {
        DC_taskCtrl.DC_modeCurrent = DC_MODE_ACC_EVENT;    
      }else{
        Tracker_exec_AGPS_mode(); //AGPS mode
      }
    }
    
    
    //*************************************************************************************
    //Collect GNSS
    if (DC_taskCtrl.DC_modeCurrent == DC_MODE_COLLECT_DATA)
    { 
      //ACC
      if (DC_taskCtrl.DC_event.flags.event_ACC)
      {
        DC_taskCtrl.DC_modeCurrent = DC_MODE_ACC_EVENT;    
      }else{
        Tracker_exec_Collect_mode(); //Collect mode
      }
    }
    
    //*************************************************************************************
    //Send data mode
    if (DC_taskCtrl.DC_modeCurrent == DC_MODE_SEND_DATA)
    {
      //ACC
      if (DC_taskCtrl.DC_event.flags.event_ACC)
      {
        DC_taskCtrl.DC_modeCurrent = DC_MODE_ACC_EVENT;    
      }else{
        Tracker_exec_Send_mode(); //Send mode
      }
    }
    
    
    //*************************************************************************************
  }
}