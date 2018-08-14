
#include "modemApi.h"
#include "stdio.h"
#include "string.h"
#include "UART.h"
#include "wialon_ips2.h"

//Buffers
static char gStr_buf[150];

QueueHandle_t MAPI_queue;

//--------------------------------------------------------------------------------------------------
//init API
DC_return_t MAPI_init(QueueHandle_t queue)
{
  MAPI_queue = queue;
  
  return DC_OK;
}
//--------------------------------------------------------------------------------------------------
//Registration
DC_return_t MAPI_cellReg()
{
  Modem_CREG_Q_ans_t reg;
  Modem_std_ans_t modem_ans;
  uint8_t reg_try_count = MAPI_REG_TRY_SEC;
  
  while(reg_try_count--)
  {
    modem_ans = MGT_getReg(&reg); //Get Registration
    
    if (modem_ans == Modem_STD_OK) //Check registration in net
    {
      switch(reg) //What make
      {
      case Modem_CREG_Q_REGISTERED: DC_debugOut("Reg OK\r\n"); DC_status.flags.flag_REG_OK = true; return DC_OK;
      case Modem_CREG_Q_ROAMING: DC_debugOut("Reg roaming\r\n"); DC_status.flags.flag_REG_OK = true; return DC_OK;
      case Modem_CREG_Q_SEARCH: DC_debugOut("Reg Search\r\n"); break;
      case Modem_CREG_Q_NOT_REG: DC_debugOut("Reg NOT\r\n"); return DC_ERROR;
      case Modem_CREG_Q_UNKNOWN: DC_debugOut("Reg UKN\r\n"); return DC_ERROR;
      case Modem_CREG_Q_DENIED: DC_debugOut("Reg Denied\r\n"); return DC_ERROR;
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
DC_return_t MAPI_GPRS_connect(char* deviceIP)
{
  uint16_t error_n;
  uint8_t GPRS_status = 0;
  Modem_std_ans_t modem_ans;
  DC_return_t DC_return;
  
  //Registration sequence
  if ((DC_return = MAPI_cellReg()) != DC_OK)
  {
    return DC_return;
  }
  
  //Setting GPRS
  if (!DC_status.flags.flag_GPRS_SETTED)
  {
    modem_ans = MGT_setAPN(Modem_DEFAUL_APN, &error_n);
    
    if (modem_ans == Modem_STD_OK)
    {
      //Check APN setted
      if (!DC_settings.settingFlags.flags.flag_APN_SETTED)
      {
        // Set APN, login, pass
        if (MGT_set_apn_login_pass(Modem_DEFAUL_APN, Modem_DEFAUL_LOGIN, Modem_DEFAUL_PASS) == Modem_STD_OK)
        {
          DC_settings.settingFlags.flags.flag_APN_SETTED = true;
          DC_debugOut("Setted APN settings, wait reboot\r\n");
          DC_reset_system(); //Reset system
        }
      }
    }
    
    if (MGT_setMUX_TCP(1) == Modem_STD_OK) //Setting multiple TCP/IP
    {
      DC_status.flags.flag_GPRS_SETTED = true;
      DC_debugOut("Check APN: OK\r\n");
    }
  }

  //Make GPRS connection
  if (!DC_status.flags.flag_GPRS_CONNECTED)
  {
    if (MGT_getGPRS(&GPRS_status) == Modem_STD_OK) //Get status
      
      if (GPRS_status == 0) //Not connected
      {
        
        if (MGT_setGPRS(1) == Modem_STD_OK) //activate GPRS
          if (MGT_getIP(deviceIP) == Modem_STD_OK) //get IP
            if (MGT_setAdrType(Modem_ADR_TYPE_IP) == Modem_STD_OK) //Set type addr
              if ( MGT_set_qihead() == Modem_STD_OK) //Set qihead
              {
                DC_status.flags.flag_GPRS_CONNECTED = true;
                DC_debugOut("GPRS OK\r\n");
                DC_debugOut("IP address: %s\r\n", deviceIP);
                return DC_OK;
              }
        
      }else{ //Already connected
        
        if (MGT_getIP(deviceIP) == Modem_STD_OK) //get IP
          if (MGT_setAdrType(Modem_ADR_TYPE_IP) == Modem_STD_OK) //Set type addr
            if (MGT_set_qihead() == Modem_STD_OK) //Set qihead
            {
              DC_status.flags.flag_GPRS_CONNECTED = true;
              DC_debugOut("GPRS OK\r\n");
              DC_debugOut("IP address: %s\r\n", deviceIP);
              return DC_OK;
            }
      }
  }else{
    
    if (MGT_getIP(deviceIP) == Modem_STD_OK) //get IP
    {
      DC_debugOut("GPRS ALREADY OK\r\n");
      DC_debugOut("IP address: %s\r\n", deviceIP);
      
      return DC_OK;
    }
    
  }

  return DC_ERROR;
}
//--------------------------------------------------------------------------------------------------
//Get celloc
DC_return_t MAPI_getCelloc(float *cellLat, float* cellLon)
{
  if (MGT_get_cell_loc(cellLat, cellLon) == Modem_STD_OK) // Get cell lock
  {    
    return DC_OK;
  }
  
  return DC_ERROR;
}
//--------------------------------------------------------------------------------------------------
//Set referent coord 
DC_return_t MAPI_setCellRef_loc()
{
  float cellLat;
  float cellLon;
  
  //Get celloc
  if (MAPI_getCelloc(&cellLat, &cellLon) == DC_OK)//Get celloc
  {
    //Set ref coord
    if (MGT_set_ref_coord_GNSS(cellLat, cellLon) == Modem_STD_OK) // Set reference location information for QuecFastFix Online
    {
      DC_debugOut("Coord referent OK\r\n");
    }else{
      DC_debugOut("Coord referent ERROR\r\n");
    }
    
    DC_debugOut("Cell loc lat:%0.3f lon:%0.3f\r\n", cellLat, cellLon);
    
    return DC_OK;
  }else{
    DC_debugOut("Cell loc error\r\n");
  }
  
  return DC_ERROR;
}
//--------------------------------------------------------------------------------------------------
//Switch on GNSS
DC_return_t MAPI_switch_on_GNSS()
{
  uint8_t status = 0;
  uint8_t try_count = DC_settings.gnss_try_count;
  uint16_t error_n = 0;
  Modem_std_ans_t ansver;
  
  while(try_count--)
  {
    status = 0;
    
    if (MGT_getPowerGNSS(&status) == Modem_STD_OK) //Get GNSS power
    {
      if (status == 1) //Already on
      {
        DC_debugOut("GNSS already on\r\n");
        return DC_OK;
      }
      if (status == 0) //Switched off
      {
        ansver = MGT_switch_GNSS(1, &error_n);
        if (ansver == Modem_STD_OK) { //Switch GNSS
          return DC_OK;
        }else if(ansver == Modem_STD_ERROR)
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
//On modem sequence
DC_return_t MAPI_switch_on_Modem()
{
  uint8_t try_on_counter = 3;
  uint8_t try_set_counter = 3;

  UART_RxEnable(); //Enale recive data
  
  if (MGT_modem_check() != Modem_STD_OK) //Check AT
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
      if (MGT_modem_check() == Modem_STD_OK) //Check AT
        if (MGT_switch_Sleep_mode(1) == Modem_STD_OK) //Switch SLEEP in
        {
          DC_debugOut("AT OK\r\n");
          
          // Set GSM time
          if(MGT_setGSM_time_synch() == Modem_STD_OK)
            DC_debugOut("GSM time synch setted\r\n");
          
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
//Get GNSS data
DC_return_t MAPI_get_GNSS(GNSS_data_t* data)
{  
  uint8_t try_count = DC_settings.gnss_try_count;
  Modem_std_ans_t ansver;
  
  while(try_count--)
  {
    ansver = MGT_get_GNSS(RMC, gStr_buf); //Get GNSS data
    if (ansver == Modem_STD_OK)
    {
      GNSS_parce_RMC(gStr_buf, data);
      
      //Check coordinate
      if ((data->lat1 > 0) && (data->lon1 > 0))
      {
        ansver = MGT_get_GNSS(GGA, gStr_buf); //Get GNSS data
        if (ansver == Modem_STD_OK)
        {
          GNSS_parce_GGA(gStr_buf, data);
        }
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
DC_return_t MAPI_getIMEI(char* retIMEI)
{
  if ( MGT_getIMEI(retIMEI) == Modem_STD_OK ) //Get IMEI
  {
    if (Modem_check_IMEI(retIMEI))  //Check IMEI
    {
      DC_debugOut("IMEI: %s\r\n", DC_settings.IMEI);
      return DC_OK;
    }
  }
  
  return DC_ERROR;
}
//--------------------------------------------------------------------------------------------------
//Send TCP str
DC_return_t MAPI_sendTCP(uint8_t index, char *str)
{
  Modem_TCP_send_t TCP_send_status = Modem_SEND_FAIL;
  
  if ( MGT_sendTCP(index, str, strlen(str), &TCP_send_status) == Modem_STD_OK) // Send TCP/UDP package index,len
    if (TCP_send_status == Modem_SEND_OK)
    {
      DC_debugOut("TCP message sended\r\n");
      return DC_OK;
    }
      
  DC_debugOut("Send fail\r\n");
  return DC_ERROR;
}
//--------------------------------------------------------------------------------------------------
//Send wialonPack
DC_return_t MAPI_sendWialonPack(uint8_t index, char *pack, char* ansver, uint16_t wait_ms)
{
  TT_mes_type tcp_task_msg;    
  uint8_t try_count = MAPI_SEND_TCP_TRY;
  
  while(try_count--)
  {
    if (MAPI_sendTCP(index, pack))//Send TCP str
    {
      //Wait message
      if ( xQueuePeek( MAPI_queue, &tcp_task_msg, wait_ms ) == pdTRUE )
      {
        //If from Modem gate task
        if (tcp_task_msg.task == TT_MGT_TASK)
          xQueueReceive( MAPI_queue, &tcp_task_msg, 0 );
        
        if (tcp_task_msg.type == EVENT_TCP_MSG) //If event message from TCP
          if (((Modem_TCP_recive_t*)(tcp_task_msg.message))->index == index)
          {
            
            memcpy(ansver, ((Modem_TCP_recive_t*)(tcp_task_msg.message))->data, ((Modem_TCP_recive_t*)(tcp_task_msg.message))->len);
            DC_debugOut("Wialon recive pack\r\n");
            return DC_OK;
            
          }else if(((Modem_TCP_recive_t*)(tcp_task_msg.message))->index == 0)
          {
            DC_debugOut("Data send fail\r\n"); //Recived other data
          }
      }
    }
    vTaskDelay(1000);
  }
  
  return DC_ERROR;
}
//--------------------------------------------------------------------------------------------------
//Wialon login
DC_return_t MAPI_wialonLogin(char* IMEI, char* pass)
{
  char ansver[20];
  WL_ansvers_t wialon_ansver = WL_ANS_UNKNOWN;
  
  WL_prepare_LOGIN(gStr_buf, IMEI, pass);
  
  //Send login pack
  if (MAPI_sendWialonPack(TCP_CONN_ID_MAIN, gStr_buf, ansver, MAPI_SEND_TCP_WAIT) == DC_OK)
  {
    //Parce answer
    WL_parce_LOGIN_ANS(ansver, &wialon_ansver);
    
    if (wialon_ansver == WL_ANS_SUCCESS)
    {
      DC_debugOut("Login OK\r\n");
      return DC_OK;
    }else{
      DC_debugOut("Login fail\r\n");
    }
  }

  return DC_ERROR;
}
//--------------------------------------------------------------------------------------------------
//Server TCP connection
DC_return_t MAPI_TCP_connect(uint8_t index, uint8_t ipList_Len, char* ipList, uint16_t port)
{
  uint8_t try_count = MAPI_SERVER_CONNECT_TRY;
  Modem_con_type_t conn_ans;
  uint8_t status_connect = 0;
  char buf_IP[16];
  
  while (try_count--){
    
    if (MGT_getConnectStat(index, &status_connect) == Modem_STD_OK) //Get connection statuses
    {
      if (status_connect == 0) //No connected
      {
        //Try open TCP connection from list
        for (int i=0; i<ipList_Len; i++)
        {
          //Get IP
          memcpy(buf_IP, (ipList+i*16), 16);
          
          if (MGT_openConn(&conn_ans, index, Modem_CON_TCP, buf_IP, port) == Modem_STD_OK) //make GPRS connection
          {
            if (conn_ans == Modem_CON_OK)
            {
              DC_params.current_data_IP = i;
              if (index == TCP_CONN_ID_MAIN)
                DC_status.flags.flag_MAIN_TCP = true;
              
              if (index == TCP_CONN_ID_SERVICE)
                DC_status.flags.flag_SERVICE_TCP = true;
              
              DC_debugOut("Data TCP connection IP: %s\r\n", buf_IP);
              return DC_OK;
            }
            
            if (conn_ans == Modem_CON_ALREADY)
            {
              DC_params.current_data_IP = i;
              
              if (index == TCP_CONN_ID_MAIN)
                DC_status.flags.flag_MAIN_TCP = true;
              
              if (index == TCP_CONN_ID_SERVICE)
                DC_status.flags.flag_SERVICE_TCP = true;

              DC_debugOut("Data TCP connected IP: %s\r\n",DC_settings.ip_dataList[i]);
              return DC_OK;
            }
            
            if (conn_ans == Modem_CON_FAIL)
            {
              if (MGT_setGPRS(0) == Modem_STD_OK) //deactivate GPRS
              {
                if (index == TCP_CONN_ID_MAIN)
                  DC_status.flags.flag_MAIN_TCP = false;
                
                if (index == TCP_CONN_ID_SERVICE)
                  DC_status.flags.flag_SERVICE_TCP = false;
              }
              MGT_close_TCP_by_index(1); // Close TCP
              DC_debugOut("Data TCP connction fail\r\n");
              return DC_ERROR;
            }
          }
        }
        return DC_ERROR;
      }else{
        
        if (index == TCP_CONN_ID_MAIN)
        {
          DC_status.flags.flag_MAIN_TCP = false;
          DC_debugOut("Main TCP already startes\r\n");
        }
        
        if (index == TCP_CONN_ID_SERVICE)
        {
          DC_status.flags.flag_SERVICE_TCP = false;
          DC_debugOut("Service TCP already startes\r\n");
        }
        
        return DC_OK;
      }
    }else{
      DC_debugOut("TCP status error\r\n");
    }
    
    vTaskDelay(1000);
  }
  
  MGT_close_TCP_by_index(1); // Close TCP
  
  return DC_TIMEOUT;
}
//--------------------------------------------------------------------------------------------------
//Get quality
DC_return_t MAPI_getQuality(uint8_t* retQuality)
{
  //Get quality
  if (MGT_getQuality(retQuality, NULL) == Modem_STD_OK)
  {
    return DC_OK;
  }
  
  return DC_ERROR;
}
//--------------------------------------------------------------------------------------------------
//AGPS
DC_return_t MAPI_set_AGPS()
{
  uint16_t error_n;

  //Set referent coord 
  if (MAPI_setCellRef_loc() != DC_OK)
    return DC_ERROR;
  
  //Time Synch sequence
  if (MAPI_SynchTime() != DC_OK)
    return DC_ERROR;
  
  //AGPS
  if (MGT_switch_EPO(1, &error_n) == Modem_STD_OK) //Switch EPO
  {
    
    if (MGT_EPO_trig() == Modem_STD_OK) //Trigger EPO
    {
      return DC_OK;
    }
    
  }else if(error_n == 7103){
    
    DC_debugOut("AGPS already ok\r\n");
    return DC_OK;
  }
  
  return DC_ERROR;
}
//--------------------------------------------------------------------------------------------------
//Time Synch
DC_return_t MAPI_ModuleSynchTime()
{
  uint8_t status;
  uint8_t try_counter = MAPI_SYNCH_TYME_TRY_SEC;
  
  while (try_counter --)
  {
    if (!DC_status.flags.flag_TYME_MODULE_SYNCH)
      if (MGT_getTimeSynch(&status) == Modem_STD_OK)
        if (status == 1)
        {
          DC_debugOut("Time sync ok\r\n");
          DC_status.flags.flag_TYME_MODULE_SYNCH = true;
          return DC_OK;
        }
    vTaskDelay(1000);
  }
  
  DC_debugOut("Time sync error\r\n");
  return DC_ERROR;
}
//--------------------------------------------------------------------------------------------------
//Synch time
DC_return_t MAPI_SynchTime()
{
  float date;
  float time; 
  
  if(!DC_status.flags.flag_TYME_MODULE_SYNCH)//It not synch
  {
    //Try get cell time
    if(MGT_get_time_network(&time, &date) == Modem_STD_OK)
    {
      CL_setDateTime(date, time);
      DC_params.UTC_time = globalUTC_time;
      DC_debugOut("Time gnss synch OK: %d\r\n", (uint64_t)DC_params.UTC_time);
      return DC_OK;
    }
    return DC_ERROR;
  }else{
    DC_params.UTC_time = globalUTC_time;
    DC_debugOut("Time cell synch OK: %d\r\n", (uint64_t)DC_params.UTC_time);
    return DC_OK;
  }
}
//--------------------------------------------------------------------------------------------------
//Send data
DC_return_t MAPI_wialonSend(DC_dataLog_t *data)
{
  char ansver[20];
  WL_ansvers_t wialon_ansver = WL_ANS_UNKNOWN; //wialon return
  
  WL_prepare_DATA(gStr_buf, &data->GNSS_data, data->MCU_temper, data->BAT_voltage, data->power, data->Cell_quality, data->Event, data->countTempSensors, data->SensorTemp); //Create package DATA
  MAPI_sendWialonPack(TCP_CONN_ID_MAIN, gStr_buf, ansver, MAPI_SEND_TCP_WAIT);
  
  WL_parce_DATA_ANS(ansver, &wialon_ansver);
  
  if(wialon_ansver != WL_ANS_SUCCESS )//data
  {
    return DC_ERROR;
  }
  
  return DC_OK;
}
//--------------------------------------------------------------------------------------------------
//BT switch on
DC_return_t MAPI_BT_switchOn()
{
  uint8_t try_counter = MAPI_SWITCH_ON_BT_TRY;
  uint8_t status;
  
  //Try BT power
  while(try_counter--)
  {  
    if (MGT_getStatusBT(&status) == Modem_STD_OK) //Get bluetooth power on status 
    {
      //Switched on
      if (status == '1')
      {
        DC_debugOut("BT switched on\r\n");
        DC_status.flags.flag_BT_POWER_ON = true;
        return DC_OK;
      }
    }else{
      MGT_powerBT(1); //Power on bluetooth
    }
    
    vTaskDelay(1000);
    
  }
  
  DC_debugOut("BT switch error\r\n");
  DC_status.flags.flag_BT_POWER_ON = false;
  return DC_ERROR;
}
//--------------------------------------------------------------------------------------------------
//Send USSD message
DC_return_t MAPI_sendUSSD(char* USSD_mes, char* USSD_ans)
{
  if (MGT_setCharacterSet("IRA") == Modem_STD_OK) //Set charecter set
    if (MGT_setUSSD_mode("1") == Modem_STD_OK) //Set USSD mode
    {
      MGT_returnUSSD(USSD_mes, USSD_ans); //Get USSD query
      MGT_setUSSD_mode("2"); //Set USSD mode
      
      return DC_OK;
    }

  return DC_ERROR;
}