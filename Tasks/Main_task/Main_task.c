
#include "Main_task.h"
#include "stdio.h"
#include "stdbool.h"
#include "GNSS.h"

#include "em_emu.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_wdog.h"
#include "em_leuart.h"
#include "em_rtc.h"

#include "Modem_gate_task.h"
#include "Monitor_task.h"
#include "Task_transfer.h"
#include "wialon_ips2.h"
#include "EXT_flash.h"
#include "MC60_lib.h"
#include "Device_ctrl.h"
#include "GNSS.h"
#include "cdc.h"
#include "UART.h"
#include "Service_ctrl.h"
#include "Date_time.h"

QueueHandle_t Main_con_Queue;
extern QueueHandle_t I2C_gate_con_Queue;
extern QueueHandle_t MGT_con_Queue;
extern QueueHandle_t Monitor_con_Queue;

uint16_t MGT_gsm_try_count = 0;
uint16_t MGT_service_period_s = 10;
uint8_t Main_quality = 0;
MC60_TCP_send_t TCP_send_status = MC60_SEND_FAIL;
TT_mes_type task_msg;
TT_status_type exec_status;
MC60_std_ans_t std_ans;
extern char synch_status;
char str_buf[170];
char ip[16];
char IMEI[17];
int server_status;

extern USBReceiveHandler USB_Handler;

//Prototypes
WL_ansvers_t Main_login(char *imei, char* pass); //Login wialon
WL_ansvers_t Main_send_short_data(GNSS_data_t *GNSS_data); //Send short pack
WL_ansvers_t Main_send_data(GNSS_data_t *GNSS_data, float *temper_sensors, uint8_t count_temper, float temp_mcu, float bat_voltage, double power, uint8_t quality, uint8_t status); //Send data pack

extern void vTimer_sample( TimerHandle_t xTimer ); //Current sample timer

//Service functions
int Main_service_send_status_req(); //Request status
int Main_service_send_fw_status_req(); //Request fw status
int Main_service_send_fw_pack_req(char *pack, int *pack_len); //Request fw pack
int Main_service_send_settings_req(); //Request settings

//--------------------------------------------------------------------------------------------------
//Connect to GPRS
uint8_t Main_GPRS_connect()
{
  uint16_t error_n;
  MC60_CREG_Q_ans_t reg;
  uint8_t GPRS_status = 0;
  
  //Check registration
  if (!DC_getStatus(DC_FLAG_REG_OK))
  {
    std_ans = MGT_getReg(&reg);
    if (std_ans == MC60_STD_OK) //Check registration in net
    {
      switch(reg) {
      case MC60_CREG_Q_REGISTERED: USB_send_str("Reg OK\r\n"); DC_setStatus(DC_FLAG_REG_OK); break;
      case MC60_CREG_Q_ROAMING: USB_send_str("Reg roaming\r\n"); DC_setStatus(DC_FLAG_REG_OK); break;
      case MC60_CREG_Q_SEARCH: USB_send_str("Reg Search\r\n"); break;
      case MC60_CREG_Q_NOT_REG: USB_send_str("Reg NOT\r\n"); break;
      case MC60_CREG_Q_UNKNOWN: USB_send_str("Reg UKN\r\n"); break;
      case MC60_CREG_Q_DENIED: USB_send_str("Reg Denied\r\n"); return 0;
      default: return 0;
      };
      vTaskDelay(50);
    }else if (std_ans == MC60_STD_ERROR)// timeout
    {
      //MGT_init(&Main_con_Queue); //Init modem gate
      USB_send_str("\r\nRESTART\r\n");
    }
  }
  
  //Set APN if registration and AT OK
  if ((!DC_getStatus(DC_FLAG_APN_OK)) && (DC_getStatus(DC_FLAG_AT_OK)) && (DC_getStatus(DC_FLAG_REG_OK)))
  {
    std_ans = MGT_setAPN(DEFAUL_APN, &error_n);
    if (std_ans == MC60_STD_OK)
    {
      //if (MGT_set_apn_login_pass(DEFAUL_APN, DEFAUL_LOGIN, DEFAUL_PASS) == MC60_STD_OK) // Set APN, login, pass
      {
        //vTaskDelay(500);
        if (MGT_setMUX_TCP(1)) //Setting multiple TCP/IP
        {
          DC_setStatus(DC_FLAG_APN_OK);
          USB_send_str("APN OK\r\n");
        }
      }
    }else if((std_ans == MC60_STD_ERROR)&&(error_n > 0)){
      DC_setStatus(DC_FLAG_APN_OK);
      USB_send_str("APN ALREADY OK\r\n");
    }
  }
  
  //Make GPRS connection
  if (DC_getStatus(DC_FLAG_APN_OK) && DC_getStatus(DC_FLAG_AT_OK) && DC_getStatus(DC_FLAG_REG_OK) && !DC_getStatus(DC_FLAG_GPRS_ACTIVE))
    if (MGT_getGPRS(&GPRS_status) == MC60_STD_OK) //Get status
      if (GPRS_status == 0)
      {
        if (MGT_setGPRS(1) == MC60_STD_OK) //activate GPRS
          if (MGT_getIP(ip) == MC60_STD_OK) //get IP
            if (MGT_setAdrType(MC60_ADR_TYPE_IP) == MC60_STD_OK) //Set type addr
              if ( MGT_set_qihead() == MC60_STD_OK) //Set qihead
              {
                DC_setStatus(DC_FLAG_GPRS_ACTIVE);
                USB_send_str("GPRS OK\r\n");
                sprintf(str_buf, "IP address: %s\r\n", ip);
                USB_send_str(str_buf);
              }
      }else{
        if (MGT_getIP(ip) == MC60_STD_OK) //get IP
          if (MGT_setAdrType(MC60_ADR_TYPE_IP) == MC60_STD_OK) //Set type addr
            if ( MGT_set_qihead() == MC60_STD_OK) //Set qihead
            {
              DC_setStatus(DC_FLAG_GPRS_ACTIVE);
              USB_send_str("GPRS OK\r\n");
              sprintf(str_buf, "IP address: %s\r\n", ip);
              USB_send_str(str_buf);
            }
      }

  return 1;
}
//--------------------------------------------------------------------------------------------------
//Service TCP connection
void Main_service_connect()
{
  MC60_con_type_t conn_ans;
  uint8_t status_connect = 0;
  
  if ( DC_getStatus(DC_FLAG_GPRS_ACTIVE) && !DC_getStatus(DC_FLAG_SERVICE_TCP) )
    if (MGT_getConnectStat(0, &status_connect) == MC60_STD_OK) //Get connection statuses
    {
      if (status_connect == 0)
      {
        if (MGT_openConn(&conn_ans, 0, MC60_CON_TCP, DC_settings.DC_SET_service_ip, DC_settings.DC_SET_service_port) == MC60_STD_OK) //make GPRS connection
        {
          if (conn_ans == MC60_CON_OK)
          {
            DC_setStatus(DC_FLAG_SERVICE_TCP);
            USB_send_str("Service TCP ok\r\n");
          }
          
          if (conn_ans == MC60_CON_ALREADY)
          {
            DC_setStatus(DC_FLAG_SERVICE_TCP);
            USB_send_str("Service TCP already\\r\n");
          }
          
          if (conn_ans == MC60_CON_FAIL)
          {
            if (MGT_setGPRS(0) == MC60_STD_OK) //deactivate GPRS
              DC_resetStatus(DC_FLAG_GPRS_ACTIVE);
            USB_send_str("Service TCP fail\r\n");
          }
        }
      }else{
        DC_setStatus(DC_FLAG_SERVICE_TCP);
        USB_send_str("Service TCP already\r\n");
      }
    }else{
      DC_resetStatuses(DC_STATUS_MASK_GNSS_SAVE); //Reset statuses
    }
}
//--------------------------------------------------------------------------------------------------
//Main TCP connection
void Main_main_connect()
{
  MC60_con_type_t conn_ans;
  uint8_t status_connect = 0;
  
  if ( DC_getStatus(DC_FLAG_GPRS_ACTIVE)  ) //&& !DC_getStatus(DC_FLAG_MAIN_TCP)
    if (MGT_getConnectStat(1, &status_connect) == MC60_STD_OK) //Get connection statuses
    {
      if (status_connect == 0)
      {
        if (MGT_openConn(&conn_ans, 1, MC60_CON_TCP, DC_settings.DC_SET_wialon_ip, DC_settings.DC_SET_wialon_port) == MC60_STD_OK) //make GPRS connection
        {
          if (conn_ans == MC60_CON_OK)
          {
            DC_resetStatus(DC_WL_LOGIN_OK);
            DC_setStatus(DC_FLAG_MAIN_TCP);
            USB_send_str("Main TCP connection ok\r\n");
          }
          
          if (conn_ans == MC60_CON_ALREADY)
          {
            DC_setStatus(DC_FLAG_MAIN_TCP);
            USB_send_str("Already connected\r\n");
          }
          
          if (conn_ans == MC60_CON_FAIL)
          {
            if (MGT_setGPRS(0) == MC60_STD_OK) //deactivate GPRS
            {
              DC_resetStatus(DC_FLAG_GPRS_ACTIVE);
              DC_resetStatus(DC_WL_LOGIN_OK);
            }
            USB_send_str("Main TCP fail\r\n");
          }
        }
      }else{
        DC_setStatus(DC_FLAG_MAIN_TCP);
        USB_send_str("Already startes\r\n");
      }
    }else{
      DC_resetStatuses(DC_STATUS_MASK_GNSS_SAVE); //Reset statuses
    }
}
//--------------------------------------------------------------------------------------------------
//Send TCP str
uint8_t Main_send_TCP(uint8_t index, char *str)
{
  if ( MGT_sendTCP(index, str, strlen(str), &TCP_send_status) == MC60_STD_OK) // Send TCP/UDP package index,len
    if (TCP_send_status == MC60_SEND_OK)
    {
      USB_send_str("TCP message sended\r\n");
      return 1;
    }else{
      DC_resetStatus(DC_FLAG_MAIN_TCP);
      USB_send_str("Send fail\r\n");
      return 0;
    }
  return 0;
}
//--------------------------------------------------------------------------------------------------
//Send pack
uint8_t Main_send_pack(uint8_t index, char *pack, char* ansver, uint16_t wait_ms)
{
  TT_mes_type tcp_task_msg;
  
  if (Main_send_TCP(index, pack))//Send TCP str
  {
    //Wait message
    if ( xQueuePeek( Main_con_Queue, &tcp_task_msg, wait_ms ) == pdTRUE )
    {
      //If from Modem gate task
      if (tcp_task_msg.task == TT_MGT_TASK)
        xQueueReceive( Main_con_Queue, &tcp_task_msg, 0 );
        if (tcp_task_msg.type == EVENT_TCP_MSG) //If event message from TCP
          if (((MC60_TCP_recive_t*)(tcp_task_msg.message))->index == index)
          {
            memcpy(ansver, ((MC60_TCP_recive_t*)(tcp_task_msg.message))->data, ((MC60_TCP_recive_t*)(tcp_task_msg.message))->len);
            USB_send_str("Wialon recive pack\r\n");
            return 1;
          }else if(((MC60_TCP_recive_t*)(tcp_task_msg.message))->index == 0)
          {
            //Recived other data
            return 0;
          }
    }
  }
  
  return 0;
}
//--------------------------------------------------------------------------------------------------
//Wakeup
void Main_Wakeup(uint8_t wakeup_mode)
{    
  DC_resetStatus(DC_FLAG_SLEEP_STAT);

  CMU_ClockEnable(cmuClock_USART0, true);
  NVIC_EnableIRQ(USART1_RX_IRQn);
  NVIC_EnableIRQ(USART1_TX_IRQn);
  //DC_switch_power(DC_DEV_MUX,1);
  
  if (wakeup_mode == WAKEUP_MODE_GNSS_ACTIVE)
  {
    DC_switch_power(DC_DEV_MUX,0);
    DC_switch_power(DC_DEV_GNSS, 1);
    DC_switch_power(DC_DEV_GSM, 1);
    vTaskDelay(50);
    vTaskResume(xHandle_Monitor);
  }

  if (wakeup_mode == WAKEUP_MODE_GSM_ACTIVE)
  {
    DC_switch_power(DC_DEV_MUX,1);
    DC_switch_power(DC_DEV_GNSS, 0);
    DC_switch_power(DC_DEV_GSM, 0);
    vTaskDelay(100);
  }

  if (wakeup_mode == WAKEUP_MODE_MODEM_ACTIVE)
  {
    DC_switch_power(DC_DEV_MUX,1);
    DC_switch_power(DC_DEV_GNSS, 1);
    vTaskDelay(50);
    DC_switch_power(DC_DEV_GSM, 0);
    vTaskDelay(100);
    vTaskResume(xHandle_Monitor);
  } 
  
  USBD_Disconnect();
  vTaskDelay( 500 );
  USBD_Connect();
  
}
//--------------------------------------------------------------------------------------------------
//Sleep
void Main_sleep(uint8_t sleep_mode, uint32_t *sec)
{    
  //ALL off // wake up: accelerometr
  if (sleep_mode == SLEEP_MODE_FULL)
  {
    USB_send_str("Sleep mode full\r\n");
    DC_switch_power(DC_DEV_GNSS, 0);
    DC_switch_power(DC_DEV_GSM, 1);
    DC_switch_power(DC_DEV_MUX,0);
    DC_resetStatuses(DC_STATUS_NOT_SAVE); //Reset statuses
  }
  
  //GNSS on // wake up: accelerometr, RTC
  if (sleep_mode == SLEEP_MODE_GNSS_ACTIVE)
  {
    USB_send_str("Sleep mode GNSS on\r\n");
    DC_switch_power(DC_DEV_GNSS, 1);
    DC_switch_power(DC_DEV_GSM, 1);
    DC_switch_power(DC_DEV_MUX,0);
    DC_resetStatuses(DC_STATUS_MASK_GNSS_SAVE); //Reset statuses
    
    DC_set_RTC_timer_s(*sec); //Set RTC timer
  }  
  
  // GSM on // wake up: accelerometr, RTC, CALL
  if (sleep_mode == SLEEP_MODE_GSM_ACTIVE)
  {
    USB_send_str("Sleep mode GSM on\r\n");
    DC_switch_power(DC_DEV_GNSS, 0);
    DC_switch_power(DC_DEV_GSM, 0);
    DC_switch_power(DC_DEV_MUX,0);
    DC_resetStatuses(DC_STATUS_NOT_SAVE); //Reset statuses
    
    DC_set_RTC_timer_s(*sec); //Set RTC timer
  }    
    
  //GNSS, GSM on // wake up: accelerometr, RTC, CALL
  if (sleep_mode == SLEEP_MODE_MODEM_ACTIVE)
  {
    USB_send_str("Sleep mode GNSS, GSM on\r\n");
    DC_switch_power(DC_DEV_GNSS, 1);
    DC_switch_power(DC_DEV_GSM, 0);
    DC_switch_power(DC_DEV_MUX, 0);
    //DC_resetStatuses(DC_STATUS_MASK_GNSS_GSM_SAVE); //Reset statuses
    
    DC_set_RTC_timer_s(*sec); //Set RTC timer
  }
  
  CMU_ClockEnable(cmuClock_USART0, false);
  NVIC_DisableIRQ(USART0_IRQn);
  MGT_reset(); //Reset task
  
  DC_setStatus(DC_FLAG_SLEEP_STAT);
  
 // USBD_Disconnect();
  
  EMU_EnterEM2(true);
  
//  if ( USBD_SafeToEnterEM2() )
//    EMU_EnterEM2(true);
//  else
//    EMU_EnterEM1(); 
}
//--------------------------------------------------------------------------------------------------
//Every time check
uint8_t Main_check_process()
{
  uint8_t try_counter = 7;
  
  while(try_counter--)
  {
    if(!DC_getStatus(DC_FLAG_AT_OK))
      return 0;
    if (MGT_getQuality(&Main_quality, &exec_status) == MC60_STD_OK) //Get quality
    {
      //If low signal
      if (Main_quality < 5)
        DC_resetStatus(DC_FLAG_REG_OK);
      
      sprintf(str_buf, "Quality: %d\r\n", Main_quality);
      USB_send_str(str_buf);
      printf(str_buf);
      return 1;
    }else{
      if (exec_status == EXEC_WAIT_ER)
      {
        printf("Modem not ansver\r\n");
        USB_send_str("Modem not ansver\r\n");
      }
    }
    vTaskDelay(1000);
  }
  
  return 0;
}
//--------------------------------------------------------------------------------------------------
//Prepare cell loc
void Main_prepare_cell_loc()
{
  GNSS_data.lat1 = GNSS_convertTo_HHMMSS(cell_lat);
  if (cell_lat>0)
    GNSS_data.lat2 = 'N';
  else
    GNSS_data.lat2 = 'S';
  GNSS_data.lon1 = GNSS_convertTo_HHMMSS(cell_lon);
  if (cell_lon>0)
    GNSS_data.lon2 = 'E';
  else
    GNSS_data.lon2 = 'W';
  
  GNSS_data.sats = 0;
}
//**************************************************************************************************
//Wialon TASK
void vMain_Task(void *pvParameters) {

  vTaskSuspend( xHandle_MGT );
  MGT_init(&Main_con_Queue); //Init modem gate
  vTaskResume( xHandle_MGT );
  
  vTaskDelay(1000);
  printf("\r\nSTART\r\n");
  USB_send_str("\r\nSTART\r\n");
  
  //Start power timer
  Timer_sample = xTimerCreate("Tsample", 1, pdTRUE, ( void * )0, vTimer_sample);
  xTimerStart(Timer_sample, 0);
  
  Main_Wakeup(WAKEUP_MODE_MODEM_ACTIVE);//Wakeup
  
  while(1)
  {     
    
  change_mode:
    
    //********************************************************************************
    //New Firmware
    if (DC_mode == DC_MODE_FIRMWARE)
    {
      
      while(1)
      {
        //Service conection
        Main_service_connect();
        
        //If service ok
        if (DC_getStatus(DC_FLAG_SERVICE_TCP) && !DC_getStatus(DC_FLAG_SERVICE_FW_STAT_OK))
        {
          if (Main_service_send_fw_status_req(&DC_fw_image) != -1) //Request fw status
            DC_setStatus(DC_FLAG_SERVICE_FW_STAT_OK);
          
          //If fw status ok
          if (DC_getStatus(DC_FLAG_SERVICE_FW_STAT_OK))
            if (DC_fw_image.imageVersion_new > DC_fw_image.imageVersion) //Check version
            {
              EXT_Flash_erace_block(EXT_FLASH_PROGRAMM_IMAGE);
              DC_setStatus(DC_FLAG_SERVICE_GET_FW);
            }
        }
        
        //Getting firmware
        if (DC_getStatus(DC_FLAG_SERVICE_GET_FW))
        {
          //Check pos
          if (DC_fw_image.imageLoaded < DC_fw_image.imageSize)
          {
            int pack_len = 0;
            if (Main_service_send_fw_pack_req(str_buf, &pack_len) != -1) //Request fw pack
            {
              DC_save_pack(DC_fw_image.imageLoaded, str_buf);
              DC_fw_image.imageLoaded += pack_len;
              DC_save_FW();
              MGT_gsm_try_count = 0;
            }
          }else{
            DC_fw_image.statusCode = COMMAND_TO_REPROGRAMM;//Firmware loaded
            DC_reset_system(); //System reset
          }
        }
        
        if (DC_getStatus(DC_FLAG_SERVICE_GET_FW))
        {
          if (MGT_gsm_try_count >= 200)
            DC_reset_system(); //System reset
          else
            MGT_gsm_try_count++;
          
          vTaskDelay(100);
        }else{
          
          if (MGT_gsm_try_count >= DC_settings.DC_SET_gsm_try_count_s)
            DC_reset_system(); //System reset
          else
            MGT_gsm_try_count++;
          
          vTaskDelay(1000);
        }
          
        WDOG_Feed(); //Watch dog;
      }
    }
    
    //********************************************************************************
    //New settings
    if (DC_mode == DC_MODE_NEW_SETTING)
    {
      while(1)
      {
        //Service conection
        Main_service_connect();
        
        //If service ok
        if (DC_getStatus(DC_FLAG_SERVICE_TCP))
        {
          if (Main_service_send_settings_req() != -1) //Request fw status
          {
            DC_save_settings(); //Save settings
            DC_reset_system(); //System reset
          }
        }
        
        if (MGT_gsm_try_count >= DC_settings.DC_SET_gsm_try_count_s)
          DC_reset_system(); //System reset
        else
          MGT_gsm_try_count++;

        WDOG_Feed(); //Watch dog;
        vTaskDelay(1000);
      }
    }
    
    //********************************************************************************
    //Collected sensors
    if (DC_mode == DC_MODE_COLLECT_SENSOR)
    {
      if (DC_getStatus(DC_FLAG_SENSORS_GETTED))
        goto main_sleep;      
    }
    
    //********************************************************************************
    //Send data
    if ((DC_mode == DC_MODE_SEND_DATA)||(DC_mode == DC_MODE_COLLECT_GNSS_AND_SEND))
    {            
      MGT_gsm_try_count = 0; //Try Count
      MGT_service_period_s = 0; //Service period
      
      //if gnss collect data
      if (DC_mode == DC_MODE_COLLECT_GNSS_AND_SEND)
        vTaskResume(xHandle_Monitor); //Wakeup monitor task
      
      //While status Main_PACK_SENED
      while (!DC_getStatus(DC_WL_PACK_SENED))
      {  
        
        //Try on modem
        if(!Main_modem_on())
        {
          printf("Can't switch on modem\r\n");
          USB_send_str("Can't switch on modem\r\n");
          DC_reset_system(); //System reset
        }
        
        //Check try count
        if (MGT_gsm_try_count >= DC_settings.DC_SET_gsm_try_count_s)
        {
          printf("Try count except\r\n");
          USB_send_str("Try count except\r\n");
          
          //If registration in gsm ok
          if (DC_getStatus(DC_FLAG_REG_OK))
          {
            goto main_sleep;
          }else{
            //If can't register in gsm
            DC_switch_power(DC_DEV_GSM, 1); //Switch off modem
            DC_resetStatuses(DC_STATUS_MASK_GNSS_SAVE); //Reset statuses
            MGT_gsm_try_count = 0; //Try Count
            MGT_service_period_s = 0; //Service period
            goto main_sleep;
          }          
          
        }else if(DC_getStatus(DC_FLAG_GNSS_RECEIVED)|| DC_getStatus(DC_FLAG_GNSS_NOT_RECEIVED)) //Inc counter if GNSS recived
        {
          MGT_gsm_try_count++;
        }     
        
        //Get IMEI
        if (!DC_getStatus(DC_FLAG_IMEI_OK))
          if ( MGT_getIMEI(IMEI) == MC60_STD_OK ) //Get IMEI
            if (MC60_check_IMEI(IMEI))  //Check IMEI
            {
              sprintf(str_buf, "IMEI: %s\r\n", IMEI);
              USB_send_str(str_buf);
              printf(str_buf);
              DC_setStatus(DC_FLAG_IMEI_OK);
            }
        
        //Connect to GPRS
        if (Main_GPRS_connect() == 0)
        {
          USB_send_str("Wait operator\r\n");
          printf("Wait operator\r\n");
          goto main_sleep;
        }
        
//        //Check service period
//        if (MGT_service_period_s >= 10)
//        {
//          //Service conection
//          Main_service_connect();
//          
//          //If service ok
//          if (DC_getStatus(DC_FLAG_SERVICE_TCP))
//          {
//            //Request status
//            server_status = Main_service_send_status_req();
//            
//            if (server_status == -1)
//            {
//              USB_send_str("Service -1\r\n");
//            }else{
//              USB_send_str("Service ok\r\n");
//              
//              //New firmware
//              if(((server_status >> SC_SERVER_NEW_FM)&0x01) == 1) { vTaskSuspend( xHandle_Monitor ); DC_mode =  DC_MODE_FIRMWARE; MGT_gsm_try_count = 0; goto change_mode;}
//              //New settings
//              if(((server_status >> SC_SERVER_NEW_SETTINGS)&0x01) == 1) { vTaskSuspend( xHandle_Monitor ); DC_mode =  DC_MODE_NEW_SETTING; goto change_mode;}
//              
//            }
//          }
//          MGT_service_period_s = 0;
//        }else{
//          MGT_service_period_s++;
//        }
        
        //If GNSS OK
        if (DC_getStatus(DC_FLAG_GNSS_RECEIVED) || DC_getStatus(DC_FLAG_GNSS_NOT_RECEIVED))
          Main_main_connect();//Main TCP connection
        
        //If main connection
        if (DC_getStatus(DC_FLAG_MAIN_TCP) && (DC_getStatus(DC_FLAG_GNSS_RECEIVED) || DC_getStatus(DC_FLAG_GNSS_NOT_RECEIVED)))
        {
          if (!DC_getStatus(DC_WL_LOGIN_OK) && (DC_getStatus(DC_FLAG_IMEI_OK)))
            if (Main_login(IMEI, DC_settings.DC_SET_wialon_pass) == WL_ANS_SUCCESS) //Make login
            {
              printf("Login OK\r\n");
              USB_send_str("Login OK\r\n");
              DC_setStatus(DC_WL_LOGIN_OK);
            }else{
              printf("Login fail\r\n");
              USB_send_str("Login fail\r\n");
              DC_resetStatus(DC_FLAG_MAIN_TCP);
            }
          
          //Send data id GNSS recived data
          if(DC_getStatus(DC_WL_LOGIN_OK))
          {
            if (DC_getStatus(DC_FLAG_GNSS_NOT_RECEIVED))
            {
              if(!DC_getStatus(DC_FLAG_GNSS_TYME_SYNCH))//It not synch
              {
                //Try get cell time
                if(MGT_get_time_network(&GNSS_data.time, &GNSS_data.date) == MC60_STD_OK)
                {
                  //RTC synch
                  DC_set_date_time(GNSS_data.date, GNSS_data.time);
                  GNSS_data.time_source = 0; //Cell
                  
                  //Check coordinate 
                  if (!((GNSS_data.lat1 > 0)&&(GNSS_data.lon1 > 0)))
                  {
                    if (MT_get_celloc())//Get celloc
                      DC_setStatus(DC_FLAG_CELL_LOC_OK);
                    else
                      DC_resetStatus(DC_FLAG_CELL_LOC_OK);
                  }
                  Main_prepare_cell_loc();
                  GNSS_data.speed = 0;
                  GNSS_data.cource = 0;
                  
                }else{ //If can't get cell time
                  
                  //Check coordinate 
                  if (!((GNSS_data.lat1 > 0)&&(GNSS_data.lon1 > 0)))
                  {
                    if (MT_get_celloc())//Get celloc
                      DC_setStatus(DC_FLAG_CELL_LOC_OK);
                    else
                      DC_resetStatus(DC_FLAG_CELL_LOC_OK);
                  }
                  Main_prepare_cell_loc();
                  DC_get_date_time(&GNSS_data.date, &GNSS_data.time);
                  GNSS_data.time_source = 2; //RTC
                  GNSS_data.speed = 0;
                  GNSS_data.cource = 0;
                }
                
              }else{ //If time from GNSS ok
                
                //Check coordinate 
                if (!((GNSS_data.lat1 > 0)&&(GNSS_data.lon1 > 0)))
                {
                  if (MT_get_celloc())//Get celloc
                    DC_setStatus(DC_FLAG_CELL_LOC_OK);
                  else
                    DC_resetStatus(DC_FLAG_CELL_LOC_OK);
                }
                Main_prepare_cell_loc();
                GNSS_data.time_source = 1; //Sat
                GNSS_data.speed = 0;
                GNSS_data.cource = 0;
              }
              
              GNSS_data.GNSS_source = 0; //Cell
            }else{
              GNSS_data.time_source = 1; //Sat
              GNSS_data.GNSS_source = 1; //GNSS
            }
            
            sprintf(str_buf,"Time source: %d Date soutce: %d\r\n",GNSS_data.time_source, GNSS_data.GNSS_source);
            printf(str_buf);
            
            if(Main_send_data(&GNSS_data, MT_temper_sensors, 2, MT_MCU_temper, (float)MT_BAT_voltage/1000, DC_getPower(), Main_quality, DC_mes_status) == WL_ANS_SUCCESS )//data
            {
              printf("Pack sended\r\n");
              USB_send_str("Pack sended\r\n");
              DC_setStatus(DC_WL_PACK_SENED);
              DC_resetStatus(DC_WL_LOGIN_OK);
              DC_resetStatus(DC_FLAG_SENSORS_GETTED);
              DC_resetStatus(DC_FLAG_GNSS_NOT_RECEIVED);
              goto main_sleep;
            }else{
              printf("Pack not sended\r\n");
              USB_send_str("Pack not sended\r\n");
              if (MGT_close_TCP_by_index(1) == MC60_STD_OK) // Close TCP
                DC_resetStatus(DC_FLAG_MAIN_TCP);
              DC_resetStatus(DC_WL_LOGIN_OK);
              DC_resetStatus(DC_FLAG_GNSS_NOT_RECEIVED);
              DC_resetStatus(DC_FLAG_CELL_LOC_OK);
              DC_resetStatus(DC_FLAG_GNSS_RECEIVED);
              vTaskResume(xHandle_Monitor); //Wakeup monitor task
            }

          }
        }
        
        //Recive message
        if ( xQueueReceive( Main_con_Queue, &task_msg, 1 ) == pdTRUE )
        {
          //Check disconnect event
          if ((task_msg.task == TT_MGT_TASK) && (task_msg.type == EVENT_TCP_CLOSE))
          {
            //Main Disconnect
            if ((*(uint8_t *)task_msg.message) == 1)
              DC_resetStatus(DC_FLAG_MAIN_TCP);
            //Service disconnect
            if ((*(uint8_t *)task_msg.message) == 0)
              DC_resetStatus(DC_FLAG_SERVICE_TCP);
          }
          //Check undervoltage event
          if ((task_msg.task == TT_MGT_TASK) && (task_msg.type == EVENT_UNDEVOLTAGE))
          {
            printf("Undervoltage\r\n");
            USB_send_str("Undervoltage\r\n");
            DC_reset_system(); //System reset
          }
        }  
        
        //Everytime make check
        if (!Main_check_process())
        {
          printf("Reset\r\n");
          USB_send_str("Reset\r\n");
          vTaskResume(xHandle_Monitor);
          DC_resetStatuses(DC_STATUS_MASK_GNSS_SAVE); //Reset statuses
        }
        
        vTaskDelay(1000);
      }
      
      //If pack sended
      if (DC_getStatus(DC_WL_PACK_SENED))
      {
        goto main_sleep;
        //Save partameters
      }

      vTaskDelay(1000);
    }
    
    //********************************************************************************
    //Sleep
    
main_sleep:
      if (MGT_close_TCP_by_index(1) == MC60_STD_OK) // Close TCP
        DC_resetStatus(DC_FLAG_MAIN_TCP);
      DC_mes_status = 0;
      WDOG_Feed(); //Watch dog
      printf("\r\nTrakcer sleep\r\n");
      USB_send_str("\r\nTrakcer sleep\r\n");
      Main_sleep(SLEEP_MODE_GNSS_ACTIVE, &DC_settings.DC_SET_data_send_period_s);
      Main_Wakeup(WAKEUP_MODE_MODEM_ACTIVE);//Wakeup
      
    vTaskDelay(1000);
  }
}
//**************************************************************************************************
//                                      Wialon functions
//**************************************************************************************************
//Login wialon
WL_ansvers_t Main_login(char *imei, char* pass)
{
  char ansver[20];
  WL_ansvers_t wialon_ansver = WL_ANS_UNKNOWN;
  
  WL_prepare_LOGIN(str_buf, imei, pass);
  Main_send_pack(1, str_buf, ansver, 8000);
  WL_parce_LOGIN_ANS(ansver, &wialon_ansver);
  
  return wialon_ansver;
}
//--------------------------------------------------------------------------------------------------
//Send short pack
WL_ansvers_t Main_send_short_data(GNSS_data_t *GNSS_data)
{ 
  char ansver[20];
  WL_ansvers_t wialon_ansver = WL_ANS_UNKNOWN;
  
  WL_prepare_SHORT_DATA(str_buf, GNSS_data);
  Main_send_pack(1, str_buf, ansver, 8000);
  WL_parce_SHORT_DATA_ANS(ansver, &wialon_ansver);
  
  return wialon_ansver;
}
//--------------------------------------------------------------------------------------------------
//Send data pack
WL_ansvers_t Main_send_data(GNSS_data_t *GNSS_data, float *temper_sensors, uint8_t count_temper, float temp_mcu, float bat_voltage, double power, uint8_t quality, uint8_t status)
{ 
  char ansver[20];
  WL_ansvers_t wialon_ansver = WL_ANS_UNKNOWN;

  WL_prepare_DATA(str_buf, GNSS_data, temper_sensors, count_temper, temp_mcu, bat_voltage, power, quality, status); //Create package DATA
  
  Main_send_pack(1, str_buf, ansver, 8000);
  WL_parce_DATA_ANS(ansver, &wialon_ansver);
  
  return wialon_ansver;
}
//**************************************************************************************************
//                                      Service functions
//**************************************************************************************************
//Request status
int Main_service_send_status_req()
{
  char ansver[30];
  int status;

  SC_prepare_status_req(str_buf, IMEI, (int*)&DC_fw_image.imageVersion_new);//Prepare status request

  if (Main_send_pack(0, str_buf, ansver, 10000) == 1)
    if (SC_parce_status_req(ansver, &status) == SC_ANS_OK)  //Parce status req
      return status;

  return -1;
}
//--------------------------------------------------------------------------------------------------
//Request fw status
int Main_service_send_fw_status_req(EXT_FLASH_image_t *image)
{
  char ansver[30];
  
  SC_prepare_fw_status_req(str_buf);//Prepare status request

  Main_send_pack(0, str_buf, ansver, 5000);
  if (SC_parce_fw_status_req(ansver, image) == SC_ANS_OK)  //Parce fw status
    return 1;

  return -1;
}
//--------------------------------------------------------------------------------------------------
//Request fw pack
int Main_service_send_fw_pack_req(char *pack, int *pack_len)
{
  char message[30];
  SC_prepare_fw_pack_req(message);//Prepare status request
  
  Main_send_pack(0, message, str_buf, 5000);
  if (SC_parce_fw_pack_req(str_buf, pack, pack_len) == SC_ANS_OK)  //Parce fw status
    return 1;
  
  return -1;
}
//--------------------------------------------------------------------------------------------------
//Request settings
int Main_service_send_settings_req()
{
  char message[30];
  SC_prepare_setting_req(message); //Prepare settings
  
  Main_send_pack(0, message, str_buf, 5000);
  if (SC_parce_settings_req(str_buf) == SC_ANS_OK)  //Parce fw status
  {
    memcpy(&DC_settings, str_buf, sizeof(DC_settings_t));
    return 1;
  }
  
  return -1;
}