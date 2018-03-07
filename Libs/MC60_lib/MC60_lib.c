
#include "MC60_lib.h"
#include "Modem_gate_task.h"
#include "Delay.h"
#include "em_gpio.h"
#include "Task_transfer.h"
#include "stdio.h"
#include "string.h"
#include "UART.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

//Cmd texts
const char cmd_text_NULL[]              = "";
const char cmd_text_AT[]                = "AT\r\n";
const char cmd_text_ATI[]               = "ATI\r\n";
const char cmd_text_CREG_Q[]            = "AT+CREG?\r\n";//Get registartion
const char cmd_text_CSQ_Q[]             = "AT+CSQ\r\n";  //Get quality
const char cmd_text_CGDCONT[]           = "AT+CGDCONT=1,\"IP\",\"%s\"\r\n"; //Set APN
const char cmd_text_CGACT[]             = "AT+CGACT=%s,1\r\n"; //Activate GPRS connection
const char cmd_text_CGACT_Q[]           = "AT+CGACT?\r\n"; //Check GPRS connection
const char cmd_text_CGPADDR[]           = "AT+CGPADDR=1\r\n"; //Get ip address
const char cmd_text_QIDNSIP[]           = "AT+QIDNSIP=%s\r\n"; //Set address type 0-IP 1-domen
const char cmd_text_QIOPEN[]            = "AT+QIOPEN=%s,\"%s\",\"%s\",\"%s\"\r\n"; //Open connection
const char cmd_text_QIMUX[]             = "AT+QIMUX=%s\r\n"; //Setting multiple TCP/IP
const char cmd_text_QISTAT_Q[]          = "AT+QISTAT\r\n"; //Get connection statuses
const char cmd_text_QISEND[]            = "AT+QISEND=%s,%s\r\n"; //Send TCP/UDP package index,len
const char cmd_text_GSN[]               = "AT+GSN\r\n"; //Request IMEI
const char cmd_text_QIHEAD[]            = "AT+QIHEAD=1\r\n"; //Set TCP head
const char cmd_text_QGNSSRD_RMC[]       = "AT+QGNSSRD=\"NMEA/RMC\"\r\n"; //Get GNSS RMC
const char cmd_text_QGNSSRD_GGA[]       = "AT+QGNSSRD=\"NMEA/GGA\"\r\n"; //Get GNSS GGA
const char cmd_text_QGNSSC[]            = "AT+QGNSSC=%s\r\n"; //Switch GNSS
const char cmd_text_QGNSSC_Q[]          = "AT+QGNSSC?\r\n"; //Query power GNSS
const char cmd_text_QGNSSTS_Q[]         = "AT+QGNSSTS?\r\n"; //Read time synchronization
const char cmd_text_QGREFLOC[]          = "AT+QGREFLOC=%s,%s\r\n"; // Set reference location information for QuecFastFix Online
const char cmd_text_QGNSSEPO[]          = "AT+QGNSSEPO=%s\r\n"; // Enable EPOTM function
const char cmd_text_QGEPOAID[]          = "AT+QGEPOAID\r\n"; // Trigget to EPO
const char cmd_text_QSCLK[]             = "AT+QSCLK=%s\r\n"; // Switch SLEEP mode
const char cmd_text_CFUN[]              = "AT+CFUN=%s\r\n"; // Switch FUN mode
const char cmd_text_QIREGAPP[]          = "AT+QIREGAPP=\"%s\",\"%s\",\"%s\"\r\n"; // Set APN, login, pass
const char cmd_text_QCELLLOC[]          = "AT+QCELLLOC=1\r\n"; // Get cell location
const char cmd_text_CCLK[]              = "AT+CCLK?\r\n"; // Get time
const char cmd_text_CTZU[]              = "AT+CTZU=%s\r\n"; //Network Time Synchronization and Update the RTC
const char cmd_text_QNITZ[]             = "AT+QNITZ=1\r\n"; // Set time synch
const char cmd_text_QICLOSE[]           = "AT+QICLOSE=%s\r\n"; //Close TCP
const char cmd_text_CUSD[]              = "AT+CUSD=1,\"%s\"\r\n"; //USSD
const char cmd_text_CSCS[]              = "AT+CSCS=\"%s\"\r\n"; //Set character set
const char cmd_text_MODECUSD[]          = "AT+CUSD=%s\r\n"; //USSD mode
const char cmd_text_CMGF[]              = "AT+CMGF=%s\r\n"; //SMS Message Format
const char cmd_text_CMGS[]              = "AT+CMGC=\"%s\"\r\n"; //Send SMS cmd

//Bluetooth cmds
const char cmd_text_QBTPWR[]            = "AT+QBTPWR=%s\r\n"; //Power BT device
const char cmd_text_QBTPWR_Q[]          = "AT+QBTPWR?\r\n";   //Check power BT device
const char cmd_text_QBTNAME[]           = "AT+QBTNAME=\"%s\"\r\n"; //Set BT name
const char cmd_text_QBTVISB[]           = "AT+QBTVISB=%s\r\n"; //Set visibility
const char cmd_text_QBTPAIRCNF[]        = "AT+QBTPAIRCNF=1,\"%s\"\r\n"; //Accept pairing
const char cmd_text_QSPPSEND[]          = "AT+QSPPSEND=%s,%s\r\n"; //Send BT
const char cmd_text_QBTCONN[]           = "AT+QBTCONN=%s,%s,%s\r\n"; //Connection BT (ID,PROFILE,MODE)

//Event text
const char event_text_NULL[]                    = " \r\n%n"; //NULL data
const char event_text_Recive_TCP[]              = " +RECEIVE: %d, %d%n%c"; //Recive TCP
const char event_text_Recive_TCP_data[]         = " IPD%%d:%%%ds%%n%%c"; //Recive TCP
const char event_text_RDY[]                     = " RDY%n"; //Ready
const char event_text_CLOSED_TCP[]              = " %d, CLOSED%n"; //Closed TCP
const char event_text_UNDER_VOLTAGE[]           = " UNDER_VOLTAGE WARNING%n"; //Voltage
const char event_text_Recive_BT_event[]         = " +QBTIND: \"%10[^\"]\",\"%20[^\"]\",%20[^,],%20[^\r]\r%n"; //BT pair
const char event_text_Recive_SMS[]              = " +CMTI: \"[^\"]\",%d%n"; //Recive SMS indicator

MC60_cmd_t MC60_current_cmd; //Curent exec cmd
extern QueueHandle_t MGT_con_Queue;
char MC60_buf[120];

//--------------------------------------------------------------------------------------------------
//Send CR
void MC60_sendCR()
{
  char cr = 0x0d;
  UART_SendData(&cr, 1);
}
//--------------------------------------------------------------------------------------------------
//Send str
void MC60_sendStr(char* str, uint16_t len)
{
  UART_SendData(str, len);
}
//--------------------------------------------------------------------------------------------------
//PrepareAndSendCmd
void MC60_PrepareAndSendCmd(MC60_str_query_t *data)
{
  MC60_current_cmd = data->cmd_id;
  
  if (MC60_current_cmd < MC60_CMD_COUNT)
  {
    sprintf(MC60_buf, MGT_cmd[MC60_current_cmd].str, data->str1, data->str2, data->str3, data->str4);
    
    UART_SendData(MC60_buf, strlen(MC60_buf));
  }
}
//--------------------------------------------------------------------------------------------------
//Check IMEI
uint8_t  MC60_check_IMEI(char *IMEI)
{
  const int m[] = {0,2,4,6,8,1,3,5,7,9}; // mapping for rule 3
  int i, odd = 1, sum = 0;
  
  if (strlen(IMEI) == 0)
    return 0;
  
  for (i = strlen(IMEI); i--; odd = !odd) {
    
    int digit = IMEI[i] - '0';
    sum += odd ? digit : m[digit];
  }
  
  return sum % 10 == 0;
}
//--------------------------------------------------------------------------------------------------
//Включить модем
void MC60_on_seq() {
  MC60_GSM_PWR_OFF;
  _delay_ms(200);
  MC60_GSM_PWR_ON;
  _delay_ms(1200);
  MC60_GSM_PWR_OFF;
}
//--------------------------------------------------------------------------------------------------
//Выключить модем
void MC60_off_seq() {
  MC60_GSM_PWR_ON;
  _delay_ms(100);
  MC60_GSM_PWR_OFF;
  _delay_ms(500);
  MC60_GSM_PWR_ON;
  _delay_ms(100);
}