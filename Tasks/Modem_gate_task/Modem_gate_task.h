
#ifndef MODEM_GATE_TASK
#define MODEM_GATE_TASK

#include "stdint.h"
#include "stdbool.h"
#include "Modem.h"
#include "Task_transfer.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "timers.h"

#define MGT_SCAN_MS             10      //Scan period
#define MGT_END_RCV_CH1         '\n'
#define MGT_END_RCV_CH2         0
#define MGT_INVITE_CH           '>'

//Parce status
typedef enum{
  MGT_PARCE_NULL = 0,
  MGT_PARCE_NEXT,
  MGT_PARCE_OK
}MGT_parce_t;

//Cmd and event
typedef struct{
  char* str;
  MGT_parce_t (*parce_func)();
  uint16_t wait_ms;
}MGT_cmd_event_t;

extern TaskHandle_t xHandle_MGT; //Task handle
extern const MGT_cmd_event_t MGT_cmd[Modem_CMD_COUNT]; //CMD list
extern uint16_t MGT_status; //Status byte

Modem_std_ans_t MGT_modem_check();//Check modem
Modem_std_ans_t MGT_getReg(Modem_CREG_Q_ans_t *reg); //Get regmodem
Modem_std_ans_t MGT_getQuality(uint8_t *level, TT_status_type* status); //Get quality
Modem_std_ans_t MGT_setAPN(char *APN, uint16_t* error_n); //setAPN
Modem_std_ans_t MGT_setGPRS(uint8_t active); //activate GPRS
Modem_std_ans_t MGT_getGPRS(uint8_t *status); //Get status GPRS
Modem_std_ans_t MGT_getIP(char *ip); //get IP
Modem_std_ans_t MGT_setAdrType(Modem_adr_type_t type); //Set address type 0-IP 1-domen
Modem_std_ans_t MGT_openConn(Modem_con_type_t *conn_ans, uint8_t index, Modem_ip_con_type_t con_type, char *domen_or_ip, uint16_t port); //make GPRS connection
Modem_std_ans_t MGT_setMUX_TCP(uint8_t mux); //Setting multiple TCP/IP
Modem_std_ans_t MGT_getConnectStat(uint8_t index, uint8_t *status); //Get connection statuses
Modem_std_ans_t MGT_sendTCP(uint8_t index, char *str, uint16_t len, Modem_TCP_send_t *TCP_send_status); // Send TCP/UDP package index,len
Modem_std_ans_t MGT_getIMEI(char* IMEI); //Request IMEI
Modem_std_ans_t MGT_set_qihead(); //Set qihead
Modem_std_ans_t MGT_get_GNSS(Modem_GNSS_data_type_t GNSS_data_type,char* GNSS_str); //Get GNSS data
Modem_std_ans_t MGT_switch_GNSS(uint8_t on, uint16_t *error_n); //Switch GNSS
Modem_std_ans_t MGT_getTimeSynch(uint8_t *synch_status); //Read time synchronization
Modem_std_ans_t MGT_set_ref_coord_GNSS(float lat, float lon); // Set reference location information for QuecFastFix Online
Modem_std_ans_t MGT_switch_EPO(uint8_t on , uint16_t *error_n); //Switch EPO
Modem_std_ans_t MGT_EPO_trig(); //Trigger EPO
Modem_std_ans_t MGT_switch_Sleep_mode(uint8_t mode); //Switch SLEEP
Modem_std_ans_t MGT_switch_FUN_mode(uint8_t mode); //Switch FUN mode
Modem_std_ans_t MGT_getPowerGNSS(uint8_t *on); //Get GNSS power
Modem_std_ans_t MGT_set_apn_login_pass(char* apn, char* login, char* pass); // Set APN, login, pass
Modem_std_ans_t MGT_get_cell_loc(float *lat, float *lon); // Get cell lock
Modem_std_ans_t MGT_setTimeSynchGSM_mode(uint8_t mode); //Network Time Synchronization and Update the RTC
Modem_std_ans_t MGT_get_time_network(float *time, float *date); // Get time synch
Modem_std_ans_t MGT_setGSM_time_synch(); // Set GSM time
Modem_std_ans_t MGT_close_TCP_by_index(uint8_t index); // Close TCP
Modem_std_ans_t MGT_setCharacterSet(char *characterSet); //Set charecter set

//**************************************************************************************************
//                                      USSD functions
//**************************************************************************************************

Modem_std_ans_t MGT_returnUSSD(char *USSD, char* returnStr); //Get USSD query
Modem_std_ans_t MGT_setUSSD_mode(char *mode); //Set USSD mode

//**************************************************************************************************
//                                      SMS functions
//**************************************************************************************************

Modem_std_ans_t MGT_setSMS_Format(char *format); //SMS Message Format 0 PDU mode 1 Text mode
Modem_std_ans_t MGT_sendSMS(char *phone, char* message); //Send SMS


//**************************************************************************************************
//                                      Bluetooth functions
//**************************************************************************************************
Modem_std_ans_t MGT_powerBT(uint8_t on); //Power on bluetooth
Modem_std_ans_t MGT_getStatusBT(uint8_t *status); //Get bluetooth power on status 
Modem_std_ans_t MGT_setNameBT(char *name); //Set BT name
Modem_std_ans_t MGT_setVisibilityBT(uint8_t on); //Set bluetooth visibility
Modem_std_ans_t MGT_acceptPairingBT(char* password, bool *pairStatus, uint8_t *pairID, char* pairName, uint16_t *ErrorCode); //Accept pairing
Modem_std_ans_t MGT_sendBT(uint8_t index, char *str, uint16_t len, Modem_BT_send_t *BT_send_status, uint16_t *ErrorCode); // Send TCP/UDP package index,len
Modem_std_ans_t MGT_connectBT(uint8_t id, Modem_BT_profile_t profile, Modem_BT_mode_t mode, uint16_t *ErrorCode); // Connect BT


void MGT_init(QueueHandle_t *MGT_event_queue_in); //Init
void MGT_switch_off_echo(); //Switch off ECHO
void MGT_switch_on_modem(); //Switch on modem
void MGT_reset(); //Reset task

#endif