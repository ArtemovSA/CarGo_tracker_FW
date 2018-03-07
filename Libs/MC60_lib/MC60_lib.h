#ifndef MC60_LIB
#define MC60_LIB

#include "stdint.h"
#include "InitDevice.h"

#define MC60_GSM_EN_PORT V_GSM_EN_PORT
#define MC60_GSM_EN_PIN V_GSM_EN_PIN

#define MC60_GSM_EN_ON GPIO_PinOutSet(MC60_GSM_EN_PORT, MC60_GSM_EN_PIN)
#define MC60_GSM_EN_OFF GPIO_PinOutClear(MC60_GSM_EN_PORT, MC60_GSM_EN_PIN)

#define MC60_GSM_PWR_PORT GSM_PWR_KEY_PORT
#define MC60_GSM_PWR_PIN GSM_PWR_KEY_PIN

#define MC60_GSM_PWR_ON GPIO_PinOutSet(MC60_GSM_PWR_PORT, MC60_GSM_PWR_PIN)
#define MC60_GSM_PWR_OFF GPIO_PinOutClear(MC60_GSM_PWR_PORT, MC60_GSM_PWR_PIN)

//*****************************************APN settings*********************************************
#define MC60_DEFAUL_APN              "internet"
#define MC60_DEFAUL_LOGIN            "gdata"
#define MC60_DEFAUL_PASS             "gdata"

//*****************************************Return statuses******************************************
//Std ansvers
typedef enum {
  MC60_STD_NULL = 0,
  MC60_STD_OK,
  MC60_STD_ERROR
} MC60_std_ans_t;

//Return
typedef enum {
  MC60_ERROR_ANS = 0x00,
  MC60_TIMEOUT,
  MC60_OK
} MC60_ans_t;

//CREG_Q ansvers
typedef enum {
  MC60_CREG_Q_NOT_REG = 0,
  MC60_CREG_Q_REGISTERED,
  MC60_CREG_Q_SEARCH,
  MC60_CREG_Q_DENIED,
  MC60_CREG_Q_UNKNOWN,
  MC60_CREG_Q_ROAMING
} MC60_CREG_Q_ans_t;


//*****************************************Commands*************************************************
//ID command
typedef enum {
  MC60_CMD_NULL = 0,
  MC60_CMD_AT,
  MC60_CMD_ATI,
  MC60_CMD_CREG_Q,
  MC60_CMD_CSQ_Q,
  MC60_CMD_CGDCONT,
  MC60_CMD_CGACT,
  MC60_CMD_CGACT_Q,
  MC60_CMD_CGPADDR,
  MC60_CMD_QIDNSIP,
  MC60_CMD_QIOPEN,
  MC60_CMD_QIMUX,
  MC60_CMD_QISTAT_Q,
  MC60_CMD_QISEND,
  MC60_CMD_GSN,
  MC60_CMD_QIHEAD,
  MC60_CMD_QGNSSRD_RMC,
  MC60_CMD_QGNSSRD_GGA,
  MC60_CMD_QGNSSC,
  MC60_CMD_QGNSSC_Q,
  MC60_CMD_QGNSSTS_Q,
  MC60_CMD_QGREFLOC,
  MC60_CMD_QGNSSEPO,
  MC60_CMD_QGEPOAID,
  MC60_CMD_QSCLK,
  MC60_CMD_CFUN,
  MC60_CMD_QIREGAPP,
  MC60_CMD_QCELLLOC,
  MC60_CMD_CTZU,
  MC60_CMD_CCLK,
  MC60_CMD_QNITZ,
  MC60_CMD_QICLOSE,
  MC60_CMD_CUSD,
  MC60_CMD_CSCS,
  MC60_CMD_MODECUSD,
  MC60_CMD_CMGF,
  MC60_CMD_CMGS,
  
  //Bluetooth
  MC60_CMD_QBTPWR,
  MC60_CMD_QBTPWR_Q,
  MC60_CMD_QBTNAME,
  MC60_CMD_QBTVISB,
  MC60_CMD_QBTPAIRCNF,
  MC60_CMD_QSPPSEND,
  MC60_CMD_QBTCONN,

  MC60_CMD_COUNT
} MC60_cmd_t;


//Cmd texts
extern const char cmd_text_NULL[];
extern const char cmd_text_AT[];
extern const char cmd_text_ATI[];
extern const char cmd_text_CREG_Q[];
extern const char cmd_text_CSQ_Q[];
extern const char cmd_text_CGDCONT[];
extern const char cmd_text_CGACT[];
extern const char cmd_text_CGACT_Q[];
extern const char cmd_text_CGPADDR[];
extern const char cmd_text_QIDNSIP[];
extern const char cmd_text_QIOPEN[];
extern const char cmd_text_QIMUX[];
extern const char cmd_text_QISTAT_Q[];
extern const char cmd_text_QISEND[];
extern const char cmd_text_GSN[];
extern const char cmd_text_QIHEAD[];
extern const char cmd_text_QGNSSRD_RMC[];
extern const char cmd_text_QGNSSRD_GGA[];
extern const char cmd_text_QGNSSC[];
extern const char cmd_text_QGNSSC_Q[];
extern const char cmd_text_QGNSSTS_Q[];
extern const char cmd_text_QGREFLOC[];
extern const char cmd_text_QGNSSEPO[];
extern const char cmd_text_QGEPOAID[];
extern const char cmd_text_QSCLK[];
extern const char cmd_text_CFUN[];
extern const char cmd_text_QIREGAPP[];
extern const char cmd_text_QCELLLOC[];
extern const char cmd_text_CTZU[];
extern const char cmd_text_CCLK[];
extern const char cmd_text_QNITZ[];
extern const char cmd_text_QICLOSE[];
extern const char cmd_text_QBTPWR[];
extern const char cmd_text_QBTPWR_Q[];
extern const char cmd_text_QBTNAME[];
extern const char cmd_text_QBTVISB[];
extern const char cmd_text_QBTPAIRCNF[];
extern const char cmd_text_QSPPSEND[];
extern const char cmd_text_QBTCONN[];
extern const char cmd_text_CUSD[];
extern const char cmd_text_CSCS[];
extern const char cmd_text_MODECUSD[];
extern const char cmd_text_CMGF[];
extern const char cmd_text_CMGS[];

//*****************************************Events***************************************************
//Events
typedef enum{
  MC60_EVENT_NULL = 0,
  MC60_EVENT_RECIVE,
  MC60_EVENT_RDY,
  MC60_EVENT_TCP_CLOSE,
  MC60_EVENT_UNDER_VOLTAGE,
  MC60_EVENT_BT_EVENT,
  MC60_EVENT_SMS_RECIVE,
  MC60_EVENT_COUNT
}MC60_event_t;

//Events texts
extern const char event_text_NULL[]; //NULL data
extern const char event_text_Recive_TCP[]; //Recive TCP
extern const char event_text_Recive_TCP_data[]; //Recive TCP
extern const char event_text_RDY[]; //Ready
extern const char event_text_CLOSED_TCP[]; //Closed TCP
extern const char event_text_UNDER_VOLTAGE[]; //Voltage
extern const char event_text_Recive_BT_event[]; //BT pair
extern const char event_text_Recive_SMS[]; //Recive SMS indicator

//**************************************************************************************************
//Connection type
typedef enum {
  MC60_CON_FAIL = 0,
  MC60_CON_OK,
  MC60_CON_ALREADY
} MC60_con_type_t;

//IP conn type
typedef enum {
  MC60_CON_TCP = 0,
  MC60_CON_UDP
} MC60_ip_con_type_t;

//TCP send type
typedef enum {
  MC60_SEND_FAIL = 0,
  MC60_CONNECT_ERR,
  MC60_SEND_OK
} MC60_TCP_send_t;

//BT send type
typedef enum {
  MC60_BT_SEND_FAIL = 0,
  MC60_BT_CONNECT_ERR,
  MC60_BT_SEND_OK
} MC60_BT_send_t;

//Adr type
typedef enum {
  MC60_ADR_TYPE_IP = 0,
  MC60_ADR_TYPE_DOMEN
} MC60_adr_type_t;

//One str param
typedef struct{
  MC60_cmd_t cmd_id;
  char *str1;
  char *str2;
  char *str3;
  char *str4;
  void *ans1;
  void *ans2;
  void *ans3;
  void *ans4;
  MC60_std_ans_t *std_ans;
} MC60_str_query_t;

//Recive TCP message
typedef struct {
  uint8_t index;
  uint16_t len;
  char *data;
} MC60_TCP_recive_t;

//GNSS data type
typedef enum{
  RMC = 0,
  GGA
}MC60_GNSS_data_type_t;

//BT event type
typedef enum{
  BT_NONE_TYPE = 0,
  BT_PAIR,
  BT_CONN,
  BT_DISCONN
}MC60_BT_event_type_t;

//BT event
typedef struct {
  MC60_BT_event_type_t type;
  char name[20];
  uint32_t numericcompare;
  uint64_t addr;
  uint8_t id;
}MC60_BT_event_t;

//SMS event
typedef struct {
  char SMS_event_place[5];
  uint16_t SMS_event_count;
}MC60_SMS_event_t;

//BT modes
typedef enum{
  BT_MODE_AT = 0,
  BT_MODE_BUF_ACCESS,
  BT_MODE_TRANSPARENT
}MC60_BT_mode_t;

//BT profiles
typedef enum{
  BT_SPP_PROFILE = 0,
  BT_HF_PROFILE = 5,
  BT_GHF_PROFILE
}MC60_BT_profile_t;

//*****************************************Functions************************************************
extern MC60_cmd_t MC60_current_cmd; //Curent exec cmd

void MC60_on_seq(); //¬ключить модем
uint8_t MC60_check_IMEI(char *IMEI); //Check IMEI
void MC60_sendCR(); //Send CR

//*****************************************Task Functions*******************************************
//Task functions
void MC60_PrepareAndSendCmd(MC60_str_query_t *data); //PrepareAndSendCmd
void MC60_sendStr(char* str, uint16_t len); //Send str

#endif