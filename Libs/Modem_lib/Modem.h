#ifndef Modem_LIB
#define Modem_LIB

#include "stdint.h"
#include "InitDevice.h"

#define Modem_GSM_EN_PORT V_GSM_EN_PORT
#define Modem_GSM_EN_PIN V_GSM_EN_PIN

#define Modem_GSM_EN_ON GPIO_PinOutSet(Modem_GSM_EN_PORT, Modem_GSM_EN_PIN)
#define Modem_GSM_EN_OFF GPIO_PinOutClear(Modem_GSM_EN_PORT, Modem_GSM_EN_PIN)

#define Modem_GSM_PWR_PORT GSM_PWR_KEY_PORT
#define Modem_GSM_PWR_PIN GSM_PWR_KEY_PIN

#define Modem_GSM_PWR_ON GPIO_PinOutSet(Modem_GSM_PWR_PORT, Modem_GSM_PWR_PIN)
#define Modem_GSM_PWR_OFF GPIO_PinOutClear(Modem_GSM_PWR_PORT, Modem_GSM_PWR_PIN)

//*****************************************APN settings*********************************************

#define Modem_DEFAUL_APN              "internet"
#define Modem_DEFAUL_LOGIN            "gdata"
#define Modem_DEFAUL_PASS             "gdata"

//*****************************************Return statuses******************************************
//Std ansvers
typedef enum {
  Modem_STD_NULL = 0,
  Modem_STD_OK,
  Modem_STD_ERROR
} Modem_std_ans_t;

//Return
typedef enum {
  Modem_ERROR_ANS = 0x00,
  Modem_TIMEOUT,
  Modem_OK
} Modem_ans_t;

//CREG_Q ansvers
typedef enum {
  Modem_CREG_Q_NOT_REG = 0,
  Modem_CREG_Q_REGISTERED,
  Modem_CREG_Q_SEARCH,
  Modem_CREG_Q_DENIED,
  Modem_CREG_Q_UNKNOWN,
  Modem_CREG_Q_ROAMING
} Modem_CREG_Q_ans_t;


//*****************************************Commands*************************************************
//ID command
typedef enum {
  Modem_CMD_NULL = 0,
  Modem_CMD_AT,
  Modem_CMD_ATI,
  Modem_CMD_CREG_Q,
  Modem_CMD_CSQ_Q,
  Modem_CMD_CGDCONT,
  Modem_CMD_CGACT,
  Modem_CMD_CGACT_Q,
  Modem_CMD_CGPADDR,
  Modem_CMD_QIDNSIP,
  Modem_CMD_QIOPEN,
  Modem_CMD_QIMUX,
  Modem_CMD_QISTAT_Q,
  Modem_CMD_QISEND,
  Modem_CMD_GSN,
  Modem_CMD_QIHEAD,
  Modem_CMD_QGNSSRD_RMC,
  Modem_CMD_QGNSSRD_GGA,
  Modem_CMD_QGNSSC,
  Modem_CMD_QGNSSC_Q,
  Modem_CMD_QGNSSTS_Q,
  Modem_CMD_QGREFLOC,
  Modem_CMD_QGNSSEPO,
  Modem_CMD_QGEPOAID,
  Modem_CMD_QSCLK,
  Modem_CMD_CFUN,
  Modem_CMD_QIREGAPP,
  Modem_CMD_QCELLLOC,
  Modem_CMD_CTZU,
  Modem_CMD_CCLK,
  Modem_CMD_QNITZ,
  Modem_CMD_QICLOSE,
  Modem_CMD_CUSD,
  Modem_CMD_CSCS,
  Modem_CMD_MODECUSD,
  Modem_CMD_CMGF,
  Modem_CMD_CMGS,
  
  //Bluetooth
  Modem_CMD_QBTPWR,
  Modem_CMD_QBTPWR_Q,
  Modem_CMD_QBTNAME,
  Modem_CMD_QBTVISB,
  Modem_CMD_QBTPAIRCNF,
  Modem_CMD_QSPPSEND,
  Modem_CMD_QBTCONN,

  Modem_CMD_COUNT
} Modem_cmd_t;


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
  Modem_EVENT_NULL = 0,
  Modem_EVENT_RECIVE,
  Modem_EVENT_RDY,
  Modem_EVENT_TCP_CLOSE,
  Modem_EVENT_UNDER_VOLTAGE,
  Modem_EVENT_BT_EVENT,
  Modem_EVENT_SMS_RECIVE,
  Modem_EVENT_COUNT
}Modem_event_t;

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
  Modem_CON_FAIL = 0,
  Modem_CON_OK,
  Modem_CON_ALREADY
} Modem_con_type_t;

//IP conn type
typedef enum {
  Modem_CON_TCP = 0,
  Modem_CON_UDP
} Modem_ip_con_type_t;

//TCP send type
typedef enum {
  Modem_SEND_FAIL = 0,
  Modem_CONNECT_ERR,
  Modem_SEND_OK
} Modem_TCP_send_t;

//BT send type
typedef enum {
  Modem_BT_SEND_FAIL = 0,
  Modem_BT_CONNECT_ERR,
  Modem_BT_SEND_OK
} Modem_BT_send_t;

//Adr type
typedef enum {
  Modem_ADR_TYPE_IP = 0,
  Modem_ADR_TYPE_DOMEN
} Modem_adr_type_t;

//One str param
typedef struct{
  uint8_t cmd_id;
  char *cmd_str;
  char *str1;
  char *str2;
  char *str3;
  char *str4;
  void *ans1;
  void *ans2;
  void *ans3;
  void *ans4;
  Modem_std_ans_t *std_ans;
} Modem_str_query_t;

//Recive TCP message
typedef struct {
  uint8_t index;
  uint16_t len;
  char *data;
} Modem_TCP_recive_t;

//GNSS data type
typedef enum{
  RMC = 0,
  GGA
}Modem_GNSS_data_type_t;

//BT event type
typedef enum{
  BT_NONE_TYPE = 0,
  BT_PAIR,
  BT_CONN,
  BT_DISCONN
}Modem_BT_event_type_t;

//BT event
typedef struct {
  Modem_BT_event_type_t type;
  char name[20];
  uint32_t numericcompare;
  uint64_t addr;
  uint8_t id;
}Modem_BT_event_t;

//SMS event
typedef struct {
  char SMS_event_place[5];
  uint16_t SMS_event_count;
}Modem_SMS_event_t;

//BT modes
typedef enum{
  BT_MODE_AT = 0,
  BT_MODE_BUF_ACCESS,
  BT_MODE_TRANSPARENT
}Modem_BT_mode_t;

//BT profiles
typedef enum{
  BT_SPP_PROFILE = 0,
  BT_HF_PROFILE = 5,
  BT_GHF_PROFILE
}Modem_BT_profile_t;

//*****************************************Functions************************************************
extern Modem_cmd_t Modem_current_cmd; //Curent exec cmd

void Modem_on_seq(); //¬ключить модем
uint8_t Modem_check_IMEI(char *IMEI); //Check IMEI
void Modem_sendCR(); //Send CR

//*****************************************Task Functions*******************************************
//Task functions
void Modem_PrepareAndSendCmd(Modem_str_query_t *data); //PrepareAndSendCmd
void Modem_sendStr(char* str, uint16_t len); //Send str

#endif