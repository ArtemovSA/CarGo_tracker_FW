
#ifndef DEVICE_CTRL_H
#define DEVICE_CTRL_H

#include "stdint.h"
#include "stdbool.h"
#include "InitDevice.h"
#include "EXT_flash.h"
#include "GNSS.h"
#include "time.h"

//RTOS
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"


//**********************************GPIO************************************************************

#define DC_DEV_1WIRE_PIN 0
#define DC_DEV_RS485_PIN 1
#define DC_DEV_FLASH_PIN 2

#define DC_DEV_GNSS_PORT V_GNSS_EN_PORT
#define DC_DEV_GNSS_PIN V_GNSS_EN_PIN

#define DC_DEV_GSM_PORT V_GSM_EN_PORT
#define DC_DEV_GSM_PIN V_GSM_EN_PIN

#define DC_GSM_MUX_PORT GSM_SLEEP_PORT
#define DC_GSM_MUX_PIN GSM_SLEEP_PIN

#define DC_GSM_VDD_SENSE_PORT GSM_VDD_SENS_PORT
#define DC_GSM_VDD_SENSE_PIN GSM_VDD_SENS_PIN

#define DC_1WIRE_LEVEL_PORT _5V_TR_EN_PORT
#define DC_1WIRE_LEVEL_PIN _5V_TR_EN_PIN

#define DC_1WIRE_POWER_PORT _5V_EN_PORT
#define DC_1WIRE_POWER_PIN _5V_EN_PIN

#define DC_1WIRE_PROTECT_PORT _5V_1WIRE_EN_PORT
#define DC_1WIRE_PROTECT_PIN _5V_1WIRE_EN_PIN

//**********************************Statuses********************************************************

//Status flags
enum{
  DC_FLAG_RDY = 0,             //Modem Ready
  DC_FLAG_AT_OK,               //Modem switchwd on
  DC_FLAG_IMEI_OK,             //IMEI OK
  DC_FLAG_REG_OK,              //Modem registered in net
  DC_FLAG_APN_OK,              //GPRS setted
  DC_FLAG_FUNC_FULL_OK,        //Modem FUN mode is full mode 
  DC_FLAG_GPRS_ACTIVE,         //GPRS active
  DC_FLAG_MAIN_TCP,            //Main TCP connection ok
  DC_FLAG_SERVICE_TCP,         //Service TCP connection OK
  DC_FLAG_SLEEP_STAT,          //Sleep status
  DC_WL_LOGIN_OK,              //Login ok
  DC_WL_PACK_SENED,            //Pack sended
  
  DC_FLAG_COORD_PRESET_OK,     //Preset coordinate
  DC_FLAG_CELL_LOC_OK,         //Cell loc
  DC_FLAG_EPO_OK,              //AGPS on
  DC_FLAG_GNSS_ON,             //GNSS switched on
  DC_FLAG_GNSS_TYME_SYNCH,     //GNSS tyme synch
  DC_FLAG_GNSS_RECEIVED,       //GNSS received
  DC_FLAG_GNSS_NOT_RECEIVED,   //GNSS not received
  
  DC_FLAG_BT_POWER_ON,         //Power on BT
  DC_FLAG_SERVICE_FW_STAT_OK,  //Service fw status
  DC_FLAG_SERVICE_GET_FW       //Get FW
};

//Device status
enum{
  DC_MES_INT_ACC = 0,          //ACC interrup 
  DC_MES_INT_RING              //RING interrup 
};

//Masks
extern const uint32_t DC_STATUS_RESET;
extern const uint32_t DC_STATUS_MASK_ONLY_GNSS_ON;
extern const uint32_t DC_STATUS_MASK_GSM_SAVE_POWER_ON;
extern const uint32_t DC_STATUS_MASK_ALL_IN_ONE;

//Dev type
typedef enum {
  DC_DEV_GNSS = 1,
  DC_DEV_GSM,
  DC_DEV_MUX,
  DC_DEV_1WIRE
}DC_dev_type;

//Return status
typedef enum {
  DC_ERROR = 0,
  DC_OK,
  DC_TIMEOUT
}DC_return_t;

//**********************************Modem modes****************************************************

//Modes
typedef enum {
  DC_MODEM_ALL_OFF = 0,          //All off
  DC_MODEM_ONLY_GNSS_ON,         //GNSS on 
  DC_MODEM_GSM_SAVE_POWER_ON,    //GSM in save power        
  DC_MODEM_ALL_IN_ONE,           //GNSS, GSM on
} DC_modemMode_t;

//Modes control
#define SWITCH_ON_GNSS DC_switch_power(DC_DEV_GNSS,1)
#define SWITCH_OFF_GNSS DC_switch_power(DC_DEV_GNSS,0)
#define SWITCH_OFF_GSM DC_switch_power(DC_DEV_GSM,0)
#define SWITCH_ON_GSM DC_switch_power(DC_DEV_GSM,1)
#define SWITCH_MUX_GNSS_IN DC_switch_power(DC_DEV_MUX,0)
#define SWITCH_MUX_ALL_IN_ONE DC_switch_power(DC_DEV_MUX,1)
#define SWITCH_1WIRE_ON DC_switch_power(DC_DEV_1WIRE,1)
#define SWITCH_1WIRE_OFF DC_switch_power(DC_DEV_1WIRE,0)

void DC_setModemMode(DC_modemMode_t mode); //Set Modem mode

//**********************************Execution status************************************************

//Ececution flags
typedef enum{
  DC_FLAG_EXEC_TIMEOUT = 1,
  DC_FLAG_EXEC_OK,
  DC_FLAG_EXEC_ERROR
}DC_flag_t;

//Ececution error
typedef enum{
  DC_ERROR_NO = 0,
  DC_ERROR_CONNECT,
  DC_ERROR_SWITCH,
  DC_ERROR_MODEM_CMD,
  DC_ERROR_GNSS_NOT_FOUND,
  DC_ERROR_NOTHING,
  DC_ERROR_TIME_SYNCH,
  DC_ERROR_LOGIN,
  DC_ERROR_SEND_DATA
}DC_error_t;

//Exeution function
typedef enum{
  DC_FUNC_NONE = 0,
  DC_FUNC_MODEM_ON,
  DC_FUNC_GNSS_ON,
  DC_FUNC_GPRS_CONN,
  DC_FUNC_TCP_CONN,
  DC_FUNC_AGPS_GET,
  DC_FUNC_GNSS_GET,
  DC_FUNC_IMEI_GET,
  DC_FUNC_TIME_SYNCH,
  DC_FUNC_LOGIN,
  DC_FUNC_SEND_DATA,
}DC_func_t;

//Execution
typedef struct{
  DC_flag_t DC_flag;
  DC_error_t DC_error;
  DC_func_t DC_func;
}DC_exec_t;

//**********************************Device mode and device status***********************************

//Device mode
typedef enum {
  DC_MODE_IDLE = 0,                     //IDLE        
  DC_MODE_AGPS_UPDATE,                  //Update AGPS and time synch
  DC_MODE_SEND_DATA,                    //Send data
  DC_MODE_REPEATED,                     //Repeated mode
  DC_MODE_GNSS_DATA,                    //Collect GNSS
  DC_MODE_FIRMWARE,                     //Firmware
  DC_MODE_NEW_SETTING                   //Reciving new settings
}DC_mode_t;

//Control task structure
typedef struct{
  DC_modemMode_t        DC_modemMode;           //Modem mode
  DC_mode_t             DC_modeCurrent;         //Mode current
  DC_mode_t             DC_modeBefore;          //Mode before
  DC_exec_t             DC_exec;                //Mode execution
  uint32_t              DC_status;              //Dev status
  uint8_t               DC_mes_status;          //dev status
}DC_taskCtrl_t;

extern DC_taskCtrl_t DC_taskCtrl; //Task control

//**********************************Default params***************************************************

#define DEBUG 1 //Debug version
#define DEBUG_OUT 1

#define DC_SET_MAGIC_CODE               0x82
#define DC_PARAMS_MAGIC_CODE            0x82

#define DC_SET_DATA_SEND_PERIOD         60
#define DC_SET_DATA_GNSS_PERIOD         30
#define DC_SET_AGPS_SYNCH_PERIOD        45
#define DC_SET_ACCEL_LEVEL              7.5
#define DC_SET_GNSS_TRY_COUNT           3
#define DC_SET_GNSS_TRY_TIME            10
#define DC_SET_GNSS_TRY_SLEEP_S         15
#define DC_SET_GSM_TRY_COUNTS           3
#define DC_SET_GSM_TRY_SLEEP_S          10
#define DC_SET_DATA_IP1                 "94.230.163.104"
#define DC_SET_DATA_IP2                 "94.230.163.104"
#define DC_SET_DATA_PORT                5039
#define DC_SET_DATA_PASS                "2476"
#define DC_SET_SERVICE_IP1              "94.230.163.104"
#define DC_SET_SERVICE_IP2              "94.230.163.104"
#define DC_SET_SERVICE_PORT             1000
#define DC_SET_IMEI                     0
#define DC_SET_BT_PASS                  "111"
#define DC_SET_PHONE_NUM1               "+79006506058"

#define DC_MAX_COUNT_PHONES             5

#define DC_FW_VERSION                   1
#define DC_FW_STATUS                    1


//**********************************Flash save struct***********************************************

//Settings
typedef struct {
  uint32_t      data_send_period;       //Send period in sec
  uint32_t      data_gnss_period;       //GNSS period in sec
  uint32_t      AGPS_synch_period;      //AGPS synch period in sec
  uint8_t       gnss_try_count;         //Count tryes
  uint8_t       gnss_try_time;          //Time for GNSS
  uint32_t      gnss_try_sleep;         //Sleep between tryes sec
  uint8_t       gsm_try_count;          //Count tryes
  uint32_t      gsm_try_sleep;          //Sleep between tryes sec 
  float         acel_level_int;         //Interrupt level in G
  char          ip_dataList[5][16];     //List IP for data transfer          
  uint16_t      dataPort;               //Data port    
  char          dataPass[5];            //Password
  char          ip_serviceList[5][16];  //Service ip list
  uint32_t      servicePort;            //Service port
  char          IMEI[19];               //IMEI
  char          BT_pass[10];            //BT password
  uint8_t       phone_count;            //Count phones
  char          phone_nums[DC_MAX_COUNT_PHONES-1][15];      //Enabled phone numbers
}DC_settings_t;

typedef struct{
  float MCU_temper;                     //Internal temperature
  float BAT_voltage;                    //Battery voltage
  GNSS_data_t GNSS_data;                //GNSS data
  uint8_t Cell_quality;                 //Cell quality
  double power;                         //Power
  uint8_t Status;                       //Global status
}DC_log_t;

//Parametrs
typedef struct {
  uint8_t       APN_setted;             //For some sim card need set APN settings
  time_t        UTC_time;               //Time in UTC
  uint64_t      power;                  //Power consumtion
  float         MCU_temper;             //Internal temperature
  float         BAT_voltage;            //Battery voltage
  uint8_t       Cell_quality;           //Cell quality
  uint8_t       Status;                 //Global status
  uint32_t      log_len;                //Len
  uint32_t      notsended_log_len;      //Len
  uint8_t       current_data_IP;        //Current data IP
  uint8_t       current_service_IP;     //Current service IP
}DC_params_t;

extern DC_log_t DC_log; //Log sample;
extern DC_settings_t DC_settings;//Settings
extern DC_params_t DC_params; //Params
extern EXT_FLASH_image_t DC_fw_image; //Image descriptor

void DC_saveLog(DC_log_t log_data); //Save log data
void DC_readLog(uint32_t log_num ,DC_log_t *log_data); //Read log data

//**********************************Common work*****************************************************

void DC_init(); //device init
void DC_set_RTC_timer_s(uint32_t sec); //Set RTC timer
void DC_sleep(uint32_t sec); //Sleep
void DC_RTC_init(); //RTC init
extern EXT_FLASH_image_t DC_fw_image; //Image descriptor
void DC_reset_system(); //System reset
double DC_getPower(); //Get power
void DC_debugOut(char *str, ...); //Out debug data

//**********************************Status LED******************************************************

void DC_ledStatus_flash(uint8_t count, uint16_t period) ; //Led flash
#define LED_STATUS_OFF GPIO_PinOutSet(STATUS_LED_PORT, STATUS_LED_PIN)
#define LED_STATUS_ON GPIO_PinOutClear(STATUS_LED_PORT, STATUS_LED_PIN)

//**********************************Status work*****************************************************

#define DC_BT_CONN_MAX 3

//Bluettoth status type
typedef struct{
  bool connStatus;              //Status connection
  uint64_t addr;                //Connection address
  char connName[20];            //Connection Name
  uint8_t ID;                   //Connection ID
}DC_BT_Status_t;

extern uint8_t DC_BT_connCount; //Count BT connections
extern DC_BT_Status_t DC_BT_Status[DC_BT_CONN_MAX]; //Bluetooth status
void DC_resetStatuses(uint32_t mask); //Reset statuses
uint8_t DC_getStatus(uint8_t flag); //Get status
void DC_resetStatus(uint8_t flag); //Reset status
void DC_setStatus(uint8_t flag); //Set status
extern uint32_t DC_nextPeriod; //Period for sleep
extern uint16_t globalSleepPeriod; //Sleep period
extern uint8_t sleepMode; //Sleep flag mode

//**********************************Power work******************************************************

void DC_switch_power(DC_dev_type device, uint8_t ctrl); //Switch power

//**********************************Sens work*******************************************************

uint8_t DC_get_GSM_VDD_sense(); //Get GSM VDD SENSE
float DC_get_bat_voltage(); //Get bat voltage
float DC_get_bat_current(); //Get bat current
float DC_get_chip_temper(); //Get chip temper

//**********************************Save and log****************************************************

void DC_save_params(); //Save params
void DC_read_params(); //Read params
void DC_save_settings(); //Save settings
void DC_read_settings(); //Read settings
void DC_save_FW(); //Save FW inf
void DC_read_FW(); //Read FW inf
void DC_save_pack(uint8_t num, char* pack); //Save save FW pack by index

//**********************************Global Timers****************************************************

void DC_startSampleTimer(uint16_t period); //Sample timer

#endif