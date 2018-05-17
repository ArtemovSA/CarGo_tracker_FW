
#ifndef DEVICE_CTRL_H
#define DEVICE_CTRL_H

#include "stdint.h"
#include "stdbool.h"
#include "InitDevice.h"
#include "EXT_flash.h"
#include "GNSS.h"
#include "time.h"
#include "ds18b20.h"

//RTOS
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"

//**********************************Default params**************************************************

#define DC_SETTINGS_MAGIC_CODE          0x03
#define DC_PARAMS_MAGIC_CODE            0x05
#define DC_VALID_MAGIC_CODE             0x17

#define DC_SET_DATA_SEND_PERIOD         300
#define DC_SET_DATA_COLLECT_PERIOD      120
#define DC_SET_AGPS_SYNCH_PERIOD        600

#define DC_SET_ACCEL_LEVEL              7.5

#define DC_SET_GNSS_TRY_COUNT           10
#define DC_SET_COLLECT_TRY_SLEEP        15
#define DC_SET_SEND_TRY_SLEEP           15
#define DC_SET_TCP_TRY_SLEEP            10

#define DC_SET_AGPS_MODE_TRY            2
#define DC_SET_COLLECT_MODE_TRY         3
#define DC_SET_SEND_MODE_TRY            3

#define DC_SET_DATA_IP_LEN               2
#define DC_SET_DATA_IP1                 "94.230.163.104"
#define DC_SET_DATA_IP2                 "94.230.163.104"
#define DC_SET_DATA_PORT                5039

#define DC_SET_DATA_PASS                "2476"

#define DC_SET_SERVICE_IP_LEN           2
#define DC_SET_SERVICE_IP1              "94.230.163.104"
#define DC_SET_SERVICE_IP2              "94.230.163.104"
#define DC_SET_SERVICE_PORT             1000

#define DC_SET_BT_PASS                  "111"

#define DC_SET_PHONE_NUM1               "+79006506058"
#define DC_MAX_COUNT_PHONES             5

#define DC_FW_VERSION                   2
#define DC_FW_STATUS                    1

//**********************************default variables***********************************************

//Max count IP addresses
#define DC_MAX_IP_COUNT                 5

//Tracker control const
#define TRY_COUNT_AGPS_MODE             2  //Try count AGPS
#define TRY_COUNT_SEND_MODE             3  //Try count Send
#define TRY_COUNT_COLLECT_MODE          3  //Try count Collect
#define TRY_COUNT_ACC_EVENT             5  //Try event

#define NON_SWITCH_OFF_PERIOD           20 //Non sleep period
#define WAIT_DATA_PERIOD                20 //Wait data period

//Connection ID
#define TCP_CONN_ID_MAIN                1
#define TCP_CONN_ID_SERVICE             2

//**********************************Misc************************************************************

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

//Modes control
#define SWITCH_ON_GNSS DC_switch_power(DC_DEV_GNSS,1)
#define SWITCH_OFF_GNSS DC_switch_power(DC_DEV_GNSS,0)
#define SWITCH_OFF_GSM DC_switch_power(DC_DEV_GSM,0)
#define SWITCH_ON_GSM DC_switch_power(DC_DEV_GSM,1)
#define SWITCH_MUX_GNSS_IN DC_switch_power(DC_DEV_MUX,0)
#define SWITCH_MUX_ALL_IN_ONE DC_switch_power(DC_DEV_MUX,1)
#define SWITCH_1WIRE_ON DC_switch_power(DC_DEV_1WIRE,1)
#define SWITCH_1WIRE_OFF DC_switch_power(DC_DEV_1WIRE,0)

//**********************************Modem modes****************************************************

//Modes
typedef enum {
  DC_MODEM_ALL_OFF = 0,          //All off
  DC_MODEM_ONLY_GNSS_ON,         //GNSS on 
  DC_MODEM_GSM_SAVE_POWER_ON,    //GSM in save power        
  DC_MODEM_ALL_IN_ONE,           //GNSS, GSM on
} DC_modemMode_t;

void DC_setModemMode(DC_modemMode_t mode); //Set Modem mode

//**********************************Execution status************************************************

//Ececution flags
typedef enum{
  DC_EXEC_FLAG_OK = 1,
  DC_EXEC_FLAG_ERROR
}DC_execFlag_t;

//Exeution function
typedef enum{
  DC_EXEC_FUNC_NONE = 0,
  DC_EXEC_FUNC_MODEM_ON,
  DC_EXEC_FUNC_GNSS_ON,
  DC_EXEC_FUNC_GPRS_CONN,
  DC_EXEC_FUNC_TCP_CONN,
  DC_EXEC_FUNC_AGPS_EXEC,
  DC_EXEC_FUNC_COORD_GNSS_GET,
  DC_EXEC_FUNC_COORD_GSM_GET,
  DC_EXEC_FUNC_IMEI_GET,
  DC_EXEC_FUNC_MODULE_TIME_SYNCH,
  DC_EXEC_FUNC_TIME_SYNCH,
  DC_EXEC_FUNC_WIALON_LOGIN,
  DC_EXEC_FUNC_GET_LOG,
  DC_EXEC_FUNC_WIALON_SEND_DATA,
}DC_execFunc_t;

//Execution
typedef struct{
  DC_execFlag_t DC_execFlag;
  DC_execFunc_t DC_execFunc;
}DC_exec_t;

//**********************************Device mode and device status***********************************

//IRQ enable
#define SW_RING_IRQ     1
#define SW_ACC_IRQ      1

//Device status
typedef union
{
  struct
  {
    bool flag_REG_OK : 1;
    bool flag_GPRS_SETTED : 1;
    bool flag_GPRS_CONNECTED : 1;
    bool flag_TYME_MODULE_SYNCH : 1;
    bool flag_MAIN_TCP : 1;
    bool flag_SERVICE_TCP : 1;
    bool flag_BT_POWER_ON : 1;
  }flags;
  uint32_t statusWord;
}DC_status_t;

//Event
typedef struct{
  struct
  {
    bool event_ACC : 1;
    bool event_RING : 1;
  }flags;
  uint32_t eventWord; 
}DC_event_t;

//Device mode
typedef enum {
  DC_MODE_IDLE = 0,                     //IDLE        
  DC_MODE_AGPS_UPDATE,                  //Update AGPS and time synch
  DC_MODE_SEND_DATA,                    //Send data
  DC_MODE_COLLECT_DATA,                 //Collect data
  DC_MODE_ACC_EVENT,                    //Event        
  DC_MODE_FIRMWARE,                     //Firmware
  DC_MODE_NEW_SETTING                   //Reciving new settings
}DC_mode_t;

//Control task structure
typedef struct{
  DC_modemMode_t        DC_modemMode;           //Modem mode
  DC_mode_t             DC_modeCurrent;         //Mode current
  DC_mode_t             DC_modeBefore;          //Mode before
  DC_exec_t             DC_exec;                //Mode execution
  DC_event_t            DC_event;               //Events
}DC_taskCtrl_t;

extern volatile DC_status_t DC_status; //Dev status
extern volatile DC_taskCtrl_t DC_taskCtrl; //Task control
extern uint32_t DC_nextPeriod; //Period for sleep
extern uint16_t globalSleepPeriod; //Sleep period
extern uint8_t sleepMode; //Sleep flag mode

//**********************************Bluetooth*******************************************************

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

//**********************************Flash save struct***********************************************

//Setted flags
typedef union
{
  struct
  {
    bool flag_APN_SETTED : 1;
  }flags;
  uint32_t flagWord;
}DC_setFlags_t;

//Settings
typedef struct {
  uint8_t       magic_key;
  
  DC_setFlags_t settingFlags;           //Flags setted
  
  uint32_t      data_send_period;       //Send period in sec
  uint32_t      data_gnss_period;       //GNSS period in sec
  uint32_t      AGPS_synch_period;      //AGPS synch period in sec
  
  uint8_t       collectMode_try_count;  //Count tryes
  uint8_t       sendMode_try_count;     //Count tryes
  uint8_t       AGPSMode_try_count;     //Count tryes
  
  uint8_t       gnss_try_count;         //Time for GNSS
  uint16_t      collect_try_sleep;      //Sleep between tryes sec
  uint16_t      send_try_sleep;         //Sleep between tryes sec 
  uint16_t      tcp_try_sleep;          //Sleep between tryes sec 

  char          ip_dataList[DC_MAX_IP_COUNT][16];     //List IP for data transfer          
  uint16_t      dataPort;               //Data port   
  uint8_t       ip_dataListLen;         //Len of IP
  
  char          ip_serviceList[DC_MAX_IP_COUNT][16];  //Service ip list
  uint32_t      servicePort;            //Service port
  uint8_t       ip_serviceListLen;      //Len of IP
  
  uint8_t       phone_count;            //Count phones
  char          phone_nums[DC_MAX_COUNT_PHONES-1][15];      //Enabled phone numbers
    
  float         acel_level_int;         //Interrupt level in G
  char          dataPass[5];            //Password
  char          IMEI[19];               //IMEI
  char          BT_pass[10];            //BT password
}DC_settings_t;

//Data log
typedef struct{
  uint8_t valid_key;                            //valid data key
  
  float MCU_temper;                             //Internal temperature
  float BAT_voltage;                            //Battery voltage
  float SensorTemp[_DS18B20_MAX_SENSORS];       //Sensors temperatures
  uint8_t countTempSensors;                     //Count temp sensors        
  GNSS_data_t GNSS_data;                        //GNSS data
  uint8_t Cell_quality;                         //Cell quality
  double power;                                 //Power
  uint32_t Status;                              //Global status
  uint32_t Event;                               //Event
}DC_dataLog_t;

//Debug log ID
typedef enum{
  DL_ID_START = 1,
  
  DL_ID_MODEM_ON,
  DL_ID_MODEM_OFF,
  DL_ID_GNSS_ON,
  DL_ID_GNSS_OFF,
  DL_ID_BT_ON,
  DL_ID_BT_OFF,
  
  DL_ID_MAIN_TCP_OK,
  DL_ID_MAIN_TCP_FAIL,
  DL_ID_SERVICE_TCP_OK,
  DL_ID_SERVICE_TCP_FAIL,
  
  DL_ID_WIALON_LOGIN_OK,
  DL_ID_WIALON_LOGIN_FAIL,
  DL_ID_WIALON_SEND_OK,
  DL_ID_WIALON_SEND_FAIL,
  
}debugLogId_t;

//Debug log
typedef struct{
  debugLogId_t  debugLogId;             //log information
  time_t        UTC_time;               //Time in UTC
}DC_debugLog_t;

//Parametrs
typedef struct {
  uint8_t       magic_key;
  
  time_t        UTC_time;               //Time in UTC
  uint64_t      power;                  //Power consumtion
  float         MCU_temper;             //Internal temperature
  float         BAT_voltage;            //Battery voltage
  uint8_t       Cell_quality;           //Cell quality
  uint8_t       countTempSensors;       //Count temperature sensors
  uint32_t      dataLog_len;            //Data log Len
  uint32_t      debugLog_len;           //Debug log Len
  uint32_t      debugLog_pos;           //Debug log pos
  uint8_t       current_data_IP;        //Current data IP
  uint8_t       current_service_IP;     //Current service IP
}DC_params_t;

extern volatile DC_debugLog_t DC_debugLog; //Debug log
extern DC_dataLog_t DC_dataLog; //Data log
extern DC_settings_t DC_settings;//Settings
extern DC_params_t DC_params; //Params
extern EXT_FLASH_image_t DC_fw_image; //Image descriptor

//Device wialon status
#define DC_WIALON_STAT_ACC      1

//Data log
DC_return_t DC_addDataLog(DC_dataLog_t dataLog); //Add data log
DC_return_t DC_readDataLog(DC_dataLog_t *log_data); //Read log data
DC_return_t DC_checkLogData(DC_dataLog_t *log_data); //Check log data

//Debug log


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
#define LED_STATUS_OFF GPIO_PinOutClear(STATUS_LED_PORT, STATUS_LED_PIN)
#define LED_STATUS_ON GPIO_PinOutSet(STATUS_LED_PORT, STATUS_LED_PIN)

//**********************************Power work******************************************************

void DC_switch_power(DC_dev_type device, uint8_t ctrl); //Switch power

//**********************************Sens work*******************************************************

uint8_t DC_get_GSM_VDD_sense(); //Get GSM VDD SENSE
float DC_get_bat_voltage(); //Get bat voltage
float DC_get_bat_current(); //Get bat current
float DC_get_chip_temper(); //Get chip temper
uint8_t DC_get_sensor_temper(float *pTempers); //Get sensor temper

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
void DC_startMonitorTimer(uint16_t period); //Start monitor timer

#endif