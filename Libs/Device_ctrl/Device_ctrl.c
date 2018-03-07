
#include "Device_ctrl.h"

//std
#include "stdio.h"
#include "stdlib.h"
#include <stdarg.h>
#include "em_device.h"
#include "em_gpio.h"
#include "em_adc.h"
#include "em_rtc.h"
#include "em_emu.h"
#include "em_dbg.h"
#include "time.h"

//RTOS
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"

//Libs
#include "Task_transfer.h"
#include "I2C_gate_task.h"
#include "task.h"
#include "ADC.h"
#include "cdc.h"
#include "crc8.h"
#include "EXT_Flash.h"
#include "Date_time.h"
#include "Clock.h"
#include "USB_ctrl.h"

//Timers
TimerHandle_t DC_TimerSample_h; //Timer handle

//Mutexs
extern xSemaphoreHandle ADC_mutex; //Mutex
extern xSemaphoreHandle extFlash_mutex; //Mutex

extern volatile void * volatile pxCurrentTCB; //Task number
DC_settings_t DC_settings;//Settings
DC_taskCtrl_t DC_taskCtrl; //Task control
DC_params_t DC_params; //Params
DC_log_t DC_log; //Log sample
EXT_FLASH_image_t DC_fw_image; //Image descriptor
uint32_t DC_nextPeriod; //Period for sleep
uint16_t globalSleepPeriod = 0; //Sleep period
uint8_t sleepMode = 0; //Sleep flag mode

uint8_t DC_BT_connCount; //Count BT connections
DC_BT_Status_t DC_BT_Status[DC_BT_CONN_MAX]; //Bluetooth status

//Masks
const uint32_t DC_STATUS_RESET = 0;
const uint32_t DC_STATUS_MASK_ONLY_GNSS_ON = 0x3FF000;
const uint32_t DC_STATUS_MASK_GSM_SAVE_POWER_ON = 0x381FFF;
const uint32_t DC_STATUS_MASK_ALL_IN_ONE = 0xFFFFFF;

const double DC_Power_const = 0.000000043741862;

//--------------------------------------------------------------------------------------------------
//Out debug data
//arg: str - string for out
void DC_debugOut(char *str, ...)
{
  va_list args;
  va_start(args, str);
  va_end(args);
  
  if (DBG_Connected())
  {
    vprintf(str, args);
    va_end(args);
  }
  
  if (CDC_Configured)
  {
    USB_send_str(str);
  }
}
//--------------------------------------------------------------------------------------------------
//Get power
double DC_getPower()
{
  return (double)DC_params.power*DC_Power_const;
}
//--------------------------------------------------------------------------------------------------
//System reset
void DC_reset_system()
{
  DC_save_FW();
  DC_save_params(); //Save params
  DC_ledStatus_flash(100,100);
  
  NVIC_SystemReset();
}   
//--------------------------------------------------------------------------------------------------
//Led flash
void DC_ledStatus_flash(uint8_t count, uint16_t period)  
{
  while(count--)
  {
    LED_STATUS_ON;
    vTaskDelay(period/2);
    LED_STATUS_OFF;
    vTaskDelay(period/2);
  }
}
//--------------------------------------------------------------------------------------------------
//device init
void DC_init()
{    
  NVIC_SetPriority(DebugMonitor_IRQn, 3);
  NVIC_SetPriority(USART1_RX_IRQn, 1);
  
  DC_debugOut("\r\nSTART\r\n");
  
  DC_ledStatus_flash(3, 100);
    
  if(EXT_Flash_init()) //Инициализация Flash
  {    
    DC_debugOut("Flash OK\r\n");
    
    DC_read_params();     //Read params
    DC_read_settings();   //Read settings
    DC_read_FW();         //Read FW inf
    DC_fw_image.imageCRC = 0;
  }
  
  //Ring IRQ
  GPIO_IntEnable(1 << RING_PIN);
  GPIO_IntConfig(RING_PORT, RING_PIN, false, true, true);
  
  //ACC IRQ settings
  GPIO_IntEnable((1 << MMA_INT1_PIN)|(1 << MMA_INT2_PIN));
  //GPIO_IntConfig(MMA_INT1_PORT, MMA_INT1_PIN, false, true, true);
  GPIO_IntConfig(MMA_INT2_PORT, MMA_INT2_PIN, false, true, true);
  
  NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
  NVIC_EnableIRQ(GPIO_EVEN_IRQn);
  
  //Set UTC time
  globalUTC_time = DC_params.UTC_time;
  DC_RTC_init(); //RTC init
  DC_debugOut("System Date Time: %d\r\n", (uint64_t)DC_params.UTC_time);
  
  INT_Enable(); //Enable global IRQ 
}
//--------------------------------------------------------------------------------------------------
//Reset statuses
void DC_resetStatuses(uint32_t mask)
{
  taskENTER_CRITICAL();
  DC_taskCtrl.DC_status = (DC_taskCtrl.DC_status & mask);
  taskEXIT_CRITICAL();
}
//--------------------------------------------------------------------------------------------------
//Get status
uint8_t DC_getStatus(uint8_t flag)
{
  return (DC_taskCtrl.DC_status >> flag) & 0x01;
}
//--------------------------------------------------------------------------------------------------
//Reset status
void DC_resetStatus(uint8_t flag)
{
  taskENTER_CRITICAL();
  DC_taskCtrl.DC_status &= ~(1<<flag);
  taskEXIT_CRITICAL();
}
//--------------------------------------------------------------------------------------------------
//Set status
void DC_setStatus(uint8_t flag)
{
  taskENTER_CRITICAL();
  DC_taskCtrl.DC_status |= (1<<flag);
  taskEXIT_CRITICAL();
}
//--------------------------------------------------------------------------------------------------
//Switch power
void DC_switch_power(DC_dev_type device, uint8_t ctrl) {
  
  switch (device){
  case DC_DEV_GNSS: if (ctrl == 1) GPIO_PinOutSet(DC_DEV_GNSS_PORT, DC_DEV_GNSS_PIN); else GPIO_PinOutClear(DC_DEV_GNSS_PORT, DC_DEV_GNSS_PIN); break;
  case DC_DEV_GSM: if (ctrl == 1) GPIO_PinOutSet(DC_DEV_GSM_PORT, DC_DEV_GSM_PIN); else GPIO_PinOutClear(DC_DEV_GSM_PORT, DC_DEV_GSM_PIN); break;
  case DC_DEV_MUX: if (ctrl == 1) GPIO_PinOutSet(DC_GSM_MUX_PORT, DC_GSM_MUX_PIN); else GPIO_PinOutClear(DC_GSM_MUX_PORT, DC_GSM_MUX_PIN); break;
  case DC_DEV_1WIRE: if (ctrl == 1) 
  {
    GPIO_PinOutSet(DC_1WIRE_POWER_PORT, DC_1WIRE_POWER_PIN); //Power
    vTaskDelay(50);
    GPIO_PinOutSet(DC_1WIRE_LEVEL_PORT, DC_1WIRE_LEVEL_PIN); //Level
    vTaskDelay(50);
    GPIO_PinOutSet(DC_1WIRE_PROTECT_PORT, DC_1WIRE_PROTECT_PIN); //Protect
  }else{
    GPIO_PinOutClear(DC_1WIRE_PROTECT_PORT, DC_1WIRE_PROTECT_PIN); //Protect
    vTaskDelay(50);
    GPIO_PinOutClear(DC_1WIRE_LEVEL_PORT, DC_1WIRE_LEVEL_PIN); //Level
    vTaskDelay(50);
    GPIO_PinOutClear(DC_1WIRE_POWER_PORT, DC_1WIRE_POWER_PIN); //Power
  }    
  break;
  
  }
}
//--------------------------------------------------------------------------------------------------
//Get GSM VDD SENSE
uint8_t DC_get_GSM_VDD_sense() {
  return GPIO_PinInGet(DC_GSM_VDD_SENSE_PORT, DC_GSM_VDD_SENSE_PIN);
}
//--------------------------------------------------------------------------------------------------
//Get chip temper
float DC_get_chip_temper() {   
  uint16_t adcSample = ADC_getValue(ADC_TEMP_CH);
  return ADC_Celsius(adcSample);
}
//--------------------------------------------------------------------------------------------------
//Get bat voltage
float DC_get_bat_voltage() { 
  uint16_t adcSample = ADC_getValue(ADC_VOLTAGE);
  return ADC_mv(adcSample) * 0.00158;
}
//--------------------------------------------------------------------------------------------------
//Get bat current in mA
float DC_get_bat_current() {
  uint16_t adcSample = ADC_getValue(ADC_CURRENT);
  return (uint16_t)(ADC_mv(adcSample) * 0.285);
}
//**************************************************************************************************
//                                      RTC
//**************************************************************************************************
//RTC init
void DC_RTC_init()
{  
  RTC_CompareSet(0, 10);
  RTC_CompareSet(1, 1);
  
  NVIC_EnableIRQ(RTC_IRQn);
  
  RTC_IntDisable(RTC_IEN_COMP0);
  RTC_IntEnable(RTC_IEN_COMP1);
  RTC_IntDisable(RTC_IEN_OF);
  RTC_IntClear(RTC_IFC_COMP0);
  RTC_IntClear(RTC_IFC_COMP1);

  RTC_Enable(true);
}
//--------------------------------------------------------------------------------------------------
//Set RTC timer
void DC_set_RTC_timer_s(uint32_t sec)
{
  uint32_t comp = (sec - 1);

  RTC_CompareSet(0, comp);
  
  RTC_IntClear(RTC_IFC_COMP0);
  RTC_IntDisable(RTC_IEN_COMP1);
  RTC_IntEnable(RTC_IEN_COMP0);

  RTC_CounterReset();
}
//**************************************************************************************************
//                                      Save and log
//**************************************************************************************************
//Save params
void DC_save_params()
{
  DC_params_t temp = DC_params;
  uint8_t crc8_value;
  uint8_t magic_key = DC_PARAMS_MAGIC_CODE;
  
  EXT_Flash_erace_sector(EXT_FLASH_PARAMS);
  EXT_Flash_writeData(EXT_FLASH_PARAMS, &magic_key, 1);
  
  crc8_value = crc8((uint8_t*)&temp,sizeof(temp), 0);
  EXT_Flash_writeData(EXT_FLASH_PARAMS+1, &crc8_value, 1);  
  
  EXT_Flash_writeData(EXT_FLASH_PARAMS+2,(uint8_t*)&temp,sizeof(DC_params_t));
}
//--------------------------------------------------------------------------------------------------
//Read params
void DC_read_params()
{
  uint8_t magic_key;
  uint8_t crc8_value;
  
  EXT_Flash_readData(EXT_FLASH_PARAMS, &magic_key, 1);
  
  if (magic_key == DC_PARAMS_MAGIC_CODE)
  {
    EXT_Flash_readData(EXT_FLASH_PARAMS+1, &crc8_value, 1);
    EXT_Flash_readData(EXT_FLASH_PARAMS+2,(uint8_t*)&DC_params,sizeof(DC_params));
    
    //Check CRC
    if (crc8_value != crc8((uint8_t*)&DC_params,sizeof(DC_params), 0))
    {
      //Try read more
      EXT_Flash_readData(EXT_FLASH_PARAMS+1, &crc8_value, 1);
      EXT_Flash_readData(EXT_FLASH_PARAMS+2,(uint8_t*)&DC_params,sizeof(DC_params));

      if (crc8_value != crc8((uint8_t*)&DC_params,sizeof(DC_params), 0))
      {
        memset(&DC_params,0,sizeof(DC_params));
        DC_save_params();
      }
    }
    
  }else{
    memset(&DC_params,0,sizeof(DC_params));
    DC_save_params();
  }
  
  DC_debugOut("Read params OK\r\n");
}
//--------------------------------------------------------------------------------------------------
//Set default settings
void DC_set_default_settings()
{
  DC_settings.acel_level_int = DC_SET_ACCEL_LEVEL;
  DC_settings.gnss_try_count = DC_SET_GNSS_TRY_COUNT;
  DC_settings.gnss_try_time = DC_SET_GNSS_TRY_TIME;
  DC_settings.gnss_try_sleep = DC_SET_GNSS_TRY_SLEEP_S;
  DC_settings.gsm_try_count = DC_SET_GSM_TRY_COUNTS;
  DC_settings.gsm_try_sleep = DC_SET_GSM_TRY_SLEEP_S;
  
  //Periods
  DC_settings.data_send_period = DC_SET_DATA_SEND_PERIOD;
  DC_settings.data_gnss_period = DC_SET_DATA_GNSS_PERIOD;
  DC_settings.AGPS_synch_period = DC_SET_AGPS_SYNCH_PERIOD;
  
  //IP addresses
  memcpy(DC_settings.ip_dataList[0], DC_SET_DATA_IP1, sizeof(DC_settings.ip_dataList));
  memcpy(DC_settings.ip_dataList[1], DC_SET_DATA_IP2, sizeof(DC_settings.ip_dataList));
  memcpy(DC_settings.ip_serviceList[0], DC_SET_SERVICE_IP1, sizeof(DC_settings.ip_serviceList));
  memcpy(DC_settings.ip_serviceList[1], DC_SET_SERVICE_IP2, sizeof(DC_settings.ip_serviceList));
  
  DC_settings.dataPort = DC_SET_DATA_PORT;
  DC_settings.servicePort = DC_SET_SERVICE_PORT;
  memcpy(DC_settings.BT_pass, DC_SET_BT_PASS, sizeof(DC_settings.BT_pass));
  
  memcpy(DC_settings.dataPass, DC_SET_DATA_PASS, sizeof(DC_settings.dataPass));
  
  //Phone numbers
  strcpy(DC_settings.phone_nums[0], DC_SET_PHONE_NUM1);
  DC_settings.phone_count = 1;
  
  DC_debugOut("Setted default settings\r\n");
}
//--------------------------------------------------------------------------------------------------
//Save settings
void DC_save_settings()
{
  uint8_t crc8_value;
  uint8_t magic_key = DC_SET_MAGIC_CODE;
  
  EXT_Flash_erace_sector(EXT_FLASH_SETTINGS);
  EXT_Flash_writeData(EXT_FLASH_SETTINGS, &magic_key, 1);
              
  crc8_value = crc8((uint8_t*)&DC_settings,sizeof(DC_settings_t), 0);
  EXT_Flash_writeData(EXT_FLASH_SETTINGS+1, &crc8_value, 1);  
  
  EXT_Flash_writeData(EXT_FLASH_SETTINGS+2,(uint8_t*)&DC_settings,sizeof(DC_settings_t)); 
  
  DC_debugOut("Save settings\r\n");
}
//--------------------------------------------------------------------------------------------------
//Read settings
void DC_read_settings()
{
  uint8_t magic_key;
  uint8_t crc8_value;
  
  EXT_Flash_readData(EXT_FLASH_SETTINGS, &magic_key, 1);
  
  if (magic_key == DC_SET_MAGIC_CODE)
  {
    EXT_Flash_readData(EXT_FLASH_SETTINGS+1, &crc8_value, 1);
    EXT_Flash_readData(EXT_FLASH_SETTINGS+2,(uint8_t*)&DC_settings,sizeof(DC_settings_t));
    
    //Check CRC
    if (crc8_value != crc8((uint8_t*)&DC_settings,sizeof(DC_settings_t), 0))
    {
      //Try read more
      EXT_Flash_readData(EXT_FLASH_SETTINGS+1, &crc8_value, 1);
      EXT_Flash_readData(EXT_FLASH_SETTINGS+2,(uint8_t*)&DC_settings,sizeof(DC_settings_t));
    
      if (crc8_value != crc8((uint8_t*)&DC_settings,sizeof(DC_settings_t), 0))
      {
        DC_set_default_settings();
        DC_save_settings();
      }
    }
          
  }else{
    DC_set_default_settings();
    DC_save_settings();
  }
  DC_debugOut("Read settings OK\r\n");
}
//--------------------------------------------------------------------------------------------------
//Save log data
void DC_saveLog(DC_log_t log_data)
{
  uint16_t logSize = sizeof(DC_log_t);
  uint32_t addresEndLog = EXT_FLASH_LOG_DATA+DC_params.log_len*logSize;

  //Check end of log size
  if (((addresEndLog+logSize) >= (EXT_FLASH_SECTOR_SIZE*EXT_FLASH_LOG_DATA_SIZE)) || (DC_params.log_len == 0))
  {
    DC_debugOut("Log full. Erace start log data\r\n");
    DC_params.log_len = 0;
    DC_params.notsended_log_len = 0;
    for (int i=0; i<EXT_FLASH_LOG_DATA_SIZE; i++)
    {
      EXT_Flash_erace_sector(EXT_FLASH_LOG_DATA+i*EXT_FLASH_SECTOR_SIZE);
    }
    addresEndLog = EXT_FLASH_LOG_DATA;
  }
    
  EXT_Flash_writeData(addresEndLog, (uint8_t*)&log_data, logSize);
  DC_params.log_len++;
}
//--------------------------------------------------------------------------------------------------
//Read log data
void DC_readLog(uint32_t log_num ,DC_log_t *log_data)
{
  uint16_t logSize = sizeof(DC_log_t);
  uint32_t addres = EXT_FLASH_LOG_DATA+log_num*logSize;
  
  EXT_Flash_readData(addres, (uint8_t*)log_data, logSize);
}
//--------------------------------------------------------------------------------------------------
//Save FW inf
void DC_save_FW()
{
  EXT_Flash_erace_sector(EXT_FLASH_CONTROL_STRUCT);
  taskENTER_CRITICAL();
  EXT_Flash_writeData(EXT_FLASH_CONTROL_STRUCT, (uint8_t*)&DC_fw_image, sizeof(EXT_FLASH_image_t));
  taskEXIT_CRITICAL();
  
  DC_debugOut("Firmware saved\r\n");
}
//--------------------------------------------------------------------------------------------------
//Read FW inf
void DC_read_FW()
{
  EXT_Flash_readData(EXT_FLASH_CONTROL_STRUCT, (uint8_t*)&DC_fw_image, sizeof( EXT_FLASH_image_t ));
  
  if ( DC_fw_image.statusCode != DC_FW_STATUS)
  {
    DC_fw_image.imageVersion = DC_FW_VERSION;
    DC_fw_image.statusCode = DC_FW_STATUS;
    DC_fw_image.imageCRC = 0;
    DC_fw_image.imageVersion_new = 0;
    DC_fw_image.imageSize = 0;
    DC_fw_image.imageLoaded = 0;
    DC_save_FW();
    DC_debugOut("Read firmware OK\r\n");
  } 
}
//--------------------------------------------------------------------------------------------------
//Save save FW pack by index
void DC_save_pack(uint8_t num, char* pack)
{
  EXT_Flash_writeData(EXT_FLASH_PROGRAMM_IMAGE+num*64, (uint8_t*)pack, 64);
}
//**************************************************************************************************
//                                      Modem modes
//**************************************************************************************************
//Set Modem mode
//arg: mode - DC_MODEM_ALL_OFF, DC_MODEM_ALL_IN_ONE, DC_MODEM_ONLY_GNSS_ON
void DC_setModemMode(DC_modemMode_t mode)
{
  //All off
  if (mode == DC_MODEM_ALL_OFF)
  {
    SWITCH_MUX_GNSS_IN;
    SWITCH_OFF_GSM;
    SWITCH_OFF_GNSS;
    DC_resetStatuses(DC_STATUS_RESET);
    DC_debugOut("Modem mode: ALL OFF\r\n");
  }   
  
  //All in one
  if (mode == DC_MODEM_ALL_IN_ONE)
  {
    SWITCH_MUX_GNSS_IN;
    SWITCH_MUX_ALL_IN_ONE;
    SWITCH_ON_GSM;
    vTaskDelay(100);
    SWITCH_ON_GNSS;
    vTaskDelay(100);
    DC_resetStatuses(DC_STATUS_MASK_ALL_IN_ONE);
    DC_debugOut("Modem mode: ALL IN ONE\r\n");
  }
  
  //GSM save power   
  if (mode == DC_MODEM_GSM_SAVE_POWER_ON)
  {
    SWITCH_OFF_GNSS;
    SWITCH_ON_GSM;
    vTaskDelay(100);
    SWITCH_MUX_GNSS_IN;
    DC_resetStatuses(DC_STATUS_MASK_GSM_SAVE_POWER_ON);
    DC_debugOut("Modem mode: SAVE POWER MODE GSM ON\r\n");
  }
  
  //Only GNSS on
  if (mode == DC_MODEM_ONLY_GNSS_ON)
  {
    SWITCH_MUX_GNSS_IN;
    SWITCH_OFF_GSM;
    SWITCH_ON_GNSS;
    vTaskDelay(100);
    DC_resetStatuses(DC_STATUS_MASK_ONLY_GNSS_ON);
    DC_debugOut("Modem mode: GNSS ON\r\n");
  }    
  
  DC_taskCtrl.DC_modemMode = mode;
}
//**************************************************************************************************
//                                      Sleep
//**************************************************************************************************
void DC_sleep(uint32_t sec)
{
  DC_set_RTC_timer_s(sec); //Set RTC timer 
  EMU_EnterEM2(true);
}
//**************************************************************************************************
//                                      Interrupts
//**************************************************************************************************
//GPIO int
//void GPIO_ODD_IRQHandler(void)
//{
//  uint32_t gpio_int = GPIO_IntGet();
//  
//  if (gpio_int & (1 << MMA_INT1_PIN))
//  {
//    DC_debugOut("@ACC IRQ\r\n");
//    DC_taskCtrl.DC_mes_status |= (1<<DC_MES_INT_ACC);
//  }  
//  
//  if (gpio_int & (1 << MMA_INT2_PIN))
//  {
//    DC_debugOut("@ACC IRQ\r\n");
//    DC_taskCtrl.DC_mes_status |= (1<<DC_MES_INT_ACC);
//  }  
//  
//  GPIO_IntClear(gpio_int);
//  NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
//}
//--------------------------------------------------------------------------------------------------
//GPIO int
void GPIO_EVEN_IRQHandler(void)
{ 
  uint32_t gpio_int = GPIO_IntGet();
  
  if (gpio_int & (1 << RING_PIN))
  {
    if (sleepMode == 1)
    {
      CL_incUTC(RTC_CounterGet()); //Add sleep counter
      RTC_CounterReset();
      RTC_IntDisable(RTC_IEN_COMP0);
      RTC_IntEnable(RTC_IEN_COMP1);
    }

    DC_debugOut("@RING IRQ\r\n");
    DC_taskCtrl.DC_mes_status |= (1<<DC_MES_INT_RING);
  }
  
//  if (gpio_int & (1 << MMA_INT1_PIN))
//  {
//    globalUTC_time -= globalSleepPeriod - RTC_CounterGet();
//    DC_debugOut("@ACC IRQ1\r\n");
//    DC_taskCtrl.DC_mes_status |= (1<<DC_MES_INT_ACC);
//  }  
  
  if (gpio_int & (1 << MMA_INT2_PIN))
  {
    if (sleepMode == 1)
    {
      CL_incUTC(RTC_CounterGet()); //Add sleep counter
      RTC_CounterReset();
      RTC_IntDisable(RTC_IEN_COMP0);
      RTC_IntEnable(RTC_IEN_COMP1);
    }
    
    DC_debugOut("@ACC IRQ2\r\n");
    DC_taskCtrl.DC_mes_status |= (1<<DC_MES_INT_ACC);
  }  
  
  GPIO_IntClear(gpio_int);
}
//--------------------------------------------------------------------------------------------------
//RTC int
void RTC_IRQHandler(void)
{
  uint32_t rtc_int = RTC_IntGet();
  
  //Sleep comparator
  if (rtc_int & RTC_IFC_COMP0)
  {
    CL_incUTC(RTC_CounterGet()); //Add sleep counter
    
    RTC_CounterReset();
    RTC_IntDisable(RTC_IEN_COMP0);
    RTC_IntEnable(RTC_IEN_COMP1);
    
    RTC_IntClear(RTC_IFC_COMP0);
    rtc_int = RTC_IntGet();
    
    DC_debugOut("@RTC IRQ\r\n");
  }
  
  //Time comparator
  if (rtc_int & RTC_IFC_COMP1)
  {
    CL_incUTC(1);
      
    RTC_CounterReset();
    RTC_IntClear(RTC_IFC_COMP1);
  }
  
  NVIC_ClearPendingIRQ(RTC_IRQn);
}

//**************************************************************************************************
//                                      Global Timers
//**************************************************************************************************
//--------------------------------------------------------------------------------------------------
//Start timer
//arg:  DC_Timer_hp - point on timer handle
//      pcTimerName - timer name
//      PeriodInTicks - periodic
//      pxCallbackFunction -- point on callback
void DC_startTimer(TimerHandle_t DC_Timer_hp, char *pcTimerName, TickType_t PeriodInTicks, TimerCallbackFunction_t pxCallbackFunction)
{
  DC_Timer_hp = xTimerCreate(pcTimerName, PeriodInTicks, pdTRUE, ( void * )0, pxCallbackFunction);
  xTimerStart(DC_Timer_hp, 0);
  DC_debugOut("Started timer\r\n"); 
}
//--------------------------------------------------------------------------------------------------
//Sample timer
void vDC_Timer_sample( TimerHandle_t xTimer )
{
  if( xSemaphoreTake( ADC_mutex, portMAX_DELAY ) == pdTRUE )
  {
    DC_params.power += ADC_getValue(ADC_CURRENT);
    xSemaphoreGive( ADC_mutex );
  }
  
  if (USB_state == USB_STATE_MSG_PROCESS)
  {
    USB_msg_process();
    USB_state = USB_STATE_WAIT_STOP1;
    USB_rx_count = 0;
  }
}
//--------------------------------------------------------------------------------------------------
//Sample timer
void DC_startSampleTimer(uint16_t period)
{
  DC_startTimer(DC_TimerSample_h,"Tsample", period, vDC_Timer_sample);
}