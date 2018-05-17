
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
#include "Delay.h"

//Timers
TimerHandle_t DC_TimerSample_h; //Timer handle
TimerHandle_t DC_TimerMonitor_h; //Timer handle

//Mutexs
extern xSemaphoreHandle ADC_mutex; //Mutex
extern xSemaphoreHandle extFlash_mutex; //Mutex

//Mode switch
extern volatile void * volatile pxCurrentTCB; //Task number
EXT_FLASH_image_t DC_fw_image; //Image descriptor
uint32_t DC_nextPeriod; //Period for sleep
uint8_t sleepMode = 0; //Sleep flag mode

uint8_t DC_BT_connCount; //Count BT connections
DC_BT_Status_t DC_BT_Status[DC_BT_CONN_MAX]; //Bluetooth status

//System parametrs
DC_settings_t DC_settings;//Settings
volatile DC_taskCtrl_t DC_taskCtrl; //Task control
DC_params_t DC_params; //Params
DC_dataLog_t DC_dataLog; //Data log
volatile DC_debugLog_t DC_debugLog; //Debug log
volatile DC_status_t DC_status; //Dev status

const double DC_Power_const = 0.000000043741862;

//--------------------------------------------------------------------------------------------------
void LEUART1_sendBuffer(char* txBuffer, int bytesToSend)
{
  LEUART_TypeDef *uart = LEUART1;
  int ii;

  /* Sending the data */
  for (ii = 0; ii < bytesToSend;  ii++)
  {
    /* Waiting for the usart to be ready */
    while (!(uart->STATUS & LEUART_STATUS_TXBL)) ;

    if (txBuffer != 0)
    {
      /* Writing next byte to USART */
      uart->TXDATA = *txBuffer;
      txBuffer++;
    }
    else
    {
      uart->TXDATA = 0;
    }
  }
//  
//  uint16_t try_counter = 1000;
//  /*Waiting for transmission of last byte */
//  while (try_counter--)
//  {
//    if (uart->STATUS & LEUART_STATUS_TXC)
//      break;
//    
//    _delay_timer_ms(5);
//  }
//  
}
//--------------------------------------------------------------------------------------------------
//Out debug data
//arg: str - string for out
void DC_debugOut(char *str, ...)
{
  char strBuffer[120];
  
  va_list args;
  va_start(args, str);
  va_end(args);
  
  vsprintf(strBuffer, str, args);
  va_end(args);
  
  if (DBG_Connected())
  {
    printf(strBuffer);
  }

  LEUART1_sendBuffer(strBuffer, strlen(strBuffer));
  va_end(args);
  
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
  DC_save_settings(); //Save settings
  
  DC_ledStatus_flash(50,100);
  
  NVIC_SystemReset();
}   
//--------------------------------------------------------------------------------------------------
//Led flash
void DC_ledStatus_flash(uint8_t count, uint16_t period)  
{
  while(count--)
  {
    LED_STATUS_OFF;
    _delay_timer_ms(period/2);
    LED_STATUS_ON;
    _delay_timer_ms(period/2);
  }
}
//--------------------------------------------------------------------------------------------------
//device init
void DC_init()
{     
  DC_debugOut("\r\nSTART\r\n");
  DC_debugOut("FreeRTOS free Heap: %d words\r\n", xPortGetFreeHeapSize());
  
  DC_ledStatus_flash(10, 100);
    
  if(EXT_Flash_init()) //Инициализация Flash
  {    
    DC_debugOut("Flash OK\r\n");
  }
  
  DC_read_settings();   //Read settings
  DC_read_params();     //Read params
  DC_read_FW();         //Read FW inf
  DC_fw_image.imageCRC = 0;
 
#ifdef SW_RING_IRQ
  
  //Ring IRQ
  GPIO_IntEnable(1 << RING_PIN);
  GPIO_IntConfig(RING_PORT, RING_PIN, false, true, true);
  
#endif
  
#ifdef SW_ACC_IRQ
  
  //ACC IRQ settings
  GPIO_IntEnable((1 << MMA_INT1_PIN)|(1 << MMA_INT2_PIN));
  GPIO_IntConfig(MMA_INT1_PORT, MMA_INT1_PIN, false, true, true);
  GPIO_IntConfig(MMA_INT2_PORT, MMA_INT2_PIN, false, true, true);
  
  NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
  NVIC_EnableIRQ(GPIO_EVEN_IRQn);
  
  NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
  NVIC_EnableIRQ(GPIO_ODD_IRQn);
  
#endif
  
  //Set UTC time
  globalUTC_time = DC_params.UTC_time;
  DC_RTC_init(); //RTC init
  DC_debugOut("System Date Time: %d\r\n", (uint64_t)DC_params.UTC_time);

  INT_Enable(); //Enable global IRQ 
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
//Get sensor temper
// Return count sensors
// params point to Tempers
uint8_t DC_get_sensor_temper(float *pTempers)
{
  uint8_t countTempers = 0;
  
  memset(&ds18b20, 0, sizeof(ds18b20));
  
  SWITCH_1WIRE_ON;
  Ds18b20_Init(_1_WIRE_EFM_PORT, _1_WIRE_EFM_PIN);
  Ds18b20_getTemperatures();
  SWITCH_1WIRE_OFF;

  for (int i=0; i<_DS18B20_MAX_SENSORS; i++)
  {
    if (ds18b20[i].DataIsValid)
    {
      memcpy(((uint8_t*)pTempers+countTempers*sizeof(float)),(uint8_t*)&ds18b20[i].Temperature, sizeof(float));
      countTempers++;
    }
  }
  
  return countTempers;
}
//--------------------------------------------------------------------------------------------------
//Get bat voltage
float DC_get_bat_voltage() { 
  uint16_t adcSample = ADC_getValue(ADC_VOLTAGE);
  return ADC_mv(adcSample) * 0.0021;
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
  temp.magic_key = DC_PARAMS_MAGIC_CODE;
  
  EXT_Flash_erace_sector(EXT_FLASH_PARAMS);
  EXT_Flash_writeData(EXT_FLASH_PARAMS,(uint8_t*)&temp,sizeof(DC_params_t));
  
  DC_debugOut("Save params\r\n");
}
//--------------------------------------------------------------------------------------------------
//Read params
void DC_read_params()
{  
  EXT_Flash_readData(EXT_FLASH_PARAMS,(uint8_t*)&DC_params ,sizeof(DC_params_t));
  
  if (DC_params.magic_key == DC_PARAMS_MAGIC_CODE)
  {
    DC_debugOut("Read params OK\r\n");
  }else{
    memset((void*)&DC_params,0,sizeof(DC_params));
    DC_save_params();
  }
}
//--------------------------------------------------------------------------------------------------
//Set default settings
void DC_set_default_settings()
{  
  //Try settings
  DC_settings.gnss_try_count = DC_SET_GNSS_TRY_COUNT;
  DC_settings.collect_try_sleep = DC_SET_COLLECT_TRY_SLEEP;
  DC_settings.send_try_sleep = DC_SET_SEND_TRY_SLEEP;
  DC_settings.tcp_try_sleep = DC_SET_TCP_TRY_SLEEP;
  
  //Periods
  DC_settings.data_send_period = DC_SET_DATA_SEND_PERIOD;
  DC_settings.data_gnss_period = DC_SET_DATA_COLLECT_PERIOD;
  DC_settings.AGPS_synch_period = DC_SET_AGPS_SYNCH_PERIOD;
  
  //Periods
  DC_settings.AGPSMode_try_count = DC_SET_AGPS_MODE_TRY;
  DC_settings.collectMode_try_count = DC_SET_COLLECT_MODE_TRY;
  DC_settings.sendMode_try_count = DC_SET_SEND_MODE_TRY;
  
  //IP addresses
  memcpy((void*)DC_settings.ip_dataList[0], DC_SET_DATA_IP1, sizeof(DC_settings.ip_dataList));
  memcpy((void*)DC_settings.ip_dataList[1], DC_SET_DATA_IP2, sizeof(DC_settings.ip_dataList));
  memcpy((void*)DC_settings.ip_serviceList[0], DC_SET_SERVICE_IP1, sizeof(DC_settings.ip_serviceList));
  memcpy((void*)DC_settings.ip_serviceList[1], DC_SET_SERVICE_IP2, sizeof(DC_settings.ip_serviceList));
  
  DC_settings.servicePort = DC_SET_SERVICE_PORT;
  DC_settings.ip_serviceListLen = DC_SET_SERVICE_IP_LEN;
  
  DC_settings.dataPort = DC_SET_DATA_PORT;
  DC_settings.ip_dataListLen = DC_SET_DATA_IP_LEN;
  memcpy((void*)DC_settings.dataPass, DC_SET_DATA_PASS, sizeof(DC_settings.dataPass));
  
  //Phone numbers
  strcpy((void*)DC_settings.phone_nums[0], DC_SET_PHONE_NUM1);
  DC_settings.phone_count = 1;
  
  //Misc
  DC_settings.acel_level_int = DC_SET_ACCEL_LEVEL;
  memcpy((void*)DC_settings.BT_pass, DC_SET_BT_PASS, sizeof(DC_settings.BT_pass));
  
  DC_debugOut("Setted default settings\r\n");
}
//--------------------------------------------------------------------------------------------------
//Save settings
void DC_save_settings()
{
  DC_settings.magic_key = DC_SETTINGS_MAGIC_CODE;
  EXT_Flash_erace_sector(EXT_FLASH_SETTINGS);
  
  EXT_Flash_writeData((EXT_FLASH_SETTINGS),(uint8_t*)&DC_settings, sizeof(DC_settings)); 
  DC_debugOut("Save settings\r\n");
}
//--------------------------------------------------------------------------------------------------
//Read settings
void DC_read_settings()
{ 
  EXT_Flash_readData(EXT_FLASH_SETTINGS,(uint8_t*)&DC_settings,sizeof(DC_settings));
  
  if (DC_settings.magic_key == DC_PARAMS_MAGIC_CODE)
  {
    DC_debugOut("Read settings OK\r\n");
  }else{
    DC_set_default_settings();
    DC_save_settings();
  }
}
//--------------------------------------------------------------------------------------------------
//Add data log
DC_return_t DC_addDataLog(DC_dataLog_t dataLog)
{
  uint16_t logSize = sizeof(DC_dataLog_t);
  uint32_t addresEndLog = EXT_FLASH_LOG_DATA+DC_params.dataLog_len*logSize;

  //Check end of log size
  if (((DC_params.dataLog_len+logSize) > EXT_FLASH_SECTOR_SIZE) || (DC_params.dataLog_len == 0))
  {
    DC_debugOut("Erace log data\r\n");

    EXT_Flash_erace_sector(EXT_FLASH_LOG_DATA);   
  }
    
  dataLog.valid_key = DC_VALID_MAGIC_CODE;
  EXT_Flash_writeData(addresEndLog, (uint8_t*)&dataLog, logSize);
  
  DC_params.dataLog_len++;
  DC_save_params();
  DC_debugOut("Add to data log #:%d\r\n", DC_params.dataLog_len);
  
  return DC_OK;
}
//--------------------------------------------------------------------------------------------------
//Check log data
DC_return_t DC_checkLogData(DC_dataLog_t *log_data)
{
  //Check magic key
  if (log_data->valid_key != DC_VALID_MAGIC_CODE)
    return DC_ERROR;
  
  //Check data time
  if ((log_data->GNSS_data.date <= 0) || (log_data->GNSS_data.time <= 0))
    return DC_ERROR;
  
  //Check
  if ((log_data->GNSS_data.cource > 360) || (log_data->GNSS_data.lock != 1))
    return DC_ERROR;
  
  return DC_OK;
}
//--------------------------------------------------------------------------------------------------
//Read log data
DC_return_t DC_readDataLog(DC_dataLog_t *log_data)
{
  uint16_t logSize = sizeof(DC_dataLog_t);
  
  if (DC_params.dataLog_len > 0)
  {
    uint32_t address = EXT_FLASH_LOG_DATA+(DC_params.dataLog_len-1)*logSize;
    DC_params.dataLog_len--;
  
    EXT_Flash_readData(address, (uint8_t*)log_data, logSize);

    if (DC_checkLogData(log_data) != DC_OK)
      return DC_ERROR;
    
    return DC_OK;
  }
  
  return DC_ERROR;
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
    DC_status.statusWord = 0;
    DC_debugOut("Modem mode: ALL OFF\r\n");
    vTaskDelay(1000);
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
    DC_debugOut("Modem mode: ALL IN ONE\r\n");
  }
  
  //GSM save power   
  if (mode == DC_MODEM_GSM_SAVE_POWER_ON)
  {
    SWITCH_OFF_GNSS;
    SWITCH_ON_GSM;
    vTaskDelay(100);
    SWITCH_MUX_GNSS_IN;
    DC_status.statusWord = 0;
    DC_debugOut("Modem mode: SAVE POWER MODE GSM ON\r\n");
  }
  
  //Only GNSS on
  if (mode == DC_MODEM_ONLY_GNSS_ON)
  {
    SWITCH_MUX_GNSS_IN;
    SWITCH_OFF_GSM;
    SWITCH_ON_GNSS;
    vTaskDelay(100);
    DC_status.statusWord = 0;
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
void GPIO_ODD_IRQHandler(void)
{
  uint32_t gpio_int = GPIO_IntGet();
  
  if (gpio_int & (1 << MMA_INT2_PIN))
  {
    if (sleepMode == 1)
    {
      CL_incUTC(RTC_CounterGet()); //Add sleep counter
      RTC_CounterReset();
      RTC_IntDisable(RTC_IEN_COMP0);
      RTC_IntEnable(RTC_IEN_COMP1);
    }
    
    DC_taskCtrl.DC_event.flags.event_ACC = true;
    DC_debugOut("@ACC IRQ2\r\n");

  }  
  
  GPIO_IntClear(gpio_int);
}

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
    
  }
  
//  if (gpio_int & (1 << MMA_INT1_PIN))
//  {
//    globalUTC_time -= globalSleepPeriod - RTC_CounterGet();
//    DC_debugOut("@ACC IRQ1\r\n");
//    DC_taskCtrl.DC_mes_status |= (1<<DC_MES_INT_ACC);
//  }  
  
  if (gpio_int & (1 << MMA_INT1_PIN))
  {
    if (sleepMode == 1)
    {
      DC_ledStatus_flash(5,200);
      CL_incUTC(RTC_CounterGet()); //Add sleep counter
      RTC_CounterReset();
      RTC_IntDisable(RTC_IEN_COMP0);
      RTC_IntEnable(RTC_IEN_COMP1);
    }
    
    DC_debugOut("@ACC IRQ1\r\n");
    DC_taskCtrl.DC_event.flags.event_ACC = true;
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
    RTC_IntEnable(RTC_IEN_COMP1);
    
    RTC_IntClear(RTC_IFC_COMP0);
    rtc_int = RTC_IntGet();
    
    //DC_debugOut("@RTC IRQ\r\n");
    RTC_IntDisable(RTC_IEN_COMP0);
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
  DC_debugOut("Started timer: %s\r\n", pcTimerName); 
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
//--------------------------------------------------------------------------------------------------
//Monitor timer
void vDC_Timer_monitor( TimerHandle_t xTimer )
{
  float sumVoltage;
  float voltage;
  
  if( xSemaphoreTake( ADC_mutex, portMAX_DELAY ) == pdTRUE )
  {
    for (int i = 0; i<10; i++)
    {
      sumVoltage += DC_get_bat_voltage();
    }
    xSemaphoreGive( ADC_mutex );
    voltage = (sumVoltage/10);
    
    if (voltage <= 2.1)
    {
      SWITCH_OFF_GSM;
      SWITCH_OFF_GNSS;
      SWITCH_MUX_GNSS_IN;
      SWITCH_1WIRE_OFF;
      DC_set_RTC_timer_s(30); //Set RTC timer
      EMU_EnterEM2(true);
    }
  }
}
//--------------------------------------------------------------------------------------------------
//Start monitor timer
void DC_startMonitorTimer(uint16_t period)
{
  DC_startTimer(DC_TimerMonitor_h,"Tmonitor", period, vDC_Timer_monitor);
}
