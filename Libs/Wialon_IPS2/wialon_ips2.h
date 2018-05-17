

#ifndef WIALON_IPS2_H
#define WIALON_IPS2_H

#include "stdint.h"
#include "GNSS.h"

//************************************Config********************************************************

#define WL_SPEED_IN_KM_PER_H       1            //Speed in km/h

//**************************************************************************************************

//Package ID
typedef enum {
  WL_ID_LOGIN = 1,
  WL_ID_LOGIN_ANS,
  WL_ID_SHORT_DATA,
  WL_ID_SHORT_DATA_ANS,
  WL_ID_DATA,
  WL_ID_DATA_ANS,
  WL_ID_PING,
  WL_ID_PING_ANS,
  WL_ID_BLACK_BOX,
  WL_ID_BLACK_BOX_ANS,
  WL_ID_MES,
  WL_ID_MES_ANS,
  WL_ID_IMG_QUERY,
  WL_ID_IMG,
  WL_ID_TAH_QUERY,
  WL_ID_TAH_INF,
  WL_ID_TAH_DATA,
  WL_ID_TAH_DATA_ANS,
  WL_ID_FW,
  WL_ID_CONF,
  WL_ID_COUNT
} WL_ID_t;

//Direction "Who send"
typedef enum {
  WL_NDKOWN = 0,
  WL_SERVER,
  WL_DEVICE,
  WL_BIDIR
} WL_DIR_t;

//Ansvers
typedef enum{
  WL_ANS_UNKNOWN = 0,
  WL_ANS_DISALLOW,
  WL_ANS_SUCCESS,
  WL_ANS_ERR,
  WL_ANS_PASS_ERR,
  WL_ANS_CRC_ERR,
  WL_ANS_PAC_STRUCT_ERR,
  WL_ANS_TIME_ERR,
  WL_ANS_COORD_ERR,
  WL_ANS_SCH_ERR,
  WL_ANS_SAT_COUNT_ERR,
  WL_ANS_IO_ERR,
  WL_ANS_ADC_ERR,
  WL_ANS_ADD_ERR,
} WL_ansvers_t;

//Prepare packages
void WL_prepare_LOGIN(char *pack, char *imei, char *pass); //Create package LOGIN
void WL_prepare_SHORT_DATA(char* pack, GNSS_data_t *data); //Create package SHORT_DATA
void WL_prepare_DATA(char* pack, GNSS_data_t *data, float temp_mcu, float bat_voltage, double power, uint8_t quality, uint8_t status, uint8_t count_temper, float *temper_sensors); //Create package DATA

//Parce
void WL_parce_LOGIN_ANS(char* pack, WL_ansvers_t *ans); //Login ansver
void WL_parce_SHORT_DATA_ANS(char* pack, WL_ansvers_t *ans); //Short data ansver
void WL_parce_DATA_ANS(char* pack, WL_ansvers_t *ans); //Data ansver

#endif