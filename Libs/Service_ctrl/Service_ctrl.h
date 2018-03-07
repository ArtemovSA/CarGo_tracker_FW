
#ifndef SERVICE_CTRL_H
#define SERVICE_CTRL_H

#include "stdint.h"
#include "EXT_flash.h"

//Ansver status
typedef enum{
  SC_ANS_UNKNOWN = 0,
  SC_ANS_OK,
  SC_ANS_CRC_ERROR,
  SC_ANS_MES_CRC_ERROR, 
  SC_ANS_SERVER_ERROR
}SC_ans_status;

//Server status
typedef enum{
  SC_SERVER_NOTHING = 0,
  SC_SERVER_NEW_FM,
  SC_SERVER_NEW_SETTINGS
}SC_server_status;

void SC_prepare_status_req(char *message, char *IMEI, int *version); //Prepare status request
SC_ans_status SC_parce_status_req(char *message, int *status); //Parce status req
void SC_prepare_fw_status_req(char *message); //Prepare fw status request
SC_ans_status SC_parce_fw_status_req(char *message, EXT_FLASH_image_t *image); //Parce fw status req
void SC_prepare_fw_pack_req(char *message); //Prepare fw pack request
SC_ans_status SC_parce_fw_pack_req(char *message, char *pack, int *pack_len); //Parce fw pack req
void SC_prepare_setting_req(char *message); //Prepare settings
SC_ans_status SC_parce_settings_req(char *message); //Parce settings

#endif