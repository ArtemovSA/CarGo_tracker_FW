
#include "Service_ctrl.h"
#include "stdio.h"
#include "string.h"
#include "crc8.h"

const char SC_MES_status_req[] = "#SR#"; //Status request
const char SC_MES_fw_status_req[] = "#FWS#"; //FW version and len request
const char SC_MES_fw_pack_req[] = "#FWP#"; //FW pack request
const char SC_MES_settings_req[] = "#STR#"; //Settings request

const char SC_MES_ansver_status[] = " #ASR#%3[^;];%3[^;];%02x%n"; //Answer status: #AS#status;ans;crc8
const char SC_MES_ansver_fw_status[] = " #AFWS#%3[^;];%i;%i;%02x;%02x%n"; //Pack version and len ansver: #AFWS#status;version;len;crc8_fw;crc8
const char SC_MES_ansver_fw_pack[] = " #AFWP#%3[^;];%i;%64[^;];%02x%n"; //Pack ansver #AFWP#%status;pack_len;pack;crc8
const char SC_MES_ansver_settings[] = " #ASTR#%3[^;];%80[^;];%02x%n"; //Settings ansver

//--------------------------------------------------------------------------------------------------
//Check status
SC_ans_status SC_check_status(char *ans_mes)
{
  if (strcmp(ans_mes, "1") == 0)  { return SC_ANS_OK; }
  if (strcmp(ans_mes, "2") == 0)  { return SC_ANS_MES_CRC_ERROR; }
  if (strcmp(ans_mes, "3") == 0)  { return SC_ANS_SERVER_ERROR; }
  
  return SC_ANS_UNKNOWN;
}
//--------------------------------------------------------------------------------------------------
//Prepare status request
void SC_prepare_status_req(char *message, char *IMEI, int *version)
{
  char crc_str[5];
  uint8_t crc_val;
  
  sprintf(message, "%s%s;%d;", SC_MES_status_req, IMEI, *version);
  crc_val = crc8((uint8_t*)message,strlen(message), 0); //calc crc
  sprintf(crc_str,"%02x\r\n", crc_val); //convert
  
  strcat(message, crc_str); //create package
}
//--------------------------------------------------------------------------------------------------
//Parce status req
SC_ans_status SC_parce_status_req(char *message, int *status)
{
  uint8_t crc_val, crc_val_cal;
  int n = 0;
  char ans_mes[3];
  char status_mes[3];
    
  if ( sscanf( message, SC_MES_ansver_status, ans_mes ,status_mes, &crc_val, &n), n){
    
    crc_val_cal = crc8((uint8_t*)message,n-2, 0); //calc crc
    
    if (crc_val_cal != crc_val)
      return SC_ANS_CRC_ERROR;
    
    sscanf(status_mes,"%d", status);
  }
  
  return SC_check_status(ans_mes);
}
//--------------------------------------------------------------------------------------------------
//Prepare fw status request
void SC_prepare_fw_status_req(char *message)
{
  char crc_str[5];
  uint8_t crc_val;
  
  sprintf(message, "%s", SC_MES_fw_status_req);
  crc_val = crc8((uint8_t*)message,strlen(message), 0); //calc crc
  sprintf(crc_str,"%02x\r\n", crc_val); //convert
  
  strcat(message, crc_str); //create package
}
//--------------------------------------------------------------------------------------------------
//Parce fw status req
SC_ans_status SC_parce_fw_status_req(char *message, EXT_FLASH_image_t *image)
{
  int crc_val, crc_val_cal;
  int n = 0;
  char ans_mes[5];
    
  if ( sscanf( message, SC_MES_ansver_fw_status, ans_mes, &image->imageVersion_new, &image->imageSize, &image->imageCRC, &crc_val, &n), n)
  {
    crc_val_cal = crc8((uint8_t*)message,n-2, 0); //calc crc
    
    if (crc_val_cal != crc_val)
      return SC_ANS_CRC_ERROR;
  }
  
  return SC_check_status(ans_mes);
}
//--------------------------------------------------------------------------------------------------
//Prepare fw pack request
void SC_prepare_fw_pack_req(char *message)
{
  char crc_str[5];
  uint8_t crc_val;
  
  sprintf(message, "%s", SC_MES_fw_pack_req);
  crc_val = crc8((uint8_t*)message,strlen(message), 0); //calc crc
  sprintf(crc_str,"%02x\r\n", crc_val); //convert
  
  strcat(message, crc_str); //create package
}
//--------------------------------------------------------------------------------------------------
//Parce fw pack req
SC_ans_status SC_parce_fw_pack_req(char *message, char *pack, int *pack_len)
{
  int crc_val, crc_val_cal;
  int n = 0;
  char ans_mes[5];
    
  if ( sscanf( message, SC_MES_ansver_fw_pack, ans_mes, pack_len, pack, &crc_val, &n), n)
  {
    crc_val_cal = crc8((uint8_t*)message,n-2, 0); //calc crc
    
    if (crc_val_cal != crc_val)
      return SC_ANS_CRC_ERROR;
  }
  
  return SC_check_status(ans_mes);
}
//--------------------------------------------------------------------------------------------------
//Prepare settings
void SC_prepare_setting_req(char *message)
{
  char crc_str[5];
  uint8_t crc_val;
  
  sprintf(message, "%s", SC_MES_settings_req);
  crc_val = crc8((uint8_t*)message,strlen(message), 0); //calc crc
  sprintf(crc_str,"%02x\r\n", crc_val); //convert
  
  strcat(message, crc_str); //create package
}
//--------------------------------------------------------------------------------------------------
//Parce settings
SC_ans_status SC_parce_settings_req(char *message)
{
  int crc_val, crc_val_cal;
  int n = 0;
  char ans_mes[5];
  
    
  if ( sscanf( message, SC_MES_ansver_settings, ans_mes, message, &crc_val, &n), n)
  {
    crc_val_cal = crc8((uint8_t*)message,n-2, 0); //calc crc
    
    if (crc_val_cal != crc_val)
      return SC_ANS_CRC_ERROR;
  }
  
  return SC_check_status(ans_mes);
}