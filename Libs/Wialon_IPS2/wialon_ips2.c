
#include "wialon_ips2.h"
#include "GNSS.h"
#include "stdint.h"
#include "stdio.h"
#include "string.h"
#include "crc16.h"

//Cmd texts
const char wl_cmd_NULL[]                = "";
const char wl_cmd_LOGIN[]               = "#L#";
const char wl_cmd_LOGIN_ANS[]           = "#AL#";
const char wl_cmd_SHORT_DATA[]          = "#SD#";
const char wl_cmd_SHORT_DATA_ANS[]      = "#ASD#";
const char wl_cmd_DATA[]                = "#D#";
const char wl_cmd_DATA_ANS[]            = "#AD#";

//**************************************************************************************************
//                                      Prepare functions
//**************************************************************************************************
//Write and cat
void WL_write_and_cat(char *write, char *pack_buf)
{
  char str_buf[15];
  sprintf(str_buf,"%s", write); strcat(pack_buf,str_buf);
}
//--------------------------------------------------------------------------------------------------
//Create package LOGIN
void WL_prepare_LOGIN(char *pack, char *imei, char *pass) 
{
  //#L#protocol_version;imei;password;crc16\r\n
  uint8_t head_len = strlen(wl_cmd_LOGIN);
  char crc_str[7];
  uint16_t crc_val = 0;
  
  sprintf(pack,"%s2.0;%s;%s;", wl_cmd_LOGIN, imei, pass == NULL ? "NA":pass); //Create root of pack for crc calc
  crc_val = crc16(pack+head_len,strlen(pack)-head_len); //calc crc
  sprintf(crc_str,"%04x\r\n", crc_val); //convert
  strcat(pack, crc_str); //create package
 
}
//----------------------------------------------------------------------------------------------------------------------------
//Create package SHORT_DATA
void WL_prepare_SHORT_DATA(char* pack, GNSS_data_t *data)
{
  uint16_t crc_val = 0;
  char str_buf[15];
  
  strcpy(pack, wl_cmd_SHORT_DATA); //Head
  if (data->date != 0.0) { snprintf(str_buf,14,"%06.0f", data->date); strcat(pack,str_buf);}else{ WL_write_and_cat("NA", pack); }//Date
  if (data->time != 0.0) { snprintf(str_buf,14,";%06.0f", data->time); strcat(pack,str_buf);}else{ WL_write_and_cat(";NA", pack); }//time
  if (data->lat1 != 0.0) { snprintf(str_buf,14,";%09.4lf", data->lat1); strcat(pack,str_buf);}else{ WL_write_and_cat(";NA", pack); }//lat1
  if (data->lat2 != '?') { snprintf(str_buf,14,";%c", data->lat2); strcat(pack,str_buf);}else{ WL_write_and_cat(";NA", pack); }//lat2
  if (data->lon1 != 0.0) { snprintf(str_buf,14,";%010.4lf", data->lon1); strcat(pack,str_buf);}else{ WL_write_and_cat(";NA", pack); }//lon1
  if (data->lon2 != '?') { snprintf(str_buf,14,";%c", data->lon2); strcat(pack,str_buf);}else{ WL_write_and_cat(";NA", pack); }//lon2
  
  #if WL_SPEED_IN_KM_PER_H
    snprintf(str_buf,14,";%.0f", data->speed*1.852); strcat(pack,str_buf);//speed
  #else
    snprintf(str_buf,14,";%.0f", data->speed); strcat(pack,str_buf);//speed
  #endif
    
  snprintf(str_buf,14,";%.0f", data->cource); strcat(pack,str_buf);//course
  if (data->alt != 0.0){snprintf(str_buf,14,";%.0f", data->alt);strcat(pack,str_buf);    }else{ WL_write_and_cat(";NA", pack); }//height
  if (data->sats != 0){snprintf(str_buf,14,";%d", data->sats); strcat(pack,str_buf);  }else{ WL_write_and_cat(";NA;", pack); }//sats
  
  uint8_t head_len = strlen(wl_cmd_SHORT_DATA);
  crc_val = crc16(pack+head_len,strlen(pack)-head_len); //calc crc
  
  sprintf(str_buf,"%04x\r\n", crc_val); //convert
  strcat(pack, str_buf); //create package
}
//----------------------------------------------------------------------------------------------------------------------------
//Create package DATA
void WL_prepare_DATA(char* pack, GNSS_data_t *data, float temp_mcu, float bat_voltage, double power, uint8_t quality, uint8_t status)
{
  uint16_t crc_val = 0;
  char str_buf[16];
  
  strcpy(pack, wl_cmd_DATA); //Head
  if (data->date != 0.0) { snprintf(str_buf,14,"%06.0f", data->date); strcat(pack,str_buf);}else{ WL_write_and_cat("NA", pack); }//Date
  if (data->time != 0.0) { snprintf(str_buf,14,";%06.0f", data->time); strcat(pack,str_buf);}else{ WL_write_and_cat(";NA", pack); }//time
  if (data->lat1 != 0.0) { snprintf(str_buf,14,";%09.4lf", data->lat1); strcat(pack,str_buf);}else{ WL_write_and_cat(";NA", pack); }//lat1
  if (data->lat2 != '?') { snprintf(str_buf,14,";%c", data->lat2); strcat(pack,str_buf);}else{ WL_write_and_cat(";NA", pack); }//lat2
  if (data->lon1 != 0.0) { snprintf(str_buf,14,";%010.4lf", data->lon1); strcat(pack,str_buf);}else{ WL_write_and_cat(";NA", pack); }//lon1
  if (data->lon2 != '?') { snprintf(str_buf,14,";%c", data->lon2); strcat(pack,str_buf);}else{ WL_write_and_cat(";NA", pack); }//lon2
  
  #if WL_SPEED_IN_KM_PER_H
    snprintf(str_buf,14,";%.0f", data->speed*1.852); strcat(pack,str_buf);//speed
  #else
    snprintf(str_buf,14,";%.0f", data->speed); strcat(pack,str_buf);//speed
  #endif
  
  snprintf(str_buf,14,";%.0f", data->cource); strcat(pack,str_buf);//course
  if (data->alt != 0.0){snprintf(str_buf,14,";%.0f", data->alt);strcat(pack,str_buf);    }else{ WL_write_and_cat(";NA", pack); }//height
  if (data->sats != 0){snprintf(str_buf,14,";%d", data->sats); strcat(pack,str_buf);  }else{ WL_write_and_cat(";NA", pack); }//sats
  if (data->hdop != 0){snprintf(str_buf,14,";%.2f", data->hdop); strcat(pack,str_buf);  }else{ WL_write_and_cat(";NA", pack); }//hdop
  WL_write_and_cat(";0", pack);//inputs
  WL_write_and_cat(";0", pack);//outputs
  WL_write_and_cat(";", pack);//adc
  WL_write_and_cat(";NA;", pack);//ibutton

//  //Temper sensors
//  for(int i=0; i<count_temper; i++)
//  {
//    snprintf(str_buf,16,"T%d:2:%.2f,", i, temper_sensors[i]); strcat(pack,str_buf);
//  }
  
  snprintf(str_buf,16,"TM:2:%.2f", temp_mcu); strcat(pack,str_buf); strcat(pack,","); //Tmcu
  snprintf(str_buf,16,"UB:2:%.2f", bat_voltage); strcat(pack,str_buf); strcat(pack,","); //Ubat 
  snprintf(str_buf,16,"P:2:%.2lf", power); strcat(pack,str_buf); strcat(pack,","); //Power
  snprintf(str_buf,16,"Q:1:%d", quality); strcat(pack,str_buf); strcat(pack,",");//Quality
  snprintf(str_buf,16,"S:1:%d", status); strcat(pack,str_buf); strcat(pack,",");//Status
  snprintf(str_buf,16,"G:1:%d,;", data->GNSS_source); strcat(pack,str_buf); //Coordinate source
  
  uint8_t head_len = strlen(wl_cmd_DATA);
  crc_val = crc16(pack+head_len,strlen(pack)-head_len); //calc crc
  
  sprintf(str_buf,"%04x\r\n", crc_val); //convert
  strcat(pack, str_buf); //create package
}
//**************************************************************************************************
//                                      Parce functions
//**************************************************************************************************
//--------------------------------------------------------------------------------------------------
//Login ansver
void WL_parce_LOGIN_ANS(char* pack, WL_ansvers_t *ans)
{
  char str[2];
  char buf[20];
  int n = 0;
  
  //Prepare parce struct
  sprintf(buf, " %s%%n%%s\r\n",wl_cmd_LOGIN_ANS);
  
  //Parce data
  if ( sscanf( pack, buf, &n, str), n){
    if (strcmp(str, "1") == 0)  { *ans = WL_ANS_SUCCESS; return; } //Autorisation ok
    if (strcmp(str, "0") == 0)  { *ans = WL_ANS_DISALLOW; return; } //Disallow
    if (strcmp(str, "01") == 0) { *ans = WL_ANS_PASS_ERR; return; } //Pass err
    if (strcmp(str, "10") == 0) { *ans = WL_ANS_CRC_ERR; return; } //CRC err   
  }
  
  *ans = WL_ANS_UNKNOWN;
}
//--------------------------------------------------------------------------------------------------
//Short data ansver
void WL_parce_SHORT_DATA_ANS(char* pack, WL_ansvers_t *ans)
{
  char str[2];
  char buf[20];
  int n = 0;
  
  //Prepare parce struct
  sprintf(buf, " %s%%n%%s\r\n",wl_cmd_SHORT_DATA_ANS);
  
  //Parce data
  if ( sscanf( pack, buf, &n, str), n){
    if (strcmp(str, "-1") == 0)  { *ans = WL_ANS_PAC_STRUCT_ERR; return; } //Struct error
    if (strcmp(str, "0") == 0)  { *ans = WL_ANS_TIME_ERR; return; } //Time error
    if (strcmp(str, "1") == 0) { *ans = WL_ANS_SUCCESS; return; } //Success
    if (strcmp(str, "10") == 0) { *ans = WL_ANS_COORD_ERR; return; } //Coord error
    if (strcmp(str, "11") == 0) { *ans = WL_ANS_SCH_ERR; return; } //Speed cource height error
    if (strcmp(str, "12") == 0) { *ans = WL_ANS_SAT_COUNT_ERR; return; } //Sat count error
    if (strcmp(str, "13") == 0) { *ans = WL_ANS_CRC_ERR; return; } //CRC err   
  }
  
  *ans = WL_ANS_UNKNOWN;
}
//--------------------------------------------------------------------------------------------------
//Data ansver
void WL_parce_DATA_ANS(char* pack, WL_ansvers_t *ans)
{
  char str[2];
  char buf[20];
  int n = 0;
  
  //Prepare parce struct
  sprintf(buf, " %s%%n%%s\r\n",wl_cmd_DATA_ANS);
  
  //Parce data
  if ( sscanf( pack, buf, &n, str), n){
    if (strcmp(str, "-1") == 0)  { *ans = WL_ANS_PAC_STRUCT_ERR; return; } //Struct error
    if (strcmp(str, "0") == 0)  { *ans = WL_ANS_TIME_ERR; return; } //Time error
    if (strcmp(str, "1") == 0) { *ans = WL_ANS_SUCCESS; return; } //Success
    if (strcmp(str, "10") == 0) { *ans = WL_ANS_COORD_ERR; return; } //Coord error
    if (strcmp(str, "11") == 0) { *ans = WL_ANS_SCH_ERR; return; } //Speed cource height error
    if (strcmp(str, "12") == 0) { *ans = WL_ANS_SAT_COUNT_ERR; return; } //Sat count error
    if (strcmp(str, "13") == 0) { *ans = WL_ANS_IO_ERR; return; } //IO err 
    if (strcmp(str, "14") == 0) { *ans = WL_ANS_ADC_ERR; return; } //ADC err
    if (strcmp(str, "15") == 0) { *ans = WL_ANS_ADD_ERR; return; } //ADD err
    if (strcmp(str, "16") == 0) { *ans = WL_ANS_CRC_ERR; return; } //CRC err 
  }
  
  *ans = WL_ANS_UNKNOWN;  
}