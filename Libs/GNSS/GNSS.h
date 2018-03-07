
#ifndef GNSS_H
#define GNSS_H

#include "stdint.h"


//GNSS data struct
typedef struct{
  float date;                   //Date ddMMyy
  float time;                   //Time hhmmhh
  float lat1;                   //Latitude, degree
  char lat2;                    //
  float lon1;                   //Longitude, degree
  char lon2;                    //
  float speed;                  //Speed km/h
  float cource;                 //Cource, degree
  float alt;                    //Altitude, meters
  int sats;                     //Count satilites
  char valid;                   //Vaild data flag
  int lock;                     //Lock flag
  float hdop;                   //
  uint8_t GNSS_source;          //Source data
} GNSS_data_t;

extern GNSS_data_t globalGNSS_data;

uint8_t GNSS_parce_RMC(char *GNSS_str, GNSS_data_t *GNSS_data); //Parce GNSS_RMC data
uint8_t GNSS_parce_GGA(char *GNSS_str, GNSS_data_t *GNSS_data); //Parce GNSS_GGA data
float GNSS_convertTo_HHMMSS(float data); //Convert to DDMMSS

#endif