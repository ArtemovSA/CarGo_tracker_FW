
#ifndef GNSS_H
#define GNSS_H

#include "stdint.h"

//Source coordinate
#define COORD_SOURCE_GNSS       1
#define COORD_SOURCE_GSM        2

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
  float hdop;                   //Heigh
  uint8_t coordSource;          //Source data
} GNSS_data_t;

uint8_t GNSS_parce_RMC(char *GNSS_str, GNSS_data_t *GNSS_data); //Parce GNSS_RMC data
uint8_t GNSS_parce_GGA(char *GNSS_str, GNSS_data_t *GNSS_data); //Parce GNSS_GGA data
float GNSS_convertTo_HHMMSS(float data); //Convert to DDMMSS

#endif