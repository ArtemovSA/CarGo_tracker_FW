
#include "GNSS.h"
#include "stdlib.h"
#include "stdint.h"
#include "stdio.h"
#include "string.h"
#include "math.h"

float time, date, latitude, longitude, knots, cource;
char valid, ns, ew, mode;
 
//--------------------------------------------------------------------------------------------------
//Parce GNSS_RMC data
uint8_t GNSS_parce_RMC(char *GNSS_str, GNSS_data_t *GNSS_data)
{
  
  //$GNRMC,133119.000,A,5955.0034,N,03021.7280,E,0.00,48.95,031117,,,D*4E
  if(strlen(GNSS_str) > 0)
  {
    //Time
    char *p = strchr(GNSS_str, ',');
    GNSS_data->time = atof(p+1);
    
    //Valid
    p = strchr(p+1, ',');
    GNSS_data->valid = p[1] == ',' ? '?' : p[1];
    
    //Latitude
    p = strchr(p+1, ',');
    GNSS_data->lat1 = atof(p+1);
    
    //NS
    p = strchr(p+1, ',');
    GNSS_data->lat2 = p[1] == ',' ? '?' : p[1];
    
    //longitude
    p = strchr(p+1, ',');
    GNSS_data->lon1 = atof(p+1);
    
    //EW
    p = strchr(p+1, ',');
    GNSS_data->lon2 = p[1] == ',' ? '?' : p[1];
    
    //Knots
    p = strchr(p+1, ',');
    GNSS_data->speed = atof(p+1);
    
    //Cource
    p = strchr(p+1, ',');
    GNSS_data->cource = atof(p+1);
    
    //Date
    p = strchr(p+1, ',');
    GNSS_data->date = atof(p+1);
    
    return 1;
  }
  return 0;
}
//--------------------------------------------------------------------------------------------------
//Parce GNSS_GGA data
uint8_t GNSS_parce_GGA(char *GNSS_str, GNSS_data_t *GNSS_data)
{
    //$GPGGA,172814.0,3723.46587704,N,12202.26957864,W,2,6,1.2,18.893,M,-25.669,M,2.0,0031*4F
    char *p = strchr(GNSS_str, ',');
    GNSS_data->time = atof(p+1); 
 
    p = strchr(p+1, ',');
    GNSS_data->lat1 = atof(p+1);
 
    p = strchr(p+1, ',');
    GNSS_data->lat2 = p[1] == ',' ? '?' : p[1];
 
    p = strchr(p+1, ',');
    GNSS_data->lon1 = atof(p+1);
 
    p = strchr(p+1, ',');
    GNSS_data->lon2 = p[1] == ',' ? '?' : p[1];
 
    p = strchr(p+1, ',');
    GNSS_data->lock = atoi(p+1);
 
    p = strchr(p+1, ',');
    GNSS_data->sats = atoi(p+1);
 
    p = strchr(p+1, ',');
    GNSS_data->hdop = atof(p+1);
 
    p = strchr(p+1, ',');
    GNSS_data->alt = atof(p+1);

  return 1;
}
//--------------------------------------------------------------------------------------------------
//Convert to DDMMSS
float GNSS_convertTo_HHMMSS(float data)
{
  uint8_t DD = (uint8_t)trunc(data);
  uint8_t MM = (uint8_t)trunc((data - DD) * 60);
  float SS = ((data - DD) * 60 - MM) * 60;
  
  return (DD*100+MM+SS/100);
}