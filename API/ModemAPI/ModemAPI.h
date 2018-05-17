
#ifndef MODEM_API_H
#define MODEM_API_H

#include "Device_ctrl.h"
#include "MC60_lib.h"
#include "Clock.h"
#include "Modem_gate_task.h"

#define MAPI_REG_TRY_SEC                15      //wait registration in cell
#define MAPI_SYNCH_TYME_TRY_SEC         5       //try synch time
#define MAPI_SWITCH_ON_BT_TRY           3       //try switch BT
#define MAPI_SERVER_CONNECT_TRY         3       //try server connect
#define MAPI_SEND_TCP_TRY               3       //try send TCP
#define MAPI_SEND_TCP_WAIT              5000    //wait send TCP

//***************************Misc*******************************************************************

DC_return_t MAPI_init(QueueHandle_t queue); //init API
DC_return_t MAPI_sendUSSD(char* USSD_mes, char* USSD_ans); //Send USSD message

//***************************Internet and TCP*******************************************************

DC_return_t MAPI_GPRS_connect(char* deviceIP); //Connect GPRS
DC_return_t MAPI_sendTCP(uint8_t index, char *str); //Send TCP str
DC_return_t MAPI_TCP_connect(uint8_t index, uint8_t ipList_Len, char* ipList, uint16_t port); //Server TCP connection

//***************************Coordinate and GNSS****************************************************

DC_return_t MAPI_getCelloc(float *lat, float* lon); //Get celloc
DC_return_t MAPI_setRef_loc(float *refLat, float* refLon); //Set referent coord 
DC_return_t MAPI_get_GNSS(GNSS_data_t* data); //Get GNSS data

//***************************Parametrs**************************************************************

DC_return_t MAPI_getQuality(uint8_t* retQuality); //Get quality
DC_return_t MAPI_getIMEI(char* retIMEI); //Get IMEI

//***************************Settings***************************************************************

DC_return_t MAPI_ModuleSynchTime(); //Module Synch Time
DC_return_t MAPI_SynchTime(); //Time Synch sequence
DC_return_t MAPI_set_AGPS(); //AGPS

//***************************Modem control**********************************************************

DC_return_t MAPI_switch_on_GNSS(); //Switch on GNSS
DC_return_t MAPI_switch_on_Modem(); //Switch nn modem 

//***************************Bluetooth**************************************************************

DC_return_t MAPI_BT_switchOn(); //BT switch on

//***************************Wialon protocol********************************************************

DC_return_t MAPI_wialonLogin(char* IMEI, char* pass); //Wialon login
DC_return_t MAPI_wialonSend(DC_dataLog_t *data); //Send data


#endif
