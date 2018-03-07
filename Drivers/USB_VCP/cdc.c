/**************************************************************************//**
 * @file cdc.c
 * @brief CDC source file
 * @author Silicon Labs
 * @version x.xx
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2014 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Silicon Labs has no
 * obligation to support this Software. Silicon Labs is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Silicon Labs will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 ******************************************************************************/

#include "em_device.h"
#include "em_usb.h"
#include "cdc.h"
#include "em_usb.h"
#include "em_usbtypes.h"
#include "em_usbhal.h"
#include "em_usbd.h"
#include "descriptors.h"

volatile bool CDC_Configured = false;

/* Define USB endpoint addresses */
#define EP_DATA_OUT       0x01  /* Endpoint for USB data reception.       */
#define EP_DATA_IN        0x81  /* Endpoint for USB data transmission.    */
#define EP_NOTIFY         0x82  /* The notification endpoint (not used).  */

#define BULK_EP_SIZE     64  /* This is the max. ep size.    */
#define USB_RX_BUF_SIZ   BULK_EP_SIZE /* Packet size when receiving on USB*/
#define USB_TX_BUF_SIZ   64    /* Packet size when transmitting on USB.  */


/* The serial port LINE CODING data structure, used to carry information  */
/* about serial port baudrate, parity etc. between host and device.       */
EFM32_PACK_START( 1 )
typedef struct
{
  uint32_t    dwDTERate;          /** Baudrate                            */
  uint8_t     bCharFormat;        /** Stop bits, 0=1 1=1.5 2=2            */
  uint8_t     bParityType;        /** 0=None 1=Odd 2=Even 3=Mark 4=Space  */
  uint8_t     bDataBits;          /** 5, 6, 7, 8 or 16                    */
  uint8_t     dummy;              /** To ensure size is a multiple of 4 bytes.*/
} __attribute__ ((packed)) cdcLineCoding_TypeDef;
EFM32_PACK_END()

/*
 * The LineCoding variable must be 4-byte aligned as it is used as USB
 * transmit and receive buffer
 */
EFM32_ALIGN(4)
EFM32_PACK_START( 1 )
cdcLineCoding_TypeDef __attribute__ ((aligned(4))) cdcLineCoding =
{
  115200, 0, 0, 8, 0
};
EFM32_PACK_END()

STATIC_UBUF(usbRxBuffer0, USB_RX_BUF_SIZ);    /* USB receive buffers.   */
static  uint8_t  *usbRxBuffer =  usbRxBuffer0;

static USBReceiveHandler _OnUSBReceiveHandler=NULL;

//static int UsbDataTransmitted(USB_Status_TypeDef status,uint32_t xferred,uint32_t remaining);
static int UsbDataReceived(USB_Status_TypeDef status,uint32_t xferred,uint32_t remaining);
void CDC_StateChange( USBD_State_TypeDef oldState, USBD_State_TypeDef newState );
int CDC_SetupCmd( const USB_Setup_TypeDef *setup );

static const USBD_Callbacks_TypeDef callbacks =
{
  .usbReset        = NULL,
  .usbStateChange  = CDC_StateChange,
  .setupCmd        = CDC_SetupCmd,
  .isSelfPowered   = NULL,
  .sofInt          = NULL
};

static const USBD_Init_TypeDef usbInitStruct =
{
  .deviceDescriptor    = &USBDESC_deviceDesc,
  .configDescriptor    = USBDESC_configDesc,
  .stringDescriptors   = USBDESC_strings,
  .numberOfStrings     = sizeof(USBDESC_strings)/sizeof(void*),
  .callbacks           = &callbacks,
  .bufferingMultiplier = USBDESC_bufferingMultiplier,
  .reserved            = 0
};


void USB_set_rx_handler();

//--------------------------------------------------------------------------------------------------
//Init
int  USB_Driver_Init( USBReceiveHandler OnUSBReceiveHandler)
{
  int result;

  result = USBD_Init(&usbInitStruct);
  /*
  * When using a debugger it is practical to uncomment the following three
  * lines to force host to re-enumerate the device.
  */
  USBD_Disconnect();
  USBTIMER_DelayMs(500);
  USBD_Connect();
  _OnUSBReceiveHandler = OnUSBReceiveHandler;
  
  return result;
}

/**************************************************************************//**
 * Called each time the USB device state is changed.
 * Starts CDC operation when device has been configured by USB host.
 *****************************************************************************/
void CDC_StateChange( USBD_State_TypeDef oldState, USBD_State_TypeDef newState )
{
  if ( newState == USBD_STATE_CONFIGURED )
  {
    CDC_Configured = true;
    USB_set_rx_handler();
  }
  else
  {
    CDC_Configured = false;
  }
}


/**************************************************************************//**
 * Called each time the USB host sends a SETUP command.
 * Implements CDC class specific commands.
 *****************************************************************************/
int CDC_SetupCmd( const USB_Setup_TypeDef *setup )
{
  int retVal = USB_STATUS_REQ_UNHANDLED;

  if ( ( setup->Type      == USB_SETUP_TYPE_CLASS          ) &&
       ( setup->Recipient == USB_SETUP_RECIPIENT_INTERFACE )    )
  {
    switch ( setup->bRequest )
    {
      case USB_CDC_GETLINECODING:
      /********************/
#if defined( NO_RAMFUNCS )
        if ( ( setup->wValue    == 0 ) &&
             ( setup->wIndex    == 0 ) &&       /* Interface no.            */
             ( setup->wLength   == 7 ) &&       /* Length of cdcLineCoding  */
             ( setup->Direction == USB_SETUP_DIR_IN ) )
#endif
        {
          /* Send current settings to USB host. */
          USBD_Write( 0, (void*)&cdcLineCoding, 7, NULL );
          retVal = USB_STATUS_OK;
        }
        break;

      case USB_CDC_SETLINECODING:
      /********************/
#if defined( NO_RAMFUNCS )
        if ( ( setup->wValue    == 0 ) &&
             ( setup->wIndex    == 0 ) &&       /* Interface no.            */
             ( setup->wLength   == 7 ) &&       /* Length of cdcLineCoding  */
             ( setup->Direction != USB_SETUP_DIR_IN ) )
#endif
        {
          /* Get new settings from USB host. */
          USBD_Read( 0, (void*)&cdcLineCoding, 7, NULL );
          retVal = USB_STATUS_OK;
        }
        break;

      case USB_CDC_SETCTRLLINESTATE:
      /********************/
#if defined( NO_RAMFUNCS )
        if ( ( setup->wIndex    == 0 ) &&       /* Interface no.  */
             ( setup->wLength   == 0 ) )        /* No data        */
#endif
        {
          /* Do nothing ( Non compliant behaviour !! ) */
          retVal = USB_STATUS_OK;
        }
        break;
    }
  }

  return retVal;
}

void USB_Send(void *data, int byteCount)
{
  USBD_Write(EP_DATA_IN, (void*)data, byteCount, NULL);
}

void USB_Recive(void *data, int byteCount)
{
  USBD_Read(EP_DATA_OUT, (void*)data, byteCount, UsbDataReceived);
}

void USB_send_str(char *str)
{
  USB_Send(str, strlen(str));
}

void USB_set_rx_callback(USBReceiveHandler handler)
{
  _OnUSBReceiveHandler = handler;
}

void USB_set_rx_handler()
{

  USBD_Ep_TypeDef *ep = USBD_GetEpFromAddr( EP_DATA_OUT );
  ep->state          = D_EP_RECEIVING;
  ep->xferCompleteCb = UsbDataReceived;
  
  USBD_ArmEp( ep );
}
  
/**************************************************************************//**
 * @brief Callback function called whenever a packet with data has been
 *        transmitted on USB
 *
 * @param[in] status    Transfer status code.
 * @param[in] xferred   Number of bytes transferred.
 * @param[in] remaining Number of bytes not transferred.
 *
 * @return USB_STATUS_OK.
 *****************************************************************************/
//  
//static int UsbDataTransmitted(USB_Status_TypeDef status,
//                              uint32_t xferred,
//                              uint32_t remaining)
//{
//  (void) xferred;              
//  (void) remaining;            
//
//  if (status == USB_STATUS_OK)
//  {
// 
////      USBD_Write(EP_DATA_IN, (void*) uartRxBuffer[ uartRxIndex ^ 1],
////                 uartRxCount, UsbDataTransmitted);
////      uartRxCount = 0;
////      USBTIMER_Start(0, RX_TIMEOUT, UartRxTimeout);
//      
//  }
//  return USB_STATUS_OK;
//}

/**************************************************************************//**
 * @brief Callback function called whenever a new packet with data is received
 *        on USB.
 *
 * @param[in] status    Transfer status code.
 * @param[in] xferred   Number of bytes transferred.
 * @param[in] remaining Number of bytes not transferred.
 *
 * @return USB_STATUS_OK.
 *****************************************************************************/
int UsbDataReceived(USB_Status_TypeDef status, uint32_t xferred, uint32_t remaining)
{
  (void) remaining;            /* Unused parameter */
  
  if ((status == USB_STATUS_OK) && (xferred > 0))
  {
    /* Start a new USB receive transfer. */
    USBD_Read(EP_DATA_OUT, (void*)usbRxBuffer, xferred, UsbDataReceived);
    if(_OnUSBReceiveHandler!=NULL)_OnUSBReceiveHandler( (void*)usbRxBuffer, xferred);
  }
  
  return USB_STATUS_OK;
}