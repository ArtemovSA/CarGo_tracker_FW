
#ifndef USB_CTRL_H
#define USB_CTRL_H

#include "stdint.h"

enum{
  USB_STATE_WAIT_STOP1 = 0,
  USB_STATE_WAIT_STOP2,
  USB_STATE_MSG_PROCESS,
};

enum{
  USB_CMD_FLASH_ARR_WRITE = 1,
  USB_CMD_FLASH_BYTE_WRITE,
  USB_CMD_FLASH_ARR_READ,
  USB_CMD_FLASH_BYTE_READ
};

extern uint8_t USB_state;
extern uint8_t USB_rx_buf[60];
extern uint8_t USB_rx_count;

void USB_msg_process();
void USBReceive_int(uint8_t *data, int byteCount);

#endif