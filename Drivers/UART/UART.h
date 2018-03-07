
#ifndef UART
#define UART

#include <stdint.h>
#include <stdbool.h>

#define BUFFERSIZE_TX 170
#define BUFFERSIZE_RX 120

typedef struct
{
  uint8_t  data[BUFFERSIZE_TX];  /* data buffer */
  uint32_t rdI;               /* read index */
  uint32_t wrI;               /* write index */
  uint32_t pendingBytes;      /* count of how many bytes are not yet handled */
  bool     overflow;          /* buffer overflow indicator */
}circularBuffer_TX_t;

typedef struct
{
  uint8_t  data[BUFFERSIZE_RX];  /* data buffer */
  uint32_t rdI;               /* read index */
  uint32_t wrI;               /* write index */
  uint32_t pendingBytes;      /* count of how many bytes are not yet handled */
  bool     overflow;          /* buffer overflow indicator */
}circularBuffer_RX_t;

void UART_RxEnable(); // Enable IRQ
void UART_RxDisable(); // Disable IRQ
uint8_t UART_ReciveByte(); //Recive Byte
uint32_t UART_GetRxCount(); //Get count rx
void UART_SendData(char* dataPtr, uint16_t dataLen); //Send Data


#endif