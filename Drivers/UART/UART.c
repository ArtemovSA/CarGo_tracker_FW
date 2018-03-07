
#include "UART.h"
#include <stdint.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_usart.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "queue.h"
#include "task.h"

extern QueueHandle_t xBinarySemaphore;
static USART_TypeDef * uart   = USART1;

volatile circularBuffer_RX_t UART_rxBuf = { {0}, 0, 0, 0, false };
volatile circularBuffer_TX_t UART_txBuf = { {0}, 0, 0, 0, false };

//--------------------------------------------------------------------------------------------------
// Enable IRQ
void UART_RxEnable() {
  NVIC_SetPriority (DebugMonitor_IRQn, 2);
  NVIC_SetPriority (USART1_RX_IRQn, 1);
  USART_IntDisable(uart, USART_IF_TXBL);
  USART_IntEnable(uart, USART_IF_RXDATAV);
}
//--------------------------------------------------------------------------------------------------
// Disable IRQ
void UART_RxDisable() {
  USART_IntDisable(uart, USART_IF_TXBL);
  USART_IntDisable(uart, USART_IF_RXDATAV);
}
//--------------------------------------------------------------------------------------------------
//Get count rx
uint32_t UART_GetRxCount()
{
  return UART_rxBuf.pendingBytes;
}
//--------------------------------------------------------------------------------------------------
//Recive Byte
uint8_t UART_ReciveByte()
{
  uint8_t ch;

  /* Copy data from buffer */
  taskENTER_CRITICAL();
  ch = UART_rxBuf.data[UART_rxBuf.rdI];
  UART_rxBuf.rdI = (UART_rxBuf.rdI + 1) % BUFFERSIZE_RX;
  
  UART_rxBuf.pendingBytes--;
  taskEXIT_CRITICAL();

  return ch;
}
//--------------------------------------------------------------------------------------------------
//Send Data
void UART_SendData(char* dataPtr, uint16_t dataLen)
{
  uint32_t i = 0;

  /* Check if buffer is large enough for data */
  if (dataLen > BUFFERSIZE_TX)
  {
    /* Buffer can never fit the requested amount of data */
    return;
  }

  /* Check if buffer has room for new data */
  if ((UART_txBuf.pendingBytes + dataLen) > BUFFERSIZE_TX)
  {
    /* Wait until room */
    while ((UART_txBuf.pendingBytes + dataLen) > BUFFERSIZE_TX) ;
  }

  /* Fill dataPtr[0:dataLen-1] into txBuffer */
  while (i < dataLen)
  {
    UART_txBuf.data[UART_txBuf.wrI] = *(dataPtr + i);
    UART_txBuf.wrI = (UART_txBuf.wrI + 1) % BUFFERSIZE_TX;
    i++;
  }

  /* Increment pending byte counter */
  UART_txBuf.pendingBytes += dataLen;
  
  /* Enable interrupt on USART TX Buffer*/
  USART_IntEnable(uart, USART_IF_TXBL);
}
//--------------------------------------------------------------------------------------------------

void USART1_RX_IRQHandler(void)
{
  //RX Data Valid
  if (uart->STATUS & USART_STATUS_RXDATAV)
  {
    USART_IntClear(uart, USART_STATUS_RXDATAV); //Clear status
    
    /* Copy data into RX Buffer */
    uint8_t rxData = USART_Rx( uart );
    UART_rxBuf.data[UART_rxBuf.wrI] = rxData;
    UART_rxBuf.wrI             = (UART_rxBuf.wrI + 1) % BUFFERSIZE_RX;
    UART_rxBuf.pendingBytes++;

    /* Flag Rx overflow */
    if (UART_rxBuf.pendingBytes > BUFFERSIZE_RX)
    {
      UART_rxBuf.overflow = true;
    }
    
  }
  
  NVIC_ClearPendingIRQ(USART1_RX_IRQn);
}

void USART1_TX_IRQHandler(void)
{
  //TX Complete
  if (uart->STATUS & USART_STATUS_TXBL)
  {
    if (UART_txBuf.pendingBytes > 0)
    {
      /* Transmit pending character */
      USART_Tx(uart, UART_txBuf.data[UART_txBuf.rdI]);
      UART_txBuf.rdI = (UART_txBuf.rdI + 1) % BUFFERSIZE_TX;
      UART_txBuf.pendingBytes--;
    }

    /* Disable Tx interrupt if no more bytes in queue */
    if (UART_txBuf.pendingBytes == 0)
    {
      USART_IntDisable(uart, USART_IF_TXBL);
    }

    USART_IntClear(uart, USART_STATUS_TXBL); //Clear status
  }      
  
  NVIC_ClearPendingIRQ(USART1_TX_IRQn);
}
