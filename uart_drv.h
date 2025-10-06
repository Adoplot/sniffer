#ifndef UART_DRV_H_
#define UART_DRV_H_

#include "uartdrv.h"
#include "sl_uartdrv_instances.h"


#define UART_DATA_BUF_SIZE 256
#define UART_RX_BUF_SIZE 1

typedef enum{
  UART_STATE_IDLE,
  UART_STATE_SEND,
  UART_STATE_TX_PENDING,
  UART_STATE_RX_PENDING,
  UART_STATE_SUCCESS,
  UART_STATE_END,
  UART_STATE_ERROR
} Uart_State_t;

typedef enum{
  UART_TYPE_RPI,
  UART_TYPE_LORA,
  UART_TYPE_SO2
} Uart_Type_t;

typedef struct{
  Uart_State_t currentState; //Current state of the UART instance
  Uart_Type_t type;
  char printType[5];    //RPI,LORA,SO2 strings for logging
  uint8_t index;        //Current index of rxDataBuf to construct msg from received bytes
  uint8_t rxByte[UART_RX_BUF_SIZE];
  uint8_t rxDataBuf[UART_DATA_BUF_SIZE];
  uint8_t rxBuf_len;
  uint8_t txDataBuf[UART_DATA_BUF_SIZE];
  uint8_t txBuf_len;
} Uart_Handle_t;


void UART_Init_State_Handles();

sl_status_t UART_runStateMachine(UARTDRV_Handle_t handle);

sl_status_t UART_Send(struct UARTDRV_HandleData *handle, uint8_t *buf,
                      uint8_t buf_len);



#endif /* UART_DRV_H_ */
