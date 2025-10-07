#ifndef RPI_DRV_H_
#define RPI_DRV_H_

#include "communication.h"
#include "uart_drv.h"


typedef enum{
  RPI_STATUS_NOT_INITIALIZED,
  RPI_STATUS_IDLE,
  RPI_STATUS_MEASURING,
  RPI_STATUS_MEAS_READY,
  RPI_STATUS_SENDING,
  RPI_STATUS_SENT,
  RPI_STATUS_ERROR,
} RPI_Status_t;

typedef struct{
  RPI_Status_t currentState;
  uint8_t buffer[256];
  uint8_t buf_len;
  uint8_t gps_buf[256];
} RPI_Handle_t;


//Interfaces
RPI_Status_t RPI_get_rpi_status();
void RPI_set_rpi_status(RPI_Status_t status);


void RPI_Init_State_Handles();
sl_status_t RPI_Handler(Comm_Msg_t msg, uint8_t *pBuf, uint8_t *pBufLen);
sl_status_t RPI_runStateMachine();
void Rpi_SetStatusIdle(void);

#endif /* RPI_DRV_H_ */
