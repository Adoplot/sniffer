#include <rpi_drv.h>
#include <sens_drv.h>
#include <string.h>
#include <uart_drv.h>
#include "uartdrv.h"
#include "sl_uartdrv_instances.h"
#include "app_log.h"
#include "communication.h"

#define UART_RESEND_TRIES_NUM   3   //how many times uart tries to resend (if failed) before setting error state

static Uart_Handle_t  rpi_handle, lora_handle, so2_handle;

static Uart_Handle_t* select_state_handle(struct UARTDRV_HandleData *handle);
static sl_status_t call_handler(Uart_Handle_t *stateHandle, uint8_t *buf, uint8_t *buf_len);
static void uart_TxCallback(struct UARTDRV_HandleData *handle, Ecode_t transferStatus,
                            uint8_t *data, UARTDRV_Count_t transferCount);
static void uart_RxCallback(struct UARTDRV_HandleData *handle, Ecode_t transferStatus,
                            uint8_t *data, UARTDRV_Count_t transferCount);
static sl_status_t send_buffer(struct UARTDRV_HandleData *handle, uint8_t *buf,
                               uint8_t buf_len);
static void process_Rx_data(Uart_Handle_t *stateHandle, uint8_t *data);


void UART_Init_State_Handles(){
  rpi_handle.currentState = UART_STATE_IDLE;
  rpi_handle.type = UART_TYPE_RPI;
  rpi_handle.index = 0;
  rpi_handle.rxBuf_len = 0;
  rpi_handle.txBuf_len = 0;
  strcpy(rpi_handle.printType, "RPI");

  lora_handle.currentState = UART_STATE_IDLE;
  lora_handle.type = UART_TYPE_LORA;
  lora_handle.index = 0;
  lora_handle.rxBuf_len = 0;
  lora_handle.txBuf_len = 0;
  strcpy(lora_handle.printType, "LORA");

  so2_handle.currentState = UART_STATE_IDLE;
  so2_handle.type = UART_TYPE_SO2;
  so2_handle.index = 0;
  so2_handle.rxBuf_len = 0;
  so2_handle.txBuf_len = 0;
  strcpy(so2_handle.printType, "SO2");
}



Uart_Handle_t* select_state_handle(struct UARTDRV_HandleData *handle) {
  if (handle == sl_uartdrv_eusart_rpi_handle) {
      return &rpi_handle;
  }
  if (handle == sl_uartdrv_eusart_lora_handle) {
      return &lora_handle;
  }
  if (handle == sl_uartdrv_usart_so2_handle) {
      return &so2_handle;
  }
  return NULL;
}


//TODO return meaningful status?
sl_status_t UART_runStateMachine(UARTDRV_Handle_t handle){

  sl_status_t call_handler_status;
  sl_status_t status;
  static uint8_t send_counter = 0;  //counts UART_STATE_SEND attempts (if failed 3 times -> UART_STATE_ERROR)

  Uart_Handle_t *stateHandle = select_state_handle(handle);

  if (!stateHandle) {
      //app_log_error("incorrect handle passed    ");
      return SL_STATUS_FAIL;
  }

  switch(stateHandle->currentState){

    case UART_STATE_IDLE:
      //do nothing
      break;


    case UART_STATE_SEND:
      //initiate sending
      status = send_buffer(handle, stateHandle->txDataBuf, stateHandle->txBuf_len);
      if (status == SL_STATUS_OK){
          send_counter = 0;
      }
      else if (send_counter < UART_RESEND_TRIES_NUM){
          //Retry sending 3 times
          stateHandle->currentState = UART_STATE_SEND;
          send_counter++;
          app_log_warning("send failed, retry sending. Try count = %d    ", send_counter);
      }
      else{
          stateHandle->currentState = UART_STATE_ERROR;
          app_log_error("send failed %d times, setting error state    ", send_counter);
          send_counter = 0;
      }
      break;


    case UART_STATE_TX_PENDING:
      //Waiting for TX callback
      break;


    case UART_STATE_RX_PENDING:
      //Waiting for RX callback
      break;


    case UART_STATE_SUCCESS:
      // buffer is ready to read
      // call corresponding handler for received cmd
      call_handler_status = call_handler(stateHandle, stateHandle->txDataBuf, &stateHandle->txBuf_len);

      if (call_handler_status){
          stateHandle->currentState = UART_STATE_ERROR;
      }
      //clear rxBuf (needed for rpi immediate reply flow)
      stateHandle->index = 0;
      stateHandle->rxBuf_len = 0;
      memset(stateHandle->rxDataBuf,'\0', UART_DATA_BUF_SIZE);
      memset(stateHandle->rxByte,'\0', UART_RX_BUF_SIZE);
      //TODO make it go through UART_STATE_END?
      break;


    case UART_STATE_ERROR:
          stateHandle->currentState = UART_STATE_END;
          app_log_error("uart error on: %s    ", stateHandle->printType);
          break;

    case UART_STATE_END:
      // Cleanup code
      // Resetting buffers and indexes
      stateHandle->index = 0;
      stateHandle->rxBuf_len = 0;
      stateHandle->txBuf_len = 0;
      memset(stateHandle->rxDataBuf,'\0', UART_DATA_BUF_SIZE);
      memset(stateHandle->rxByte,'\0', UART_RX_BUF_SIZE);
      memset(stateHandle->txDataBuf,'\0', UART_DATA_BUF_SIZE);

      //Only for RPI, to enable recv after receiving invalid cmd
      if (stateHandle->type == UART_TYPE_RPI){
          UARTDRV_Receive(handle, stateHandle->rxByte, 1, uart_RxCallback);
          stateHandle->currentState = UART_STATE_RX_PENDING;
      }else{
          stateHandle->currentState = UART_STATE_IDLE;
      }
      break;
  }

  return SL_STATUS_OK;
}


sl_status_t call_handler(Uart_Handle_t *stateHandle, uint8_t *buf, uint8_t *buf_len){
  Comm_Msg_t msg;
  sl_status_t status = SL_STATUS_FAIL;

  switch (stateHandle->type){

    case UART_TYPE_RPI:
      // Copy buffer into msg and pass msg by value
      memcpy(msg.data.buffer, stateHandle->rxDataBuf, stateHandle->rxBuf_len);
      msg.data.buf_len = stateHandle->rxBuf_len;
      msg.device = COMM_DEVICE_RPI;

      status = RPI_Handler(msg, buf, buf_len);

      if (!status){
          stateHandle->currentState = UART_STATE_SEND;  //send reply immediately
      }else{
          stateHandle->currentState = UART_STATE_ERROR;
      }
      break;


    case UART_TYPE_LORA:
      //status = Lora_Handler();
      status = SL_STATUS_OK; //TODO
      break;


    case UART_TYPE_SO2:
      memcpy(msg.data.buffer, stateHandle->rxDataBuf, stateHandle->rxBuf_len);
      msg.data.buf_len = stateHandle->rxBuf_len;
      msg.device = COMM_DEVICE_SO2;

      status = SENS_Handler(msg);  //TODO
      if (!status){
          stateHandle->currentState = UART_STATE_END;
      }else{
          stateHandle->currentState = UART_STATE_ERROR;
      }
      status = SL_STATUS_OK;
      break;

  }
  return status;
}


bool isTxAvailable(Uart_Handle_t *stateHandle){
  if (stateHandle->currentState == UART_STATE_IDLE){
      return true; }
  else{
      return false; }
}


sl_status_t UART_Send(struct UARTDRV_HandleData *handle, uint8_t *buf,
                      uint8_t buf_len){
  Uart_Handle_t *stateHandle = select_state_handle(handle);
  memcpy(stateHandle->txDataBuf, buf, buf_len);
  stateHandle->txBuf_len = buf_len;

  if (isTxAvailable(stateHandle)){
      stateHandle->currentState = UART_STATE_SEND;
      return SL_STATUS_OK;
  }
  else{
      return SL_STATUS_TRANSMIT_BUSY;
  }
}


sl_status_t send_buffer(struct UARTDRV_HandleData *handle, uint8_t *buf,
                        uint8_t buf_len){

  Uart_Handle_t *stateHandle = select_state_handle(handle);

  if (!stateHandle) {
      app_log_error("incorrect handle passed    ");
      return SL_STATUS_FAIL;
  }

  stateHandle->currentState = UART_STATE_TX_PENDING;

  Ecode_t tx_status;
  tx_status = UARTDRV_Transmit(handle, buf, (UARTDRV_Count_t)buf_len, uart_TxCallback);

  if (tx_status == ECODE_EMDRV_UARTDRV_OK){
      return SL_STATUS_OK;
  }
  else{
      return SL_STATUS_FAIL;
  }
}


void uart_TxCallback(struct UARTDRV_HandleData *handle, Ecode_t transferStatus,
                     uint8_t *data, UARTDRV_Count_t transferCount){
  (void)data;
  (void)transferCount;

  Uart_Handle_t *stateHandle = select_state_handle(handle);

  if (!stateHandle) {
      app_log_error("incorrect handle passed    ");
  }
  else{
      if (transferStatus == ECODE_EMDRV_UARTDRV_OK){
          //Start receiving
          Ecode_t rx_status;
          rx_status = UARTDRV_Receive(handle, stateHandle->rxByte, 1, uart_RxCallback);

          if (rx_status == ECODE_EMDRV_UARTDRV_OK){
              stateHandle->currentState = UART_STATE_RX_PENDING;
          }
          else{
              stateHandle->currentState = UART_STATE_ERROR;
              app_log_error("recv error on %s, transferStatus: %d    ",
                            stateHandle->printType, (int)rx_status);
          }
      }
  }
}


void uart_RxCallback(struct UARTDRV_HandleData *handle, Ecode_t transferStatus,
                     uint8_t *data, UARTDRV_Count_t transferCount){
  (void)transferCount;
  Uart_Handle_t *stateHandle = select_state_handle(handle);

  if (!stateHandle) {
      app_log_error("incorrect handle passed    ");
  }
  else{

      if (transferStatus == ECODE_EMDRV_UARTDRV_OK){

          process_Rx_data(stateHandle, data);
      }
      else{
          app_log_error("recv fail on %s, transferStatus: %d    ",
                        stateHandle->printType, (int)transferStatus);
          stateHandle->currentState = UART_STATE_ERROR;
      }

      // Continue receiving bytes until transfer aborted
      if ((transferStatus != ECODE_EMDRV_UARTDRV_ABORTED)
          && (stateHandle->currentState == UART_STATE_RX_PENDING)){

          UARTDRV_Receive(handle, stateHandle->rxByte, 1, uart_RxCallback);
      }

  }
}


/***************************************************************************//**
 * @brief
 *    Processes received buffer and saves it.
 *    Due to different message limiters (\r\n) in every device, specific
 *    processing is required.
 ******************************************************************************/
void process_Rx_data(Uart_Handle_t *stateHandle, uint8_t *data){
  uint8_t *index = &stateHandle->index;

  switch (stateHandle->type){

    case UART_TYPE_RPI:

      //Null-terminate if end of msg received
      if (*data == '\n') {
          stateHandle->rxBuf_len = *index;
          stateHandle->currentState = UART_STATE_SUCCESS;
          app_log("%s recv message: %s    ", stateHandle->printType, stateHandle->rxDataBuf);
      }
      else {
          if (*index < UART_DATA_BUF_SIZE - 1) {
              stateHandle->rxDataBuf[*index] = *data;
              //app_log_append("%c", *data);
              (*index)++;
          } else {
              //reset if overflow
              *index = 0;
              stateHandle->currentState = UART_STATE_ERROR;
              app_log_error("recv buffer overflow on %s, resetting index    ", stateHandle->printType);
          }
      }
      break;



    case UART_TYPE_LORA:
      break;


    case UART_TYPE_SO2:

      //Null-terminate if end of msg received
      if (*data == '\n') {
          stateHandle->rxBuf_len = *index;
          stateHandle->currentState = UART_STATE_SUCCESS;
          app_log("%s recv message: %s    ", stateHandle->printType, stateHandle->rxDataBuf);
      }
      else {
          if (*index < UART_DATA_BUF_SIZE - 1) {
              stateHandle->rxDataBuf[*index] = *data;
              //app_log_append("%c", *data);
              (*index)++;
          } else {
              //reset if overflow
              *index = 0;
              stateHandle->currentState = UART_STATE_ERROR;
              app_log_error("recv buffer overflow on %s, resetting index    ", stateHandle->printType);
          }
      }
      break;

  }
}

