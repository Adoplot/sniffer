/*
 * dgs2_so2.c
 *
 * Driver for SO2 sensor on DGS2 board
 */

#include "dgs2_so2.h"
#include "sl_sleeptimer.h"
#include "uart_drv.h"
#include "communication.h"
#include <string.h>

#define SENS_SERIAL_NUM             "092822010140"
#define DGS2_RESPONSE_TIMEOUT_MS    3000

typedef struct {
  char *data;
  uint8_t length;
} Dgs2_CmdBuf_t;

typedef enum{
  //SO2 commands
  DGS2_CMD__SINGLE_MEASUREMENT,
  //Other
  DGS2_CMD__UNKNOWN,
  DGS2_CMD__COUNT
} Dgs2_Cmd_t;


const Dgs2_CmdBuf_t getCmdBuf[DGS2_CMD__COUNT] = {
    { "\r",                         1}  //SINGLE_MEASUREMENT
};

static Dgs2_Config_t hSensor;
static sl_sleeptimer_timer_handle_t hTimer;


static void Dgs2_initialize();
static sl_status_t Dgs2_send(Dgs2_Cmd_t cmd);
static sl_status_t Dgs2_stopTimer(sl_sleeptimer_timer_handle_t *handle);
static void Dgs2_cbTimerResponse(sl_sleeptimer_timer_handle_t *handle, void *data);
static sl_status_t Dgs2_parseData(Comm_Msg_t *msg);


sl_status_t Dgs2_RunStateMachine(void){
  sl_status_t status = SL_STATUS_FAIL;
  uint32_t timeout = 0;

  switch(hSensor.currentState){

    case DGS2_STATE__IDLE:
      //do nothing
      if (!Dgs2_isInitialized()){
          Dgs2_initialize();
      }

      status = SL_STATUS_OK;
      break;


    case DGS2_STATE__SEND_REQUEST:
      status = Dgs2_stopTimer(&hTimer);

      if (status == SL_STATUS_OK){
          timeout = sl_sleeptimer_ms_to_tick(DGS2_RESPONSE_TIMEOUT_MS);
          status = sl_sleeptimer_start_timer(&hTimer, timeout, Dgs2_cbTimerResponse,
                                             NULL, 0, 0);
      }

      if (status == SL_STATUS_OK){
          status = Dgs2_send(DGS2_CMD__SINGLE_MEASUREMENT);
      }

      if (status == SL_STATUS_OK){
          hSensor.currentState = DGS2_STATE__REQUESTED;
      }else{
          hSensor.currentState = DGS2_STATE__ERROR;
          app_log_error("SO2 failed to send request");
      }

      break;


    case DGS2_STATE__REQUESTED:
      //waiting for answer from sensor
      //answer handled in Dgs2_Handler()
      status = SL_STATUS_OK;
      break;


    case DGS2_STATE__MEAS_READY:

      // Initialization successful
      if (hSensor.initState == DGS2_INIT__IN_PROCESS){
          hSensor.initState = DGS2_INIT__INITIALIZED;
      }

      status = Dgs2_stopTimer(&hTimer);

      if (status == SL_STATUS_OK){
          hSensor.currentState = DGS2_STATE__IDLE;
      }else{
          hSensor.currentState = DGS2_STATE__ERROR;
      }

      break;


    case DGS2_STATE__ERROR:
      status = Dgs2_stopTimer(&hTimer);

      app_log_error("SO2 general error");
      hSensor.currentState = DGS2_STATE__IDLE;
      break;
  }
  return status;
}


/***************************************************************************//**
 * @brief
 *    Handles msg received from UART driver
 ******************************************************************************/
sl_status_t Dgs2_Handler(Comm_Msg_t msg){
  sl_status_t status = SL_STATUS_FAIL;

  if (hSensor.currentState == DGS2_STATE__REQUESTED){
      status = Dgs2_parseData(&msg);
  }else{
      app_log_error("SO2 data received out of state");
  }

  if (status == SL_STATUS_OK){
      hSensor.currentState = DGS2_STATE__MEAS_READY;
  }else{
      hSensor.currentState = DGS2_STATE__ERROR;
  }
  return status;
}


void Dgs2_InitializeConfiguration(void){
  hSensor.currentState = DGS2_STATE__IDLE;
  hSensor.initState = DGS2_INIT__NOT_INITIALIZED;
  hSensor.data.so2Ppb = 0;
  hSensor.data.temp_x100 = 0;
  hSensor.data.rh_x100 = 0;
  hSensor.bufLen = 0;
}


void Dgs2_initialize(){
  hSensor.initState = DGS2_INIT__IN_PROCESS;
  hSensor.currentState = DGS2_STATE__SEND_REQUEST;
}


bool Dgs2_isInitialized(void){
  if (hSensor.initState == DGS2_INIT__INITIALIZED){
      return true;
  }else{
      return false;
  }
}


/***************************************************************************//**
 * @brief
 *    Parses So2 (ppb), T (C), RH (%) from buf to int16
 ******************************************************************************/
sl_status_t Dgs2_parseData(Comm_Msg_t *msg){
  sl_status_t status = SL_STATUS_FAIL;
  char buf[256];

  memcpy(buf, msg->data.buffer, msg->data.buf_len);
  buf[msg->data.buf_len] = '\0';

  char *token = strtok(buf, ",");

  int16_t values[4];
  uint8_t count = 0;

  while (token && count < 4) {
      values[count] = (int32_t)strtol(token, NULL, 10); //TODO value[0] has overflow, but it is not used
      count++;
      token = strtok(NULL, ",");
  }

  if (count < 4) {
      app_log_error("not enough values in SO2 buffer");
      status = SL_STATUS_FAIL; //not enough values
  }else{
      hSensor.data.so2Ppb     = values[1];
      hSensor.data.temp_x100  = values[2];
      hSensor.data.rh_x100    = values[3];
      status = SL_STATUS_OK;
  }
  return status;
}


sl_status_t Dgs2_send(Dgs2_Cmd_t cmd){
  sl_status_t status = SL_STATUS_FAIL;

  const Dgs2_CmdBuf_t msg = getCmdBuf[cmd];

  status = UART_Send(sl_uartdrv_usart_so2_handle, (uint8_t*)msg.data, msg.length);

  return status;
}


// Checks if timer is running and then stops it
//TODO make common func
sl_status_t Dgs2_stopTimer(sl_sleeptimer_timer_handle_t *handle){
  sl_status_t status = SL_STATUS_FAIL;
  bool isRunning;

  // Check if timer is running
  status = sl_sleeptimer_is_timer_running(&hTimer, &isRunning);

  if (status == SL_STATUS_OK){
      if (isRunning){
          status = sl_sleeptimer_stop_timer(handle);
      }else{
          status = SL_STATUS_OK;
          app_log_warning("SO2: tried to stop inactive timer");
      }
  }

  if (status == SL_STATUS_FAIL){
      app_log_error("SO2: failed to stop timer");
  }
  return status;
}


// If cb occurred then sensor is not responding to requests
void Dgs2_cbTimerResponse(sl_sleeptimer_timer_handle_t *handle, void *data){
  (void)data;
  (void)handle;

  hSensor.currentState = DGS2_STATE__ERROR;
  app_log_warning("SO2 response timeout");

}
