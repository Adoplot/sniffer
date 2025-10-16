/*
 * Handles communication with CO2 and SO2 sensors
 */

#include <sens_drv.h>
#include "scd41_co2.h"
#include <string.h>
#include <uart_drv.h>
#include <stdbool.h>
#include "app_log.h"
#include "communication.h"
#include "sl_uartdrv_instances.h"

#define SENS_TIMEOUT_MS_SO2         3000 //ms
#define SENS_TIMEOUT_MS_CO2         3000 //ms
#define SENS_TIMER_PRIORITY_SO2     0
#define SENS_TIMER_PRIORITY_CO2     1



typedef struct {
  char *data;
  uint8_t length;
} SENS_Cmd_Buf_t;

typedef enum{
  //SO2 commands
  SENS_CMD_GET_VALUE_SO2,
  //Other
  SENS_CMD_UNKNOWN,
  SENS_CMD_COUNT
} SENS_Cmd_t;


static SENS_Handle_t hSo2Config;
static SENS_Handle_t hCo2Config;
static sl_sleeptimer_timer_handle_t hMeasTimerSo2;
static sl_sleeptimer_timer_handle_t hMeasTimerCo2;
static sl_sleeptimer_timer_handle_t hInitTimerSo2;
static sl_sleeptimer_timer_handle_t hInitTimerCo2;

static void           Sens_initialize       (SENS_Handle_t *handle);
static sl_status_t    Sens_handleSendRequest(SENS_Handle_t *pHandle);
static sl_status_t    sens_handle_so2       (Comm_Msg_t *msg);
static sl_status_t    sens_handle_co2       (Comm_Msg_t *msg);
static SENS_Handle_t* select_device         (Comm_Device_t device);
static sl_status_t    parse_measured_values (Comm_Msg_t *msg, SENS_Handle_t *handle);
static sl_status_t    sens_send             (SENS_Handle_t *pHandle, SENS_Cmd_t cmd);
static sl_status_t    Sens_startMeasTimer   (SENS_Type_t type);
static sl_status_t    Sens_stopMeasTimer    (SENS_Type_t type);
static sl_status_t    Sens_startInitTimer   (SENS_Type_t type);
static sl_status_t    Sens_stopInitTimer    (SENS_Type_t type);
static void           Sens_cbMeasTimer      (sl_sleeptimer_timer_handle_t *handle, void *data);
static void           Sens_cbInitTimer      (sl_sleeptimer_timer_handle_t *handle, void *data);


//TODO TEMP
void SENS_setState(Comm_Device_t device, SENS_Status_t state){
  if (device == COMM_DEVICE_SO2) {
      hSo2Config.currentState = state;
  }
  else if (device == COMM_DEVICE_CO2) {
      hCo2Config.currentState = state;
  }
  else{
      app_log_error("incorrect device passed    ");
  }
}


const SENS_Cmd_Buf_t sens_get_cmd_buf[SENS_CMD_COUNT] = {
    { "\r",                         1}
};


SENS_Handle_t* select_device(Comm_Device_t device) {
  if (device == COMM_DEVICE_SO2) {
      return &hSo2Config;
  }
  if (device == COMM_DEVICE_CO2) {
      return &hCo2Config;
  }
  app_log_error("incorrect device passed    ");
  return NULL;
}


sl_status_t SENS_runStateMachine(Comm_Device_t device){

  SENS_Handle_t *handle = select_device(device);
  sl_status_t timer_status = SL_STATUS_OK;
  sl_status_t status = SL_STATUS_FAIL;

  switch (handle->currentState) {

    case SENS_STATUS_IDLE:
      //check if sensor works correctly
      if (handle->initState == SENS_INIT_NOT_INITIALIZED){
          Sens_initialize(handle);
      }
      //do nothing
      status = SL_STATUS_OK;
      break;


    case SENS_STATUS_SEND_REQUEST:
      //send request to sensors
      status = Sens_handleSendRequest(handle);

      break;


    case SENS_STATUS_REQUESTED:
      //waiting for answer from sensor
      //answer handled in SENS_Handler()
      status = SL_STATUS_OK;
      break;


    case SENS_STATUS_MEAS_READY:
      //measurement ready for reading
      //waiting for lora_handler.c to copy values and mark as sent

      if (handle->initState == SENS_INIT_IN_PROCESS){
          //Initialization successful
          handle->initState = SENS_INIT_INITIALIZED;
          timer_status = Sens_stopInitTimer(handle->type);
          handle->currentState = SENS_STATUS_IDLE;
      }
      else{
          timer_status = Sens_stopMeasTimer(handle->type);
      }
      status = SL_STATUS_OK;
      break;


    case SENS_STATUS_ERROR:
      //TODO error handling
      app_log_error("error state on %s    ", handle->printType);
      handle->currentState = SENS_STATUS_IDLE;
      status = SL_STATUS_OK;
      break;
  }

  if (timer_status == SL_STATUS_FAIL){
      handle->currentState = SENS_STATUS_ERROR;
      app_log_error("timer failure on %s    ", handle->printType);
  }


  if (timer_status == SL_STATUS_FAIL) {
      status = timer_status;
  }
  return status;
}


sl_status_t Sens_handleSendRequest(SENS_Handle_t *pHandle){
  sl_status_t status = SL_STATUS_FAIL;
  sl_status_t timer_status = SL_STATUS_OK;  //OK, because timer can be skipped

  // Set timer if it is not init send request (different timer applies)
  if (pHandle->initState != SENS_INIT_IN_PROCESS){
      timer_status = Sens_startMeasTimer(pHandle->type);
  }

  switch (pHandle->type){

    case SENS_TYPE_SO2:
      status = sens_send(pHandle, SENS_CMD_GET_VALUE_SO2);

      if (status == SL_STATUS_OK){
          pHandle->currentState = SENS_STATUS_REQUESTED;
      }else{
          pHandle->currentState = SENS_STATUS_ERROR;
          app_log_error("error when sending request to %s sensor, error num %lu    ",
                        pHandle->printType, status);
      }
      break;


    case SENS_TYPE_CO2:
      //TODO sens_send(I2C);
      status = SL_STATUS_OK;
      break;
  }


  if (timer_status != SL_STATUS_OK){
      app_log_error("meas timer error");
      status = timer_status;
  }

  return status;
}


void Sens_initialize(SENS_Handle_t *handle){
  sl_status_t status = SL_STATUS_FAIL;

  handle->initState = SENS_INIT_IN_PROCESS;

  status = Sens_startInitTimer(handle->type);

  if (status == SL_STATUS_OK){
      handle->currentState = SENS_STATUS_SEND_REQUEST;
  }else{
      handle->currentState = SENS_STATUS_ERROR;
  }
}


bool SENS_isSensorInitialized(Comm_Device_t device){
  SENS_Handle_t *handle = select_device(device);

  if (handle->initState == SENS_INIT_INITIALIZED){
      return true;
  }
  else{
      return false;
  }
}


void Sens_GetSensorData(void){

}


bool SENS_isMeasReady(Comm_Device_t device){
  SENS_Handle_t *handle = select_device(device);

  if (handle->currentState == SENS_STATUS_MEAS_READY){
      return true;
  }
  else{
      return false;
  }
}


void SENS_Init_State_Handles(){
  hSo2Config.currentState = SENS_STATUS_IDLE;
  hSo2Config.initState = SENS_INIT_NOT_INITIALIZED;
  hSo2Config.type = SENS_TYPE_SO2;
  hSo2Config.buf_len = 0;
  hSo2Config.xO2_value = 0;
  hSo2Config.temperature = 0;
  hSo2Config.humidity = 0;
  strcpy(hSo2Config.printType, "SO2");

  hCo2Config.currentState = SENS_STATUS_IDLE;
  hCo2Config.initState = SENS_INIT_NOT_INITIALIZED;
  hCo2Config.type = SENS_TYPE_CO2;
  hCo2Config.buf_len = 0;
  hCo2Config.xO2_value = 0;
  hCo2Config.temperature = 0;
  hCo2Config.humidity = 0;
  strcpy(hCo2Config.printType, "CO2");
  //TODO
  //Dgs2_InitializeConfiguration();

  Scd41_InitializeConfiguration();
}



sl_status_t SENS_Handler(Comm_Msg_t msg){
  sl_status_t status;

  if (msg.device == COMM_DEVICE_SO2){
      status = sens_handle_so2(&msg);
      if (!status){
          hSo2Config.currentState = SENS_STATUS_MEAS_READY;
      }
      else{
          hSo2Config.currentState = SENS_STATUS_ERROR;
      }
  }
  else if (msg.device == COMM_DEVICE_CO2){
      //handle CO2 via i2c
      status = sens_handle_co2(&msg);
      if (!status){
          hCo2Config.currentState = SENS_STATUS_MEAS_READY;
      }
      else{
          hCo2Config.currentState = SENS_STATUS_ERROR;
      }
  }
  else{
      app_log_error("invalid device passed    ");
      status = SL_STATUS_FAIL;
  }
  return status;
}


sl_status_t sens_handle_so2(Comm_Msg_t *msg){
  sl_status_t status;

  switch (hSo2Config.currentState){
    case SENS_STATUS_REQUESTED:
      status = parse_measured_values(msg, &hSo2Config);
      hSo2Config.currentState = SENS_STATUS_MEAS_READY;
      return status;
    default:
      app_log_error("sensor data received out of context    ");
      break;
  }

  return SL_STATUS_OK;
}


sl_status_t sens_handle_co2(Comm_Msg_t *msg){
  (void)msg;
  return SL_STATUS_FAIL;
}


sl_status_t parse_measured_values(Comm_Msg_t *msg, SENS_Handle_t *handle){
  char buf[256];

  memcpy(buf, msg->data.buffer, msg->data.buf_len);
  buf[msg->data.buf_len] = '\0';

  char *token = strtok(buf, ",");

  int32_t values[4];
  uint8_t count = 0;

  while (token && count < 4) {
          values[count] = (int32_t)strtol(token, NULL, 10); //TODO value[0] has overflow
          count++;
          token = strtok(NULL, ",");
      }

  if (count < 4) {
      app_log_error("not enough values in %s buffer    ", handle->printType);
          return SL_STATUS_FAIL; //not enough values
      }

  handle->xO2_value   = values[1];
  handle->temperature = values[2];
  handle->humidity    = values[3];

  return SL_STATUS_OK;
}


sl_status_t sens_send(SENS_Handle_t *pHandle, SENS_Cmd_t cmd){
  const SENS_Cmd_Buf_t msg = sens_get_cmd_buf[cmd];
  sl_status_t status = SL_STATUS_FAIL;

  switch (pHandle->type){

    case SENS_TYPE_SO2:
      status = UART_Send(sl_uartdrv_usart_so2_handle, (uint8_t*)msg.data, msg.length);
      break;

    case SENS_TYPE_CO2:
      //TODO send I2C msg
      status = SL_STATUS_OK;
      break;
  }

  return status;
}


sl_status_t Sens_startMeasTimer(SENS_Type_t type){
  sl_status_t status = SL_STATUS_FAIL;
  uint32_t timeoutTicks;

  switch (type){
    case SENS_TYPE_SO2:
      timeoutTicks = sl_sleeptimer_ms_to_tick(SENS_TIMEOUT_MS_SO2);
      status = sl_sleeptimer_start_timer(&hMeasTimerSo2,
                                         timeoutTicks,
                                         Sens_cbMeasTimer,
                                         NULL,
                                         SENS_TIMER_PRIORITY_SO2,
                                         0);
      break;

    case SENS_TYPE_CO2:
      timeoutTicks = sl_sleeptimer_ms_to_tick(SENS_TIMEOUT_MS_CO2);
      status = sl_sleeptimer_start_timer(&hMeasTimerCo2,
                                         timeoutTicks,
                                         Sens_cbMeasTimer,
                                         NULL,
                                         SENS_TIMER_PRIORITY_CO2,
                                         0);
      break;
  }
  return status;
}


sl_status_t Sens_startInitTimer(SENS_Type_t type){
  sl_status_t status = SL_STATUS_FAIL;
  uint32_t timeoutTicks;

  switch (type){
    case SENS_TYPE_SO2:
      timeoutTicks = sl_sleeptimer_ms_to_tick(SENS_TIMEOUT_MS_SO2);
      status = sl_sleeptimer_start_timer(&hInitTimerSo2,
                                         timeoutTicks,
                                         Sens_cbInitTimer,
                                         NULL,
                                         SENS_TIMER_PRIORITY_SO2,
                                         0);
      break;

    case SENS_TYPE_CO2:
      timeoutTicks = sl_sleeptimer_ms_to_tick(SENS_TIMEOUT_MS_CO2);
      status = sl_sleeptimer_start_timer(&hInitTimerCo2,
                                         timeoutTicks,
                                         Sens_cbInitTimer,
                                         NULL,
                                         SENS_TIMER_PRIORITY_CO2,
                                         0);
      break;
  }
  return status;
}


sl_status_t Sens_stopInitTimer(SENS_Type_t type){
  sl_status_t status = SL_STATUS_FAIL;

  switch (type){
    case SENS_TYPE_SO2:
      status = sl_sleeptimer_stop_timer(&hInitTimerSo2);
      break;

    case SENS_TYPE_CO2:
      status = sl_sleeptimer_stop_timer(&hInitTimerCo2);
          break;
  }
  return status;
}


sl_status_t Sens_stopMeasTimer(SENS_Type_t type){
  sl_status_t status = SL_STATUS_FAIL;

  switch (type){
    case SENS_TYPE_SO2:
      status = sl_sleeptimer_stop_timer(&hMeasTimerSo2);
      break;

    case SENS_TYPE_CO2:
      status = sl_sleeptimer_stop_timer(&hMeasTimerCo2);
          break;
  }
  return status;
}


//If callback received means sensor timeout
void Sens_cbMeasTimer(sl_sleeptimer_timer_handle_t *handle, void *data){
  (void)data;

  if (handle == &hMeasTimerSo2){
      hSo2Config.currentState = SENS_STATUS_ERROR;
      app_log_warning("SO2 response timeout    ");
  }
  else if (handle == &hMeasTimerCo2){
      hCo2Config.currentState = SENS_STATUS_ERROR;
      app_log_warning("CO2 response timeout    ");
  }
  else {
      app_log_error("unrecognized handle    ");
  }
}


/**************************************************************************//**
 * If sensor is not initialized before timer ends, then error is issued and
 * init state is set to NOT_INITIALIZED
 *****************************************************************************/
void Sens_cbInitTimer(sl_sleeptimer_timer_handle_t *handle, void *data){
  (void)data;

    if (handle == &hInitTimerSo2){
        if (hSo2Config.initState != SENS_INIT_INITIALIZED){

            hSo2Config.initState = SENS_INIT_NOT_INITIALIZED;
            hSo2Config.currentState = SENS_STATUS_ERROR;
            app_log_warning("SO2 init timeout    ");
        }
    }
    else if (handle == &hInitTimerCo2){
        if (hCo2Config.initState != SENS_INIT_INITIALIZED){

            hCo2Config.initState = SENS_INIT_NOT_INITIALIZED;
            hCo2Config.currentState = SENS_STATUS_ERROR;
            app_log_warning("CO2 init timeout    ");
        }
    }
    else {
        app_log_error("unrecognized handle    ");
    }
  }

