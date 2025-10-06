/*
 * Handles communication with CO2 and SO2 sensors
 */

#include <sens_drv.h>
#include <string.h>
#include <uart_drv.h>
#include "app_log.h"
#include "communication.h"
#include "sl_uartdrv_instances.h"

#define SENS_TIMEOUT_MS_SO2         1000 //ms
#define SENS_TIMEOUT_MS_CO2         1000 //ms
#define SENS_TIMER_PRIORITY_SO2     0
#define SENS_TIMER_PRIORITY_CO2     1

#define SENS_SERIAL_NUM             "092822010140"

typedef struct {
  char *data;
  uint8_t length;
} SENS_Cmd_Buf_t;

typedef enum{
  SENS_CMD_GET_VALUE_SO2,
  SENS_CMD_CONTINIOUS_MEAS_SO2,
  SENS_CMD_UNKNOWN,
  SENS_CMD_COUNT
} SENS_Cmd_t;


static SENS_Handle_t hSo2Config;
static SENS_Handle_t hCo2Config;
static sl_sleeptimer_timer_handle_t hSo2Timer;
static sl_sleeptimer_timer_handle_t hCo2Timer;

static sl_status_t sens_handle_so2(Comm_Msg_t *msg);
static sl_status_t sens_handle_co2(Comm_Msg_t *msg);
static SENS_Handle_t* select_device(Comm_Device_t device);
static sl_status_t parse_measured_values(Comm_Msg_t *msg, SENS_Handle_t *handle);
static sl_status_t sens_send(Comm_Device_t device, SENS_Cmd_t cmd);
static sl_status_t start_timer(Comm_Device_t device);
static sl_status_t stop_timer(Comm_Device_t device);
static void sens_timer_Callback(sl_sleeptimer_timer_handle_t *handle, void *data);


//TODO TEMP
void SENS_setState(Comm_Device_t device, SENS_Status_t status){
  if (device == COMM_DEVICE_SO2) {
      hSo2Config.currentState = status;
  }
  else if (device == COMM_DEVICE_CO2) {
      hCo2Config.currentState = status;
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


void SENS_runStateMachine(Comm_Device_t device){

  SENS_Handle_t *handle = select_device(device);
  sl_status_t timer_status;
  sl_status_t status;

  switch (handle->currentState) {

    case SENS_STATUS_IDLE:
      //do nothing
      break;


    case SENS_STATUS_SEND_REQUEST:
      //send request to sensors
      if (device == COMM_DEVICE_SO2){
          status = sens_send(device, SENS_CMD_GET_VALUE_SO2);
          timer_status = start_timer(device);

          if (status == SL_STATUS_OK){
              handle->currentState = SENS_STATUS_REQUESTED;
          }
          else{
              handle->currentState = SENS_STATUS_ERROR;
              app_log_error("error when sending request to %s sensor, error num %lu    ", handle->printType, status);
          }
      }
      else if (device == COMM_DEVICE_CO2){
          timer_status = start_timer(device);
          //I2C_Send();
      }
      else{
          handle->currentState = SENS_STATUS_ERROR;
          app_log_error("incorrect device passed    ");
      }
      break;


    case SENS_STATUS_REQUESTED:
      //waiting for answer from sensor
      //answer handled in SENS_Handler()
      break;


    case SENS_STATUS_MEAS_READY:
      //measurement ready for reading
      //waiting for lora_handler.c to copy values and mark as sent
      timer_status = stop_timer(device);

      break;


    case SENS_STATUS_ERROR:
      //TODO error handling
      app_log_error("error state on %s    ", handle->printType);
      handle->currentState = SENS_STATUS_IDLE;
      break;
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


sl_status_t sens_send(Comm_Device_t device, SENS_Cmd_t cmd){
  const SENS_Cmd_Buf_t msg = sens_get_cmd_buf[cmd];
  sl_status_t status;

  if (device == COMM_DEVICE_SO2){
      status = UART_Send(sl_uartdrv_usart_so2_handle, (uint8_t*)msg.data, msg.length);
  }
  else if (device == COMM_DEVICE_CO2){
      //send I2C msg
      status = SL_STATUS_OK;
  }
  else{
      app_log_error("invalid device passed    ");
      status = SL_STATUS_FAIL;
  }
  return status;
}


sl_status_t start_timer(Comm_Device_t device){
  sl_status_t status;
  if (device == COMM_DEVICE_SO2){
      status = sl_sleeptimer_start_periodic_timer_ms(&hSo2Timer,
                                                     SENS_TIMEOUT_MS_SO2,
                                                     sens_timer_Callback,
                                                     NULL,
                                                     SENS_TIMER_PRIORITY_SO2,
                                                     0);
  }
  else if (device == COMM_DEVICE_CO2){
      status = sl_sleeptimer_start_periodic_timer_ms(&hCo2Timer,
                                                     SENS_TIMEOUT_MS_CO2,
                                                     sens_timer_Callback,
                                                     NULL,
                                                     SENS_TIMER_PRIORITY_CO2,
                                                     0);
  }
  else{
      app_log_error("invalid device passed    ");
      status = SL_STATUS_FAIL;
  }
  return status;
}


sl_status_t stop_timer(Comm_Device_t device){
  sl_status_t status;
  if (device == COMM_DEVICE_SO2){
      status = sl_sleeptimer_stop_timer(&hSo2Timer);
  }
  else if (device == COMM_DEVICE_CO2){
      status = sl_sleeptimer_stop_timer(&hCo2Timer);
  }
  else{
      app_log_error("invalid device passed    ");
      status = SL_STATUS_FAIL;
  }
  return status;
}


//If callback received means sensor timeout
void sens_timer_Callback(sl_sleeptimer_timer_handle_t *handle, void *data){
  (void)data;

  if (handle == &hSo2Timer){
      hSo2Config.currentState = SENS_STATUS_ERROR;
      app_log_warning("SO2 response timeout    ");
  }
  else if (handle == &hCo2Timer){
      hCo2Config.currentState = SENS_STATUS_ERROR;
      app_log_warning("CO2 response timeout    ");
  }
  else {
      app_log_error("unrecognized handle    ");
  }
}


