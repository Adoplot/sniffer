/*
 * Handles communication with Raspberry Pi 5
 */

#include <lora_drv.h>
#include <rpi_drv.h>
#include <sens_drv.h>
#include "app_log.h"
#include <string.h>
#include <uart_drv.h>


RPI_Handle_t hRpiConfig;


typedef struct {
  char *data;
  uint8_t length;
} Rpi_CmdBuf_t;

typedef enum{
  RPI_CMD_INIT_CHECK,
  RPI_CMD_INIT_IN_PROCESS,
  RPI_CMD_INIT_READY,
  RPI_CMD_INIT_ERR,
  RPI_CMD_START,
  RPI_CMD_START_ACK,
  RPI_CMD_START_ERR,
  RPI_CMD_START_MEAS_IN_PROCESS,
  RPI_CMD_START_MEAS_SENT,
  RPI_CMD_STOP,
  RPI_CMD_STOP_ACK,
  RPI_CMD_STOP_ERR,
  RPI_CMD_UNKNOWN,
  RPI_CMD_COUNT
} Rpi_Cmd_t;

typedef struct {
  Rpi_Cmd_t cmd;
  Rpi_CmdBuf_t cmd_data;
} Rpi_Msg_t;


static sl_status_t rpi_send(Rpi_Cmd_t cmd);
static bool isAllInitialized();
static sl_status_t handle_init();


const Rpi_CmdBuf_t rpi_get_cmd_buf[RPI_CMD_COUNT] = {
    { "init_check\n",             11 },
    { "init_in_process\n",        16 },
    { "init_ready\n",             11 },
    { "init_err\n",               9 },
    { "start\n",                  6 },
    { "start_ack\n",              10 },
    { "start_err\n",              10 },
    { "start_meas_in_process\n",  22 },
    { "start_meas_sent\n",        16 },
    { "stop\n",                   5 },
    { "stop_ack\n",               9 },
    { "stop_err\n",               9 }
};


sl_status_t RPI_runStateMachine(){
  sl_status_t status;

  switch (hRpiConfig.currentState){

    case RPI_STATUS_NOT_INITIALIZED:
      //wait for init cmd from rpi
      //is handled in RPI_Handler()
      status = SL_STATUS_OK;
      break;

    case RPI_STATUS_IDLE:
      //do nothing
      //wait for start cmd from rpi
      status = SL_STATUS_OK;
      break;


    case RPI_STATUS_MEASURING:
      //check if both measurements are ready
      //TODO check both sensors
#if 0
      if ((SENS_isMeasReady(COMM_DEVICE_SO2)) &&
          (SENS_isMeasReady(COMM_DEVICE_CO2))) {
          hRpiConfig.currentState = RPI_STATUS_MEAS_READY;
      }
#else
      if (SENS_isMeasReady(COMM_DEVICE_SO2)) {
          hRpiConfig.currentState = RPI_STATUS_MEAS_READY;
      }
#endif
      status = SL_STATUS_OK;
      break;


    case RPI_STATUS_MEAS_READY:
      //
      status = SL_STATUS_OK;
      break;

    case RPI_STATUS_SENDING:
      //do nothing
      status = SL_STATUS_OK;
      break;


    case RPI_STATUS_SENT:
      //do nothing
      status = SL_STATUS_OK;
      break;


    case RPI_STATUS_ERROR:
      hRpiConfig.currentState = RPI_STATUS_IDLE;
      app_log_error("RPI error");
      status = SL_STATUS_OK;
      break;

  }
  return status;
}




void RPI_Init_State_Handles(){
  hRpiConfig.currentState = RPI_STATUS_NOT_INITIALIZED;
}


/***************************************************************************//**
 * @brief
 *    Checks if all sensors and the lora module are initialized
 * @return
 *    @ref true/false
 ******************************************************************************/
bool isAllInitialized(){
  if (SENS_isSensorInitialized(COMM_DEVICE_SO2) &&
      SENS_isSensorInitialized(COMM_DEVICE_CO2) &&
      LORA_isInitialized()) {

      return true;
  }
  else{
      return false;
  }
}


/***************************************************************************//**
 * @brief
 *    Checks if all sensors and the lora module are initialized
 ******************************************************************************/
bool isMeasReady(){

  if (SENS_isSensorInitialized(COMM_DEVICE_SO2) &&
      SENS_isSensorInitialized(COMM_DEVICE_CO2) &&
      LORA_isInitialized()) {

      return true;
  }
  else{
      return false;
  }
}



/***************************************************************************//**
 * @brief
 *    Checks what cmd is in buffer
 * @return
 *    @ref cmd
 ******************************************************************************/
Rpi_Msg_t parse_rpi_cmd(const uint8_t *buf, const uint8_t buf_len) {
  Rpi_Msg_t msg;
  msg.cmd = RPI_CMD_UNKNOWN;
  msg.cmd_data.data = NULL;
  msg.cmd_data.length = 0;

  //Null-terminate copy of the buffer
  char temp[256];

  memcpy(temp, buf, buf_len);
  temp[buf_len] = '\0';

  //get pointer to delimiter
  char *delimiter = strchr(temp, ':');

  if (delimiter == NULL) {
      if (strncmp((char*)buf, "stop", buf_len) == 0){
          msg.cmd = RPI_CMD_STOP;
      }
      else if (strncmp((char*)buf, "init_check", buf_len) == 0){
          msg.cmd = RPI_CMD_INIT_CHECK;
      }
      else{
        app_log_warning("unrecognized cmd when parsing on RPI");
      }
  }else{
      *delimiter = '\0';
      char *cmd = temp;
      char *gps_data = delimiter + 1;

      if (strcmp(cmd, "start") == 0) {
          msg.cmd = RPI_CMD_START;
          msg.cmd_data.length = strlen(gps_data);
          msg.cmd_data.data = malloc(msg.cmd_data.length + 1);
          if (msg.cmd_data.data) {
              strcpy(msg.cmd_data.data, gps_data);
          }
      }
  }

  return msg;
}


/***************************************************************************//**
 * @brief
 *    Handles cmd received from RPI
 * @return
 *    @ref SL_STATUS_OK
 ******************************************************************************/
sl_status_t RPI_Handler(Comm_Msg_t msg){
  Rpi_Msg_t rpi_msg;
  sl_status_t status = SL_STATUS_FAIL;

  rpi_msg = parse_rpi_cmd(msg.data.buffer, msg.data.buf_len);

  switch (rpi_msg.cmd){
    case RPI_CMD_START:
      //Start measuring

      if (hRpiConfig.currentState == RPI_STATUS_IDLE){

          //Request measurement from sensors
          SENS_setState(COMM_DEVICE_SO2, SENS_STATUS_SEND_REQUEST);
          SENS_setState(COMM_DEVICE_CO2, SENS_STATUS_SEND_REQUEST);

          rpi_send(RPI_CMD_START_ACK);

          //Copy cmd buf to gps data
          memcpy(hRpiConfig.gps_buf, rpi_msg.cmd_data.data, rpi_msg.cmd_data.length);

          hRpiConfig.currentState = RPI_STATUS_MEASURING;
          status = SL_STATUS_OK;

      }else if (hRpiConfig.currentState == RPI_STATUS_MEASURING ||
          hRpiConfig.currentState == RPI_STATUS_MEAS_READY ||
          hRpiConfig.currentState == RPI_STATUS_SENDING){

          rpi_send(RPI_CMD_START_MEAS_IN_PROCESS);
      }else{
          app_log_error("received start in invalid state");
          hRpiConfig.currentState = RPI_STATUS_ERROR;
          status = SL_STATUS_FAIL;
      }

      return status;



    case RPI_CMD_STOP_ACK:
      //Stop measuring

      if (hRpiConfig.currentState == RPI_STATUS_MEASURING ||
          hRpiConfig.currentState == RPI_STATUS_SENDING   ||
          hRpiConfig.currentState == RPI_STATUS_SENT        ){

          //SENS_setState(COMM_DEVICE_SO2, SENS_STATUS_STOP_CYCLIC_MEASUREMENT);
          //SENS_setState(COMM_DEVICE_CO2, SENS_STATUS_STOP_CYCLIC_MEASUREMENT);

          hRpiConfig.currentState = RPI_STATUS_MEASURING;

          status = SL_STATUS_OK;
      }else{
          app_log_error("received stop in invalid state");
          hRpiConfig.currentState = RPI_STATUS_ERROR;
          status = SL_STATUS_FAIL;
      }

      return status;



    case RPI_CMD_INIT_CHECK:
      // Checks if all devices initialized and then sends reply to rpi

      if (hRpiConfig.currentState == RPI_STATUS_NOT_INITIALIZED){
          status = handle_init();
      }else{
          hRpiConfig.currentState = RPI_STATUS_ERROR;
          status = rpi_send(RPI_CMD_INIT_ERR);
          app_log_warning("probably RPI already initialized");
      }

      return status;



    default:
      app_log_error("invalid cmd received    ");
      return SL_STATUS_FAIL;
  }
}


sl_status_t handle_init(){
  sl_status_t status;

  if (isAllInitialized()){
      status = rpi_send(RPI_CMD_INIT_READY);
      if (!status){
          hRpiConfig.currentState = RPI_STATUS_IDLE;
      } else{
          hRpiConfig.currentState = RPI_STATUS_ERROR; }
  }
  else{
      status = rpi_send(RPI_CMD_INIT_IN_PROCESS);
      if (!status){
          hRpiConfig.currentState = RPI_STATUS_NOT_INITIALIZED;
      } else{
          hRpiConfig.currentState = RPI_STATUS_ERROR; }
  }

  return status;
}


sl_status_t rpi_send(Rpi_Cmd_t cmd){
  const Rpi_CmdBuf_t msg = rpi_get_cmd_buf[cmd];
  sl_status_t status;

  status = UART_Send(sl_uartdrv_eusart_rpi_handle, (uint8_t*)msg.data, msg.length);

  return status;
}

void Rpi_SetStatusIdle(void){
  hRpiConfig.currentState = RPI_STATUS_IDLE;
}






