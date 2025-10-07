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
  char data[128];
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


static bool isAllInitialized();
static void handle_init(uint8_t *pBuf, uint8_t *pBufLen);
static sl_status_t handle_start(Rpi_Msg_t *rpi_msg, uint8_t *pBuf, uint8_t *pBufLen);
static void Rpi_reply(Rpi_Cmd_t cmd, uint8_t *pBuf, uint8_t *pBufLen);


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
#if 0
  if (SENS_isSensorInitialized(COMM_DEVICE_SO2) &&
      SENS_isSensorInitialized(COMM_DEVICE_CO2) &&
      LORA_isInitialized()) {

      return true;
  }
  else{
      return false;
  }
#else
  if (SENS_isSensorInitialized(COMM_DEVICE_SO2)){
      return true;
  }else{
      return false;
  }
#endif
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
  msg.cmd_data.length = 0;

  //Null-terminate copy of the buffer
  char tmp[256];

  memcpy(tmp, buf, buf_len);
  tmp[buf_len] = '\0';

  //get pointer to delimiter
  char *delimiter = strchr(tmp, ':');

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
      char *cmd = tmp;
      char *gps_data = delimiter + 1;

      if (strcmp(cmd, "start") == 0) {
          msg.cmd = RPI_CMD_START;
          msg.cmd_data.length = strlen(gps_data);

          memcpy(msg.cmd_data.data, gps_data, msg.cmd_data.length);
          msg.cmd_data.data[msg.cmd_data.length] = '\0';
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
sl_status_t RPI_Handler(Comm_Msg_t msg, uint8_t *pBuf, uint8_t *pBufLen){
  Rpi_Msg_t rpi_msg;
  sl_status_t status = SL_STATUS_FAIL;

  rpi_msg = parse_rpi_cmd(msg.data.buffer, msg.data.buf_len);

  switch (rpi_msg.cmd){

    case RPI_CMD_START:
      //Start measuring
      status = handle_start(&rpi_msg, pBuf, pBufLen);
      break;


    case RPI_CMD_STOP_ACK:  //TODO
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
      break;


    case RPI_CMD_INIT_CHECK:
      // Checks if all devices initialized and then sends reply to rpi

      if (hRpiConfig.currentState == RPI_STATUS_NOT_INITIALIZED){
          handle_init(pBuf, pBufLen);
      }else{
          Rpi_reply(RPI_CMD_INIT_ERR, pBuf, pBufLen);

          hRpiConfig.currentState = RPI_STATUS_ERROR;

          app_log_warning("probably RPI already initialized");
      }
      status = SL_STATUS_OK;
      break;

//TODO BUG: can be initialized without init: NOT_INIT->ERROR->IDLE
    case RPI_CMD_UNKNOWN:
      hRpiConfig.currentState = RPI_STATUS_ERROR;

      status = SL_STATUS_FAIL;
      app_log_warning("unrecognized cmd received");
      break;

    default:
      hRpiConfig.currentState = RPI_STATUS_ERROR;

      status = SL_STATUS_FAIL;
      app_log_error("invalid cmd received    ");
      break;
  }

  return status;
}


void handle_init(uint8_t *pBuf, uint8_t *pBufLen){

  if (isAllInitialized()){

      Rpi_reply(RPI_CMD_INIT_READY, pBuf, pBufLen);
      hRpiConfig.currentState = RPI_STATUS_IDLE;

  }else{

      Rpi_reply(RPI_CMD_INIT_IN_PROCESS, pBuf, pBufLen);
      hRpiConfig.currentState = RPI_STATUS_NOT_INITIALIZED;
  }
}


sl_status_t handle_start(Rpi_Msg_t *rpi_msg, uint8_t *pBuf, uint8_t *pBufLen){

  sl_status_t status = SL_STATUS_FAIL;
  switch (hRpiConfig.currentState){

    case RPI_STATUS_IDLE:
      //Request measurement from sensors
      SENS_setState(COMM_DEVICE_SO2, SENS_STATUS_SEND_REQUEST);
      SENS_setState(COMM_DEVICE_CO2, SENS_STATUS_SEND_REQUEST);

      Rpi_reply(RPI_CMD_START_ACK, pBuf, pBufLen);

      //Copy cmd buf to gps data
      memcpy(hRpiConfig.gps_buf, rpi_msg->cmd_data.data, rpi_msg->cmd_data.length);

      hRpiConfig.currentState = RPI_STATUS_MEASURING;
      status = SL_STATUS_OK;
      break;


    case RPI_STATUS_MEASURING:
    case RPI_STATUS_MEAS_READY:
    case RPI_STATUS_SENDING:

      Rpi_reply(RPI_CMD_START_MEAS_IN_PROCESS, pBuf, pBufLen);
      status = SL_STATUS_OK;
      break;


    case RPI_STATUS_SENT:

      Rpi_reply(RPI_CMD_START_MEAS_SENT, pBuf, pBufLen);
      status = SL_STATUS_OK;
      break;


    case RPI_STATUS_NOT_INITIALIZED:
    case RPI_STATUS_ERROR:

      Rpi_reply(RPI_CMD_START_ERR, pBuf, pBufLen);

      hRpiConfig.currentState = RPI_STATUS_ERROR;

      status = SL_STATUS_OK;
      app_log_warning("received START cmd in invalid state    ");
      break;
  }
  return status;
}


void Rpi_reply(Rpi_Cmd_t cmd, uint8_t *pBuf, uint8_t *pBufLen){
  const Rpi_CmdBuf_t msg = rpi_get_cmd_buf[cmd];

  memcpy(pBuf, msg.data, msg.length);
  *pBufLen = msg.length;
}


void Rpi_SetStatusIdle(void){
  hRpiConfig.currentState = RPI_STATUS_IDLE;
}






