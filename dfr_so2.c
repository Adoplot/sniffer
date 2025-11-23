/*
 * dgs2_so2.c
 *
 * Driver for SO2 sensor on DFR board
 */

#include "dfr_so2.h"
#include "sl_sleeptimer.h"
#include "uart_drv.h"
#include "communication.h"
#include <string.h>

#define DFR_RESPONSE_TIMEOUT_MS    3000

typedef struct {
  const uint8_t *data;
  uint8_t length;
} Dfr_CmdBuf_t;

typedef enum{
  //SO2 commands
  DFR_CMD__SINGLE_MEASUREMENT,
  DFR_CMD__SET_RESPONSE_MODE,
  //Other
  DFR_CMD__UNKNOWN,
  DFR_CMD__COUNT
} Dfr_Cmd_t;


static const uint8_t cmd_SingleMeasurement[] = {
    0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
static const uint8_t cmd_SetResponseMode[] = {
    0xFF, 0x01, 0x78, 0x04, 0x00, 0x00, 0x00, 0x00, 0x83};

const Dfr_CmdBuf_t Dfr_getCmdBuf[DFR_CMD__COUNT] = {
    [DFR_CMD__SINGLE_MEASUREMENT] = {
        cmd_SingleMeasurement,
        sizeof(cmd_SingleMeasurement)
    },
    [DFR_CMD__SET_RESPONSE_MODE] = {
        cmd_SetResponseMode,
        sizeof(cmd_SetResponseMode)
    }
};

static Dfr_Config_t hSensor;
static sl_sleeptimer_timer_handle_t hTimer;


static void Dfr_initialize();
static sl_status_t Dfr_send(Dfr_Cmd_t cmd);
static sl_status_t Dfr_stopTimer(sl_sleeptimer_timer_handle_t *handle);
static void Dfr_cbTimerResponse(sl_sleeptimer_timer_handle_t *handle, void *data);
static sl_status_t Dfr_parseData(Comm_Msg_t *msg);


sl_status_t Dfr_RunStateMachine(void){
  sl_status_t status = SL_STATUS_FAIL;
  uint32_t timeout = 0;

  switch(hSensor.currentState){

    case DFR_STATE__IDLE:
      //do nothing
      if (!Dfr_isInitialized()){
          Dfr_initialize();
      }

      status = SL_STATUS_OK;
      break;


    case DFR_STATE__SEND_REQUEST:
      status = Dfr_stopTimer(&hTimer);

      if (status == SL_STATUS_OK){
          timeout = sl_sleeptimer_ms_to_tick(DFR_RESPONSE_TIMEOUT_MS);
          status = sl_sleeptimer_start_timer(&hTimer, timeout, Dfr_cbTimerResponse,
                                             NULL, 0, 0);
      }

      if (status == SL_STATUS_OK){
          status = Dfr_send(DFR_CMD__SINGLE_MEASUREMENT);
      }

      if (status == SL_STATUS_OK){
          hSensor.currentState = DFR_STATE__REQUESTED;
      }else{
          hSensor.currentState = DFR_STATE__ERROR;
          app_log_error("SO2 failed to send request");
      }

      break;


    case DFR_STATE__REQUESTED:
      //waiting for answer from sensor
      //answer handled in Dfr_Handler()
      hSensor.isMeasReady = false;    //resetting "ready" flag because new meas requested
      status = SL_STATUS_OK;
      break;


    case DFR_STATE__MEAS_READY:
      //measurement is ready, stop timeout timer, set flag to ready
      //or if initializing then set to initialized
      if (hSensor.isMeasReady == false){
          status = Dfr_stopTimer(&hTimer);
      }

      if (status == SL_STATUS_OK){
          hSensor.isMeasReady = true;
          // Waiting for Rpi to read and set to Idle state
          //hSensor.currentState = DFR_STATE__IDLE;

      }else{
          hSensor.currentState = DFR_STATE__ERROR;
      }

      // Initialization successful
      if (hSensor.initState == DFR_INIT__IN_PROCESS){
          hSensor.isMeasReady = false;  // rewrite flag to false when initializing
          hSensor.initState = DFR_INIT__INITIALIZED;
          hSensor.currentState = DFR_STATE__SEND_REQUEST;
      }
      break;


    case DFR_STATE__ERROR:
      status = Dfr_stopTimer(&hTimer);

      hSensor.isMeasReady = false;

      app_log_error("SO2 general error");
      hSensor.currentState = DFR_STATE__IDLE;
      break;
  }
  return status;
}


void Dfr_SetState(Dfr_State_t state){
  hSensor.currentState = state;
}


void Dfr_GetData(Dfr_Data_t *sensorData){
  (void)sensorData;
  *sensorData = hSensor.data;
}


/***************************************************************************//**
 * @brief
 *    Handles msg received from UART driver
 ******************************************************************************/
sl_status_t Dfr_Handler(Comm_Msg_t msg){
  sl_status_t status = SL_STATUS_FAIL;

  if (hSensor.currentState == DFR_STATE__REQUESTED){
      status = Dfr_parseData(&msg);
  }else{
      app_log_error("SO2 data received out of state");
  }

  if (status == SL_STATUS_OK){
      hSensor.currentState = DFR_STATE__MEAS_READY;
  }else{
      hSensor.currentState = DFR_STATE__ERROR;
  }
  return status;
}


void Dfr_InitializeConfiguration(void){
  hSensor.currentState = DFR_STATE__IDLE;
  hSensor.initState = DFR_INIT__INITIALIZED;  //TODO skip init sequence for now
  hSensor.isMeasReady = false;
  hSensor.data.so2Ppb = 0;
  hSensor.data.temp_x100 = 0;
  hSensor.data.rh_x100 = 0;
  hSensor.bufLen = 0;
}


void Dfr_initialize(){
  hSensor.initState = DFR_INIT__IN_PROCESS;
  hSensor.currentState = DFR_STATE__SEND_REQUEST;
}


bool Dfr_isInitialized(void){
  if (hSensor.initState == DFR_INIT__INITIALIZED){
      return true;
  }else{
      return false;
  }
}


bool Dfr_isMeasReady(void){
  if (hSensor.isMeasReady == true){
      return true;
  }else{
      return false;
  }
}


/***************************************************************************//**
 * @brief
 *    Parses So2 (ppb), from buf to int16
 ******************************************************************************/
sl_status_t Dfr_parseData(Comm_Msg_t *msg){
  sl_status_t status = SL_STATUS_OK;
  int16_t so2PpbX100 = 0;

  const uint8_t *buf = msg->data.buffer;

  if (msg->data.buf_len < 9) {
      app_log_error("SO2: response too short");
      status = SL_STATUS_FAIL;
  }

  if (buf[0] != 0xFF) {
      app_log_error("SO2: invalid [start] byte");
      status = SL_STATUS_FAIL;
  }

  if (buf[1] != 0x86) {
      app_log_error("SO2: unexpected [command] byte");
      status = SL_STATUS_FAIL;
  }

  if (status == SL_STATUS_OK){
      if (buf[5] == 0){
          so2PpbX100 = (buf[2]*256 + buf[3])*100;
      } else if (buf[5] == 1){
          so2PpbX100 = (buf[2]*256 + buf[3])*10;
      } else if (buf[5] == 2){
          so2PpbX100 = (buf[2]*256 + buf[3]);
      } else{
          app_log_error("SO2: unexpected [decimal places] byte");
          status = SL_STATUS_FAIL;
      }
  }

  if (status == SL_STATUS_OK){
      hSensor.data.so2Ppb     = so2PpbX100/100; //int division will round up to smallest abs value
      hSensor.data.temp_x100  = 0;
      hSensor.data.rh_x100    = 0;
  }

  return status;
}


sl_status_t Dfr_send(Dfr_Cmd_t cmd){
  sl_status_t status = SL_STATUS_FAIL;

  const Dfr_CmdBuf_t msg = Dfr_getCmdBuf[cmd];

  status = UART_Send(sl_uartdrv_usart_so2_handle, (uint8_t*)msg.data, msg.length);

  return status;
}


// Checks if timer is running and then stops it
//TODO make common func
sl_status_t Dfr_stopTimer(sl_sleeptimer_timer_handle_t *handle){
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
void Dfr_cbTimerResponse(sl_sleeptimer_timer_handle_t *handle, void *data){
  (void)data;
  (void)handle;

  hSensor.currentState = DFR_STATE__ERROR;
  app_log_warning("SO2 response timeout");

}


