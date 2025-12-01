/*
 * manager.c
 *
 * Manages sensors and LoRa module to read data and send it to gateway
 */

#include "manager.h"
#include "lora_drv.h"
#include "dfr_so2.h"
#include "scd41_co2.h"
#include "app_log.h"
#include "sl_sleeptimer.h"

#define MANAGER__TIME_BETWEEN_MEAS_MS  1000

static sl_sleeptimer_timer_handle_t hTimer;
static Manager_Handle_t hConfig;
static Dfr_Data_t so2Data;
static Scd41_Data_t co2Data;

static bool isFirst = true;

static void Manager_cbTimerRequestMeasurement(sl_sleeptimer_timer_handle_t *handle, void *data);

sl_status_t Manager_RunStateMachine(){
  sl_status_t status;
  uint32_t timeout = 0;
  double fsc;

  switch (hConfig.currentState){

    case MANAGER_STATUS_IDLE:
      //do nothing
      status = SL_STATUS_OK;
      break;


    case MANAGER_STATUS_REQUEST_MEASUREMENT:
      //initiating measurement
      //start timer 6s
      timeout = sl_sleeptimer_ms_to_tick(MANAGER__TIME_BETWEEN_MEAS_MS);
      status = sl_sleeptimer_start_timer(&hTimer, timeout, Manager_cbTimerRequestMeasurement,
                                         NULL, 0, 0);

      //Start periodic measurement for CO2
      if (isFirst){
      //Scd41_SetState(SCD41_STATE__START_MEASURING);
      isFirst = false;
      }

      hConfig.currentState = MANAGER_STATUS_IDLE;
      status = SL_STATUS_OK;
      break;


    case MANAGER_STATUS_READ_MEASUREMENTS:
      Dfr_GetData(&so2Data);
      hConfig.sensorData.so2Ppb = so2Data.so2Ppb;

      Scd41_GetData(&co2Data);
      hConfig.sensorData.co2Ppm = co2Data.co2Ppm;
      hConfig.sensorData.temp_x100 = co2Data.temp_x100;
      hConfig.sensorData.rh_x100 = co2Data.rh_x100;

      if (hConfig.sensorData.co2Ppm == 0){
          hConfig.sensorData.co2Ppm = 400;
          app_log_warning("co2 value is 0");
      }

      fsc = ((double)hConfig.sensorData.so2Ppb / (double)hConfig.sensorData.co2Ppm) * 0.232f;

      hConfig.sensorData.fsc = (uint16_t)(fsc * 1000);

      app_log("SO2 = %d | CO2 = %d | FSC = %d | T = %d | RH = %d", hConfig.sensorData.so2Ppb,
              hConfig.sensorData.co2Ppm, hConfig.sensorData.fsc, hConfig.sensorData.temp_x100,
              hConfig.sensorData.rh_x100);

      hConfig.currentState = MANAGER_STATUS_SEND;

      status = SL_STATUS_OK;
      break;


    case MANAGER_STATUS_SEND:
      //set sending state to lora drv
      Lora_SetState(LORA_STATUS__SEND_DATA);
      hConfig.currentState = MANAGER_STATUS_SEND_IN_PROCESS;
      status = SL_STATUS_OK;
      break;


    case MANAGER_STATUS_SEND_IN_PROCESS:
      //wait till lora_drv confirms msg is sent
      status = SL_STATUS_OK;
      break;


    case MANAGER_STATUS_SENT:
      app_log("Data sent via LoRa");
      hConfig.currentState = MANAGER_STATUS_REQUEST_MEASUREMENT;
      status = SL_STATUS_OK;
      break;


    case MANAGER_STATUS_ERROR:
      status = SL_STATUS_OK;
      break;
  }

  return status;
}

void Manager_InitializeConfiguration(){
  hConfig.currentState = MANAGER_STATUS_REQUEST_MEASUREMENT;
  hConfig.loraBufLen = 0;
  hConfig.sensorData.co2Ppm = 400;
  hConfig.sensorData.fsc = 0;
  hConfig.sensorData.so2Ppb = 0;
  hConfig.sensorData.rh_x100 = 0;
  hConfig.sensorData.temp_x100 = 0;
}


void Manager_SetState(Manager_Status_t state){
  hConfig.currentState = state;
}

void Manager_GetData(Manager_SensorData_t *sensorData){
  *sensorData = hConfig.sensorData;
}


void Manager_cbTimerRequestMeasurement(sl_sleeptimer_timer_handle_t *handle, void *data){
  (void)data;
  (void)handle;

  if (hConfig.currentState == MANAGER_STATUS_IDLE){
      Dfr_SetState(DFR_STATE__SEND_REQUEST);
      hConfig.currentState = MANAGER_STATUS_READ_MEASUREMENTS;
  }else{
      app_log_warning("manager's timer recv callback, but manager is not idle");
  }
}
