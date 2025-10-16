/*
 * scd41_co2.c
 *
 * Driver for CO2 sensor SCD41
 */

#include "scd41_co2.h"
#include "app_log.h"
#include "sl_i2cspm.h"
#include "sl_i2cspm_instances.h"
#include "sl_sleeptimer.h"
#include <limits.h>


#define SCD41__I2C_ADDR                   (0x62UL)    // 7-bit I2C slave address
// Timeouts
#define SCD41__READ_DATA_TIMEOUT_MS       1           // time to wait between read cmd and reading data
#define SCD41__READ_TIMEOUT_MS            5000        // time to wait before next read
#define SCD41__STOP_TIMEOUT_MS            500         // time to wait after stop
// Timers
#define SCD41_TIMER_PRIORITY__CMD_STOP    0           // time to wait after stop


typedef enum{
  // Basic commands
  SCD41_CMD__START_PERIODIC_MEASUREMENT,
  SCD41_CMD__READ_MEASUREMENT,
  SCD41_CMD__STOP_PERIODIC_MEASUREMENT,

  // Low power periodic measurement mode
  SCD41_CMD__START_LOW_POWER_PERIODIC_MEASUREMENT,
  SCD41_CMD__GET_DATA_READY_STATUS,

  // Single shot measurement mode
  SCD41_CMD__MEASURE_SINGLE_SHOT,
  SCD41_CMD__POWER_DOWN,
  SCD41_CMD__WAKE_UP,

  // Other
  SCD41_CMD__UNKNOWN,
  SCD41_CMD__COUNT
} Scd41_Cmd_t;


static const uint16_t Scd41_getCmdCode[SCD41_CMD__COUNT] = {
    // Basic commands
    0x21B1,   // START_PERIODIC_MEASUREMENT
    0xec05,   // READ_MEASUREMENT
    0x3f86,   // STOP_PERIODIC_MEASUREMENT

    // Low power periodic measurement mode
    0x21ac,   // START_LOW_POWER_PERIODIC_MEASUREMENT
    0xe4b8,   // GET_DATA_READY_STATUS

    //Single shot measurement mode
    0x219d,   // MEASURE_SINGLE_SHOT
    0x36e0,   // POWER_DOWN
    0x36f6,   // WAKE_UP
};

static Scd41_Config_t hSensor;
static sl_sleeptimer_timer_handle_t hTimer;

static void Scd41_initialize(void);
static Scd41_I2cStatus_t Scd41_i2cWriteCmd(I2C_TransferSeq_TypeDef *seq, Scd41_Cmd_t cmd);
static Scd41_I2cStatus_t Scd41_i2cRead(I2C_TransferSeq_TypeDef *seq, uint8_t *rxData, uint8_t rxDataLen);
static void Scd41_cmd16ToBytes(uint16_t cmd, uint8_t *buf);
//static sl_status_t Scd41_startTimer(uint8_t timeout, sl_sleeptimer_timer_callback_t callback);
static sl_status_t Scd41_stopTimer(sl_sleeptimer_timer_handle_t *handle);
static void Scd41_cbTimerStop(sl_sleeptimer_timer_handle_t *handle, void *data);
static void Scd41_cbTimerReadyToRead(sl_sleeptimer_timer_handle_t *handle, void *data);


sl_status_t Scd41_RunStateMachine(void){
  sl_status_t status = SL_STATUS_FAIL;
  uint32_t timeout = 0;

  switch(hSensor.currentState){
    case SCD41_STATE__IDLE:
      //check if initialized
      if (!Scd41_isInitialized()){
          Scd41_initialize();
      }
      status = SL_STATUS_OK;
      break;



    case SCD41_STATE__START_MEASURING:
      // start measurement mode
      status = Scd41_StartPeriodicMeasurement();

      if (status == SL_STATUS_OK){
          //set timer 500ms
          timeout = sl_sleeptimer_ms_to_tick(SCD41__READ_TIMEOUT_MS);
          status = sl_sleeptimer_start_timer(&hTimer, timeout, Scd41_cbTimerReadyToRead,
                                             NULL, SCD41_TIMER_PRIORITY__CMD_STOP, 0);
      }
      if (status == SL_STATUS_OK){
          hSensor.currentState = SCD41_STATE__WAITING_FOR_MEASUREMENT;
      }
      if (status == SL_STATUS_FAIL){
          hSensor.currentState = SCD41_STATE__ERROR;
          app_log_error("CO2 failed to start periodic meas");
      }
      break;



    case SCD41_STATE__STOP_MEASURING:
      // Stop timers, send Stop cmd, set timer that will change state to IDLE after 500ms
      status = Scd41_stopTimer(&hTimer);

      if (status == SL_STATUS_OK){
      status = Scd41_StopPeriodicMeasurement();
      }

      if (status == SL_STATUS_OK){
          //set timer 500ms
          timeout = sl_sleeptimer_ms_to_tick(SCD41__STOP_TIMEOUT_MS);
          status = sl_sleeptimer_start_timer(&hTimer, timeout, Scd41_cbTimerStop,
                                             NULL, SCD41_TIMER_PRIORITY__CMD_STOP, 0);
      }

      if (status == SL_STATUS_OK){
          hSensor.currentState = SCD41_STATE__WAITING_STOP;
      }

      if (status == SL_STATUS_FAIL){
          hSensor.currentState = SCD41_STATE__ERROR;
          app_log_error("CO2 failed to stop periodic meas");
      }
      break;



    case SCD41_STATE__WAITING_STOP:
      // waiting stop timer callback to switch to IDLE state (500ms)
      status = SL_STATUS_OK;
      break;


    case SCD41_STATE__WAITING_FOR_MEASUREMENT:
      // waiting ReadyToRead timer callback to switch to READY_TO_READ state (500ms)
      status = SL_STATUS_OK;
      break;


    case SCD41_STATE__READY_TO_READ:
      status = SL_STATUS_OK;
      break;


    case SCD41_STATE__READ:
      //Read values and set new timer
      status = Scd41_ReadMeasurement();

      if (status == SL_STATUS_OK){
          //set timer 500ms
          timeout = sl_sleeptimer_ms_to_tick(SCD41__READ_TIMEOUT_MS);
          status = sl_sleeptimer_start_timer(&hTimer, timeout, Scd41_cbTimerReadyToRead,
                                             NULL, SCD41_TIMER_PRIORITY__CMD_STOP, 0);
      }
      if (status == SL_STATUS_OK){
          hSensor.currentState = SCD41_STATE__WAITING_FOR_MEASUREMENT;
      }
      if (status == SL_STATUS_FAIL){
          hSensor.currentState = SCD41_STATE__ERROR;
          app_log_error("CO2 failed to start periodic meas");
      }

      // Set initialized if data was read successfully and reset to idle via stop
      if (!Scd41_isInitialized()){
          if (status == SL_STATUS_OK){
              hSensor.initState = SCD41_INIT__INITIALIZED;
              //Reset to IDLE
              hSensor.currentState = SCD41_STATE__STOP_MEASURING;
          }else{
              hSensor.initState = SCD41_INIT__NOT_INITIALIZED;
          }
      }

      break;


    case SCD41_STATE__ERROR:
      //Stop all timers
      status = Scd41_stopTimer(&hTimer);

      app_log_error("CO2 general error");
      hSensor.currentState = SCD41_STATE__IDLE;
      break;
  }

  return status;
}


void Scd41_InitializeConfiguration(void){
  hSensor.currentState = SCD41_STATE__IDLE;
  hSensor.initState = SCD41_INIT__NOT_INITIALIZED;
  hSensor.data.co2Ppm = 0;
  hSensor.data.temp_x100 = 0;
  hSensor.data.rh_x100 = 0;
}


void Scd41_initialize(void){
  if (hSensor.initState == SCD41_INIT__IN_PROCESS){
      hSensor.currentState = SCD41_STATE__START_MEASURING;
  }else{
      //reset sensor to idle state
      hSensor.currentState = SCD41_STATE__STOP_MEASURING;
  }


  //hSensor.initState = SCD41_INIT__INITIALIZED;
}


bool Scd41_isInitialized(void){
  if (hSensor.initState == SCD41_INIT__INITIALIZED){
      return true;
  }else{
      return false;
  }
}


/***************************************************************************//**
 * @brief
 *    Transmits SCD41 command via I2C
 ******************************************************************************/
Scd41_I2cStatus_t Scd41_i2cWriteCmd(I2C_TransferSeq_TypeDef *seq, Scd41_Cmd_t cmd){
  Scd41_I2cStatus_t status = SL_STATUS_FAIL;
  I2C_TransferReturn_TypeDef ret;
  uint8_t txData[2];

  Scd41_cmd16ToBytes(Scd41_getCmdCode[cmd], txData);

  seq->addr = SCD41__I2C_ADDR << 1;  // 7bit addr from 8bit definition
  seq->flags = I2C_FLAG_WRITE;
  seq->buf[0].data = txData;
  seq->buf[0].len = sizeof(txData) / sizeof(txData[0]);

  ret = I2CSPM_Transfer(sl_i2cspm_co2, seq);

  if (ret != i2cTransferDone) {
      app_log_error("Error while transmitting i2c");
      status = SL_STATUS_FAIL;
  }else{
      status = SL_STATUS_OK;
  }

  return status;
}


/***************************************************************************//**
 * @brief
 *    Reads data from SCD41 via I2C
 ******************************************************************************/
Scd41_I2cStatus_t Scd41_i2cRead(I2C_TransferSeq_TypeDef *seq, uint8_t *rxData, uint8_t rxDataLen){
  Scd41_I2cStatus_t status = SL_STATUS_FAIL;
  I2C_TransferReturn_TypeDef ret;

  // Reading data from sensor
  seq->flags = I2C_FLAG_READ;
  seq->buf[0].data = rxData;
  seq->buf[0].len = rxDataLen;  //expecting 3x3b = 9 bytes (2 bytes DATA + 1 byte CRC)

  ret = I2CSPM_Transfer(sl_i2cspm_co2, seq);

  if (ret != i2cTransferDone) {
      app_log_error("Error while transmitting i2c");
      status = SL_STATUS_FAIL;
  }else{
      status = SL_STATUS_OK;
  }

  return status;
}


void Scd41_parseSensorData(uint8_t *pBuf, uint16_t bufLen, Scd41_Data_t *sensorData);


/***************************************************************************//**
 * @brief
 *    Divides 16bit command to 2 bytes with MSB first
 ******************************************************************************/
void Scd41_cmd16ToBytes(uint16_t cmd, uint8_t *buf){
  buf[0] = (uint8_t)(cmd >> 8);   // MSB
  buf[1] = (uint8_t)(cmd & 0xFF); // LSB
}


/***************************************************************************//**
 * @brief
 *    Starts the periodic measurement mode. The signal update interval
 *    is 5 seconds
 *    While the periodic measurement mode is running, no other commands may be
 *    issued, with exception of read_measurement, get_data_ready_status,
 *    stop_periodic_measurement, set_ambient_pressure and get_ambient_pressure
 ******************************************************************************/
Scd41_I2cStatus_t Scd41_StartPeriodicMeasurement(void){
  Scd41_I2cStatus_t status = SL_STATUS_FAIL;
  I2C_TransferSeq_TypeDef seq;

  status = Scd41_i2cWriteCmd(&seq, SCD41_CMD__START_PERIODIC_MEASUREMENT);

  return status;
}


/***************************************************************************//**
 * @brief
 *  Reads the sensor output. The measurement data can only be read out once
 *  per signal update interval as the buffer is emptied upon read-out.
 *  If no data is available in the buffer, the sensor returns a NACK
 *  Response: CO2=500ppm, T=25°C, RH=37%
 ******************************************************************************/
Scd41_I2cStatus_t Scd41_ReadMeasurement(void){
  Scd41_I2cStatus_t status = SL_STATUS_FAIL;
  I2C_TransferSeq_TypeDef seq;

  uint8_t rxData[9];

  status = Scd41_i2cWriteCmd(&seq, SCD41_CMD__READ_MEASUREMENT);

  if (status == SL_STATUS_OK){
      // Has to wait for sensor to prepare data for reading
      sl_sleeptimer_delay_millisecond(SCD41__READ_DATA_TIMEOUT_MS);

      uint8_t rxDataLen = sizeof(rxData) / sizeof(rxData[0]);
      status = Scd41_i2cRead(&seq, rxData , rxDataLen);

  }
  if (status == SL_STATUS_OK){
      Scd41_parseSensorData(rxData, (sizeof(rxData) / sizeof(rxData[0])), &hSensor.data);

      app_log("Co2 = %d ppm | ", hSensor.data.co2Ppm);
      app_log_append("T = %.2f C | ", hSensor.data.temp_x100/100.0);
      app_log_append("RH = %.2f %%", hSensor.data.rh_x100/100.0);
  }else{
      app_log_warning("fail reading from CO2");
  }

  return status;
}


/***************************************************************************//**
 * @brief
 *    Command returns a sensor running in periodic measurement mode or low
 *    power periodic measurement mode back to the idle state, e.g. to then
 *    allow changing the sensor configuration or to save power. Note that the
 *    sensor will only respond to other commands 500 ms after the
 *    stop_periodic_measurement command has been issued
 ******************************************************************************/
Scd41_I2cStatus_t Scd41_StopPeriodicMeasurement(void){
  Scd41_I2cStatus_t status = SL_STATUS_FAIL;
  I2C_TransferSeq_TypeDef seq;

  status = Scd41_i2cWriteCmd(&seq, SCD41_CMD__STOP_PERIODIC_MEASUREMENT);

  return status;
}


void Scd41_parseSensorData(uint8_t *pBuf, uint16_t bufLen, Scd41_Data_t *sensorData){
  (void)bufLen;

  uint16_t rawCo2     = ((pBuf[0] << 8) | pBuf[1]);
  uint16_t rawTemp    = ((pBuf[3] << 8) | pBuf[4]);
  uint16_t rawRh      = ((pBuf[6] << 8) | pBuf[7]);

  if (rawCo2 <= SHRT_MAX){
      sensorData->co2Ppm = (int16_t)rawCo2;
  }else{
      rawCo2 = 0;
      app_log_error("overflow when reading CO2 value");
  }

  if (rawTemp <= SHRT_MAX){
      sensorData->temp_x100 = (int16_t)(((int32_t)17500 * rawTemp) / 65535 - 4500);
  }else{
      rawTemp = 0;
      app_log_error("overflow when reading Temp value");
  }

  if (rawRh <= SHRT_MAX){
       sensorData->rh_x100 = (int16_t)(((int32_t)10000 * rawRh)  / 65535);
   }else{
       rawRh = 0;
       app_log_error("overflow when reading RH value");
   }
}

#if 0
sl_status_t Scd41_startTimer(uint8_t timeout, sl_sleeptimer_timer_callback_t callback){
  sl_status_t status = SL_STATUS_FAIL;
  uint32_t timeoutTicks;

  timeoutTicks = sl_sleeptimer_ms_to_tick(SCD41__STOP_TIMEOUT_MS);
  status = sl_sleeptimer_start_timer(&hTimer,
                                     timeoutTicks,
                                     Scd41_cbTimer,
                                     NULL,
                                     SCD41_TIMER_PRIORITY__CMD_STOP,
                                     0);
  return status;
}
#endif


// Checks if timer is running and then stops it
sl_status_t Scd41_stopTimer(sl_sleeptimer_timer_handle_t *handle){
  sl_status_t status = SL_STATUS_FAIL;
  bool isRunning;

  // Check if timer is running
  status = sl_sleeptimer_is_timer_running(&hTimer, &isRunning);

  if (status == SL_STATUS_OK){
      if (isRunning){
          status = sl_sleeptimer_stop_timer(handle);
      }else{
          status = SL_STATUS_OK;
          app_log_warning("CO2: tried to stop inactive timer");
      }
  }

  if (status == SL_STATUS_FAIL){
      app_log_error("CO2: failed to stop timer");
  }

  return status;
}


// Stop periodic measurement 500ms timeout
void Scd41_cbTimerStop(sl_sleeptimer_timer_handle_t *handle, void *data){
  (void)data;
  (void)handle;

  if(!Scd41_isInitialized()){
      hSensor.initState = SCD41_INIT__IN_PROCESS;
  }
  if (hSensor.currentState == SCD41_STATE__WAITING_STOP){
      hSensor.currentState = SCD41_STATE__IDLE;
      app_log("CO2 stopped periodic meas");

  }else{
      hSensor.currentState = SCD41_STATE__ERROR;
      app_log_error("incorrect state when CO2 Stop callback received");
  }
}


// Data is ready to read 5000ms timeout
void Scd41_cbTimerReadyToRead(sl_sleeptimer_timer_handle_t *handle, void *data){
  (void)data;
  (void)handle;

  if (hSensor.currentState == SCD41_STATE__WAITING_FOR_MEASUREMENT){
      hSensor.currentState = SCD41_STATE__READ;
      app_log("CO2 is ready to read");

  }else{
      hSensor.currentState = SCD41_STATE__ERROR;
      app_log_error("incorrect state when CO2 ReadyToRead callback received");
  }
}



