/*
 * scd41_co2.c
 *
 * Driver for CO2 sensor SCD41
 */

#include "scd41_co2.h"
#include "app_log.h"
#include "sl_i2cspm.h"
#include "sl_i2cspm_instances.h"


#define SCD41__I2C_ADDR     (0x62UL)    // 7-bit I2C slave address
#define SCD41__READ_TIMEOUT 5000        // ms time to wait for next read



typedef enum{
  // Basic commands
  SCD41_CMD__START_PERIODIC_MEASUREMENT,
  SCD41_CMD__READ_MEASUREMENT,
  SCD41_CMD__STOP_PERIODIC_MEASUREMENT,

  // Low power periodic measurement mode
  SCD41_CMD__START_LOW_POWER_PERIODIC_MEASUREMENT,
  SCD41_CMD__GET_DATA_READY_STATUS,

  // Single shot measurement mode
  SCD41_CMD__POWER_DOWN,
  SCD41_CMD__WAKE_UP,

  // Other
  SCD41_CMD__UNKNOWN,
  SCD41_CMD__COUNT
}Scd41_Cmd_t;


static const uint16_t Scd41_getCmdCode[SCD41_CMD__COUNT] = {
    // Basic commands
    0x21B1,   // START_PERIODIC_MEASUREMENT
    0xec05,   // READ_MEASUREMENT
    0x3f86,   // STOP_PERIODIC_MEASUREMENT

    // Low power periodic measurement mode
    0x21ac,   // START_LOW_POWER_PERIODIC_MEASUREMENT
    0xe4b8,   // GET_DATA_READY_STATUS

    //Single shot measurement mode
    0x36e0,   // POWER_DOWN
    0x36f6,   // WAKE_UP
};


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
scd41_I2cStatus_t Scd41_StartPeriodicMeasurement(void){
  scd41_I2cStatus_t status;
  I2C_TransferSeq_TypeDef seq;
  I2C_TransferReturn_TypeDef ret;
  uint8_t tx_data[2];

  Scd41_cmd16ToBytes(Scd41_getCmdCode[SCD41_CMD__START_PERIODIC_MEASUREMENT], tx_data);

  seq.addr = SCD41__I2C_ADDR << 1;  // 7bit addr from 8bit definition
  seq.flags = I2C_FLAG_WRITE;
  seq.buf[0].data = tx_data;
  seq.buf[0].len = sizeof(tx_data) / sizeof(tx_data[0]);

  ret = I2CSPM_Transfer(sl_i2cspm_co2, &seq);

  if (ret != i2cTransferDone) {
      app_log_error("Error while transmitting firmware message through i2c");
      status = SL_STATUS_FAIL;
  }

  return status;
}


