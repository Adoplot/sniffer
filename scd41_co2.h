/*
 * scd41_co2.h
 *
 * Driver for CO2 sensor SCD41
 */

#ifndef SCD41_CO2_H_
#define SCD41_CO2_H_

#include "app_log.h"

typedef sl_status_t Scd41_I2cStatus_t;

typedef enum{
  SCD41_STATE__IDLE,
  SCD41_STATE__START_MEASURING,
  SCD41_STATE__STOP_MEASURING,
  SCD41_STATE__WAITING_STOP,
  SCD41_STATE__WAITING_FOR_MEASUREMENT,
  SCD41_STATE__READY_TO_READ, //TODO del
  SCD41_STATE__READ,
  SCD41_STATE__ERROR
} Scd41_State_t;

typedef enum{
  SCD41_INIT__NOT_INITIALIZED,
  SCD41_INIT__IN_PROCESS,
  SCD41_INIT__INITIALIZED
} Scd41_Init_t;

typedef struct{
  int16_t co2Ppm;       // ppm
  int16_t temp_x100;    // C/100
  int16_t rh_x100;      // %RH/100
} Scd41_Data_t;

typedef struct{
  Scd41_State_t currentState;
  Scd41_Init_t  initState;
  Scd41_Data_t  data;
} Scd41_Config_t;

sl_status_t Scd41_RunStateMachine(void);
void Scd41_InitializeConfiguration(void);
bool Scd41_isInitialized(void);
//TODO change cmds to static
Scd41_I2cStatus_t Scd41_StartPeriodicMeasurement(void);
Scd41_I2cStatus_t Scd41_ReadMeasurement(void);
Scd41_I2cStatus_t Scd41_StopPeriodicMeasurement(void);
Scd41_I2cStatus_t Scd41_measureSingleShot(void);


#endif /* SCD41_CO2_H_ */
