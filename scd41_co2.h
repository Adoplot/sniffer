/*
 * scd41_co2.h
 *
 * Driver for CO2 sensor SCD41
 */

#ifndef SCD41_CO2_H_
#define SCD41_CO2_H_

#include "app_log.h"

typedef sl_status_t Scd41_I2cStatus_t;

typedef struct{
  int16_t co2Ppm;       // ppm
  int16_t temp_x100;    // C/100
  int16_t rh_x100;      // %RH/100
} Scd41_Data_t;

Scd41_I2cStatus_t Scd41_StartPeriodicMeasurement(void);
Scd41_I2cStatus_t Scd41_ReadMeasurement(void);


#endif /* SCD41_CO2_H_ */
