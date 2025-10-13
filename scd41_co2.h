/*
 * scd41_co2.h
 *
 * Driver for CO2 sensor SCD41
 */

#ifndef SCD41_CO2_H_
#define SCD41_CO2_H_

#include "app_log.h"

typedef sl_status_t scd41_I2cStatus_t;

scd41_I2cStatus_t Scd41_StartPeriodicMeasurement(void);


#endif /* SCD41_CO2_H_ */
