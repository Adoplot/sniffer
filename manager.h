/*
 * manager.h
 *
 * Manages sensors and LoRa module to read data and send it to gateway
 */

#ifndef MANAGER_H_
#define MANAGER_H_

#include <stdint.h>
#include "app_log.h"

typedef enum{
  MANAGER_STATUS_IDLE,
  MANAGER_STATUS_REQUEST_MEASUREMENT,
  MANAGER_STATUS_READ_MEASUREMENTS,
  MANAGER_STATUS_SEND,
  MANAGER_STATUS_SEND_IN_PROCESS,
  MANAGER_STATUS_SENT,
  MANAGER_STATUS_ERROR,
} Manager_Status_t;

typedef struct __attribute__((packed)){
  uint16_t so2Ppb;
  uint16_t co2Ppm;
  uint16_t fsc;        //calculated sulfur content of the fuel
  int16_t temp_x100;
  int16_t rh_x100;
} Manager_SensorData_t;

typedef struct{
  Manager_Status_t currentState;
  Manager_SensorData_t sensorData;    //TODO make common struct for sensor data?
  uint8_t loraBuf[256];
  uint8_t loraBufLen;
} Manager_Handle_t;


sl_status_t Manager_RunStateMachine();
void Manager_InitializeConfiguration();
void Manager_SetState(Manager_Status_t state);
void Manager_GetData(Manager_SensorData_t *sensorData);

#endif /* MANAGER_H_ */
