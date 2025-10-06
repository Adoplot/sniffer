#ifndef SENS_DRV_H_
#define SENS_DRV_H_

#include "app_log.h"
#include "communication.h"

typedef enum{
  SENS_INIT_NOT_INITIALIZED,
  SENS_INIT_INITIALIZED
} SENS_Init_t;


typedef enum{
  SENS_TYPE_SO2,
  SENS_TYPE_CO2
} SENS_Type_t;


typedef enum{
  SENS_STATUS_IDLE,
  SENS_STATUS_SEND_REQUEST,
  SENS_STATUS_REQUESTED,
  SENS_STATUS_MEAS_READY,
  SENS_STATUS_ERROR
} SENS_Status_t;


typedef struct{
  SENS_Status_t currentState;
  SENS_Init_t initState;
  SENS_Type_t type;
  char printType[4];    //SO2 or CO2 (used for logging)
  uint8_t buffer[256];
  uint8_t buf_len;
  int32_t xO2_value;
  int32_t temperature;
  int32_t humidity;
}SENS_Handle_t;

bool SENS_isSensorInitialized(Comm_Device_t device);
bool SENS_isMeasReady(Comm_Device_t device);
void SENS_runStateMachine(Comm_Device_t device);
void SENS_Init_State_Handles();
sl_status_t SENS_Handler(Comm_Msg_t msg);

//TODO TEMP
void SENS_setState(Comm_Device_t device, SENS_Status_t status);

#endif /* SENS_DRV_H_ */
