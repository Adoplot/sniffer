#ifndef COMMUNICATION_H_
#define COMMUNICATION_H_

#include "stdint.h"

typedef enum{
  COMM_DEVICE_RPI,
  COMM_DEVICE_LORA,
  COMM_DEVICE_SO2,
  COMM_DEVICE_CO2,
}Comm_Device_t;


typedef enum{
  COMM_RESULT_SUCCESS,
  COMM_RESULT_FAIL
}Comm_Result_t;


typedef struct {
  uint8_t buffer[256];
  uint8_t buf_len;
} Comm_Data_t;


typedef struct{
  Comm_Device_t device;
  Comm_Result_t result;
  Comm_Data_t data;
} Comm_Msg_t;


#endif /* COMMUNICATION_H_ */
