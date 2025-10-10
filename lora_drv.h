#ifndef LORA_DRV_H_
#define LORA_DRV_H_

#include <stdbool.h>
#include "uart_drv.h"


typedef enum{
  LORA_STATUS__IDLE,
  LORA_STATUS__SEND_DATA,
  LORA_STATUS__SENDING,
  LORA_STATUS__SENT_SUCCESSFUL,
  LORA_STATUS__ERROR,
} Lora_State_t;

typedef enum{
  LORA_INIT__NOT_INITIALIZED,
  LORA_INIT__IN_PROCESS,
  LORA_INIT__INITIALIZED,
} Lora_Init_t;

typedef struct{
  Lora_State_t currentState;
  Lora_Init_t initState;
  int32_t so2ppb;
  int32_t so2Temperature;
  int32_t so2Humidity;
  int32_t co2Ppm;
  int32_t co2Temperature;
  int32_t co2Humidity;
  uint8_t gpsBuf[256];
  uint8_t gpsBufLen;
  uint8_t loraBuf[512];
  uint16_t loraBufLen;
} Lora_Config_t;

sl_status_t Lora_RunStateMachine();
void Lora_SetState(Lora_State_t state);
void LORA_Init_State_Handles();
bool LORA_isInitialized();



#endif /* LORA_DRV_H_ */
