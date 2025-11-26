#ifndef LORA_DRV_H_
#define LORA_DRV_H_

#include <stdbool.h>
#include "manager.h"
#include "communication.h"
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
  uint8_t payload[256];
  uint8_t payload_len;
} Lora_Config_t;



sl_status_t Lora_RunStateMachine();
sl_status_t Lora_Handler(Comm_Msg_t msg, uint8_t *pBuf, uint8_t *pBufLen);
void Lora_SetState(Lora_State_t state);
void LORA_InitializeConfiguration();
bool LORA_isInitialized();



#endif /* LORA_DRV_H_ */
