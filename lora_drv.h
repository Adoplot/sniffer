#ifndef LORA_DRV_H_
#define LORA_DRV_H_

#include <stdbool.h>

typedef enum{
  LORA_STATUS_IDLE,
  LORA_STATUS_SENDING,
  LORA_STATUS_SENT,
  LORA_STATUS_ERR,
} Lora_Status_t;

typedef enum{
  LORA_INIT_NOT_INITIALIZED,
  LORA_INIT_INITIALIZED
} Lora_Init_t;

void LORA_Init_State_Handles();
bool LORA_isInitialized();



#endif /* LORA_DRV_H_ */
