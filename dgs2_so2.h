/*
 * dgs2_so2.h
 *
 *  Driver for SO2 sensor on DGS2 board
 */

#ifndef DGS2_SO2_H_
#define DGS2_SO2_H_

#include <stdint.h>
#include "app_log.h"
#include "communication.h"

typedef enum{
  DGS2_STATE__IDLE,
  DGS2_STATE__SEND_REQUEST,
  DGS2_STATE__REQUESTED,
  DGS2_STATE__MEAS_READY,
  DGS2_STATE__ERROR
} Dgs2_State_t;

typedef enum{
  DGS2_INIT__NOT_INITIALIZED,
  DGS2_INIT__IN_PROCESS,
  DGS2_INIT__INITIALIZED
} Dgs2_Init_t;

typedef struct{
  int16_t so2Ppb;       // ppb
  int16_t temp_x100;    // C/100
  int16_t rh_x100;      // %RH/100
} Dgs2_Data_t;

typedef struct{
  Dgs2_State_t  currentState;
  Dgs2_Init_t   initState;
  bool          isMeasReady;
  Dgs2_Data_t   data;
  uint8_t       buf[256];
  uint8_t       bufLen;
} Dgs2_Config_t;


sl_status_t Dgs2_RunStateMachine(void);
void Dgs2_SetState(Dgs2_State_t state);
sl_status_t Dgs2_Handler(Comm_Msg_t msg);
void Dgs2_InitializeConfiguration(void);
bool Dgs2_isInitialized(void);


#endif /* DGS2_SO2_H_ */
