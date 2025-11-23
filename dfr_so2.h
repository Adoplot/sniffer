/*
 * dgs2_so2.h
 *
 *  Driver for SO2 sensor on DFR board
 */

#ifndef DFR_SO2_H_
#define DFR_SO2_H_

#include <stdint.h>
#include "app_log.h"
#include "communication.h"

typedef enum{
  DFR_STATE__IDLE,
  DFR_STATE__SEND_REQUEST,
  DFR_STATE__REQUESTED,
  DFR_STATE__MEAS_READY,
  DFR_STATE__ERROR
} Dfr_State_t;

typedef enum{
  DFR_INIT__NOT_INITIALIZED,
  DFR_INIT__IN_PROCESS,
  DFR_INIT__INITIALIZED
} Dfr_Init_t;

typedef struct{
  int16_t so2Ppb;       // ppb
  int16_t temp_x100;    // C/100
  int16_t rh_x100;      // %RH/100
} Dfr_Data_t;

typedef struct{
  Dfr_State_t  currentState;
  Dfr_Init_t   initState;
  bool          isMeasReady;
  Dfr_Data_t   data;
  uint8_t       buf[256];
  uint8_t       bufLen;
} Dfr_Config_t;


sl_status_t Dfr_RunStateMachine(void);
void Dfr_SetState(Dfr_State_t state);
sl_status_t Dfr_Handler(Comm_Msg_t msg);
void Dfr_InitializeConfiguration(void);
bool Dfr_isInitialized(void);

#endif /* DFR_SO2_H_ */
