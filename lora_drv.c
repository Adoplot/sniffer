/*
 * Handles communication with rak3272s LoRa PTP
 */
#include "lora_drv.h"
#include "app_log.h"
#include "communication.h"
#include <string.h>
#include "uart_drv.h"
#include "sens_drv.h"


#define LORA_PARAM_PFREQ	"868000000"   //Frequency
#define LORA_PARAM_PSF  	"9"           //Spreading factor
#define LORA_PARAM_PBW  	"125"         //Bandwidth
#define LORA_PARAM_PCR  	"0"           //Code rate
#define LORA_PARAM_PPL  	"8"           //Preamble length
#define LORA_PARAM_PTP  	"15"          //TX power
#define LORA_PARAM_ALLTOGETHER LORA_PARAM_PFREQ ":" LORA_PARAM_PSF ":" LORA_PARAM_PBW ":" LORA_PARAM_PCR ":" LORA_PARAM_PPL ":" LORA_PARAM_PTP


typedef struct {
  char *data;
  uint8_t length;
} Lora_CmdBuf_t;

typedef enum{
  LORA_CMD__CONFIGURE_P2P,
  LORA_CMD__PSEND,
  LORA_CMD__OK,
  LORA_CMD__SEND_COMPLETE,
  LORA_CMD__PARAM_ERROR,
  LORA_CMD__UNKNOWN,
  LORA_CMD__COUNT
} Lora_Cmd_t;


const Lora_CmdBuf_t lora_get_cmd_buf[LORA_CMD__COUNT] = {
    { "AT+P2P=" LORA_PARAM_ALLTOGETHER,   sizeof("AT+P2P=" LORA_PARAM_ALLTOGETHER)},
    { "AT_PSEND=",                        sizeof("AT_PSEND=")},
    { "\r\nOK\r\n",                       sizeof("\r\nOK\r\n")},
    { "\r\n+EVT:TXP2P DONE\r\n",          sizeof("\r\n+EVT:TXP2P DONE\r\n")},
    { "\r\nAT_PARAM_ERROR\r\n",           sizeof("\r\nAT_PARAM_ERROR\r\n")},
};

static Lora_Config_t hLoraConfig;

//static sl_status_t lora_uart_send(Lora_Cmd_t cmd, char *payload, uint8_t payload_len);
//static sl_status_t lora_init_handler();


sl_status_t Lora_RunStateMachine(){
  sl_status_t status = SL_STATUS_FAIL;

  switch (hLoraConfig.currentState){

    case LORA_STATUS__IDLE:
      //do nothing
      status = SL_STATUS_OK;
      break;


    case LORA_STATUS__SEND_DATA:
      //get data from sensors
      Sens_GetSensorData();
      status = SL_STATUS_OK;
      break;


    case LORA_STATUS__SENDING:
      status = SL_STATUS_OK;
      break;


    case LORA_STATUS__SENT_SUCCESSFUL:
      status = SL_STATUS_OK;
      break;


    case LORA_STATUS__ERROR:
      status = SL_STATUS_OK;
      break;
  }

  return status;
}


void Lora_SetState(Lora_State_t state){
  hLoraConfig.currentState = state;
}


void LORA_Init_State_Handles(){
  hLoraConfig.loraBufLen = 0;
  hLoraConfig.currentState = LORA_STATUS__IDLE;
  hLoraConfig.initState = LORA_INIT__NOT_INITIALIZED;
}


//TODO Lora initialization check
bool LORA_isInitialized(){
  return true;
}


//TODO
#if 0
sl_status_t lora_init_handler(){
  sl_status_t status;

  if (hLoraConfig.initState == LORA_INIT__NOT_INITIALIZED){
      status = lora_uart_send(LORA_CMD__CONFIGURE_P2P, "send_check", 10);
  }
  else{
      app_log_warning("lora already initialized");
      return SL_STATUS_OK;
  }

  return status;
}
#endif


Lora_Cmd_t parse_lora_cmd(const uint8_t *buf, const uint8_t buf_len) {
  if (strncmp((char*)buf, "\r\nOK\r\n", buf_len) == 0)                return LORA_CMD__OK;
  if (strncmp((char*)buf, "\r\n+EVT:TXP2P DONE\r\n", buf_len) == 0)   return LORA_CMD__SEND_COMPLETE;
  if (strncmp((char*)buf, "init_check\n", buf_len) == 0)              return LORA_CMD__PARAM_ERROR;
  return LORA_CMD__UNKNOWN;
}


//TODO: add uart recv, and LORA_stateMachine?
//TODO: handle received uart cmd
sl_status_t LORA_Handler(Comm_Msg_t msg){
  (void)msg;
  return SL_STATUS_FAIL;

}


