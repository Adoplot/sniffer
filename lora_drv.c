/*
 * Handles communication with rak3272s LoRa PTP
 */
#include <lora_drv.h>
#include "app_log.h"
#include "communication.h"
#include <string.h>
#include <uart_drv.h>


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
} Lora_Cmd_Buf_t;

typedef enum{
  LORA_CMD_CONFIGURE_P2P,
  LORA_CMD_PSEND,
  LORA_CMD_OK,
  LORA_CMD_SEND_COMPLETE,
  LORA_CMD_PARAM_ERROR,
  LORA_CMD_UNKNOWN,
  LORA_CMD_COUNT
} Lora_Cmd_t;


const Lora_Cmd_Buf_t lora_get_cmd_buf[LORA_CMD_COUNT] = {
    { "AT+P2P=" LORA_PARAM_ALLTOGETHER,   sizeof("AT+P2P=" LORA_PARAM_ALLTOGETHER)},
    { "AT_PSEND=",                        sizeof("AT_PSEND=")},
    { "\r\nOK\r\n",                       sizeof("\r\nOK\r\n")},
    { "\r\n+EVT:TXP2P DONE\r\n",          sizeof("\r\n+EVT:TXP2P DONE\r\n")},
    { "\r\nAT_PARAM_ERROR\r\n",           sizeof("\r\nAT_PARAM_ERROR\r\n")},
};


static Lora_Status_t lora_status;
static Lora_Init_t lora_init_status;

static sl_status_t lora_uart_send(Lora_Cmd_t cmd, char *payload, uint8_t payload_len);
static sl_status_t lora_init_handler();


void LORA_Init_State_Handles(){
  lora_status = LORA_STATUS_IDLE;
  lora_init_status = LORA_INIT_NOT_INITIALIZED;
}

//TODO Lora initialization check
bool LORA_isInitialized(){
  return true;
}


sl_status_t lora_init_handler(){
  sl_status_t status;

  if (lora_init_status == LORA_INIT_NOT_INITIALIZED){
      status = lora_uart_send(LORA_CMD_CONFIGURE_P2P, "send_check", 10);
  }
  else{
      app_log_warning("lora already initialized");
      return SL_STATUS_OK;
  }

  return status;
}


Lora_Cmd_t parse_lora_cmd(const uint8_t *buf, const uint8_t buf_len) {
  if (strncmp((char*)buf, "\r\nOK\r\n", buf_len) == 0)                return LORA_CMD_OK;
  if (strncmp((char*)buf, "\r\n+EVT:TXP2P DONE\r\n", buf_len) == 0)   return LORA_CMD_SEND_COMPLETE;
  if (strncmp((char*)buf, "init_check\n", buf_len) == 0)              return LORA_CMD_PARAM_ERROR;
  return LORA_CMD_UNKNOWN;
}


//TODO: add uart recv, and LORA_stateMachine?
//TODO: handle received uart cmd
sl_status_t LORA_Handler(Comm_Msg_t msg){

  return SL_STATUS_FAIL;

}


