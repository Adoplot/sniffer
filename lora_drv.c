/*
 * Handles communication with rak3272s LoRa PTP
 */
#include "lora_drv.h"
#include "manager.h"
#include "app_log.h"
#include "communication.h"
#include <string.h>
#include "uart_drv.h"


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

typedef struct {
  Lora_Cmd_t cmd;
  Lora_CmdBuf_t cmd_data;
} Lora_Msg_t;

const Lora_CmdBuf_t lora_getCmdBuf[LORA_CMD__COUNT] = {
    { "AT+P2P=" LORA_PARAM_ALLTOGETHER,   sizeof("AT+P2P=" LORA_PARAM_ALLTOGETHER)},
    { "AT+PSEND=",                        sizeof("AT_PSEND=")},
    { "\r\nOK\r\n",                       sizeof("\r\nOK\r\n")},
    { "\r\n+EVT:TXP2P DONE\r\n",          sizeof("\r\n+EVT:TXP2P DONE\r\n")},
    { "\r\nAT_PARAM_ERROR\r\n",           sizeof("\r\nAT_PARAM_ERROR\r\n")},
};

static Lora_Config_t hLoraConfig;

static sl_status_t Lora_send(Lora_Cmd_t cmd, uint8_t *payload, uint8_t payload_len);
//static sl_status_t lora_init_handler();
static Lora_Cmd_t Lora_parseCmd(const uint8_t *buf, const uint8_t buf_len);
static void Lora_bytes2Hex(const uint8_t *bytes, uint16_t len, uint8_t *out_hex, uint8_t *out_hexLen);


sl_status_t Lora_RunStateMachine(){
  sl_status_t status = SL_STATUS_FAIL;
  Manager_SensorData_t sensorData;

  switch (hLoraConfig.currentState){

    case LORA_STATUS__IDLE:
      //do nothing
      status = SL_STATUS_OK;
      break;


    case LORA_STATUS__SEND_DATA:
      //pack all data to payload
      Manager_GetData(&sensorData);
      Lora_bytes2Hex((const uint8_t*)&sensorData, sizeof(sensorData), hLoraConfig.payload, &hLoraConfig.payload_len);

      //send it
      Lora_send(LORA_CMD__PSEND, hLoraConfig.payload, hLoraConfig.payload_len);

      hLoraConfig.currentState = LORA_STATUS__SENDING;
      status = SL_STATUS_OK;
      break;


    case LORA_STATUS__SENDING:
      //wait till lora_handler confirms msg sent successfully
      status = SL_STATUS_OK;
      break;


    case LORA_STATUS__SENT_SUCCESSFUL:
      Manager_SetState(MANAGER_STATUS_SENT);
      hLoraConfig.currentState = LORA_STATUS__IDLE;
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


void LORA_InitializeConfiguration(){
  hLoraConfig.currentState = LORA_STATUS__IDLE;
  hLoraConfig.initState = LORA_INIT__INITIALIZED; //TODO initialized from start
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


/***************************************************************************//**
 * @brief
 *    Concatenates cmd and payload in one buf and sends it to LoRa module via
 *    UART
 ******************************************************************************/
sl_status_t Lora_send(Lora_Cmd_t cmd, uint8_t *payload, uint8_t payload_len){
  sl_status_t status = SL_STATUS_FAIL;

  const Lora_CmdBuf_t msg = lora_getCmdBuf[cmd];

  const char lineEnd[2] = {0x0D,0x0A};
  char buf[256];  //TODO payload can be 256 bytes + cmd buf + \CR\LF
  uint16_t total_len = msg.length - 1 + payload_len + 2;

  if (total_len <= sizeof(buf)){
      status = SL_STATUS_OK;
  }

  if (status == SL_STATUS_OK){
      memcpy(buf, msg.data, msg.length - 1);  // copy without '\0'
      memcpy(&buf[msg.length - 1], payload, payload_len);
      memcpy(&buf[msg.length - 1 + payload_len], lineEnd, 2);

      status = UART_Send(sl_uartdrv_eusart_lora_handle, (uint8_t*)buf, total_len);
  }

  return status;
}


/***************************************************************************//**
 * @brief
 *  Converts bytes to string of HEX
 ******************************************************************************/
void Lora_bytes2Hex(const uint8_t *bytes, uint16_t len, uint8_t *out_hex, uint8_t *out_hexLen){
  //Check if packed struct is still 10bytes
  _Static_assert(sizeof(Manager_SensorData_t) == 10, "Manager_SensorData_t must be 10 bytes");

  static const uint8_t hex_digits[] = "0123456789ABCDEF";

  for (uint16_t i = 0; i < len; i++) {
      uint8_t b = bytes[i];

      //0x0F keeps only the lower 4 bits (a value from 0 to 15)
      out_hex[2*i]     = hex_digits[(b >> 4) & 0x0F]; //convert upper 4 bits to HEX char
      out_hex[2*i + 1] = hex_digits[b & 0x0F];  //convert lower 4 bits to HEX char
  }

  //out_hex[2*len] = '\0';  // null-terminate for sending as C string
  *out_hexLen = 2*len;
}


Lora_Cmd_t Lora_parseCmd(const uint8_t *buf, const uint8_t buf_len) {
  if (strncmp((char*)buf, "OK\r", buf_len) == 0)                      return LORA_CMD__OK;
  if (strncmp((char*)buf, "\r\n+EVT:TXP2P DONE\r\n", buf_len) == 0)   return LORA_CMD__SEND_COMPLETE;
  if (strncmp((char*)buf, "\r\nAT_PARAM_ERROR\r\n", buf_len) == 0)    return LORA_CMD__PARAM_ERROR;
  return LORA_CMD__UNKNOWN;
}


sl_status_t Lora_Handler(Comm_Msg_t msg, uint8_t *pBuf, uint8_t *pBufLen){
  (void)pBuf;
  (void)pBufLen;
  Lora_Msg_t lora_msg;
  lora_msg.cmd = Lora_parseCmd(msg.data.buffer, msg.data.buf_len);

  if (lora_msg.cmd == LORA_CMD__OK){

      hLoraConfig.currentState = LORA_STATUS__SENT_SUCCESSFUL;
  }
  else{
      hLoraConfig.currentState = LORA_STATUS__ERROR;
  }

  return SL_STATUS_OK;
}


