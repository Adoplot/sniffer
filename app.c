/***************************************************************************//**
 * @file
 * @brief Top level application functions
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

#include <rpi_drv.h>
#include <sens_drv.h>
#include "app_log.h"
#include "communication.h"
#include <string.h>
#include <uart_drv.h>


/***************************************************************************//**
 * Initialize application.
 ******************************************************************************/
void app_init(void)
{
  app_log("....Starting application    ");
  UART_Init_State_Handles();
  RPI_Init_State_Handles();
  SENS_Init_State_Handles();

  char *buf;
  buf = "initiate_recv\n";
  UART_Send(sl_uartdrv_eusart_rpi_handle, (uint8_t*)buf, 14);

  //SENS_setState(COMM_DEVICE_SO2, SENS_STATUS_SEND_REQUEST);
#if 0
  char *pBuf = "start:123456789\n";
  uint8_t buf_len = 16;
  Comm_Msg_t msg;
  memcpy(msg.data.buffer, pBuf, buf_len);
  msg.data.buf_len = buf_len;
  msg.device = COMM_DEVICE_RPI;
  msg.result = COMM_RESULT_SUCCESS;

  Rpi_SetStatusIdle();
  RPI_Handler(msg);
#endif
}

/***************************************************************************//**
 * App ticking function.
 ******************************************************************************/
void app_process_action(void)
{
  UART_runStateMachine(sl_uartdrv_eusart_rpi_handle);
  //UART_runStateMachine(sl_uartdrv_eusart_lora_handle);
  UART_runStateMachine(sl_uartdrv_usart_so2_handle);

  RPI_runStateMachine();

  SENS_runStateMachine(COMM_DEVICE_SO2);
  //SENS_runStateMachine(COMM_DEVICE_CO2);


}
