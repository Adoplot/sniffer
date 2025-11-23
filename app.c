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

#include "rpi_drv.h"
#include "scd41_co2.h"
#include "lora_drv.h"
#include "scd41_co2.h"
#include "dfr_so2.h"
#include "manager.h"
#include "app_log.h"
#include "communication.h"
#include <string.h>
#include "uart_drv.h"
#include "sl_sleeptimer.h"  //TODO remove
#include <limits.h> //TODO remove


/***************************************************************************//**
 * Initialize application.
 ******************************************************************************/
void app_init(void)
{
  app_log("....Starting application    ");
  UART_Init_State_Handles();
  RPI_Init_State_Handles();
  Scd41_InitializeConfiguration();
  Dfr_InitializeConfiguration();
  Manager_InitializeConfiguration();

  char *buf;
  buf = "initiate_recv\n";
  UART_Send(sl_uartdrv_eusart_rpi_handle, (uint8_t*)buf, 14);

  //SENS_setState(COMM_DEVICE_SO2, SENS_STATUS_SEND_REQUEST);
}

/***************************************************************************//**
 * App ticking function.
 ******************************************************************************/
void app_process_action(void)
{
  UART_runStateMachine(sl_uartdrv_eusart_rpi_handle);
  UART_runStateMachine(sl_uartdrv_eusart_lora_handle);
  UART_runStateMachine(sl_uartdrv_usart_so2_handle);

  Scd41_RunStateMachine();
  Dfr_RunStateMachine();

  Manager_RunStateMachine();
  //RPI_runStateMachine();
  Lora_RunStateMachine();
  //SENS_runStateMachine(COMM_DEVICE_SO2);
  //SENS_runStateMachine(COMM_DEVICE_CO2);


}
