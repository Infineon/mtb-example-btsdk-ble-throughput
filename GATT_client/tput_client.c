/******************************************************************************
* File Name:   tput_client.c
*
* Description: This file contains APPLICATION_START(), which is the starting
*              point of code execution. In this file, PUART is initialized for
*              UART trace messages and bluetooth controller and host stack are
*              initialized.
*
* Related Document: See README.md
*
*******************************************************************************
* Copyright 2020-2021, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
******************************************************************************/

#include "wiced_bt_trace.h"
#include "wiced_bt_stack.h"
#include "app_bt_cfg.h"
#include "tput_client_le.h"
#include "sparcommon.h"

/*******************************************************************************
* Function Name: application_start
********************************************************************************
* Summary: Initialize Transport configuration; Register callback for Bluetooth
*          LE management events.
*
* Parameters:
*   None
*
* Return:
*   None
*
*******************************************************************************/
void application_start(void)
{
    wiced_result_t status = WICED_BT_SUCCESS;

    wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_PUART);
    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    WICED_BT_TRACE("\x1b[2J\x1b[;H");

    /* Initialize Bluetooth controller and host stack */
    status = wiced_bt_stack_init(tput_management_callback,
                                 &wiced_bt_cfg_settings,
                                 wiced_bt_cfg_buf_pools);
    if (WICED_BT_SUCCESS != status)
    {
        WICED_BT_TRACE("Bluetooth LE Stack initialization failure\r\n");
    }
    else
    {
        WICED_BT_TRACE("*********** BLUETOOTH LE THROUGHPUT MEASUREMENT : "
        "CLIENT DEVICE *********** \r\n");
        WICED_BT_TRACE("Press Switch SW3 on your kit to start scanning...\r\n");
    }

}

