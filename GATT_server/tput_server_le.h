/******************************************************************************
* File Name:   tput_server_le.h
*
* Description: This is the header file for tput_server_le.h. It contains macros
* and structures used by the functions in tput_server_le.c. It also contains
* function prototypes that can be used by other files.
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

#ifndef TPUT_SERVER_LE_H_
#define TPUT_SERVER_LE_H_

#include "cycfg_gatt_db.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_trace.h"

/******************************************************************************
 *                           Macros
 ******************************************************************************/
/* Enable VERBOSE_THROUGHPUT_OUTPUT using WICED_TRUE to have verbose messages on
 * console for debugging */
#define VERBOSE_THROUGHPUT_OUTPUT           WICED_FALSE

/* GATT advertisement element length */
#define ADV_ELEMENT_LEN                     (3u)
/* Actual Connection interval: (21 * 1.25) = 26.25ms */
#define CONNECTION_INTERVAL                 (21u)
/* Minimum tx buffer length to send notification*/
#define MINIMUM_TX_BUFFER_LEN               (1u)

#define SECONDS_TIMER_TIMEOUT               (1u)
#define GATT_CONNECT_LED                    LED1
#define NOTIFY_LED                          LED2

/******************************************************************************
 *                     Macros for Connection Parameters
 ******************************************************************************/
/* Bluetooth LE connection timeout value */
#define CONN_INTERVAL_TIMEOUT               (512u)
/* Connection interval multiplication factor to get actual interval */
#define CONN_INTERVAL_MULTIPLIER            (125u)
/* To print float value of connection interval after multiplying with the factor 125*/
#define CONN_INTERVAL_MAJOR(a)              (a / 100)
#define CONN_INTERVAL_MINOR(a)              (a % 100)

/******************************************************************************
 *                     Macros for RTOS Thread
 ******************************************************************************/
/* Send notification thread related settings */
#define NOTIFY_THREAD_PRIORITY              (1u)
#define NOTIFY_THREAD_SIZE                  (1024u)
#define NO_TX_THREAD_SLEEP_TIMEOUT          (1000u)
#define TX_THREAD_SLEEP_TIMEOUT             (5u)

/******************************************************************************
 *                     Macros for GATT operations
 ******************************************************************************/
/* Data packet sizes when 247 <= ATT MTU <= 498 */
#define DATA_PACKET_SIZE_1                  (244u)
#define DATA_PACKET_SIZE_2                  (495u)
#define ATT_HEADER                          (3u)

extern const wiced_bt_cfg_settings_t        wiced_bt_cfg_settings;
extern const wiced_bt_cfg_buf_pool_t        wiced_bt_cfg_buf_pools[];

/*****************************************************************************
 *                              Function Prototypes
 *****************************************************************************/
wiced_bt_dev_status_t tput_management_callback(wiced_bt_management_evt_t,
                                            wiced_bt_management_evt_data_t*);

/******************************************************************************
 *                              Structures
 ******************************************************************************/
typedef struct
{
    BD_ADDR     remote_addr;  /* remote peer device address */
    uint16_t    conn_id;      /* connection ID referenced by the stack */
    uint16_t    mtu;          /* MTU for the connection */
} tput_conn_state_t;

#endif /* TPUT_SERVER_LE_H_ */
