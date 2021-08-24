/******************************************************************************
* File Name:   tput_client_le.h
*
* Description: This is the header file for tput_client_le.h. It contains macros,
* enumns and structures used by the functions in tput_client_le.c. It also
* contains function prototypes that can be used by other files.
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

#ifndef TPUT_CLIENT_LE_H_
#define TPUT_CLIENT_LE_H_

/******************************************************************************
 *                                Macros
 ******************************************************************************/
/* Enable VERBOSE_THROUGHPUT_OUTPUT using WICED_TRUE to have verbose messages on
 * console for debugging */
#define VERBOSE_THROUGHPUT_OUTPUT        WICED_FALSE
/* Print throughput output every one second */
#define TIMER_PRINT_TPUT                 (1u)
/* Minimum tx buffer length to send gatt write */
#define MINIMUM_TX_BUFFER_LEN            (1u)

#define GATT_CONNECT_LED                 LED1
#define GATT_WRITE_LED                   LED2
/******************************************************************************
 *                     Macros for Connection Parameters
 ******************************************************************************/
/* Connection interval multiplication factor to get actual interval */
#define CONN_INTERVAL_MULTIPLIER         (125u)
/* To print float value of connection interval after multiplying with the factor 125*/
#define CONN_INTERVAL_MAJOR(a)           (a / 100)
#define CONN_INTERVAL_MINOR(a)           (a % 100)

/******************************************************************************
 *                     Macros for RTOS Thread
 ******************************************************************************/
#define SEND_GATT_WRITE_THREAD_PRIORITY  (1u)
#define SEND_GATT_WRITE_THREAD_SIZE      (1024u)
#define NO_TX_THREAD_SLEEP_TIMEOUT       (1000u)

/******************************************************************************
 *                     Macros for GATT operations
 ******************************************************************************/
/* CCCD handle in GATT DB */
#define GATT_CCCD_HANDLE                 (3u)
#define CCCD_LENGTH                      (2u)
#define CCCD_WRITE_MAX_RETRIES           (5u)
/* Handle to write field in GATT DB */
#define GATT_WRITE_HANDLE                (5u)
#define GATT_WRITE_BYTES_MAX_LEN         (495u)
#define TPUT_SERVICE_UUID                {0xCCu, 0x7Bu, 0xCBu, 0x32u, 0x07u, \
                                          0x08u, 0x17u, 0xAFu, 0xD3u, 0x43u, \
                                          0x1Eu, 0x5Du, 0x20u, 0x0Du, 0xECu, \
                                          0x1Au}
/* Data packet sizes when 247 <= ATT MTU <= 498 */
#define DATA_PACKET_SIZE_1               (244u)
#define DATA_PACKET_SIZE_2               GATT_WRITE_BYTES_MAX_LEN
#define ATT_HEADER                       (3u)

/*******************************************************************************
 *                         Function Prototypes
 ******************************************************************************/
wiced_bt_dev_status_t tput_management_callback(wiced_bt_management_evt_t,
                                               wiced_bt_management_evt_data_t*);

/******************************************************************************
 *                      Structures and Enumerations
 ******************************************************************************/
typedef struct
{
    BD_ADDR remote_addr;   /* remote peer device address */
    uint16_t conn_id;      /* connection ID referenced by the stack */
    uint16_t mtu;          /* MTU used for the connection */
} tput_conn_state_t;

typedef enum
{
    GATT_NOTIFY_SERVER_TO_CLIENT,       /* Server to Client Notifications */
    GATT_WRITE_CLIENT_TO_SERVER,        /* Client to Server GATT writes   */
    GATT_NOTIFY_AND_WRITE               /* Both Notifications and GATT writes */
}tput_mode_t;

typedef struct
{
    uint32_t gatt_write_tx_bytes;   /* Successful number of GATT write command bytes */
    uint8_t enable_cccd;            /* Flag to indicate whether to enable notifications */
    uint8_t gatt_write_tx;          /* Flag to inidicate whether to send GATT write commands */
    uint16_t packet_size;           /* Packet size decided based on MTU exchanged */
    wiced_bt_gatt_value_t *p_write; /* Pointer to memory which holds all information required to send a GATT write command*/
} tput_client_tx_data_struct_t;

typedef struct
{
    wiced_bool_t button_pressed; /* Flag to indicate button press by user */
    wiced_bool_t scan_flag;      /* Flag to Scan only for first button press */
    tput_mode_t mode_flag;       /* Variable to switch between different data transfer modes */
}tput_client_flags_t;

#endif /* TPUT_CLIENT_LE_H_ */
