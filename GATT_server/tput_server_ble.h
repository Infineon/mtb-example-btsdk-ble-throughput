/******************************************************************************
* File Name:   tput_server_ble.h
*
* Description: Header file for tput_server_ble.c
*
* Related Document: See README.md
*
*******************************************************************************
* (c) 2020, Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
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
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*******************************************************************************/

#ifndef TPUT_SERVER_BLE_H_
#define TPUT_SERVER_BLE_H_

#include "cycfg_gatt_db.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_trace.h"

/*****************************************************************************
 *                              EXTERNS and FUNCTIONS
 *****************************************************************************/
wiced_bt_dev_status_t           tput_management_callback( wiced_bt_management_evt_t, wiced_bt_management_evt_data_t* );
static void                     tput_app_sec_timeout( uint32_t );
static void                     tput_app_msec_timeout( uint32_t );
static void                     tput_set_advertisement_data( void );
static void                     tput_send_notification( void );
static void                     tput_init( void );
static void                     tput_gatts_connection_down( void );
static void                     tput_gatts_connection_up( wiced_bt_gatt_connection_status_t *p_status );
static void                     tput_gatts_conn_status_cb( wiced_bt_gatt_connection_status_t *p_status );
static wiced_bt_gatt_status_t   tput_gatts_callback( wiced_bt_gatt_evt_t, wiced_bt_gatt_event_data_t* );
static wiced_bt_gatt_status_t   tput_gatts_req_cb( wiced_bt_gatt_attribute_request_t *p_data );
static wiced_bt_gatt_status_t   tput_gatts_req_write_handler( uint16_t conn_id, wiced_bt_gatt_write_t * p_data );
static wiced_bt_gatt_status_t   tput_gatts_req_read_handler( uint16_t conn_id, wiced_bt_gatt_read_t * p_read_data );
static gatt_db_lookup_table_t*  tput_get_attribute( uint16_t handle );
static uint16_t                 get_notification_packet_size( uint16_t att_mtu_size );

/******************************************************************************
 *                               Structures
 ******************************************************************************/
typedef struct
{
    BD_ADDR     remote_addr;  // remote peer device address
    uint16_t    conn_id;      // connection ID referenced by the stack
    uint16_t    peer_mtu;     // peer MTU
} tput_conn_state_t;

extern const wiced_bt_cfg_settings_t     wiced_bt_cfg_settings;
extern const wiced_bt_cfg_buf_pool_t     wiced_bt_cfg_buf_pools[];


/******************************************************************************
 *                           Constants and Enumerations
 ******************************************************************************/
#define CONNECTION_INTERVAL                 (21)  // (21 * 1.25) = 26.25ms
#define ADV_ELEM_LEN                        (3)   // GATT advertisement element length
#define MINIMUM_TX_BUFFER_LEN               (1)   // Minimum tx buffer length
#define BLE_CONN_TIMEOUT                    (512) // BLE connection timeout value
#define CONN_INTERVAL_MAJOR(a)              (a / 100)
#define CONN_INTERVAL_MINOR(a)              (a % 100)
#define CONN_INTERVAL_MULTIPLIER            (125)
#define ONE_BYTE                            (8)
#define SECONDS_TIMER_TIMEOUT               (1)
#define MILLISECOND_TIMER_TIMEOUT           (1)
#define GATT_CONNECT_LED                    LED1
#define CONGESTION_LED                      LED2

#endif /* TPUT_SERVER_BLE_H_ */
