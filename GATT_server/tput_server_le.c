/******************************************************************************
* File Name:   tput_server_le.c
*
* Description: This file has implementation for functions which do the following:
*              * Bluettoth LE initialization(registering gatt db, gatt callback,
*                setting adv data etc..)
*              * Bluetooth LE stack event handler - to process the Bluetooth
*                   LE events.
*              * 1 second timer init and callback - for throughput calculation.
*              * RTOS thread to send notifications when enabled to the client
*                device.
*
* Related Document: See README.md
*
*******************************************************************************
* Copyright 2020-2022, Cypress Semiconductor Corporation (an Infineon company) or
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

#include "wiced_bt_dev.h"
#include "wiced_bt_cfg.h"
#include "wiced_transport.h"
#include "wiced_bt_l2c.h"
#include "wiced_bt_trace.h"
#include "cycfg_pins.h"
#include "sparcommon.h"
#include "wiced_result.h"
#include "wiced_bt_stack.h"
#include "wiced_timer.h"
#include "app_bt_cfg.h"
#include "wiced_gki.h"
#include "tput_server_le.h"
#include "wiced_hal_wdog.h"
#include "wiced_rtos.h"

/*******************************************************************************
 *                                Variable Definitions
 ******************************************************************************/
/* Variables to hold GATT notification bytes sent and GATT Write bytes received
 * successfully*/
static uint32_t             gatt_notify_tx_bytes  = 0;
static uint32_t             gatt_write_rx_bytes  = 0;
/* No of bytes rejected for transfer */
static uint32_t             failed_to_send       = 0;
/* Application seconds timer */
static wiced_timer_t        tput_app_sec_timer;
/* Variable to store peer device details after connection */
static tput_conn_state_t    tput_conn_state;
/* Variable to store packet size decided based on MTU exchanged */
static uint16_t             packet_size          = 0;
/* Pointer to store memory allocated to the new thread while thread creation */
wiced_thread_t*             notify_thread    = NULL;

/*******************************************************************************
 *                                Function Prototypes
 ******************************************************************************/
static void tput_app_sec_timeout(uint32_t cb_param);
static void tput_set_advertisement_data(void);
static void tput_server_init(void);
static void tput_gatts_connection_down(wiced_bt_gatt_connection_status_t *p_status);
static void tput_gatts_connection_up(wiced_bt_gatt_connection_status_t *p_status);
static void tput_gatts_conn_status_cb(wiced_bt_gatt_connection_status_t *p_status);

static wiced_bt_gatt_status_t tput_gatts_callback(wiced_bt_gatt_evt_t,
                                                  wiced_bt_gatt_event_data_t*);
static wiced_bt_gatt_status_t tput_gatts_req_cb(wiced_bt_gatt_attribute_request_t *p_data);
static wiced_bt_gatt_status_t tput_gatts_req_write_handler(uint16_t conn_id,
                                             wiced_bt_gatt_write_t *p_data);
static wiced_bt_gatt_status_t tput_gatts_req_read_handler(uint16_t conn_id,
                                        wiced_bt_gatt_read_t *p_read_data);
static wiced_bt_gatt_status_t tput_gatts_att_get_value(uint16_t attr_handle,
                                                       uint16_t conn_id,
                                                       uint8_t *p_val,
                                                       uint16_t max_len,
                                                       uint16_t *p_len);
static wiced_bt_gatt_status_t tput_gatts_att_set_value(uint16_t attr_handle,
                                                       uint16_t conn_id,
                                                       uint8_t *p_val,
                                                       uint16_t len);
static uint16_t get_notification_packet_size(uint16_t att_mtu_size);
static void tput_notify_thread(uint32_t thread_arg);

/*******************************************************************************
 *                                Function Definitions
 ******************************************************************************/

/*******************************************************************************
* Function Name: tput_management_callback
********************************************************************************
* Summary:
*   This is a Bluetooth management event handler function to receive events from
*   Bluetooth LE stack and process as needed by the application.
*
* Parameters:
*   wiced_bt_management_evt_t event             : Bluetooth LE event code of
*                                                 one byte length
*   wiced_bt_management_evt_data_t *p_event_data: Pointer to Bluetooth LE
*                                                management event structures
*
* Return:
*  wiced_result_t: Error code from WICED_RESULT_LIST or BT_RESULT_LIST
*
*******************************************************************************/
wiced_result_t tput_management_callback(wiced_bt_management_evt_t event,
                                        wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_result_t result = WICED_BT_SUCCESS;
    uint16_t conn_interval = 0;

#if VERBOSE_THROUGHPUT_OUTPUT
    WICED_BT_TRACE("Throughput GATT server management callback event: %d\r\n", event);
#endif

    switch (event)
    {
        case BTM_ENABLED_EVT:
            /* Handle initialize sequence for the application */
            tput_server_init();
            break;

        case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
            /* No IO capabilities on this platform */
            p_event_data->pairing_io_capabilities_ble_request.local_io_cap = BTM_IO_CAPABILITIES_NONE;
            p_event_data->pairing_io_capabilities_ble_request.oob_data     = BTM_OOB_NONE;
            p_event_data->pairing_io_capabilities_ble_request.auth_req     = BTM_LE_AUTH_REQ_NO_BOND;
            p_event_data->pairing_io_capabilities_ble_request.max_key_size = 0x10;
            p_event_data->pairing_io_capabilities_ble_request.init_keys    = 0;
            p_event_data->pairing_io_capabilities_ble_request.resp_keys    = BTM_LE_KEY_PENC|BTM_LE_KEY_PID;
            break;

        case BTM_BLE_ADVERT_STATE_CHANGED_EVT:
#if VERBOSE_THROUGHPUT_OUTPUT
            if(p_event_data->ble_advert_state_changed == BTM_BLE_ADVERT_OFF)
            {
                WICED_BT_TRACE("Advertisement State Change: OFF\r\n");
            }
            else if(p_event_data->ble_advert_state_changed == BTM_BLE_ADVERT_UNDIRECTED_HIGH)
            {
                WICED_BT_TRACE("Advertisement State Change: Undirected High\r\n");
            }
            else if(p_event_data->ble_advert_state_changed == BTM_BLE_ADVERT_UNDIRECTED_LOW)
            {
                WICED_BT_TRACE("Advertisement State Change: Undirected Low\r\n");
            }
#endif
            break;

        case BTM_BLE_PHY_UPDATE_EVT:
            WICED_BT_TRACE("Selected TX PHY - %dM\r\nSelected RX PHY - %dM\r\n",
                            p_event_data->ble_phy_update_event.tx_phy,
                            p_event_data->ble_phy_update_event.rx_phy);

            /* send connection interval update if required */
            wiced_bt_l2cap_update_ble_conn_params(tput_conn_state.remote_addr,
                                                          CONNECTION_INTERVAL,
                                                          CONNECTION_INTERVAL,
                                                                            0,
                                                        CONN_INTERVAL_TIMEOUT);

            WICED_BT_TRACE("Connection Interval update requested \r\n");
            break;

        case BTM_BLE_CONNECTION_PARAM_UPDATE:
            /* Connection parameters updated */
            if(WICED_SUCCESS == p_event_data->ble_connection_param_update.status)
            {
                conn_interval = (p_event_data->ble_connection_param_update.conn_interval)
                                * CONN_INTERVAL_MULTIPLIER;
                WICED_BT_TRACE("New connection parameters:"
                               "\r\nConnection interval = %d.%dms\r\n",
                               CONN_INTERVAL_MAJOR(conn_interval),
                               CONN_INTERVAL_MINOR(conn_interval));
            }
            else
            {
                WICED_BT_TRACE("Connection parameters update failed, Error Code: %d\r\n",
                                p_event_data->ble_connection_param_update.status);
            }
            break;

        default:
#if VERBOSE_THROUGHPUT_OUTPUT
            WICED_BT_TRACE("Unhandled management callback event: %d\r\n", event);
#endif
            break;
    }

    return (result);
}

/*******************************************************************************
* Function Name: tput_server_init
********************************************************************************
* Summary:
*   This function initializes/registers the interrupt(s), a timer, an rtos
*   thread, GATT database as needed by the application.
*
* Parameters:
*   None
*
* Return:
*   None
*
*******************************************************************************/
void tput_server_init(void)
{
    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_ERROR;
    wiced_result_t result = WICED_ERROR;

    /* Register with stack to receive GATT callback */
    gatt_status = wiced_bt_gatt_register(tput_gatts_callback);
    if (WICED_BT_GATT_SUCCESS != gatt_status)
    {
        WICED_BT_TRACE("GATT registration failed\r\n");
    }

    /* Tell the bluetooth stack to use our GATT database */
    gatt_status = wiced_bt_gatt_db_init(gatt_database, gatt_database_len);
    if (WICED_BT_GATT_SUCCESS != gatt_status)
    {
        WICED_BT_TRACE("GATT database init failed\r\n");
    }

    /* Create a thread to send notifications when enabled */
    notify_thread = wiced_rtos_create_thread();
    if(NULL != notify_thread)
    {
        result = wiced_rtos_init_thread(notify_thread,
                                        NOTIFY_THREAD_PRIORITY,
                                        "Send_notify",
                                        tput_notify_thread,
                                        NOTIFY_THREAD_SIZE,
                                        NULL);
        if(WICED_BT_SUCCESS != result)
        {
            WICED_BT_TRACE("RTOS Thread initialization failed!\r\n");
        }
    }
    else
    {
        WICED_BT_TRACE("RTOS Thread creation failed!\r\n");
    }

    /* Start the timer which will be used to calculate throughput and display it */
    if (WICED_BT_SUCCESS != wiced_init_timer(&tput_app_sec_timer,
                                            tput_app_sec_timeout,
                                            0,
                                            WICED_SECONDS_PERIODIC_TIMER))
    {
        WICED_BT_TRACE("Seconds timer init failed\r\n");
    }

    /* Pairing not needed for throughput test */
    wiced_bt_set_pairable_mode(WICED_FALSE, 0);

    /* Set the advertising parameters */
    tput_set_advertisement_data();

    /* Start advertisements */
    if(WICED_BT_SUCCESS == wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH,
                                                        BLE_ADDR_PUBLIC,
                                                        NULL))
    {
        WICED_BT_TRACE("Advertising......\r\n");
    }
}

/*******************************************************************************
* Function Name: tput_set_advertisement_data
********************************************************************************
* Summary:
*   Setup advertisement data with the device name and throughput
*   service uuid.
*
* Parameters:
*   None
*
* Return:
*   None
*
*******************************************************************************/
void tput_set_advertisement_data(void)
{
    wiced_bt_ble_advert_elem_t adv_elem[ADV_ELEMENT_LEN] = {0};
    uint8_t num_elem = 0;
    uint8_t flag = BTM_BLE_GENERAL_DISCOVERABLE_FLAG | BTM_BLE_BREDR_NOT_SUPPORTED;
    uint8_t tput_service_uuid[LEN_UUID_128] = {__UUID_SERVICE_THROUGHPUT_MEASUREMENT};

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_FLAG;
    adv_elem[num_elem].len          = sizeof(uint8_t);
    adv_elem[num_elem].p_data       = &flag;
    num_elem++;

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_128SRV_COMPLETE;
    adv_elem[num_elem].len          = LEN_UUID_128;
    adv_elem[num_elem].p_data       = tput_service_uuid;
    num_elem++;


    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE;
    adv_elem[num_elem].len          = app_gap_device_name_len;
    adv_elem[num_elem].p_data       = (uint8_t *)app_gap_device_name;
    num_elem++;

    if (WICED_BT_SUCCESS != wiced_bt_ble_set_raw_advertisement_data(num_elem, adv_elem))
    {
        WICED_BT_TRACE("Setup advertisement data failed\r\n");
    }
}

/*******************************************************************************
* Function Name: tput_app_sec_timeout
********************************************************************************
* Summary:
*   One second timer callback.
*   Print the TX and RX data throughput over PUART console.
*
* Parameters:
*   uint32_t cb_param: Variable registered by wiced_init_timer to pass to
*                      timer callback. Not used for now.
*
* Return:
*   None
*
*******************************************************************************/
void tput_app_sec_timeout(uint32_t cb_param)
{
    /* Calculate TX and RX throughput
     *
     * throughput(kbps) = (number of bytes sent/received in 1 second) * 8(bits)
     *                    -----------------------------------------------------
     *                                           1000
     */

    /* Display GATT TX throughput result */
    if ((tput_conn_state.conn_id) && (app_throughput_measurement_notify_descriptor[0]))
    {
        gatt_notify_tx_bytes = (gatt_notify_tx_bytes * 8)/1000;
        WICED_BT_TRACE("GATT NOTIFICATION : Server Throughput (TX)= %d kbps\r\n",
                          gatt_notify_tx_bytes);
#if VERBOSE_THROUGHPUT_OUTPUT
        WICED_BT_TRACE("Number of GATT notification Send API failure = %d\r\n"
                       , failed_to_send);
#endif
        /* Reset the GATT notification byte counter */
        gatt_notify_tx_bytes = 0;
        /* Reset the rejected byte counter */
        failed_to_send  = 0;
    }

    /* Display GATT RX throughput result */
    if (tput_conn_state.conn_id && gatt_write_rx_bytes)
    {
        gatt_write_rx_bytes = (gatt_write_rx_bytes * 8)/1000;
        WICED_BT_TRACE("GATT WRITE        : Server Throughput (RX)= %d kbps\r\n",
                          gatt_write_rx_bytes);
        /* Reset the GATT write byte counter */
        gatt_write_rx_bytes = 0;
    }
}

/*******************************************************************************
* Function Name: tput_notify_thread
********************************************************************************
* Summary:
*   Thread to send notifications to client device.
*
* Parameters:
*   uint32_t thread_arg: not used
*
* Return:
*   None
*
*******************************************************************************/
void tput_notify_thread(uint32_t thread_arg)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_SUCCESS;
    uint32_t buf_left = 0;

    while(WICED_TRUE)
    {
        if(!app_throughput_measurement_notify_descriptor[0])
        {
            /* Turn off LED2 indicating there is no data transfer form the device*/
            wiced_hal_gpio_set_pin_output(NOTIFY_LED , GPIO_PIN_OUTPUT_HIGH);

            /* Thread has nothing to do. Keep it in blocked state till the
               timeout specified */
            wiced_rtos_delay_milliseconds(NO_TX_THREAD_SLEEP_TIMEOUT,
                                          ALLOW_THREAD_TO_SLEEP);
        }
        else
        {
            /* Notifications are enabled. Send notification packet if Bluetooth
               LE TX buffer is available */
            buf_left = wiced_bt_ble_get_available_tx_buffers();
            if (buf_left > MINIMUM_TX_BUFFER_LEN)
            {
                status = wiced_bt_gatt_send_notification(tput_conn_state.conn_id,
                                        HDLC_THROUGHPUT_MEASUREMENT_NOTIFY_VALUE,
                                                                    packet_size,
                                            app_throughput_measurement_notify);
                if (status == WICED_BT_GATT_SUCCESS)
                {
                    /* Accumulate the successfully sent bytes to calculate
                       throughput after 1 second */
                    gatt_notify_tx_bytes += packet_size;
                    /* LED 2 ON to indicate data transfer from device */
                    wiced_hal_gpio_set_pin_output(NOTIFY_LED, GPIO_PIN_OUTPUT_LOW);
                }
                else
                {
                    failed_to_send++;
                    /* LED 2 OFF since TX failed */
                    wiced_hal_gpio_set_pin_output(NOTIFY_LED, GPIO_PIN_OUTPUT_HIGH);
                }
            }
            else
            {
                wiced_rtos_delay_milliseconds(TX_THREAD_SLEEP_TIMEOUT,
                                              ALLOW_THREAD_TO_SLEEP);
            }
            wiced_hal_wdog_restart();
        }
    }
}

/*******************************************************************************
 * Function Name: tput_gatts_callback
 *******************************************************************************
 * Summary: Callback for various GATT events.
 *
 * Parameters:
 *   wiced_bt_gatt_evt_t event          : GATT event code.
 *   wiced_bt_gatt_event_data_t *p_data : GATT event information handle.
 *
 * Return:
 *   wiced_bt_gatt_status_t: Error code from WICED_RESULT_LIST or BT_RESULT_LIST
 *
 ******************************************************************************/
wiced_bt_gatt_status_t tput_gatts_callback(wiced_bt_gatt_evt_t event,
                                           wiced_bt_gatt_event_data_t *p_data)
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_SUCCESS;

    switch(event)
    {
        case GATT_CONNECTION_STATUS_EVT:
            tput_gatts_conn_status_cb(&p_data->connection_status);
            break;
        case GATT_ATTRIBUTE_REQUEST_EVT:
            result = tput_gatts_req_cb(&p_data->attribute_request);
            break;
        default:
#if VERBOSE_THROUGHPUT_OUTPUT
            WICED_BT_TRACE("tput_gatts_callback: Event %d unhandled\r\n", event);
#endif
            break;
    }
    return result;
}

/*******************************************************************************
* Function Name: tput_gatts_conn_status_cb
********************************************************************************
* Summary: This function is invoked on GATT connection status change
*
* Parameters:
*   wiced_bt_gatt_connection_status_t *p_status: GATT connection status handle
*
* Return:
*   None
*
*******************************************************************************/
void tput_gatts_conn_status_cb(wiced_bt_gatt_connection_status_t *p_status)
{
    if (p_status->connected)
    {
        tput_gatts_connection_up(p_status);
    }
    else
    {
        tput_gatts_connection_down(p_status);
    }
}

/*******************************************************************************
* Function Name: tput_gatts_connection_up
********************************************************************************
* Summary: This function is invoked when GATT connection is established
*
* Parameters:
*   wiced_bt_gatt_connection_status_t *p_status: GATT connection status handle
*
* Return:
*   None
*
*******************************************************************************/
void tput_gatts_connection_up(wiced_bt_gatt_connection_status_t *p_status)
{
    WICED_BT_TRACE("GATT connected\r\n");

    /* Update the connection handler */
    tput_conn_state.conn_id = p_status->conn_id;

    /* Save BD address of the connected device */
    memcpy(tput_conn_state.remote_addr, p_status->bd_addr, sizeof(BD_ADDR));

    /* GATT Connected - LED1 ON */
    wiced_hal_gpio_set_pin_output(GATT_CONNECT_LED, GPIO_PIN_OUTPUT_LOW);

    /* Stop advertising */
    if(WICED_BT_SUCCESS != wiced_bt_start_advertisements(BTM_BLE_ADVERT_OFF,
            BLE_ADDR_PUBLIC, NULL))
    {
        WICED_BT_TRACE("Advertisement Stop failed\r\n");
    }

    /* Trigger 2M PHY update request */
    wiced_bt_ble_phy_preferences_t phy_preferences;
    wiced_bt_dev_status_t status;
    phy_preferences.rx_phys = BTM_BLE_PREFER_2M_PHY;
    phy_preferences.tx_phys = BTM_BLE_PREFER_2M_PHY;
    memcpy(phy_preferences.remote_bd_addr, tput_conn_state.remote_addr,
           sizeof(BD_ADDR));
    status = wiced_bt_ble_set_phy(&phy_preferences);
    if (status == WICED_BT_SUCCESS)
    {
        WICED_BT_TRACE("TPUT: Request sent to switch PHY to %dM\r\n",
                       phy_preferences.tx_phys);
    }
    else
    {
        WICED_BT_TRACE("TPUT: PHY switch request failed, result: %d\r\n",
                       status);
    }

    /* Start one second timer to calculate throughput */
    if (WICED_BT_SUCCESS != wiced_start_timer(&tput_app_sec_timer,
                                              SECONDS_TIMER_TIMEOUT))
    {
        WICED_BT_TRACE("Seconds timer start failed\r\n");
    }
}

/*******************************************************************************
* Function Name: tput_gatts_connection_down
********************************************************************************
* Summary: This function is invoked on GATT disconnect
*
* Parameters:
*   wiced_bt_gatt_connection_status_t *p_status: GATT connection status handle
*
* Return:
*   None
*
*******************************************************************************/
void tput_gatts_connection_down(wiced_bt_gatt_connection_status_t *p_status)
{
    gatt_db_lookup_table_t *puAttribute;

    /* Resetting the device info */
    memset(tput_conn_state.remote_addr, 0, sizeof(BD_ADDR));
    tput_conn_state.conn_id = 0;
    gatt_notify_tx_bytes = 0;
    gatt_write_rx_bytes = 0;
    failed_to_send = 0;

    WICED_BT_TRACE("Disconnected from peer, Reason: %d\r\n", p_status->reason);

    /* GATT Disconnected - LED1 and LED2 OFF */
    wiced_hal_gpio_set_pin_output(GATT_CONNECT_LED, GPIO_PIN_OUTPUT_HIGH);
    wiced_hal_gpio_set_pin_output(NOTIFY_LED , GPIO_PIN_OUTPUT_HIGH);

    /* We are disconnected now. Stop the timer */
    if (WICED_BT_SUCCESS != wiced_stop_timer(&tput_app_sec_timer))
    {
        WICED_BT_TRACE("TPUT: seconds timer stop failed\r\n");
    }

    /* Reset the CCCD value */
    app_throughput_measurement_notify_descriptor[0] = 0u;
    app_throughput_measurement_notify_descriptor[1] = 0u;

    /* If disconnection was caused by the peer, start advertisements, so that,
     * peer can connect when it wakes up.
     */
    if(WICED_BT_SUCCESS == wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH,
                                                         BLE_ADDR_PUBLIC, NULL))
    {
        WICED_BT_TRACE("Advertising......\r\n");
    }
    else
    {
        WICED_BT_TRACE("Advertising failed!!\r\n");
    }
}

/*******************************************************************************
 * Function Name: tput_gatts_req_cb
 *******************************************************************************
 * Summary: Process GATT Read/Write/MTU request from the peer.
 *
 * Parameters:
 *   wiced_bt_gatt_attribute_request_t *p_data: GATT request information handle.
 *
 * Return:
 *   wiced_bt_gatt_status_t: Error code from WICED_RESULT_LIST or BT_RESULT_LIST
 *
 ******************************************************************************/
wiced_bt_gatt_status_t tput_gatts_req_cb(wiced_bt_gatt_attribute_request_t *p_data)
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_SUCCESS;

    switch (p_data->request_type)
    {
        case GATTS_REQ_TYPE_READ:
            result = tput_gatts_req_read_handler(p_data->conn_id,
                                                 &(p_data->data.read_req));
            break;

        case GATTS_REQ_TYPE_WRITE:
            result = tput_gatts_req_write_handler(p_data->conn_id,
                                                  &(p_data->data.write_req));
            break;

        case GATTS_REQ_TYPE_MTU:
            WICED_BT_TRACE("MTU exchanged: %d\r\n", p_data->data.mtu);
            tput_conn_state.mtu = p_data->data.mtu;
            packet_size = get_notification_packet_size(tput_conn_state.mtu);
            break;

        default:
            WICED_BT_TRACE("GATT request unhandled..\r\n");
            break;
    }
    return result;
}

/*******************************************************************************
* Function Name: tput_gatts_req_read_handler
********************************************************************************
* Summary: GATT attribute read function.
*          Process read command from peer device.
*
* Parameters:
*   uint16_t conn_id                    : GATT connection id
*   wiced_bt_gatt_read_t * p_read_data  : GATT read attribute handle
*
* Return:
*   wiced_bt_gatt_status_t: result for read operation
*
*******************************************************************************/
wiced_bt_gatt_status_t tput_gatts_req_read_handler(uint16_t conn_id,
                                                   wiced_bt_gatt_read_t* p_read_data)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_INVALID_HANDLE;

    /* Attempt to perform the Read Request */
    status = tput_gatts_att_get_value(p_read_data->handle,
                                      conn_id,
                                      p_read_data->p_val,
                                      *p_read_data->p_val_len,
                                      p_read_data->p_val_len);

    return status;
}

/*******************************************************************************
* Function Name: tput_gatts_req_write_handler
********************************************************************************
* Summary: GATT attribute write function.
*          Process write request or write command from peer device.
*
* Parameters:
*   uint16_t conn_id                : GATT connection id
*   wiced_bt_gatt_write_t * p_data  : GATT write attribute handle
*
* Return:
*   wiced_bt_gatt_status_t: result for write operation
*
*******************************************************************************/
wiced_bt_gatt_status_t tput_gatts_req_write_handler(uint16_t conn_id,
                                                    wiced_bt_gatt_write_t * p_data)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_INVALID_HANDLE;

    /* Attempt to perform the Write Request */
    status = tput_gatts_att_set_value(p_data->handle, conn_id, p_data->p_val,
                                      p_data->val_len);

    return status;

}

/*******************************************************************************
* Function Name: tput_gatts_att_get_value
********************************************************************************
* Summary:
*   This function handles reading of the attribute value from the GATT database
*   and passing the data to the BT stack. The value read from the GATT database
*   is stored in a buffer whose starting address is passed as one of the function
*   parameters
*
* Parameters:
*   uint16_t attr_handle    : Attribute handle for read operation
*   uint16_t conn_id        : Connection ID
*   uint8_t *p_val          : Pointer to the buffer to store read data
*   uint16_t max_len        : Maximum buffer length available to
*                                                         store the read data
*   uint16_t *p_len         : Actual length of data copied to the buffer
*
* Return:
*   wiced_bt_gatt_status_t: See possible status codes in wiced_bt_gatt_status_e
*                                    in wiced_bt_gatt.h
*
*******************************************************************************/
wiced_bt_gatt_status_t tput_gatts_att_get_value(uint16_t attr_handle,
                                                uint16_t conn_id,
                                                uint8_t *p_val,
                                                uint16_t max_len,
                                                uint16_t *p_len)
{
    int i = 0;
    wiced_bool_t isHandleInTable = WICED_FALSE;
    wiced_bt_gatt_status_t res = WICED_BT_GATT_INVALID_HANDLE;

    /* Check for a matching handle entry */
    for (i = 0; i < app_gatt_db_ext_attr_tbl_size; i++)
    {
        if (app_gatt_db_ext_attr_tbl[i].handle == attr_handle)
        {
            /* Detected a matching handle in external lookup table */
            isHandleInTable = WICED_TRUE;
            /* Detected a matching handle in the external lookup table */
            if (app_gatt_db_ext_attr_tbl[i].cur_len <= max_len)
            {
                /* Value fits within the supplied buffer; copy over the value */
                *p_len = app_gatt_db_ext_attr_tbl[i].cur_len;
                memcpy(p_val, app_gatt_db_ext_attr_tbl[i].p_data,
                       app_gatt_db_ext_attr_tbl[i].cur_len);
                res = WICED_BT_GATT_SUCCESS;
            }
            else
            {
                /* Value to read will not fit within the buffer */
                res = WICED_BT_GATT_INVALID_ATTR_LEN;
            }
            break;
        }
    }

    if (!isHandleInTable)
    {
        /* The read operation was not performed for the indicated handle */
        WICED_BT_TRACE("Read Request to Invalid Handle: 0x%x\r\n",
                        attr_handle);
        res = WICED_BT_GATT_READ_NOT_PERMIT;
    }
    return res;
}

/*******************************************************************************
* Function Name: tput_gatts_att_set_value
********************************************************************************
* Summary:
*   This function handles writing to the attribute handle in the GATT database
*   using the data passed from the BT stack. The value to write is stored in a
*   buffer whose starting address is passed as one of the function parameters
*
* Parameters:
*   uint16_t attr_handle          : Attribute handle for write operation
*   uint16_t conn_id              : Connection ID
*   uint8_t *p_val                : Pointer to the buffer that stores
*                                                    the data to be written
*   uint16_t len                  : Length of data to be written
*
* Return:
*   wiced_bt_gatt_status_t: See possible status codes in wiced_bt_gatt_status_e
*                                                 in wiced_bt_gatt.h
*
*******************************************************************************/
wiced_bt_gatt_status_t tput_gatts_att_set_value(uint16_t attr_handle,
                                                uint16_t conn_id,
                                                uint8_t *p_val,
                                                uint16_t len )
{
    int i = 0;
    wiced_bool_t isHandleInTable = WICED_FALSE;
    wiced_bool_t validLen = WICED_FALSE;
    wiced_bt_gatt_status_t res = WICED_BT_GATT_INVALID_HANDLE;

    /* Check for a matching handle entry */
    for (i = 0; i < app_gatt_db_ext_attr_tbl_size; i++)
    {
        if (app_gatt_db_ext_attr_tbl[i].handle == attr_handle)
        {
            /* Detected a matching handle in external lookup table */
            isHandleInTable = WICED_TRUE;
            /* Verify that size constraints have been met */
            validLen = (app_gatt_db_ext_attr_tbl[i].max_len >= len);
            if (validLen)
            {
                /* Value fits within the supplied buffer; copy over the value */
                app_gatt_db_ext_attr_tbl[i].cur_len = len;
                memcpy(app_gatt_db_ext_attr_tbl[i].p_data, p_val, len);
                res = WICED_BT_GATT_SUCCESS;

                switch ( attr_handle )
                {
                /* By writing into Characteristic Client Configuration descriptor
                 * peer can enable or disable notification  */
                case HDLD_THROUGHPUT_MEASUREMENT_NOTIFY_DESCRIPTOR:
                    gatt_notify_tx_bytes = 0;
                    gatt_write_rx_bytes = 0;
                    if(app_throughput_measurement_notify_descriptor[0])
                    {
                        WICED_BT_TRACE("Notifications enabled\r\n");
                    }
                    else
                    {
                        WICED_BT_TRACE("Notifications disabled\r\n");
                    }
                    break;

                case HDLC_THROUGHPUT_MEASUREMENT_WRITEME_VALUE:
                    /* Receive GATT write commands from client
                    * and update the counter with number of
                    * bytes received.
                    */
                    gatt_write_rx_bytes += len;
                    break;

                default:
                    WICED_BT_TRACE("Write is not supported \r\n");
                }
            }
            else
            {
                /* Value to write does not meet size constraints */
                res = WICED_BT_GATT_INVALID_ATTR_LEN;
            }
            break;
        }
    }

    if (!isHandleInTable)
    {
            /* The write operation was not performed for the indicated handle */
            WICED_BT_TRACE("Write Request to Invalid Handle: 0x%x\r\n",
                            attr_handle);
            res = WICED_BT_GATT_WRITE_NOT_PERMIT;
    }
    return res;
}

/*******************************************************************************
* Function Name: get_notification_packet_size
********************************************************************************
* Summary: This function decides size of notification packet based on the
*   attribute MTU exchanged. This is done to utilize the LL payload space
*   effectively.
*
* Parameters:
*   uint16_t att_mtu_size: MTU value exchaged after connection.
*
* Return:
*   uint16_t: Size of notification packet derived based on MTU.
*
*******************************************************************************/
static uint16_t get_notification_packet_size( uint16_t att_mtu_size )
{
    if(att_mtu_size < DATA_PACKET_SIZE_1 + ATT_HEADER)
    {
       /* Packet Length = ATT_MTU_SIZE - ATT_HANDLE(2 bytes) - ATT_OPCODE(1 byte)
        * Reason: With DLE enabled, LL payload is 251 bytes. So if an MTU less
        * than 247 is exchanged, the data can be accommodated in a single LL
        * packet */
        packet_size = att_mtu_size - ATT_HEADER;
    }
    else if((att_mtu_size >= DATA_PACKET_SIZE_1 + ATT_HEADER) &&
            (att_mtu_size < DATA_PACKET_SIZE_2 + ATT_HEADER))
    {
        /* If MTU is between 247 and 498, if a packet size other than 244 bytes
         * is used, the data will be split and the LL payload space is not
         * utilized effectively. Refer README for details */
        packet_size = DATA_PACKET_SIZE_1;
    }
    else
    {
        /* For MTU value greater than 498, if a packet size other than 495(or 244)
         * is used, the LL payload space is not utilized effectively.
         * 495 bytes will go as two LL packets: 244 bytes + 251 bytes */
        packet_size = DATA_PACKET_SIZE_2;
    }
    return packet_size;
}

