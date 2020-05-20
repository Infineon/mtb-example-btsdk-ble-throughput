/******************************************************************************
* File Name:   tput_server_ble.c
*
* Description: This file has implementation for functions which do the following:
*              * BLE initialization(registering gatt db, gatt callback, setting
*                adv data etc..)
*              * BLE stack event handler - to process the BLE events.
*              * 1 second timer init and callback - for throughput calculation.
*              * 1 msec timer init and callback - for sending BLE notifications.
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
#include "tput_server_ble.h"

/*******************************************************************************
 *                                Variable Definitions
 ******************************************************************************/

/* Variables to hold GATT notification bytes sent and GATT Write bytes received
 * successfully*/
static uint32_t             gatt_notif_tx_packets  = 0;
static uint32_t             gatt_write_rx_packets  = 0;
/* No of bytes rejected for transfer */
static uint32_t             failed_to_send         = 0;
/* Application seconds timer */
static wiced_timer_t        tput_app_sec_timer;
/* Application milliseconds timer */
static wiced_timer_t        tput_app_msec_timer;
/* Variable to store peer device details after connection */
static tput_conn_state_t    tput_conn_state;
/* Variable to store attribute MTU that is exchanged after connection */
static uint16_t             att_mtu_size           = 247;
/* Variable to store packet size decided based on MTU exchanged */
static uint16_t             packet_size            = 0;

/*******************************************************************************
 *                                Function Definitions
 ******************************************************************************/

/*******************************************************************************
* Function Name: tput_management_callback()
********************************************************************************
* Summary:
*   This is a Bluetooth management event handler function to receive events from
*   BLE stack and process as needed by the application.
*
* Parameters:
*   wiced_bt_management_evt_t event             : BLE event code of one byte
*                                                 length
*   wiced_bt_management_evt_data_t *p_event_data: Pointer to BLE management
*                                                 event structures
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

#ifdef VERBOSE_THROUGHPUT_OUTPUT
    WICED_BT_TRACE("\rThroughput GATT server management callback event: %d\n", event);
#endif

    switch (event)
    {
        case BTM_ENABLED_EVT:
            /* Handle initialize sequence for the application */
            tput_init();
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
#ifdef VERBOSE_THROUGHPUT_OUTPUT
            if(p_event_data->ble_advert_state_changed == BTM_BLE_ADVERT_OFF)
            {
                WICED_BT_TRACE("\rAdvertisement State Change: OFF\n");
            }
            else if(p_event_data->ble_advert_state_changed == BTM_BLE_ADVERT_UNDIRECTED_HIGH)
            {
                WICED_BT_TRACE("\rAdvertisement State Change: Undirected High\n");
            }
            else if(p_event_data->ble_advert_state_changed == BTM_BLE_ADVERT_UNDIRECTED_LOW)
            {
                WICED_BT_TRACE("\rAdvertisement State Change: Undirected Low\n");
            }
#endif
            break;

        case BTM_BLE_PHY_UPDATE_EVT:
#ifdef VERBOSE_THROUGHPUT_OUTPUT
            WICED_BT_TRACE("\rReceived BTM_BLE_PHY_UPDATE_EVT: PHY status: "
                                             "%d, RX_PHY: %d, TX_PHY: %d\n",
                                  p_event_data->ble_phy_update_event.status,
                                  p_event_data->ble_phy_update_event.rx_phy,
                                 p_event_data->ble_phy_update_event.tx_phy);
#endif

            WICED_BT_TRACE("\rSelected PHY - %dM\n",
                            p_event_data->ble_phy_update_event.tx_phy);

            /* send connection interval update if required */
            wiced_bt_l2cap_update_ble_conn_params(tput_conn_state.remote_addr,
                                                          CONNECTION_INTERVAL,
                                                          CONNECTION_INTERVAL,
                                                                            0,
                                                            BLE_CONN_TIMEOUT);

            WICED_BT_TRACE("\rConnection Interval update requested \n");
            break;

        case BTM_BLE_CONNECTION_PARAM_UPDATE:
            /* Connection parameters updated */
            if(WICED_SUCCESS == p_event_data->ble_connection_param_update.status)
            {
                conn_interval = (p_event_data->ble_connection_param_update.conn_interval) * CONN_INTERVAL_MULTIPLIER;
                WICED_BT_TRACE("\rNew connection parameters:"
                               "\n\rConnection interval = %d.%dms\n",
                                  CONN_INTERVAL_MAJOR(conn_interval),
                                 CONN_INTERVAL_MINOR(conn_interval));
            }
            else
            {
#ifdef VERBOSE_THROUGHPUT_OUTPUT
                WICED_BT_TRACE("\rConnection parameters update failed\n");
#endif
            }
            break;

        default:
#ifdef VERBOSE_THROUGHPUT_OUTPUT
            WICED_BT_TRACE("\rUnhandled management callback event: %d\n", event);
#endif
            break;
    }

    return (result);
}

/*******************************************************************************
* Function Name: tput_init()
********************************************************************************
* Summary:
*   This function initializes/registers the interrupt(s), timer(s),
*   GATT database as needed by the application.
*
* Parameters:
*   None
*
* Return:
*   None
*
*******************************************************************************/
void tput_init(void)
{
    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_ERROR;

    /* Register with stack to receive GATT callback */
    gatt_status = wiced_bt_gatt_register(tput_gatts_callback);
    if (WICED_BT_GATT_SUCCESS != gatt_status)
    {
        WICED_BT_TRACE("\rGATT registration failed\n");
    }

    /* Tell the bluetooth stack to use our GATT database */
    gatt_status = wiced_bt_gatt_db_init(gatt_database, gatt_database_len);
    if (WICED_BT_GATT_SUCCESS != gatt_status)
    {
        WICED_BT_TRACE("\rGATT database init failed\n");
    }

    /* Start app timers */
    if (wiced_init_timer(&tput_app_sec_timer, tput_app_sec_timeout, 0,
                         WICED_SECONDS_PERIODIC_TIMER) != WICED_BT_SUCCESS)
    {
#ifdef VERBOSE_THROUGHPUT_OUTPUT
        WICED_BT_TRACE("\rSeconds timer init failed\n");
#endif
    }

    if (wiced_init_timer(&tput_app_msec_timer, tput_app_msec_timeout, 0,
                         WICED_MILLI_SECONDS_PERIODIC_TIMER) != WICED_BT_SUCCESS)
    {
#ifdef VERBOSE_THROUGHPUT_OUTPUT
        WICED_BT_TRACE("\rMillisecond timer init failed\n");
#endif
    }

    /* Pairing not needed for throughput test */
    wiced_bt_set_pairable_mode(WICED_FALSE, 0);

    /* Set the advertising parameters */
    tput_set_advertisement_data();

    /* Start advertisements */
    if(wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH,
                         BLE_ADDR_PUBLIC, NULL) == WICED_BT_SUCCESS)
    {
        WICED_BT_TRACE("\rAdvertising......\n");
    }
}

/*******************************************************************************
* Function Name: tput_set_advertisement_data()
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
    wiced_bt_ble_advert_elem_t adv_elem[ADV_ELEM_LEN] = {0};
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

    if (wiced_bt_ble_set_raw_advertisement_data(num_elem, adv_elem) != WICED_BT_SUCCESS)
    {
        WICED_BT_TRACE("\rSetup advertisement data failed\n");
    }
}

/*******************************************************************************
* Function Name: tput_app_sec_timeout()
********************************************************************************
* Summary:
*   One second timer callback.
*   Print the TX and RX data throughput over PUART console.
*
* Parameters:
*   uint32_t unused: Variable registered by wiced_init_timer to pass to
*                    timer callback. Not used for now.
*
* Return:
*   None
*
*******************************************************************************/
void tput_app_sec_timeout(uint32_t unused)
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
        gatt_notif_tx_packets = (gatt_notif_tx_packets * 8)/1000;
        WICED_BT_TRACE("\rGATT NOTIFICATION : Server Throughput (TX)= %d kbps\n",
                          gatt_notif_tx_packets);
#ifdef VERBOSE_THROUGHPUT_OUTPUT
        WICED_BT_TRACE("\rNumber of GATT notification packets failed to send = %d\n"
                       , failed_to_send);
#endif
        /* Reset the GATT notification byte counter */
        gatt_notif_tx_packets = 0;
        /* Reset the rejected byte counter */
        failed_to_send  = 0;
    }

    /* Display GATT RX throughput result */
    if (tput_conn_state.conn_id && gatt_write_rx_packets)
    {
        gatt_write_rx_packets = (gatt_write_rx_packets * 8)/1000;
        WICED_BT_TRACE("\rGATT WRITE        : Server Throughput (RX)= %d kbps\n",
                          gatt_write_rx_packets);
        /* Reset the GATT write byte counter */
        gatt_write_rx_packets = 0;

    }
}

/*******************************************************************************
* Function Name: tput_app_msec_timeout()
********************************************************************************
* Summary:
*   One millisecond timer callback.
*   Send GATT notifications if enabled by the GATT Client.
*
* Parameters:
*   uint32_t unused: Variable registered by wiced_init_timer to pass to
*                    timer callback. Not used for now.
*
* Return:
*   None
*
*******************************************************************************/
void tput_app_msec_timeout(uint32_t unused)
{
    /* Send GATT Notification */
    if (app_throughput_measurement_notify_descriptor[0])
    {
        /* Sending GATT notification. */
        tput_send_notification();
    }
}

/*******************************************************************************
* Function Name: tput_send_notification()
********************************************************************************
* Summary:
*   Send GATT notification every millisecond.
*
* Parameters:
*   None
*
* Return:
*   None
*
*******************************************************************************/
static void tput_send_notification(void)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_SUCCESS;
    uint32_t buf_left = 0;

    buf_left = wiced_bt_ble_get_available_tx_buffers();
    if (buf_left > MINIMUM_TX_BUFFER_LEN)
    {
        status = wiced_bt_gatt_send_notification(tput_conn_state.conn_id,
                                HDLC_THROUGHPUT_MEASUREMENT_NOTIFY_VALUE,
                                                             packet_size,
                                      app_throughput_measurement_notify);
        if (status == WICED_BT_GATT_SUCCESS)
        {
            gatt_notif_tx_packets += packet_size;
            /* LED 2 ON */
            wiced_hal_gpio_set_pin_output(CONGESTION_LED, GPIO_PIN_OUTPUT_LOW);
        }
        else
        {
            failed_to_send++;
            /* LED 2 OFF since TX failed */
            wiced_hal_gpio_set_pin_output(CONGESTION_LED, GPIO_PIN_OUTPUT_HIGH);
        }
    }
    else
    {
        failed_to_send++;
        /* LED 2 OFF since TX failed */
        wiced_hal_gpio_set_pin_output(CONGESTION_LED, GPIO_PIN_OUTPUT_HIGH);
    }
}

/*******************************************************************************
 * Function Name: tput_gatts_callback()
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
wiced_bt_gatt_status_t
tput_gatts_callback(wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data)
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
#ifdef VERBOSE_THROUGHPUT_OUTPUT
            WICED_BT_TRACE("\rtput_gatts_callback: Event %d unhandled\n", event);
#endif
            break;
    }
    return result;
}

/*******************************************************************************
* Function Name: tput_gatts_conn_status_cb()
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
        tput_gatts_connection_down();
    }
}

/*******************************************************************************
* Function Name: tput_gatts_connection_up()
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
    WICED_BT_TRACE("\rGATT connected\n");

    /* Update the connection handler */
    tput_conn_state.conn_id = p_status->conn_id;

    /* Save BD address of the connected device */
    memcpy(tput_conn_state.remote_addr, p_status->bd_addr, sizeof(BD_ADDR));

    /* GATT Connected - LED1 ON */
    wiced_hal_gpio_set_pin_output(GATT_CONNECT_LED, GPIO_PIN_OUTPUT_LOW);

    /* Stop advertising */
    if(wiced_bt_start_advertisements(BTM_BLE_ADVERT_OFF, BLE_ADDR_PUBLIC, NULL)
                                                           != WICED_BT_SUCCESS)
    {
        WICED_BT_TRACE("\rAdvertisement Stop failed\n");
    }

    /* Trigger 2M PHY update request */
    wiced_bt_ble_phy_preferences_t phy_preferences;
    wiced_bt_dev_status_t status;
    phy_preferences.rx_phys = BTM_BLE_PREFER_2M_PHY;
    phy_preferences.tx_phys = BTM_BLE_PREFER_2M_PHY;
    memcpy(phy_preferences.remote_bd_addr, tput_conn_state.remote_addr, sizeof(BD_ADDR));
    status = wiced_bt_ble_set_phy(&phy_preferences);
    if (status == WICED_BT_SUCCESS)
    {
        WICED_BT_TRACE("\rTPUT: Request sent to switch PHY to %dM\n",
                                            phy_preferences.tx_phys);
    }
    else
    {
        WICED_BT_TRACE("\rTPUT: PHY switch request failed, result: %d\n", status);
    }

    /* Start one second timer to calculate throughput */
    if (wiced_start_timer(&tput_app_sec_timer, SECONDS_TIMER_TIMEOUT) != WICED_BT_SUCCESS)
    {
        WICED_BT_TRACE("\rSeconds timer start failed\n");
    }
}

/*******************************************************************************
* Function Name: tput_gatts_connection_down()
********************************************************************************
* Summary: This function is invoked on GATT disconnect
*
* Parameters:
*   None
*
* Return:
*   None
*
*******************************************************************************/
void tput_gatts_connection_down(void)
{
    gatt_db_lookup_table_t *puAttribute;

    /* Resetting the device info */
    memset(tput_conn_state.remote_addr, 0, sizeof(BD_ADDR));
    tput_conn_state.conn_id = 0;
    gatt_notif_tx_packets = 0;
    gatt_write_rx_packets = 0;
    failed_to_send = 0;

    WICED_BT_TRACE("\rDisconnected from peer.\n");

    /* GATT Disconnected - LED1 OFF */
    wiced_hal_gpio_set_pin_output(GATT_CONNECT_LED, GPIO_PIN_OUTPUT_HIGH);
    wiced_hal_gpio_set_pin_output(CONGESTION_LED , GPIO_PIN_OUTPUT_HIGH);

    /* We are disconnected now. Stop the timers */
    if (WICED_BT_SUCCESS != wiced_stop_timer(&tput_app_sec_timer))
    {
        WICED_BT_TRACE("\rTPUT: seconds timer stop failed\n");
    }
    if (WICED_BT_SUCCESS != wiced_stop_timer(&tput_app_msec_timer))
    {
        WICED_BT_TRACE("\rTPUT: millisecond timer stop failed\n");
    }

    /* Disable GATT notification */
    app_throughput_measurement_notify_descriptor[0] = 0u;

    /* Update GATT DB */
    if ((puAttribute = tput_get_attribute(HDLD_THROUGHPUT_MEASUREMENT_NOTIFY_DESCRIPTOR)) != NULL)
    {
        memcpy(puAttribute->p_data, (uint8_t *)app_throughput_measurement_notify_descriptor,
               puAttribute->max_len);
    }

    /* If disconnection was caused by the peer, start advertisements, so that,
     * peer can connect when it wakes up.
     */
    if(wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, BLE_ADDR_PUBLIC, NULL) == WICED_BT_SUCCESS)
    {
        WICED_BT_TRACE("\rAdvertising......\n");
    }
    else
    {
        WICED_BT_TRACE("\rAdvertising failed!!\n");
    }
}

/*******************************************************************************
 * Function Name: tput_gatts_req_cb()
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
            result = tput_gatts_req_read_handler(p_data->conn_id, &(p_data->data.read_req));
            break;

        case GATTS_REQ_TYPE_WRITE:
            result = tput_gatts_req_write_handler(p_data->conn_id, &(p_data->data.write_req));
            break;

        case GATTS_REQ_TYPE_MTU:
            WICED_BT_TRACE("\rMTU exchanged: %d\n", p_data->data.mtu);
            att_mtu_size = p_data->data.mtu;
            packet_size = get_notification_packet_size(att_mtu_size);
            break;

        default:
            WICED_BT_TRACE("\rGATT request unhandled..\n");
            break;
    }
    return result;
}

/*******************************************************************************
* Function Name: tput_gatts_req_read_handler()
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
wiced_bt_gatt_status_t
tput_gatts_req_read_handler(uint16_t conn_id, wiced_bt_gatt_read_t* p_read_data)
{
    gatt_db_lookup_table_t *server_attribute = NULL;
    uint16_t attribute_length = 0;
    uint16_t max_attribute_length = 0;

    /* Get the attribute requested by client using the handle */
    if ((server_attribute = tput_get_attribute(p_read_data->handle)) == NULL)
    {
        /* If the attribute is not found, return */
        WICED_BT_TRACE("\rAttribute handle not found for read request: %x\n",
                                                       p_read_data->handle );
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    /* Copy the length of server attribute found in GATT DB */
    attribute_length = server_attribute->cur_len;
    /* Copy the length provided by stack */
    max_attribute_length = *p_read_data->p_val_len;

    /* Check if attribute(present in GATT DB) length does not exceed length
       provided by stack */
    if (attribute_length <= max_attribute_length)
    {
        memcpy( p_read_data->p_val, server_attribute->p_data, attribute_length);
        return WICED_BT_GATT_SUCCESS;
    }
    else
    {
        return WICED_BT_GATT_INVALID_ATTR_LEN;
    }
}

/*******************************************************************************
* Function Name: tput_gatts_req_write_handler()
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
wiced_bt_gatt_status_t
tput_gatts_req_write_handler(uint16_t conn_id, wiced_bt_gatt_write_t * p_data)
{
    wiced_bt_gatt_status_t result    = WICED_BT_GATT_SUCCESS;
    uint8_t                *p_attr   = p_data->p_val;
    gatt_db_lookup_table_t *puAttribute;

    switch (p_data->handle)
    {
    /* By writing into Characteristic Client Configuration descriptor
     * peer can enable or disable notification
     */
    case HDLD_THROUGHPUT_MEASUREMENT_NOTIFY_DESCRIPTOR:

        if (p_data->val_len != app_throughput_measurement_notify_descriptor_len)
        {
            WICED_BT_TRACE("Invalid attribute length\n");
            return WICED_BT_GATT_INVALID_ATTR_LEN;
        }

        app_throughput_measurement_notify_descriptor[0] = p_attr[0];
        app_throughput_measurement_notify_descriptor[1] = p_attr[1];

        /* Update GATT DB */
        if ((puAttribute = tput_get_attribute(p_data->handle)) != NULL)
        {
            memcpy(puAttribute->p_data, (uint8_t *)app_throughput_measurement_notify_descriptor,
                                                        puAttribute->max_len);
        }

        if(app_throughput_measurement_notify_descriptor[0])
        {
            WICED_BT_TRACE("\rNotifications enabled\n");
            gatt_write_rx_packets = 0;
            if (wiced_start_timer(&tput_app_msec_timer, MILLISECOND_TIMER_TIMEOUT) != WICED_BT_SUCCESS)
            {
#ifdef VERBOSE_THROUGHPUT_OUTPUT
                WICED_BT_TRACE("\rTPUT: millisecond timer start failed\n");
#endif
            }
        }
        else
        {
            WICED_BT_TRACE("\rNotifications disabled\n");
            wiced_stop_timer(&tput_app_msec_timer);
            gatt_notif_tx_packets = 0;
        }

        /* LED 2 ON(notify enabled); OFF(notify disabled) */
        wiced_hal_gpio_set_pin_output(CONGESTION_LED, !app_throughput_measurement_notify_descriptor[0]);

        break;

    case HDLC_THROUGHPUT_MEASUREMENT_WRITEME_VALUE:
        /* Receive GATT write commands from client
         * and update the counter with number of
         * bytes received.
         */
        gatt_write_rx_packets += p_data->val_len;
        result = WICED_BT_GATT_SUCCESS;
        break;

    default:
        break;
    }

    return result;
}

/*******************************************************************************
* Function Name: get_notification_packet_size()
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
    if(att_mtu_size < 247u)
    {
       /* Packet Length = ATT_MTU_SIZE - ATT_HANDLE(2 bytes) - ATT_OPCODE(1 byte)
        * Reason: With DLE enabled, LL payload is 251 bytes. So if an MTU less
        * than 247 is exchanged, the data can be accommodated in a single LL
        * packet */
        packet_size = att_mtu_size - 3u;
    }
    else if((att_mtu_size >= 247u) && (att_mtu_size < 498u))
    {
        /* If MTU is between 247 and 498, if a packet size other than 244 bytes
         * is used, the data will be split and the LL payload space is not
         * utilized effectively. Refer README for details */
        packet_size = 244u;
    }
    else
    {
        /* For MTU value greater than 498, if a packet size other than 495(or 244)
         * is used, the LL payload space is not utilized effectively.
         * 495 bytes will go as two LL packets: 244 bytes + 251 bytes */
        packet_size = 495u;
    }
    return packet_size;
}

/*******************************************************************************
* Function Name: tput_get_attribute()
********************************************************************************
* Summary: Find attribute description by handle
*
* Parameters:
*   uint16_t handle: Attribute handle
*
* Return:
*   gatt_db_lookup_table_t*: Pointer to BLE GATT attribute handle
*
*******************************************************************************/
gatt_db_lookup_table_t* tput_get_attribute(uint16_t handle)
{
    for (uint16_t i = 0; i < app_gatt_db_ext_attr_tbl_size; i++)
    {
        if (app_gatt_db_ext_attr_tbl[i].handle == handle)
        {
            return (&app_gatt_db_ext_attr_tbl[i]);
        }
    }
#ifdef VERBOSE_THROUGHPUT_OUTPUT
    WICED_BT_TRACE("\rTPUT[%s]: Attr not found:%x\n", __func__, handle);
#endif
    return NULL;
}




