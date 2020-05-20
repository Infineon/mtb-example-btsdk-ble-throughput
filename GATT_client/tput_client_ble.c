/******************************************************************************
* File Name:   tput_client_ble.c
*
* Description: This file has implementation for functions which do the following:
*              * BLE initialization(registering gatt db, gatt callback, setting
*                adv data etc..)
*              * BLE stack event handler - to process the BLE events.
*              * Button interrupt registration and callback implementation.
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
#include "wiced_rtos.h"
#include "GeneratedSource/cycfg_pins.h"
#include "sparcommon.h"
#include "wiced_result.h"
#include "cycfg_gatt_db.h"
#include "wiced_bt_stack.h"
#include "wiced_timer.h"
#include "wiced_bt_uuid.h"
#include "app_bt_cfg.h"
#include "wiced_gki.h"
#include "wiced_memory.h"
#include "tput_client_ble.h"

/*******************************************************************************
 *                                Global variables
 ******************************************************************************/
/* Variables to hold GATT notification bytes received and GATT Write bytes sent
 * successfully*/
static uint32_t           gatt_notify_rx_packets          = 0;
static uint32_t           gatt_write_tx_packets           = 0;
/* Enable or Disable notification from server */
static uint8_t            enable_cccd                     = WICED_TRUE;
static wiced_bool_t       update_notif_flag               = WICED_FALSE;
/* Flag to enable or disable GATT write */
static uint8_t            gatt_write_tx                   = WICED_FALSE;
/* Flag to indicate GATT congestion */
static wiced_bool_t       gatt_congestion_status          = WICED_FALSE;
/* Flag to used to Scan only for first button press */
static wiced_bool_t       scan_flag                       = WICED_TRUE;
/* Flag used to decide if a button press should change data transfer modes */
static wiced_bool_t       button_flag                     = WICED_FALSE;
/* Handle to the Throughput Measurement service */
static uint16_t           tput_service_handle             = 0;
static uint8_t            tput_service_uuid[LEN_UUID_128] = DISCOVER_TPUT_SERVICE_UUID;
static tput_conn_state_t  tput_conn_state;
/* Timer for printing throughput data */
static wiced_timer_t      timer_print_tput;
/* Timer for sending GATT writes */
static wiced_timer_t      timer_generate_data;
/* Variable to switch between different data transfer modes */
static tput_mode_t        mode_flag                       = GATT_Notif_StoC;
/* Variable to store attribute MTU that is exchanged after connection */
static uint16_t           att_mtu_size           = 247;
/* Variable to store packet size decided based on MTU exchanged */
static uint16_t           packet_size            = 0;

/*******************************************************************************
* Function Name: tput_management_callback()
********************************************************************************
* Summary:
*   This is a BLE management event handler function to receive events from
*   the stack and process as needed by the application.
*
* Parameters:
*   wiced_bt_management_evt_t event             : BLE event code of one byte
*                                                 length
*   wiced_bt_management_evt_data_t *p_event_data: Pointer to BLE management
*                                                 event structures
*
* Return:
*   wiced_result_t: Error code from WICED_RESULT_LIST or BT_RESULT_LIST
*
*******************************************************************************/
wiced_result_t tput_management_callback( wiced_bt_management_evt_t event,
                           wiced_bt_management_evt_data_t *p_event_data )
{
    wiced_result_t result   = WICED_BT_SUCCESS;
    uint16_t conn_interval  = 0;

#ifdef VERBOSE_THROUGHPUT_OUTPUT
    WICED_BT_TRACE("\rThroughput GATT client management callback event: %d \n", event);
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

        case BTM_BLE_SCAN_STATE_CHANGED_EVT:
#ifdef VERBOSE_THROUGHPUT_OUTPUT
            WICED_BT_TRACE("\rScan %s \n", (BTM_BLE_SCAN_TYPE_NONE !=
                              p_event_data->ble_scan_state_changed) ?
                                                "started":"stopped");
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
            WICED_BT_TRACE("\rSelected PHY - %dM\n", p_event_data->ble_phy_update_event.tx_phy);
            break;

        case BTM_BLE_CONNECTION_PARAM_UPDATE:
            result = p_event_data->ble_connection_param_update.status;
            /* Connection parameters updated */
            if(WICED_SUCCESS == result)
            {
                conn_interval = (p_event_data->ble_connection_param_update.conn_interval) * CONN_INTERVAL_MULTIPLIER;
                WICED_BT_TRACE("\rNew connection parameters:"
                               "\n\rConnection interval = %d.%dms\n",
                                  CONN_INTERVAL_MAJOR(conn_interval),
                                 CONN_INTERVAL_MINOR(conn_interval));
            }

#ifdef VERBOSE_THROUGHPUT_OUTPUT
            else
            {
                WICED_BT_TRACE("\rConnection parameters update failed\n");
            }
#endif

            /* Trigger 2M PHY update request */
            wiced_bt_ble_phy_preferences_t phy_preferences;
            wiced_bt_dev_status_t status;
            phy_preferences.rx_phys = BTM_BLE_PREFER_2M_PHY;
            phy_preferences.tx_phys = BTM_BLE_PREFER_2M_PHY;
            memcpy(phy_preferences.remote_bd_addr, tput_conn_state.remote_addr, sizeof(BD_ADDR));

            status = wiced_bt_ble_set_phy(&phy_preferences);
            if (status == WICED_BT_SUCCESS)
            {
                WICED_BT_TRACE("\rRequest sent to switch PHY to %dM\n",
                                              phy_preferences.tx_phys);
            }
            else
            {
                WICED_BT_TRACE("\rTPUT: PHY switch request failed, result: %d\n", status);
            }

            break;

        case  BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
              result = WICED_BT_ERROR;
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
void tput_init( void )
{
    wiced_bt_gatt_status_t gatt_status = WICED_BT_SUCCESS;

    /* Configure button interrupt callback */
    wiced_platform_register_button_callback(SW3, tput_btn_interrupt_handler,
                                    NULL, WICED_PLATFORM_BUTTON_RISING_EDGE);

    /* Register with stack to receive GATT callback */
    gatt_status = wiced_bt_gatt_register(tput_gattc_callback);
    if (WICED_BT_SUCCESS != gatt_status)
    {
        WICED_BT_TRACE("\rGATT registration failed\n");
    }

    /* Tell the bluetooth stack to use the GATT database */
    gatt_status = wiced_bt_gatt_db_init(gatt_database, gatt_database_len);
    if (WICED_BT_SUCCESS != gatt_status)
    {
        WICED_BT_TRACE("\rGATT database init failed\n");
    }

    /* One second timer: To calculate and display the throughput result.
     * Start the timer on GATT Connection UP event.
     * Stop the timer on GATT Connection DOWN event.
     */
    if (wiced_init_timer(&timer_print_tput, tput_app_sec_timeout, 0,
                     WICED_SECONDS_PERIODIC_TIMER) != WICED_BT_SUCCESS)
    {
#ifdef VERBOSE_THROUGHPUT_OUTPUT
        WICED_BT_TRACE("\rSeconds timer init failed\n");
#endif
    }

    /* Millisecond timer: Mainly used for sending GATT data to the server.
     * Start the timer on GATT Connection UP event.
     * Stop the timer on GATT Connection DOWN event.
     */
    if (wiced_init_timer(&timer_generate_data, tput_app_msec_timeout, 0,
                   WICED_MILLI_SECONDS_PERIODIC_TIMER) != WICED_BT_SUCCESS)
    {
#ifdef VERBOSE_THROUGHPUT_OUTPUT
        WICED_BT_TRACE("\rMillisecond timer init failed\n");
#endif
    }

    /* Pairing is not needed for throughput test */
    wiced_bt_set_pairable_mode(WICED_FALSE, 0);

}

/*******************************************************************************
* Function Name: tput_scan_result_cback()
********************************************************************************
* Summary:
*   This function is registered as a callback to handle the scan results.
*   When the desired device is found, it will try to establish connection with
*   that device.
*
* Parameters:
*   wiced_bt_ble_scan_results_t *p_scan_result: Details of the new device found.
*   uint8_t                     *p_adv_data      : Advertisement data.
*
* Return:
*   None
*
*******************************************************************************/
void tput_scan_result_cback( wiced_bt_ble_scan_results_t *p_scan_result, uint8_t *p_adv_data )
{
    wiced_result_t         status = WICED_BT_SUCCESS;
    uint8_t                length = 0u;
    uint8_t                *p_data = NULL;
    uint8_t                server_device_name[5] = {'T','P','U','T','\0'};
    if (p_scan_result)
    {
        p_data = wiced_bt_ble_check_advertising_data(p_adv_data,
                              BTM_BLE_ADVERT_TYPE_NAME_COMPLETE,
                                                       &length);

        if(p_data != NULL)
        {
            /* Check if the peer device's name is "TPUT" */
            if ((length = strlen((const char *)server_device_name)) &&
                (memcmp(p_data, (uint8_t *)server_device_name, length) == 0))
            {
                WICED_BT_TRACE("\rScan completed\n");
                WICED_BT_TRACE("\rFound Peer Device : %B \n", p_scan_result->remote_bd_addr);
                scan_flag = WICED_FALSE;

                /* Device found. Stop scan. */
                if((status = wiced_bt_ble_scan(BTM_BLE_SCAN_TYPE_NONE,
                                            WICED_TRUE,
                                            tput_scan_result_cback))!= 0)
                {
                    WICED_BT_TRACE("\rscan off status %d\n", status);
                }
                /* Delay here to let the BLE stack to process the pending requests.
                * Note that, the delay here does not have any impact on throughput
                * as it is done before BLE connection.
                */
                wiced_rtos_delay_milliseconds(2000, KEEP_THREAD_ACTIVE);

                /* Initiate the connection */
                if(wiced_bt_gatt_le_connect(p_scan_result->remote_bd_addr,
                                            p_scan_result->ble_addr_type,
                                                BLE_CONN_MODE_HIGH_DUTY,
                                                WICED_TRUE)!= WICED_TRUE)
                {
                    WICED_BT_TRACE("\rwiced_bt_gatt_connect failed\n");
                }
            }
            else
            {
                return; //Skip - This is not the device we are looking for.
            }
        }
    }
}

/*******************************************************************************
* Function Name: tput_app_sec_timeout()
********************************************************************************
* Summary:
*   One second timer callback. Calculate and print the TX and RX rate over PUART
*   console. Whenever there is button interrupt, notification enable/disable
*   request is sent according to the current throughput mode.
*
* Parameters:
*   uint32_t unused: Variable registered by wiced_init_timer to pass to
*                    timer callback. Not used for now.
*
* Return:
*   None
*
*******************************************************************************/
void tput_app_sec_timeout( uint32_t unused )
{
    if(update_notif_flag == WICED_TRUE)
    {
        update_notif_flag = WICED_FALSE;
        /* Clear GATT Tx packet */
        gatt_write_tx_packets = 0;

        if (WICED_BT_GATT_SUCCESS != enable_disable_gatt_notification(enable_cccd))
        {
            WICED_BT_TRACE("\rEnable/Disable notification failed\n");
        }
    }

    /* Calculate TX and RX throughput
     *
     * throughput(kbps) = (number of bytes sent/received in 1 second) * 8(bits)
     *                    -----------------------------------------------------
     *                                           1000
     */

    if (tput_conn_state.conn_id && gatt_notify_rx_packets )
    {
        gatt_notify_rx_packets = (gatt_notify_rx_packets * 8)/1000;
        WICED_BT_TRACE("\rGATT NOTIFICATION : Client Throughput (RX) = %d kbps\n",
                       gatt_notify_rx_packets);
        gatt_notify_rx_packets = 0; //Reset the byte counter
    }

    if ((tput_conn_state.conn_id) && gatt_write_tx_packets )
    {
        gatt_write_tx_packets = (gatt_write_tx_packets * 8)/1000;
        WICED_BT_TRACE("\rGATT WRITE        : Client Throughput (TX) = %d kbps\n",
                       gatt_write_tx_packets);
        gatt_write_tx_packets = 0; //Reset the byte counter
    }
}

/*******************************************************************************
* Function Name: tput_app_msec_timeout()
********************************************************************************
* Summary:
*   One millisecond timer callback. It sends out GATT Write(with no response)
*   commands to the server only when no GATT congestion and if GATT writes
*   are allowed to send by the application.
*
* Parameters:
*   uint32_t unused: Variable registered by wiced_init_timer to pass to
*                    timer callback. Not used for now.
*
* Return:
*   None
*
*******************************************************************************/
void tput_app_msec_timeout( uint32_t unused )
{
    wiced_bt_gatt_value_t   *p_write;
    wiced_bt_gatt_status_t  status = WICED_BT_GATT_SUCCESS;
    static uint8_t gatt_write_bytes[GATT_WRITE_BYTES_MAX_LEN];
    /* Return if not connected. Nothing for throughput calculation */
    if (!tput_conn_state.conn_id)
    {
        return;
    }

    /* Send GATT write(with no response) commands to the server only
     * when there is no GATT congestion and no GATT notifications are being
     * received. In data transfer mode 3(Both TX and RX), the GATT write
     * commands will be sent irrespective of GATT notifications being received
     * or not.
     */
    if (gatt_congestion_status == WICED_FALSE && gatt_write_tx == WICED_TRUE)
    {
        p_write = (wiced_bt_gatt_value_t *)wiced_bt_get_buffer(
                                                sizeof(wiced_bt_gatt_value_t)
                                                + GATT_WRITE_BYTES_MAX_LEN);

        if (p_write)
        {
            p_write->handle   = (tput_service_handle) + GATT_WRITE_HANDLE;
            p_write->len      = packet_size;
            p_write->auth_req = GATT_AUTH_REQ_NONE;

            memcpy(p_write->value, gatt_write_bytes, packet_size);

            status = wiced_bt_gatt_send_write(tput_conn_state.conn_id,
                                          GATT_WRITE_NO_RSP, p_write);

            if (status == WICED_BT_GATT_SUCCESS)
            {
                /* Update the GATT TX byte counter */
                gatt_write_tx_packets    += packet_size;
            }

            wiced_bt_free_buffer(p_write);
        }
    }
}

/*******************************************************************************
* Function Name: tput_btn_interrupt_handler()
********************************************************************************
* Summary:
*   User Button1 interrupt callback function.
*
* Parameters:
*   void* unused_user_data  : Not used.
*   uint8_t unused          : Not used.
*
* Return:
*   None
*
*******************************************************************************/
void tput_btn_interrupt_handler( void *unused_user_data, uint8_t unused )
{
    wiced_result_t status = WICED_BT_SUCCESS;

    if (!tput_conn_state.conn_id)
    {
        if (scan_flag)
        {
            /* Start scan */
            status = wiced_bt_ble_scan(BTM_BLE_SCAN_TYPE_HIGH_DUTY, WICED_TRUE,
                                                       tput_scan_result_cback);
            if ((WICED_BT_PENDING == status) || (WICED_BT_BUSY == status))
            {
                WICED_BT_TRACE("\rScanning.....\n");
            }
            else
            {
                WICED_BT_TRACE("\rError: Starting scan failed. Error code: %d\n"
                               , status);
                return;
            }
        }
    }

    if(button_flag)
    {
        /* After connection pressing the user button will change the throughput
         * modes as follows :
         * GATT_Notif_StoC -> GATT_Write_CtoS -> GATT_NotifandWrite -> Roll back
         * to GATT_Notif_StoC
         */

        /* Stop ongoing GATT writes when enabling/disabling server notification,
         * to prevent command failure due to GATT congestion that may occur.
         * The timer will be enabled on GATT event callback based on the
         * status of the GATT operation.
         */
        wiced_stop_timer(&timer_generate_data);

        update_notif_flag = WICED_TRUE;

        /* Change data transfer modes upon interrupt. Based on the current mode,
         * set flags to enable/disable notifications and set/clear GATT write
         * flag
         */
        mode_flag = (mode_flag == GATT_NotifandWrite)? GATT_Notif_StoC : mode_flag + 1u;
        switch(mode_flag)
        {
            case GATT_Notif_StoC:
                enable_cccd = WICED_TRUE;
                gatt_write_tx = WICED_FALSE;
                break;

            case GATT_Write_CtoS:
                enable_cccd = WICED_FALSE;
                gatt_write_tx = WICED_TRUE;
                break;

            case GATT_NotifandWrite:
                enable_cccd = WICED_TRUE;
                gatt_write_tx = WICED_TRUE;
                break;

            default:
                WICED_BT_TRACE("\rInvalid Data Transfer Mode\n");
                break;
        }
    }
}

/*******************************************************************************
* Function Name: tput_gattc_conn_status_cb()
********************************************************************************
* Summary: This function is invoked on GATT connection status change
*
* Parameters:
*   wiced_bt_gatt_connection_status_t *p_status: Handler to connection status
*                                                structure
*
* Return:
*   None
*
*******************************************************************************/
void tput_gattc_conn_status_cb( wiced_bt_gatt_connection_status_t *p_status )
{
    if (p_status->connected)
    {
        tput_gattc_connection_up(p_status);
    }
    else
    {
        tput_gattc_connection_down();
    }
}

/*******************************************************************************
* Function Name: tput_gattc_connection_up()
********************************************************************************
* Summary: This function is invoked when GATT connection is established
*
* Parameters:
*   wiced_bt_gatt_connection_status_t *p_status: Handler to connection status
*                                                structure
*
* Return:
*   None
*
*******************************************************************************/
void tput_gattc_connection_up( wiced_bt_gatt_connection_status_t *p_status )
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_SUCCESS;

    /* Update connected device information */
    tput_conn_state.conn_id = p_status->conn_id;
    WICED_BT_TRACE("\rGATT connected. Connection ID: %d\n", p_status->conn_id);
    memcpy(tput_conn_state.remote_addr, p_status->bd_addr, sizeof(BD_ADDR));

    /* GATT Connected - LED1 ON */
    wiced_hal_gpio_set_pin_output(GATT_CONNECT_LED, GPIO_PIN_OUTPUT_LOW);

    /* Send MTU exchange request */
    result = wiced_bt_gatt_configure_mtu(p_status->conn_id, wiced_bt_cfg_settings.gatt_cfg.max_mtu_size);

    if(result != WICED_BT_GATT_SUCCESS)
    {
        WICED_BT_TRACE("\r%s: GATT MTU configure failed. Error code: %d\n",
                                                     __FUNCTION__, result);
    }

    /* Start the timer which calculates throughput */
    if (wiced_start_timer(&timer_print_tput, TIMER_PRINT_TPUT) != WICED_BT_SUCCESS)
    {
        WICED_BT_TRACE("\rTPUT: seconds timer start failed\n");
    }
}

/*******************************************************************************
* Function Name: tput_gattc_connection_down()
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
void tput_gattc_connection_down(void)
{
    /* Reset all the flags and connection information */
    tput_conn_state.conn_id = 0;
    button_flag = WICED_FALSE;
    scan_flag = WICED_TRUE;
    mode_flag = GATT_Notif_StoC;
    enable_cccd = WICED_TRUE;
    gatt_write_tx = WICED_FALSE;
    update_notif_flag = WICED_FALSE;

    /* Clear tx and rx packet count */
    gatt_notify_rx_packets = 0;
    gatt_write_tx_packets = 0;

    WICED_BT_TRACE("\rTPUT: Disconnected from peer\n");
    WICED_BT_TRACE("\rPress Switch SW3 on your kit to start scanning.....\n");
    /* GATT Disconnected - LED 1 OFF */
    wiced_hal_gpio_set_pin_output(GATT_CONNECT_LED, GPIO_PIN_OUTPUT_HIGH);
    /*LED 2 off - no data transfer*/
    wiced_hal_gpio_set_pin_output(CONGESTION_LED, GPIO_PIN_OUTPUT_HIGH);

    /* We are disconnected now. Stop the timers */
    if (WICED_BT_SUCCESS != wiced_stop_timer(&timer_print_tput))
    {
        WICED_BT_TRACE("\rTPUT: seconds timer stop failed\n");
    }
    if (WICED_BT_SUCCESS != wiced_stop_timer(&timer_generate_data))
    {
        WICED_BT_TRACE("\rTPUT: millisecond timer stop failed\n");
    }
}

/*******************************************************************************
 * Function Name: tput_gattc_callback()
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
tput_gattc_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data )
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_SUCCESS;
    wiced_bt_gatt_value_t *p_write = NULL;

    switch(event)
    {
    case GATT_CONNECTION_STATUS_EVT:
        tput_gattc_conn_status_cb(&p_data->connection_status);
        break;

    case GATT_DISCOVERY_RESULT_EVT:
#ifdef VERBOSE_THROUGHPUT_OUTPUT
        WICED_BT_TRACE("\rstart_handle: %x, end_handle: %x, UUID16: %02x\n",
               p_data->discovery_result.discovery_data.group_value.s_handle,
               p_data->discovery_result.discovery_data.group_value.e_handle,
               p_data->discovery_result.discovery_data.group_value.service_type.uu.uuid16);
#endif
        /* Check if it is throughput service uuid */
        if (!memcmp(&p_data->discovery_result.discovery_data.group_value.service_type.uu.uuid128,
                    &tput_service_uuid, sizeof(LEN_UUID_128)))
        {
            /* Update the handle to TPUT service uuid, Throughput service GATT handle : 0x0009 */
            tput_service_handle = p_data->discovery_result.discovery_data.group_value.s_handle;
            /* After discovery is complete, subscribe for notifications */
            update_notif_flag = WICED_TRUE;
        }
        break;

    case GATT_OPERATION_CPLT_EVT:
        switch (p_data->operation_complete.op)
        {
            case GATTC_OPTYPE_READ:
                WICED_BT_TRACE("\rHandle: %d, Length: %d\n",
                            p_data->operation_complete.response_data.handle,
                            p_data->operation_complete.response_data.att_value.len);
                break;

            case GATTC_OPTYPE_WRITE:
                /* Check if GATT operation of enable/disable notification is success. */
                if ((p_data->operation_complete.response_data.handle == (tput_service_handle + GATT_CCCD_HANDLE))
                    && (WICED_BT_GATT_SUCCESS != p_data->operation_complete.status))
                {
                    WICED_BT_TRACE("\rCCCD update failed. Error: %d\n", p_data->operation_complete.status);
                }

                else if ((p_data->operation_complete.response_data.handle == (tput_service_handle + GATT_CCCD_HANDLE)))
                {
                    button_flag = WICED_TRUE;
                    WICED_BT_TRACE("\rNotifications %s\n", (enable_cccd)?"enabled":"disabled");
                    wiced_hal_gpio_set_pin_output(CONGESTION_LED, enable_cccd);
                    /* Start msec timer only for GATT writes */
                    if(gatt_write_tx)
                    {
                        /* Clear GATT Tx packets */
                        gatt_notify_rx_packets = 0;
                        wiced_start_timer(&timer_generate_data, TIMER_GENERATE_DATA);
                    }
                }
                break;

            case GATTC_OPTYPE_NOTIFICATION:
                /* Receive GATT Notifications from server */
                gatt_notify_rx_packets += p_data->operation_complete.response_data.att_value.len;
                break;

            case GATTC_OPTYPE_CONFIG:
                att_mtu_size = p_data->operation_complete.response_data.mtu;
                WICED_BT_TRACE("\rNegotiated MTU Size: %d\n", att_mtu_size);
                packet_size = get_write_cmd_pkt_size(att_mtu_size);

                /* Send GATT service discovery request */
                wiced_bt_gatt_discovery_param_t gatt_discovery_setup;
                memset(&gatt_discovery_setup, 0, sizeof(gatt_discovery_setup));
                gatt_discovery_setup.s_handle = 1;
                gatt_discovery_setup.e_handle = 0xFFFF;
                gatt_discovery_setup.uuid.len = LEN_UUID_128;
                memcpy(gatt_discovery_setup.uuid.uu.uuid128, tput_service_uuid,
                                                    sizeof(tput_service_uuid));

                if(WICED_BT_GATT_SUCCESS != (result = wiced_bt_gatt_send_discover
                                                    (tput_conn_state.conn_id,
                                                GATT_DISCOVER_SERVICES_BY_UUID,
                                                        &gatt_discovery_setup)))
                {
                    WICED_BT_TRACE("\rGATT Discovery request failed. Error code: %d,\
                                Conn id: %d\n", result, tput_conn_state.conn_id);
                }
                default:
#ifdef VERBOSE_THROUGHPUT_OUTPUT
                WICED_BT_TRACE("\rUnhandled gatt operation complete event: %d\n"
                                                ,p_data->operation_complete.op);
#endif
                break;
        }
        break;
    case GATT_DISCOVERY_CPLT_EVT:
            WICED_BT_TRACE("\rAttributes discovery complete\n");
            break;

    case GATT_CONGESTION_EVT:
        /* GATT congestion event from stack */
        gatt_congestion_status = p_data->congestion.congested;

        if (gatt_congestion_status)
        {
            wiced_hal_gpio_set_pin_output(CONGESTION_LED, GPIO_PIN_OUTPUT_HIGH);
        }
        else
        {
            wiced_hal_gpio_set_pin_output(CONGESTION_LED, GPIO_PIN_OUTPUT_LOW);
        }
        break;

    default:
        WICED_BT_TRACE("\rtput_gattc_callback: Event %d unhandled\n", event);
        break;
    }
    return result;
}

/*******************************************************************************
* Function Name: enable_disable_gatt_notification()
********************************************************************************
* Summary:
*   Enable or Disable GATT notification from the server.
*
* Parameters:
*   uint8_t notify          : Variable indicating whether to turn On/Off GATT
*                             notification.
*
* Return:
*   wiced_bt_gatt_status_t  : Status code from wiced_bt_gatt_status_e.
*
*******************************************************************************/
wiced_bt_gatt_status_t enable_disable_gatt_notification( uint8_t notify )
{
    wiced_bt_gatt_value_t     *p_write = NULL;
    wiced_bt_gatt_status_t     status = WICED_BT_GATT_SUCCESS;

    p_write = (wiced_bt_gatt_value_t *)wiced_bt_get_buffer(sizeof(wiced_bt_gatt_value_t) + 2);

    if (p_write)
    {
        memset(p_write, 0, sizeof(wiced_bt_gatt_value_t) + CCCD_LENGTH);
        p_write->handle   = (tput_service_handle) + GATT_CCCD_HANDLE;
        p_write->len      = CCCD_LENGTH;
        p_write->value[0] = notify; //enable or disable notification;

        status = wiced_bt_gatt_send_write(tput_conn_state.conn_id, GATT_WRITE,
                                                                     p_write);

        if (status != WICED_BT_GATT_SUCCESS)
        {
#ifdef VERBOSE_THROUGHPUT_OUTPUT
            WICED_BT_TRACE("\r[%s]: CCCD write failed. Error code: %d\n",
                                                   __FUNCTION__, status);
#endif
        }
        wiced_bt_free_buffer(p_write);
    }
    return status;
}

/*******************************************************************************
* Function Name: get_write_cmd_pkt_size()
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
static uint16_t get_write_cmd_pkt_size( uint16_t att_mtu_size )
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


