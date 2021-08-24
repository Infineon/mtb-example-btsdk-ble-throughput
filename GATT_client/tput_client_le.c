/******************************************************************************
* File Name:   tput_client_le.c
*
* Description: This file has implementation for functions which do the following:
*   * Bluetooth LE initialization(registering gatt db, gatt callback,
*                setting adv data etc..)
*   * Bluetooth LE stack event handler - to process the Bluetooth LE events.
*   * Button interrupt registration and callback implementation.
*   * 1 second timer init and callback - for throughput calculation.
*   * RTOS Thread to handle button press events and send GATT write commands.
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
#include "tput_client_le.h"
#include "wiced_hal_wdog.h"
#include "wiced_rtos.h"

/*******************************************************************************
 *                                Global variables
 ******************************************************************************/
/* Variables to hold GATT notification bytes received */
static uint32_t           gatt_notify_rx_bytes    = 0;
/* Flag to indicate GATT congestion */
static wiced_bool_t       gatt_congestion_status  = WICED_FALSE;
/* Handle to the Throughput Measurement service */
static uint16_t           tput_service_handle     = 0;
/* To store connection related deatils */
static tput_conn_state_t  tput_conn_state;
/* Timer for printing throughput data */
static wiced_timer_t      timer_print_tput;
/* Pointer to store memory allocated to the new thread while thread creation */
wiced_thread_t*           send_gatt_write_thread   = NULL;

/* Variable to hold all details related to Client data transmission */
static tput_client_tx_data_struct_t client_tx_data = {
    .gatt_write_tx_bytes = 0,
    .enable_cccd = WICED_TRUE,
    .gatt_write_tx = WICED_FALSE,
    .packet_size = 0,
    .p_write = NULL
};

/* Variable to hold flag values required for the application */
static tput_client_flags_t tput_client_flags = {
    .button_pressed = WICED_FALSE,
    .scan_flag = WICED_TRUE,
    .mode_flag = GATT_NOTIFY_SERVER_TO_CLIENT
};

const uint8_t tput_service_uuid[LEN_UUID_128] = TPUT_SERVICE_UUID;

/*******************************************************************************
 *                              Function Protoypes
 ******************************************************************************/
static wiced_bt_gatt_status_t tput_gattc_callback(wiced_bt_gatt_evt_t event,
                                 wiced_bt_gatt_event_data_t *p_event_data );
static wiced_bt_gatt_status_t tput_set_gatt_notification(uint8_t notify);
static void gattc_operation_complete_event_handler
                               (wiced_bt_gatt_operation_complete_t *event_data);
static void tput_scan_result_cback(wiced_bt_ble_scan_results_t *p_scan_result,
                                   uint8_t *p_adv_data);
static void tput_app_sec_timeout(uint32_t cb_param);
static void tput_btn_interrupt_callback(void* user_data, uint8_t pin);
static void tput_btn_press_handler(void);
static void tput_client_init(void);
static void tput_gattc_connection_down(wiced_bt_gatt_connection_status_t *p_status);
static void tput_gattc_connection_up(wiced_bt_gatt_connection_status_t *p_status);
static void tput_gattc_conn_status_cb(wiced_bt_gatt_connection_status_t *p_status);
static void tput_send_gatt_write_thread(uint32_t thread_arg);
static uint16_t tput_get_write_packet_size(uint16_t att_mtu_size);


/*******************************************************************************
* Function Name: tput_management_callback
********************************************************************************
* Summary:
*   This is a Bluetooth LE management event handler function to receive events
*   from the stack and process as needed by the application.
*
* Parameters:
*   wiced_bt_management_evt_t event             : Bluetooth LE event code of one
*                                                 byte length
*   wiced_bt_management_evt_data_t *p_event_data: Pointer to Bluetooth LE
*                                                 management event structures
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

#if VERBOSE_THROUGHPUT_OUTPUT
    WICED_BT_TRACE("Throughput GATT client management callback event: %d \r\n", event);
#endif

    switch (event)
    {
        case BTM_ENABLED_EVT:
            /* Handle initialize sequence for the application */
            tput_client_init();
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
#if VERBOSE_THROUGHPUT_OUTPUT
            WICED_BT_TRACE("Scan %s \r\n", (BTM_BLE_SCAN_TYPE_NONE !=
                              p_event_data->ble_scan_state_changed) ?
                                                "started":"stopped");
#endif
            break;

        case BTM_BLE_PHY_UPDATE_EVT:
            WICED_BT_TRACE("Selected TX PHY - %dM \r\nSelected RX PHY - %dM \r\n",
                                      p_event_data->ble_phy_update_event.tx_phy,
                                      p_event_data->ble_phy_update_event.tx_phy);
            break;

        case BTM_BLE_CONNECTION_PARAM_UPDATE:
            result = p_event_data->ble_connection_param_update.status;
            /* Connection parameters updated */
            if(WICED_SUCCESS == result)
            {
                conn_interval = (p_event_data->ble_connection_param_update.conn_interval)
                                * CONN_INTERVAL_MULTIPLIER;
                WICED_BT_TRACE("New connection parameters:"
                               "\r\nConnection interval = %d.%dms \r\n",
                               CONN_INTERVAL_MAJOR(conn_interval),
                               CONN_INTERVAL_MINOR(conn_interval));
            }
            else
            {
                WICED_BT_TRACE("Connection parameters update failed, Error Code: %d\r\n",
                                p_event_data->ble_connection_param_update.status);
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
                WICED_BT_TRACE("Request sent to switch PHY to %dM \r\n",
                                              phy_preferences.tx_phys);
            }
            else
            {
                WICED_BT_TRACE("TPUT: PHY switch request failed, result: %d \r\n",
                               status);
            }
            break;

        case  BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
              result = WICED_BT_ERROR;
            break;

        default:
#if VERBOSE_THROUGHPUT_OUTPUT
            WICED_BT_TRACE("Unhandled management callback event: %d \r\n", event);
#endif
            break;
    }
    return (result);
}

/*******************************************************************************
* Function Name: tput_client_init
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
void tput_client_init( void )
{
    wiced_bt_gatt_status_t gatt_status = WICED_BT_SUCCESS;
    wiced_result_t result = WICED_ERROR;
    /* Configure button interrupt callback */
    wiced_platform_register_button_callback(USER_BUTTON1,
                                            tput_btn_interrupt_callback,
                                            NULL,
                                            WICED_PLATFORM_BUTTON_RISING_EDGE);

    /* Register with stack to receive GATT callback */
    gatt_status = wiced_bt_gatt_register(tput_gattc_callback);
    if (WICED_BT_SUCCESS != gatt_status)
    {
        WICED_BT_TRACE("GATT registration failed \r\n");
    }

    /* Tell the bluetooth stack to use the GATT database */
    gatt_status = wiced_bt_gatt_db_init(gatt_database, gatt_database_len);
    if (WICED_BT_SUCCESS != gatt_status)
    {
        WICED_BT_TRACE("GATT database init failed \r\n");
    }

    /* Create a thread to send notifications when enabled, to the client device */
    send_gatt_write_thread = wiced_rtos_create_thread();
    if(NULL != send_gatt_write_thread)
    {
        result = wiced_rtos_init_thread(send_gatt_write_thread,
                                    SEND_GATT_WRITE_THREAD_PRIORITY,
                                    "Send_gatt_write",
                                    tput_send_gatt_write_thread,
                                    SEND_GATT_WRITE_THREAD_SIZE,
                                    NULL);
        if(WICED_BT_SUCCESS != result)
        {
            WICED_BT_TRACE("Thread initialization failed!\r\n");
        }
    }
    else
    {
        WICED_BT_TRACE("Thread creation failed!\r\n");
    }
    
    /* One second timer: To calculate and display the throughput result. */
    if (WICED_BT_SUCCESS != wiced_init_timer(&timer_print_tput,
                                             tput_app_sec_timeout,
                                             0,
                                             WICED_SECONDS_PERIODIC_TIMER))
    {
        WICED_BT_TRACE("Seconds timer init failed\r\n");
    }

    /* Pairing is not needed for throughput test */
    wiced_bt_set_pairable_mode(WICED_FALSE, 0);

}

/*******************************************************************************
* Function Name: tput_scan_result_cback
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
void tput_scan_result_cback(wiced_bt_ble_scan_results_t *p_scan_result,
                            uint8_t *p_adv_data)
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
                WICED_BT_TRACE("Scan completed\r\n");
                WICED_BT_TRACE("Found Peer Device : %B \r\n",
                                p_scan_result->remote_bd_addr);
                tput_client_flags.scan_flag = WICED_FALSE;

                /* Device found. Stop scan. */
                if((status = wiced_bt_ble_scan(BTM_BLE_SCAN_TYPE_NONE,
                                            WICED_TRUE,
                                            tput_scan_result_cback))!= 0)
                {
                    WICED_BT_TRACE("scan off status %d\r\n", status);
                }

                /* Initiate the connection */
                if(wiced_bt_gatt_le_connect(p_scan_result->remote_bd_addr,
                                            p_scan_result->ble_addr_type,
                                                BLE_CONN_MODE_HIGH_DUTY,
                                                WICED_TRUE)!= WICED_TRUE)
                {
                    WICED_BT_TRACE("GATT connection API failure\r\n");
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
* Function Name: tput_app_sec_timeout
********************************************************************************
* Summary:
*   One second timer callback. Calculate and print the TX and RX rate over PUART
*   console.
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
    if (tput_conn_state.conn_id && gatt_notify_rx_bytes )
    {
        gatt_notify_rx_bytes = (gatt_notify_rx_bytes * 8)/1000;
        WICED_BT_TRACE("GATT NOTIFICATION : Client Throughput (RX) = %d kbps\r\n",
                       gatt_notify_rx_bytes);
        gatt_notify_rx_bytes = 0; //Reset the byte counter
    }

    if ((tput_conn_state.conn_id) && client_tx_data.gatt_write_tx_bytes )
    {
        client_tx_data.gatt_write_tx_bytes = (client_tx_data.gatt_write_tx_bytes * 8)/1000;
        WICED_BT_TRACE("GATT WRITE        : Client Throughput (TX) = %d kbps\r\n",
                       client_tx_data.gatt_write_tx_bytes);
        client_tx_data.gatt_write_tx_bytes = 0; //Reset the byte counter
    }
}

/*******************************************************************************
* Function Name: tput_send_gatt_write_thread
********************************************************************************
* Summary:
*   The thread handles button press events - to start scanning or to decide the
*   data transfer mode. It also sends out GATT Write(with no response) commands
*   to the server when bluetooth LE TX buffer is available and if GATT writes
*   are allowed to send by the application.
*
* Parameters:
*   uint32_t thread_arg: not used
*
* Return:
*   None
*
*******************************************************************************/
void tput_send_gatt_write_thread(uint32_t thread_arg)
{
    wiced_bt_gatt_status_t status;
    uint32_t buf_left = 0;

    while(WICED_TRUE)
    {
        if(tput_client_flags.button_pressed)
        {
            /* When button is pressed, check if it is to start scan or change data
            tranfer mode */
            tput_btn_press_handler();
        }

        /* Start sending GATT write commands if the user has switched to mode
         * 2 or 3 */
        if(!client_tx_data.gatt_write_tx)
        {
            /* Thread has nothing to do. Put the thread to blocked state until
            *  the timeout */
            wiced_rtos_delay_milliseconds(NO_TX_THREAD_SLEEP_TIMEOUT,
                                          ALLOW_THREAD_TO_SLEEP);
        }
        else
        {
            /* Send GATT write(with no response) commands to the server if
             * Bluetooth LE TX buffer is available*/
            buf_left = wiced_bt_ble_get_available_tx_buffers();
            if (buf_left > MINIMUM_TX_BUFFER_LEN)
            {
                status = wiced_bt_gatt_send_write(tput_conn_state.conn_id,
                                                  GATT_WRITE_NO_RSP,
                                                  client_tx_data.p_write);

                if (status == WICED_BT_GATT_SUCCESS)
                {
                    /* Turn on LED2 to show GATT write is happening */
                    wiced_hal_gpio_set_pin_output(GATT_WRITE_LED,
                                                  GPIO_PIN_OUTPUT_LOW);
                }
                else
                {
                    /* Turn off LED2 when no GATT writes are happening */
                    wiced_hal_gpio_set_pin_output(GATT_WRITE_LED,
                                                  GPIO_PIN_OUTPUT_HIGH);
                }
            }
        }
    }
}

/*******************************************************************************
* Function Name: tput_btn_interrupt_callback
********************************************************************************
* Summary:
*   User Button1 interrupt callback function. Makes the button pressed flag as
*    true so that the rtos thread can process this event.
*
* Parameters:
*   void* user_data  : Not used.
*   uint8_t pin      : Not used.
*
* Return:
*   None
*
*******************************************************************************/
void tput_btn_interrupt_callback(void* user_data, uint8_t pin)
{
    tput_client_flags.button_pressed = WICED_TRUE;
}

/*******************************************************************************
* Function Name: tput_btn_press_handler
********************************************************************************
* Summary:
*   Handler function to start scan or switch to different modes of throughput on
*   button press.
*
* Parameters:
*   None
*
* Return:
*   None
*
*******************************************************************************/
static void tput_btn_press_handler(void)
{
    wiced_result_t result = WICED_BT_SUCCESS;
    wiced_bt_gatt_status_t  status = WICED_BT_GATT_ERROR;
    static uint8_t cccd_write_retry_count = 0;

    /* Check if the device is not connected to the peer yet and start scanning */
    if (!tput_conn_state.conn_id && tput_client_flags.scan_flag)
    {
        /* Start scan */
        result = wiced_bt_ble_scan(BTM_BLE_SCAN_TYPE_HIGH_DUTY, WICED_TRUE,
                                                tput_scan_result_cback);
        if ((WICED_BT_PENDING == result) || (WICED_BT_BUSY == result))
        {
            WICED_BT_TRACE("Scanning.....\r\n");
        }
        else
        {
            WICED_BT_TRACE("Starting scan failed! Error code: %d\r\n", result);
            return;
        }
    }
    else if(tput_service_handle)
    {
        /* After connection pressing the user button will change the throughput
        * modes as follows :
        * GATT_NOTIFY_SERVER_TO_CLIENT -> GATT_WRITE_CLIENT_TO_SERVER ->
        * GATT_NOTIFY_AND_WRITE -> Roll back to GATT_NOTIFY_SERVER_TO_CLIENT
        */
        /* Stop ongoing GATT writes when enabling/disabling server notification,
         * to prevent command failure due to GATT congestion that may occur.
         */
        client_tx_data.gatt_write_tx = WICED_FALSE;
        wiced_hal_gpio_set_pin_output(GATT_WRITE_LED, GPIO_PIN_OUTPUT_HIGH);

        /* Change data transfer modes upon interrupt. Based on the current mode,
            * set flag to enable/disable notifications.
            */
        tput_client_flags.mode_flag = (tput_client_flags.mode_flag ==
                                       GATT_NOTIFY_AND_WRITE)?
                                       GATT_NOTIFY_SERVER_TO_CLIENT :
                                       tput_client_flags.mode_flag + 1u;

        if((tput_client_flags.mode_flag == GATT_NOTIFY_SERVER_TO_CLIENT) ||
           (tput_client_flags.mode_flag == GATT_NOTIFY_AND_WRITE))
        {
            client_tx_data.enable_cccd = WICED_TRUE;
        }
        else
        {
            client_tx_data.enable_cccd = WICED_FALSE;
        }

        /* CCCD write may not go through at the first attempt because of
         * GATT congestion. If the API returns failure, retry CCCD write
         * until the max retires set. */
        cccd_write_retry_count = 0;
        status = WICED_BT_GATT_ERROR;
        while((status != WICED_BT_SUCCESS) &&
              (cccd_write_retry_count < CCCD_WRITE_MAX_RETRIES))
        {
            if(!gatt_congestion_status)
            {
                status = tput_set_gatt_notification(client_tx_data.enable_cccd);
            }
            /* Give a small delay before calling the function again on failure */
            wiced_rtos_delay_microseconds(100);
            cccd_write_retry_count++;
        }
#if VERBOSE_THROUGHPUT_OUTPUT
        WICED_BT_TRACE("Enable/Disable CCCD failed after %d retires, "
                       "error code = %x\r\n", CCCD_WRITE_MAX_RETRIES, status);
#endif
    }
    else
    {
        WICED_BT_TRACE("Throughput Service not found\r\n");
    }
    tput_client_flags.button_pressed = WICED_FALSE;
}
/*******************************************************************************
* Function Name: tput_gattc_conn_status_cb
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
void tput_gattc_conn_status_cb(wiced_bt_gatt_connection_status_t *p_status)
{
    if (p_status->connected)
    {
        tput_gattc_connection_up(p_status);
    }
    else
    {
        tput_gattc_connection_down(p_status);
    }
}

/*******************************************************************************
* Function Name: tput_gattc_connection_up
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
void tput_gattc_connection_up(wiced_bt_gatt_connection_status_t *p_status)
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_SUCCESS;

    /* Update connected device information */
    tput_conn_state.conn_id = p_status->conn_id;
    WICED_BT_TRACE("GATT connected. Connection ID: %d\r\n", p_status->conn_id);
    memcpy(tput_conn_state.remote_addr, p_status->bd_addr, sizeof(BD_ADDR));

    /* GATT Connected - LED1 ON */
    wiced_hal_gpio_set_pin_output(GATT_CONNECT_LED, GPIO_PIN_OUTPUT_LOW);

    /* Send MTU exchange request */
    result = wiced_bt_gatt_configure_mtu(p_status->conn_id,
                                 wiced_bt_cfg_settings.gatt_cfg.max_mtu_size);

    if(result != WICED_BT_GATT_SUCCESS)
    {
        WICED_BT_TRACE("GATT MTU configure failed. Error code: %d\r\n", result);
    }

    /* Start the timer which calculates throughput */
    if (WICED_BT_SUCCESS != wiced_start_timer(&timer_print_tput, TIMER_PRINT_TPUT))
    {
        WICED_BT_TRACE("TPUT: seconds timer start failed\r\n");
    }
}

/*******************************************************************************
* Function Name: tput_gattc_connection_down
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
void tput_gattc_connection_down(wiced_bt_gatt_connection_status_t *p_status)
{
    /* Reset all the flags and connection information */
    tput_conn_state.conn_id = 0;
    client_tx_data.enable_cccd = WICED_TRUE;
    client_tx_data.gatt_write_tx = WICED_FALSE;
    tput_client_flags.button_pressed = WICED_FALSE;
    tput_client_flags.scan_flag = WICED_TRUE;
    tput_client_flags.mode_flag = GATT_NOTIFY_SERVER_TO_CLIENT;

    /* Clear tx and rx packet count */
    gatt_notify_rx_bytes = 0;
    client_tx_data. gatt_write_tx_bytes = 0;

    WICED_BT_TRACE("Disconnected from peer, Reason: %d\r\n", p_status->reason);
    WICED_BT_TRACE("Press Switch SW3 on your kit to start scanning.....\r\n");
    /* GATT Disconnected - LED 1 OFF */
    wiced_hal_gpio_set_pin_output(GATT_CONNECT_LED, GPIO_PIN_OUTPUT_HIGH);
    /*LED 2 off - no data transfer*/
    wiced_hal_gpio_set_pin_output(GATT_WRITE_LED, GPIO_PIN_OUTPUT_HIGH);

    /* We are disconnected now. Stop the timer */
    if (WICED_BT_SUCCESS != wiced_stop_timer(&timer_print_tput))
    {
        WICED_BT_TRACE("Seconds timer stop API failed\r\n");
    }
}

/*******************************************************************************
 * Function Name: tput_gattc_callback
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
tput_gattc_callback(wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data)
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_SUCCESS;

    switch(event)
    {
    case GATT_CONNECTION_STATUS_EVT:
        tput_gattc_conn_status_cb(&p_data->connection_status);
        break;

    case GATT_DISCOVERY_RESULT_EVT:
#if VERBOSE_THROUGHPUT_OUTPUT
        WICED_BT_TRACE("start_handle: %x, end_handle: %x, UUID16: %02x\r\n",
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
        }
        break;

    case GATT_OPERATION_CPLT_EVT:
        gattc_operation_complete_event_handler(&p_data->operation_complete);
        break;

    case GATT_DISCOVERY_CPLT_EVT:
            WICED_BT_TRACE("Attributes discovery complete\r\n");
            client_tx_data.p_write = (wiced_bt_gatt_value_t *)wiced_bt_get_buffer(
                                        sizeof(wiced_bt_gatt_value_t)
                                        + GATT_WRITE_BYTES_MAX_LEN);
            if (client_tx_data.p_write)
            {
                client_tx_data.p_write->handle   = (tput_service_handle) + GATT_WRITE_HANDLE;
                client_tx_data.p_write->len      = client_tx_data.packet_size;
                client_tx_data.p_write->auth_req = GATT_AUTH_REQ_NONE;
            }
            /* Enable notifications */
            tput_set_gatt_notification(WICED_TRUE);
            break;

    case GATT_CONGESTION_EVT:
        /* GATT congestion event from stack */
        gatt_congestion_status = p_data->congestion.congested;
        break;

    default:
        WICED_BT_TRACE("TPUT gatt client callback: Event %d unhandled\r\n", event);
        break;
    }
    return result;
}

/*******************************************************************************
* Function Name: gattc_operation_complete_event_handler
********************************************************************************
* Summary:
*   Handle GATT operation complete event.
*
* Parameters:
*   wiced_bt_gatt_operation_complete_t *op_complete: Poniter to event data
*
* Return:
*   None
*
*******************************************************************************/
static void gattc_operation_complete_event_handler
                                (wiced_bt_gatt_operation_complete_t *event_data)
{
    wiced_bt_gatt_status_t result;
    switch (event_data->op)
    {
        case GATTC_OPTYPE_READ:
            WICED_BT_TRACE("Handle: %d, Length: %d\r\n",
                            event_data->response_data.handle,
                            event_data->response_data.att_value.len);
            break;

        case GATTC_OPTYPE_WRITE:
            /* Check if GATT operation of enable/disable notification is success. */
            if ((event_data->response_data.handle ==
                (tput_service_handle + GATT_CCCD_HANDLE)) &&
                (WICED_BT_GATT_SUCCESS != event_data->status))
            {
                WICED_BT_TRACE("CCCD update failed. Error Code: %d\r\n", event_data->status);
            }

            else if ((event_data->response_data.handle ==
                     (tput_service_handle + GATT_CCCD_HANDLE)))
            {
                WICED_BT_TRACE("Notifications %s\r\n",
                               (client_tx_data.enable_cccd)?"enabled":"disabled");

                client_tx_data.gatt_write_tx_bytes = 0;
                gatt_notify_rx_bytes = 0;

                if((tput_client_flags.mode_flag == GATT_WRITE_CLIENT_TO_SERVER) ||
                   (tput_client_flags.mode_flag == GATT_NOTIFY_AND_WRITE))
                {
                    client_tx_data.gatt_write_tx = WICED_TRUE;
                }
                else
                {
                    client_tx_data.gatt_write_tx = WICED_FALSE;
                }
            }

            else if((event_data->response_data.handle ==
                     tput_service_handle + GATT_WRITE_HANDLE))
            {
                /* Accumulate the successfully sent bytes to calculate
                * throughput after 1 second */
                client_tx_data.gatt_write_tx_bytes += client_tx_data.packet_size;
            }
            break;

        case GATTC_OPTYPE_NOTIFICATION:
            /* Receive GATT Notifications from server */
            gatt_notify_rx_bytes += event_data->response_data.att_value.len;
            break;

        case GATTC_OPTYPE_CONFIG:
            tput_conn_state.mtu = event_data->response_data.mtu;
            WICED_BT_TRACE("Negotiated MTU Size: %d\r\n", tput_conn_state.mtu);
            client_tx_data.packet_size = tput_get_write_packet_size(tput_conn_state.mtu);

            /* Send GATT service discovery request */
            wiced_bt_gatt_discovery_param_t gatt_discovery_setup;
            memset(&gatt_discovery_setup, 0, sizeof(gatt_discovery_setup));
            gatt_discovery_setup.s_handle = 1;
            gatt_discovery_setup.e_handle = 0xFFFF;
            gatt_discovery_setup.uuid.len = LEN_UUID_128;
            memcpy(gatt_discovery_setup.uuid.uu.uuid128, tput_service_uuid,
                                                sizeof(tput_service_uuid));

            result = wiced_bt_gatt_send_discover(tput_conn_state.conn_id,
                                                 GATT_DISCOVER_SERVICES_BY_UUID,
                                                 &gatt_discovery_setup);
            if(WICED_BT_GATT_SUCCESS != result)
            {
                WICED_BT_TRACE("GATT Discovery request failed. Error code: %d,\
                            Conn id: %d\r\n", result, tput_conn_state.conn_id );
            }
            break;
        default:
#if VERBOSE_THROUGHPUT_OUTPUT
            WICED_BT_TRACE("Unhandled gatt operation complete event: %d\r\n",
                            event_data->op);
#endif
            break;
    }
}
/*******************************************************************************
* Function Name: tput_set_gatt_notification
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
wiced_bt_gatt_status_t tput_set_gatt_notification( uint8_t notify )
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
#if VERBOSE_THROUGHPUT_OUTPUT
            WICED_BT_TRACE("CCCD write failed. Error code: %d\r\n", status);
#endif
        }
        wiced_bt_free_buffer(p_write);
    }
    return status;
}

/*******************************************************************************
* Function Name: tput_get_write_packet_size
********************************************************************************
* Summary: This function decides size of notification packet based on the
*   attribute MTU exchanged. This is done to utilize the LL payload space
*   effectively.
*
* Parameters:
*   uint16_t att_mtu_size: MTU value exchanged after connection.
*
* Return:
*   uint16_t: Size of notification packet derived based on MTU.
*
*******************************************************************************/
static uint16_t tput_get_write_packet_size( uint16_t att_mtu_size )
{
    if(att_mtu_size < DATA_PACKET_SIZE_1 + ATT_HEADER)
    {
       /* Packet Length = ATT_MTU_SIZE - ATT_HANDLE(2 bytes) - ATT_OPCODE(1 byte)
        * Reason: With DLE enabled, LL payload is 251 bytes. So if an MTU less
        * than 247 is exchanged, the data can be accommodated in a single LL
        * packet */
        client_tx_data.packet_size = att_mtu_size - ATT_HEADER;
    }
    else if((att_mtu_size >= DATA_PACKET_SIZE_1 + ATT_HEADER) &&
            (att_mtu_size < DATA_PACKET_SIZE_2 + ATT_HEADER))
    {
        /* If MTU is between 247 and 498, if a packet size other than 244 bytes
         * is used, the data will be split and the LL payload space is not
         * utilized effectively. Refer README for details */
        client_tx_data.packet_size = DATA_PACKET_SIZE_1;
    }
    else
    {
        /* For MTU value greater than 498, if a packet size other than 495(or 244)
         * is used, the LL payload space is not utilized effectively.
         * 495 bytes will go as two LL packets: 244 bytes + 251 bytes */
        client_tx_data.packet_size = DATA_PACKET_SIZE_2;
    }
    return client_tx_data.packet_size;
}


