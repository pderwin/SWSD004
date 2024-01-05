/*!
 * @ingroup   tracker_application
 * @file      main_tracker_application.c
 *
 * @brief     LoRa Basics Modem LR11XX tracker application implementation.
 *
 * @copyright
 * @parblock
 * The Clear BSD License
 * Copyright Semtech Corporation 2021. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Semtech corporation nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT
 * NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 * @endparblock
 */

/*!
 * @addtogroup tracker_application
 * LoRa Basics Modem LR11XX tracker application
 * @{
 */

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/trace.h>
#include <stdio.h>
#include <string.h>
#include "firmware_update.h"
#include "main_tracker_application.h"
#include "tracker_utility.h"
#include "lorawan_key_config.h"
#include "smtc_board.h"

#define SEMTRACKER_PRIORITY  (4)
#define SEMTRACKER_STACKSIZE (2048)

K_THREAD_STACK_DEFINE(semtracker_stack_area, SEMTRACKER_STACKSIZE);
struct k_thread	semtracker_kthread;

#ifdef PHIL


#include "smtc_hal.h"
#endif

#include "apps_utilities.h"
#include "apps_modem_common.h"
#include "apps_modem_event.h"

#ifdef PHIL
#include "lr11xx_gnss.h"
#endif // PHIL
#include "lr11xx_system.h"

#ifdef PHIL

#include "lr11xx_bootloader.h"

#endif // PHIL

#include "smtc_hal_dbg_trace.h"
#include "smtc_modem_hal.h"
#include "smtc_basic_modem_lr11xx_api_extension.h"

#if PHIL
#include "smtc_modem_test_api.h"
#endif


#include "smtc_modem_utilities.h"
#include "smtc_board_ralf.h"
#include "ralf.h"
#include "lr1110_trk1xks_board.h"


#ifdef PHIL
// #include "smtc_lr11xx_board.h"
// #include "ble_thread.h"

#endif // PHIL
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*!
 * @brief Force a new tracker context in flash memory
 */
#define FORCE_NEW_TRACKER_CONTEXT 1

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*!
 * @brief Tracker context structure
 */
extern tracker_ctx_t tracker_ctx;

/*!
 * @brief Stack identifier
 */
static uint8_t stack_id = 0;

/*!
 * @brief Modem radio
 */
ralf_t* tracker_modem_radio;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*!
 * @brief Init the applicativ tracker context
 *
 * @param [in] dev_eui LoRaWAN Device Eui
 * @param [in] join_eui LoRaWAN Join Eui
 * @param [in] app_key LoRaWAN Application Key
 */
static void tracker_app_init_context( uint8_t* dev_eui, uint8_t* join_eui, uint8_t* app_key );

/*!
 * @brief Lorawan default init
 */
static void tracker_app_modem_configure_lorawan_params( void );

/*!
 * @brief   Check if the tracker is in static mode
 *
 * @returns  [true : tracker is static, false : tracker in movement]
 */
static bool tracker_app_is_tracker_in_static_mode( void );

/*!
 * @brief Parse the received downlink
 *
 * @param [in] port LoRaWAN port
 * @param [in] payload Payload Buffer
 * @param [in] size Payload size
 */
static void tracker_app_parse_downlink_frame( uint8_t port, const uint8_t* payload, uint8_t size );

/*!
 * @brief Update in flash context the accumulated charge if has changed
 *
 * @param [in] charge_mAh the actual charge
 */
static void tracker_app_store_new_accumulated_charge( uint32_t charge_mAh );

/*!
 * @brief read and build sensor payload then send it
 */
static void tracker_app_read_and_send_sensors( void );

/*!
 * @brief build tracker settings payload in TLV format
 *
 * @param [in] buffer Buffer containing the tracker settings
 * @param [in] len Len of the buffer
 */
static void tracker_app_build_and_send_tracker_settings( const uint8_t* buffer, uint8_t len );

/*!
 * @brief Get and update in flash context the gnss assistance position if has changed
 *
 * @param [in] gnss_scan_results GNSS scan results \ref gnss_mw_event_data_scan_done_t
 */
static void tracker_gnss_store_new_assistance_position( gnss_mw_event_data_scan_done_t* gnss_scan_results );

/*!
 * @brief Get, Check and display the lr11xx firmware version
 */
static bool tracker_app_lr11xx_check_firmware_version( const void* context );

/*!
 * @brief Get, Check and display the LoRa Basics Modem firmware version
 */
static void tracker_app_display_lbm_version( void );

/*!
 * @brief Set the ADR according to the used region
 */
static void tracker_app_set_adr( void );

/**
 * @brief Set the maximum TX output power following the region used
 *
 * @param [in] region the region used by the modem, see \ref smtc_modem_region_t
 * @param [in] sub_region the sub region used by the modem, see \ref smtc_modem_sub_region_t
 */
static void tracker_app_set_max_tx_power_supported( smtc_modem_region_t region, smtc_modem_sub_region_t sub_region );

/*!
 * @addtogroup basics_modem_evt_callback
 * LoRa Basics Modem event callbacks
 * @{
 */

/*!
 * @brief Reset event callback
 *
 * @param [in] reset_count reset counter from the modem
 */
static void on_modem_reset( uint16_t reset_count );

/*!
 * @brief Network Joined event callback
 */
static void on_modem_network_joined( void );

/*!
 * @brief Downlink data event callback.
 *
 * @param [in] rssi       RSSI in signed value in dBm + 64
 * @param [in] snr        SNR signed value in 0.25 dB steps
 * @param [in] rx_window  RX window
 * @param [in] port       LoRaWAN port
 * @param [in] payload    Received buffer pointer
 * @param [in] size       Received buffer size
 */
static void on_modem_down_data( int8_t rssi, int8_t snr, smtc_modem_event_downdata_window_t rx_window, uint8_t port,
                                const uint8_t* payload, uint8_t size );

/*!
 * @brief Clock synchronisation event callback
 *
 * @param [in] time_status Status returned by the TIME event
 */
static void on_modem_clk_synch( smtc_modem_event_time_status_t time_status );

/*!
 * @brief GNSS Almanac update event callback
 *
 * @param [in] status Status returned by the ALMANAC_UPDATE event
 */
static void on_modem_almanac_update( smtc_modem_event_almanac_update_status_t status );

/*!
 * @brief GNSS middleware event callback
 *
 * @param [in] pending_events Events returned by the GNSS middleware event
 */
static void on_middleware_gnss_event( uint8_t pending_events );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */
const void *lr11xx_hal_get_radio_context( void );

extern void     debug_gpio_init      (void);
extern void on_wifi_event (uint8_t pending_events);

extern uint32_t accel_has_moved (void);
extern void     wifi_scan_start (void);

/**
 * @brief Main application entry point.
 */
void semtracker_application( void *p1, void *p2, void *p3)
{
    uint32_t
       curr_time_secs,
       last_wifi_scan_secs = 0,
       sleep_time_ms       = 0;
    static uint8_t
       app_key[16] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe },
       dev_eui[8]  = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0b, 0x03 },
       join_eui[8] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x03, 0x31, 0xC9 };

    static apps_modem_event_callback_t smtc_event_callback = {
        .adr_mobile_to_static  = NULL,
        .alarm                 = NULL,
        .almanac_update        = on_modem_almanac_update,
        .down_data             = on_modem_down_data,
        .join_fail             = NULL,
        .joined                = on_modem_network_joined,
        .link_status           = NULL,
        .mute                  = NULL,
        .new_link_adr          = NULL,
        .reset                 = on_modem_reset,
        .set_conf              = NULL,
        .stream_done           = NULL,
        .time_updated_alc_sync = on_modem_clk_synch,
        .tx_done               = NULL,
        .upload_done           = NULL,
        .user_radio_access     = NULL,
        .middleware_1          = on_middleware_gnss_event,
        .middleware_2          = on_wifi_event,
    };

    /*
     * Get some debug pins initialized
     */
    debug_gpio_init();

    /* Initialise the ralf_t object corresponding to the board */
    tracker_modem_radio = smtc_board_initialise_and_get_ralf( );

    /*
     * Some board-specific things that need to be moved out of here some day
     */
#ifndef CONFIG_BOARD_MOKO_LW008
    {
      const struct device *lna_dev;

      lna_dev = device_get_binding("gpio@50000000");

      if (lna_dev) {
         gpio_pin_configure(lna_dev, 25, GPIO_OUTPUT_ACTIVE);
         gpio_pin_set(lna_dev, 25, 1);
      }
      else {
         printk("%s: GPIO_1 block not found\n", __func__);
      }
   }

#define FORCE_GPIO_1_7_HIGH 1
#if	defined(FORCE_GPIO_1_7_HIGH)
    {
       const struct device *dev1;

       /*
        * This appears to be a UART DTE signal
        */
       dev1 = device_get_binding("gpio@50000300");
       if (dev1) {
          gpio_pin_configure(dev1, 7, GPIO_OUTPUT_ACTIVE);
          gpio_pin_set(dev1, 7, 1);
       }
       else {
          printk("%s: GPIO_1 block not found\n", __func__);
       }
    }
#endif
#endif

    /*
     * TODO: the context really should be given to the modem_radio as it is
     *       initizlized above.  This setting of the context is a kludge.
     */
    tracker_modem_radio->ral.context = lr11xx_hal_get_radio_context();

    /* Disable IRQ to avoid unwanted behaviour during init */
//    hal_mcu_disable_irq( );

    /* Init board and peripherals */
//    hal_mcu_init( );
//    smtc_board_init_periph( );

      /* Init the Lora Basics Modem event callbacks */
    apps_modem_event_init( &smtc_event_callback );

    /* Init the modem and use apps_modem_event_process as event callback, please note that the callback will be called
     * immediately after the first call to modem_run_engine because of the reset detection */
    smtc_modem_init( tracker_modem_radio, &apps_modem_event_process );

  /* Check LR11XX Firmware version */
    if( tracker_app_lr11xx_check_firmware_version( tracker_modem_radio->ral.context ) != true )
    {
       HAL_DBG_TRACE_INFO( "Something goes wrong with the LR11XX firmware, stay in BLE mode and update it\n" );
       hal_mcu_enable_irq( );
       tracker_app_init_context( dev_eui, join_eui, app_key );
       while( 1 )
       {
          /* Stay in BLE while the LR11XX firmware is not installed */
//          start_ble_thread( 0 );
          k_sleep(K_MSEC(1000));
          printk("%s %d waiting for BLE\n", __func__, __LINE__);

       }
    }

    /* Re-enable IRQ */
//    hal_mcu_enable_irq( );

    /* Notify user that the board is initialized */
    smtc_board_leds_blink( smtc_board_get_led_all_mask( ), 100, 2 );

    HAL_DBG_TRACE_MSG( "\n" );
    HAL_DBG_TRACE_INFO( "###### ===== LoRa Basics Modem Tracker application ==== ######\n\n" );
    HAL_DBG_TRACE_PRINTF( "APP VERSION : %d.%d.%d\n\n", TRACKER_MAJOR_APP_VERSION, TRACKER_MINOR_APP_VERSION,
                          TRACKER_SUB_MINOR_APP_VERSION );

    HAL_DBG_TRACE_INFO( "###### ===== LoRa Basics Modem Version ==== ######\n" );
    tracker_app_display_lbm_version( );

    /* Init Tracker context */
    tracker_app_init_context( dev_eui, join_eui, app_key );

    while( 1 )
    {
       TRACE(TAG_TRACKER_RUN_ENGINE);

        /* Execute modem runtime, this function must be called again in sleep_time_ms milliseconds or sooner. */
        sleep_time_ms = smtc_modem_run_engine( );

        if( get_hall_effect_irq_state( ) == true )
        {
    printk("%s %d (from %p) \n", __func__, __LINE__, __builtin_return_address(0) );
            /* Effect hall irq, reset the board*/
            smtc_modem_hal_reset_mcu( );
        }
        else
        {
           /* go in low power */
           TRACE(TAG_TRACKER_SLEEP);
           hal_mcu_set_sleep_for_ms( sleep_time_ms );

           TRACE1(TAG_TRACKER_WAKE,tracker_ctx.airplane_mode  );

           /*
            * Erwin -- changed this test so that we are looking for the accelerometer to
            *          cause a Wifi scan.
            */
           if( tracker_ctx.airplane_mode == true /* false */ )
            {
                /* Wake up from static mode thanks the accelerometer ? */
                if( ( get_accelerometer_irq1_state( ) == true ) &&
                    ( tracker_app_is_tracker_in_static_mode( ) == true ) )
                {
                    /* Start Hall Effect sensors while the tracker moves */
                    smtc_board_hall_effect_enable( true );

                    /* abort and relauch middlewares */
                    HAL_DBG_TRACE_INFO( "WAKE UP ABORT CURRENT RP TASKS\n" );
                    gnss_mw_scan_cancel( );
                    wifi_mw_scan_cancel( );
                }
            }
            else
            {
                /* Wake up thanks the accelerometer and in airplane mode ? */
                if( accel_has_moved( ) == true )
                {

                   /*
                    * Don't do a new scan within 5 seconds of the last one.
                    */
                   curr_time_secs =  smtc_modem_hal_get_time_in_s( );

                   TRACE2(TAG_SEMTRACKER_ACCEL, curr_time_secs, last_wifi_scan_secs);

                   if (curr_time_secs - last_wifi_scan_secs > 10) {

                      last_wifi_scan_secs = curr_time_secs;

                      wifi_scan_start();
                   }

//                    smtc_board_hall_effect_enable_for_duration( TRACKER_AIRPLANE_HALL_EFFECT_TIMEOUT_MS );
//                    HAL_DBG_TRACE_PRINTF( "Start hall effect sensor for %ds\n",
//                                          TRACKER_AIRPLANE_HALL_EFFECT_TIMEOUT_MS / 1000 );
                }
            }
        } // else
    } // while(1)
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- TRACKER GNSS FUNCTION TYPES ---------------------------------------------
 */
static void tracker_gnss_store_new_assistance_position( gnss_mw_event_data_scan_done_t* gnss_scan_results )
{
    float latitude_dif, longitude_dif;

    latitude_dif =
        fabs( gnss_scan_results->context.aiding_position_latitude - tracker_ctx.gnss_assistance_position_latitude );
    longitude_dif =
        fabs( gnss_scan_results->context.aiding_position_longitude - tracker_ctx.gnss_assistance_position_longitude );

    /* Store the new assistance position only if the difference is greater than the conversion error */
    if( ( latitude_dif > ( float ) 0.03 ) || ( longitude_dif > ( float ) 0.03 ) )
    {
        HAL_DBG_TRACE_MSG( "New assistance position stored\n" );

        tracker_ctx.gnss_assistance_position_latitude  = gnss_scan_results->context.aiding_position_latitude;
        tracker_ctx.gnss_assistance_position_longitude = gnss_scan_results->context.aiding_position_longitude;

        tracker_store_app_ctx( );
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- TRACKER LORAWAN FUNCTION TYPES ------------------------------------------
 */

static void tracker_app_init_context( uint8_t* dev_eui, uint8_t* join_eui, uint8_t* app_key )
{
    /* Store or restore the Tracker context */
    if( ( tracker_restore_app_ctx( ) != TRACKER_SUCCESS ) || ( FORCE_NEW_TRACKER_CONTEXT == 1 ) )
    {
        /* When the production keys are used, DevEUI = ChipEUI and JoinEUI is the one defined in
         * lorawan_key_config.h
         */
        if( tracker_ctx.has_lr11xx_trx_firmware )
        {
            smtc_modem_get_chip_eui( stack_id, dev_eui );
        }

        /* Init the tracker global context and store it only if there is a lr11xx firmware */
        tracker_init_app_ctx( dev_eui, join_eui, app_key, tracker_ctx.has_lr11xx_trx_firmware );

        /* Init the tracker internal log context */
        if( tracker_init_internal_log_ctx( ) != TRACKER_SUCCESS )
        {
            HAL_DBG_TRACE_ERROR( "tracker_init_internal_log_ctx failed\n" );
        }
    }
    else
    {
        /* Restore the tracker internal log context */
        if( tracker_restore_internal_log_ctx( ) != TRACKER_SUCCESS )
        {
            /* Init the tracker internal log context */
            if( tracker_init_internal_log_ctx( ) != TRACKER_SUCCESS )
            {
                HAL_DBG_TRACE_ERROR( "tracker_init_internal_log_ctx failed\n" );
            }
        }

        /* Set the restored LoRaWAN Keys */
        memcpy( dev_eui, tracker_ctx.dev_eui, LORAWAN_DEVICE_EUI_LEN );
        memcpy( join_eui, tracker_ctx.join_eui, LORAWAN_JOIN_EUI_LEN );
        memcpy( app_key, tracker_ctx.app_key, LORAWAN_APP_KEY_LEN );
    }

    /* Init tracker context volatile parameters */
    tracker_ctx.accelerometer_move_history = 1;
    tracker_ctx.voltage                    = hal_mcu_get_vref_level( );
    tracker_ctx.system_sanity_check        = ( tracker_system_sanity_check_mask_t ) 0;
    tracker_ctx.gnss_scan_charge_uAh       = 0;
    tracker_ctx.wifi_scan_charge_uAh       = 0;
    tracker_ctx.reset_board_asked          = false;

    smtc_board_select_gnss_antenna( tracker_ctx.gnss_antenna_sel );

    /* Set the maximum authorized transmit output power supported by the board following the region */
    tracker_app_set_max_tx_power_supported( tracker_ctx.lorawan_region, tracker_ctx.lorawan_sub_region );

    /* Set the battery level */
    smtc_board_set_battery_level( tracker_get_battery_level( ) );
}

static void tracker_app_modem_configure_lorawan_params( void )
{
    smtc_modem_return_code_t rc = SMTC_MODEM_RC_OK;

    HAL_DBG_TRACE_INFO( "LoRaWAN parameters:\n" );

    rc = smtc_modem_get_chip_eui( stack_id, tracker_ctx.chip_eui );

    if( rc == SMTC_MODEM_RC_OK )
    {
        HAL_DBG_TRACE_ARRAY( "ChipEIU", tracker_ctx.chip_eui, SMTC_MODEM_EUI_LENGTH );
    }
    else
    {
        HAL_DBG_TRACE_ERROR( "smtc_modem_get_chip_eui failed (%d)\n", rc );
    }

    rc = smtc_modem_set_deveui( stack_id, tracker_ctx.dev_eui );
    if( rc == SMTC_MODEM_RC_OK )
    {
        HAL_DBG_TRACE_ARRAY( "DevEUI", tracker_ctx.dev_eui, SMTC_MODEM_EUI_LENGTH );
    }
    else
    {
        HAL_DBG_TRACE_ERROR( "smtc_modem_get_deveui failed (%d)\n", rc );
    }

    rc = smtc_modem_set_joineui( stack_id, tracker_ctx.join_eui );
    if( rc == SMTC_MODEM_RC_OK )
    {
        HAL_DBG_TRACE_ARRAY( "JoinEUI", tracker_ctx.join_eui, SMTC_MODEM_EUI_LENGTH );
    }
    else
    {
        HAL_DBG_TRACE_ERROR( "smtc_modem_get_joineui failed (%d)\n", rc );
    }

    /* The Derive keys is done thought the smtc_modem_get_pin command */
    rc = smtc_modem_get_pin( stack_id, tracker_ctx.lorawan_pin );
    if( rc == SMTC_MODEM_RC_OK )
    {
        HAL_DBG_TRACE_ARRAY( "PIN", tracker_ctx.lorawan_pin, 4 );
    }
    else
    {
        HAL_DBG_TRACE_ERROR( "smtc_modem_get_pin failed (%d)\n", rc );
    }

    if (tracker_ctx.use_semtech_join_server) {
       printk("%s %d ERROR: should not use semtech server.  Forcing off (from %p) \n", __func__, __LINE__, __builtin_return_address(0) );
       tracker_ctx.use_semtech_join_server = false;
    }

       if( tracker_ctx.use_semtech_join_server == false )
    {
        rc = smtc_modem_set_nwkkey( stack_id, tracker_ctx.app_key );
        if( rc == SMTC_MODEM_RC_OK )
        {
            HAL_DBG_TRACE_ARRAY( "AppKey", tracker_ctx.app_key, SMTC_MODEM_KEY_LENGTH );
        }
        else
        {
            HAL_DBG_TRACE_ERROR( "smtc_modem_set_nwkkey failed (%d)\n", rc );
        }
    }
    else
    {

        HAL_DBG_TRACE_MSG( "AppKey : Use Semtech Join Server\n" );
    }

    ASSERT_SMTC_MODEM_RC( smtc_modem_set_class( stack_id, LORAWAN_CLASS ) );
    modem_class_to_string( LORAWAN_CLASS );

    ASSERT_SMTC_MODEM_RC( smtc_modem_set_region( stack_id, tracker_ctx.lorawan_region ) );
    modem_region_to_string( tracker_ctx.lorawan_region );

    /* Configure modem DM status for regular almanac status update */
    smtc_modem_dm_info_interval_format_t format   = SMTC_MODEM_DM_INFO_INTERVAL_IN_DAY;
    uint8_t                              interval = 1;
    ASSERT_SMTC_MODEM_RC( smtc_modem_dm_set_info_interval( format, interval ) );

    /* Active almanac update OTA - WARNING: will remove all other DM message */
    uint8_t info_field = SMTC_MODEM_DM_FIELD_ALMANAC_STATUS;
    ASSERT_SMTC_MODEM_RC( smtc_modem_dm_set_info_fields( &info_field, 1 ) );

    /* Start time sync (ALC sync), necessary for GNSS scan:
The interval_s indicates how often the LBM will request a time sync from the DAS.
If no time sync downlink has been received from the DAS after the invalid_delay_s is elapsed,
the LBM will report SMTC_MODEM_RC_NO_TIME on smtc_modem_get_time() call. */
    /* -- */
    ASSERT_SMTC_MODEM_RC( smtc_modem_time_set_sync_interval_s( APP_ALC_TIMING_INTERVAL ) );     /* keep call order */
    ASSERT_SMTC_MODEM_RC( smtc_modem_time_set_sync_invalid_delay_s( APP_ALC_TIMING_INVALID ) ); /* keep call order */
    /* Start the service */
//    ASSERT_SMTC_MODEM_RC( smtc_modem_time_start_sync_service( stack_id, SMTC_MODEM_TIME_ALC_SYNC ) );
}

/*
 * -------------------------------------------------------------------------
 * --- TRACKER APP FUNCTION TYPES ------------------------------------------
 */

static void tracker_app_build_and_send_tracker_settings( const uint8_t* buffer, uint8_t len )
{
    uint8_t tx_max_payload = 0;

    ASSERT_SMTC_MODEM_RC( smtc_modem_get_next_tx_max_payload( stack_id, &tx_max_payload ) );
    HAL_DBG_TRACE_PRINTF( "tx_max_payload %d \n", tx_max_payload );

    HAL_DBG_TRACE_PRINTF( " - Tracker settings (%d bytes) : ", len + 2 );

    if( tx_max_payload < ( len + 2 ) )
    {
        HAL_DBG_TRACE_ERROR( "TX max payload < payload len\n" );
    }
    else
    {
        uint8_t payload_len = 0;
        uint8_t lorawan_payload[242];

        /* Add tracker settings value */
        lorawan_payload[payload_len++] = TLV_TRACKER_SETTINGS_TAG;  // Tracker settings TAG
        lorawan_payload[payload_len++] = len;                       // Tracker settings LEN is variable

        memcpy( lorawan_payload + payload_len, buffer, len );
        payload_len += len;

        HAL_DBG_TRACE_MSG( "Send data\n" );
        smtc_modem_request_uplink( stack_id, TRACKER_REQUEST_MSG_PORT, false, lorawan_payload, payload_len );
    }
}

static void tracker_app_read_and_send_sensors( void )
{
    uint8_t  tx_max_payload = 0;
    uint8_t  payload_len    = 0;
    uint8_t  lorawan_payload[242];
    uint32_t charge_mah = 0;

    /* SENSORS DATA */
    HAL_DBG_TRACE_INFO( "*** sensors collect ***\n" );

    /* Move history */
    HAL_DBG_TRACE_PRINTF( "Move history : %d\n", tracker_ctx.accelerometer_move_history );

    /* Temperature */
    tracker_ctx.temperature = smtc_modem_hal_get_temperature( );
    HAL_DBG_TRACE_PRINTF( "Temperature : %d *C\n", tracker_ctx.temperature );

    /* Modem charge */
    smtc_modem_get_charge( &charge_mah );
    HAL_DBG_TRACE_PRINTF( "LBM Charge value : %d mAh\n", charge_mah );
    HAL_DBG_TRACE_PRINTF( "GNSS scan charge value : %d mAh\n", tracker_ctx.gnss_scan_charge_uAh / 1000 );
    HAL_DBG_TRACE_PRINTF( "Wi-Fi scan charge value : %d mAh\n", tracker_ctx.wifi_scan_charge_uAh / 1000 );
    tracker_app_store_new_accumulated_charge( charge_mah + ( tracker_ctx.gnss_scan_charge_uAh / 1000 ) +
                                              ( tracker_ctx.wifi_scan_charge_uAh / 1000 ) );
    HAL_DBG_TRACE_PRINTF( "Accumulated charge value : %d mAh\n", tracker_ctx.accumulated_charge_mAh );

    /* Board voltage charge */
    tracker_ctx.voltage = smtc_modem_hal_get_voltage( ) * 20;
    HAL_DBG_TRACE_PRINTF( "Board voltage : %d mV\n\n", tracker_ctx.voltage );

    ASSERT_SMTC_MODEM_RC( smtc_modem_get_next_tx_max_payload( stack_id, &tx_max_payload ) );
    HAL_DBG_TRACE_PRINTF( "tx_max_payload %d \n", tx_max_payload );

    /* Temperature */
    lorawan_payload[payload_len++] = tracker_ctx.temperature >> 8;
    lorawan_payload[payload_len++] = tracker_ctx.temperature;

    /* Accumulated Charge in mAh */
    lorawan_payload[payload_len++] = tracker_ctx.accumulated_charge_mAh >> 8;
    lorawan_payload[payload_len++] = tracker_ctx.accumulated_charge_mAh;

    /* Board Voltage */
    lorawan_payload[payload_len++] = tracker_ctx.voltage >> 8;
    lorawan_payload[payload_len++] = tracker_ctx.voltage;

    if( tx_max_payload < payload_len )
    {
        HAL_DBG_TRACE_ERROR( "TX max payload < payload len\n" );
    }
    else
    {
        HAL_DBG_TRACE_MSG( "Use classic uplink to send data\n" );
        smtc_modem_request_uplink( stack_id, TRACKER_APP_SENSOR_PORT, false, lorawan_payload, payload_len );
    }
}

static void tracker_app_store_new_accumulated_charge( uint32_t charge_mAh )
{
    static uint32_t previous_charge_mAh = 0;  // Previous modem charge before read from LBM plus scans, keep
                                              // the historic even after leave the function because of the static

    /* Store the new accumulated charge only if the modem charge has changed */
    if( charge_mAh != previous_charge_mAh )
    {
        tracker_ctx.accumulated_charge_mAh += charge_mAh - previous_charge_mAh;
        HAL_DBG_TRACE_MSG( "New accumulated charge stored\n" );
        tracker_store_app_ctx( );

        previous_charge_mAh = charge_mAh;

        /* Set the new battery level */
        smtc_board_set_battery_level( tracker_get_battery_level( ) );
    }
}

static bool tracker_app_is_tracker_in_static_mode( void )
{
    if( ( ( tracker_ctx.accelerometer_move_history & TRACKER_SEND_TWO_MORE_SCANS_ONCE_STATIC ) == 0 ) &&
        ( tracker_ctx.accelerometer_used == 1 ) )
    {
        return true;
    }
    else
    {
        return false;
    }
}

static void tracker_app_parse_downlink_frame( uint8_t port, const uint8_t* payload, uint8_t size )
{
    /* Forward downlink to GNSS middleware to handle it if necessary */
    gnss_mw_handle_downlink( port, payload, size );

    switch( port )
    {
    case TRACKER_REQUEST_MSG_PORT:
    {
        uint8_t tag           = 0;
        uint8_t len           = 0;
        uint8_t payload_index = 0;

        while( payload_index < size )
        {
            tag = payload[payload_index++];
            len = payload[payload_index++];

            switch( tag )
            {
            case GET_APP_TRACKER_SETTINGS_CMD:
            {
                uint8_t settings_buffer[240];
                uint8_t tracker_settings_payload_max_len = 0;
                memcpy( settings_buffer, payload + payload_index, len );

                ASSERT_SMTC_MODEM_RC(
                    smtc_modem_get_next_tx_max_payload( stack_id, &tracker_settings_payload_max_len ) );

                HAL_DBG_TRACE_INFO( "###### ===== TRACKER CONFIGURATION SETTINGS PAYLOAD RECEIVED ==== ######\n\n" );

                tracker_ctx.tracker_settings_payload_len =
                    tracker_parse_cmd( stack_id, settings_buffer, tracker_ctx.tracker_settings_payload,
                                       tracker_settings_payload_max_len, false );

                /* Store the new values here if it's asked */
                if( ( tracker_ctx.new_value_to_set ) == true )
                {
                    tracker_ctx.new_value_to_set = false;
                    tracker_store_app_ctx( );
                }

                if( ( tracker_ctx.lorawan_parameters_have_changed == true ) ||
                    ( tracker_ctx.reset_board_asked == true ) )
                {
                    /* reset device because of LoRaWAN Parameters */
                    HAL_DBG_TRACE_INFO( "###### ===== RESET TRACKER ==== ######\n\n" );
                    smtc_modem_hal_reset_mcu( );
                }

                tracker_app_build_and_send_tracker_settings( tracker_ctx.tracker_settings_payload,
                                                             tracker_ctx.tracker_settings_payload_len );

                break;
            }
            case GET_MODEM_DATE_CMD:
            {
                uint8_t        buffer[6];
                const uint32_t modem_date = apps_modem_common_get_utc_time( );

                buffer[0] = GET_MODEM_DATE_CMD;
                buffer[1] = GET_MODEM_DATE_ANSWER_LEN;
                buffer[2] = modem_date >> 24;
                buffer[3] = modem_date >> 16;
                buffer[4] = modem_date >> 8;
                buffer[5] = modem_date;

                /* Use the emergency TX to reduce the latency */
                smtc_modem_request_emergency_uplink( stack_id, port, false, buffer, 6 );

                HAL_DBG_TRACE_INFO( "###### ===== SEND LBM DATE IN EMERGENCY TX ==== ######\n\n" );

                break;
            }
            default:
                payload_index += len;
                break;
            }
        }
    }
    default:
        break;
    }
}

/*-------------------------------------------------------------------------
 *
 * name:         tracker_app_lr11xx_check_firmware_version
 *
 * description:
 *
 * input:
 *
 * output:
 *
 *-------------------------------------------------------------------------*/
static bool tracker_app_lr11xx_check_firmware_version( const void* context )
{
    lr11xx_status_t status;
    bool            has_expected_fw_version = false;
    uint8_t         lr11xx_busy_pin_state   = 0;

    /*
     * Original code would reset the radio here.  This caused issues that the join request would not get
     * transmitted, so removed it.
     */

    /* Check if the LR1110 is in transceiver mode or Modem-E mode, busy pin in low for transceiver mode, high for
     * Modem-E */
    lr11xx_busy_pin_state = smtc_board_read_busy_pin( context );

    if( lr11xx_busy_pin_state == 0 )
    {
        /* LR1110 is in transceiver mode */
        status = lr11xx_system_get_version( tracker_modem_radio->ral.context, &tracker_ctx.lr11xx_fw_version );
        HAL_DBG_TRACE_PRINTF( "type 0x%04X \n", tracker_ctx.lr11xx_fw_version.type );

        if( ( status == LR11XX_STATUS_OK ) && ( tracker_ctx.lr11xx_fw_version.type == 0x0001 ) )
        {
            tracker_ctx.has_lr11xx_trx_firmware = true;
            if( tracker_ctx.lr11xx_fw_version.fw < LR11XX_FW_VERSION )
            {
#if 1
               /*
                * Update the sensor firmware with code that is built into this image.
                */
               firmware_update(tracker_modem_radio->ral.context);
#else
                /* LR11XX firmware version is not the expected one, sstay in BLE and
                 * update the LR11XX firmware */
                HAL_DBG_TRACE_ERROR( "Wrong LR11XX_FW_VERSION, current version is 0x%04X, expected is 0x%04X\n",
                                     tracker_ctx.lr11xx_fw_version.fw, LR11XX_FW_VERSION );
#endif
            }
            else
            {
                HAL_DBG_TRACE_PRINTF( "LR11XX FW : 0x%04X\n", tracker_ctx.lr11xx_fw_version.fw );
                has_expected_fw_version = true;
            }
        }
        else
        {
            /* LR1110 is in Modem-E mode */
            tracker_ctx.lr11xx_fw_version.fw =
                0; /* Set the lr11xx_fw_version to 0 to be able to update it thought BLE */
            tracker_ctx.has_lr11xx_trx_firmware = false;
            has_expected_fw_version             = false;
        }
    }
    else
    {
        tracker_ctx.lr11xx_fw_version.fw = 0; /* Set the lr11xx_fw_version to 0 to be able to update it thought BLE */
        tracker_ctx.has_lr11xx_trx_firmware = false;
        has_expected_fw_version             = false;
    }

    return has_expected_fw_version;
}

static void tracker_app_display_lbm_version( void )
{
    smtc_modem_return_code_t modem_response_code = SMTC_MODEM_RC_OK;

    modem_response_code = smtc_modem_get_lorawan_version( &tracker_ctx.lorawan_version );
    if( modem_response_code == SMTC_MODEM_RC_OK )
    {
        HAL_DBG_TRACE_INFO( "LoRaWAN version: %.2x.%.2x.%.2x.%.2x\n", tracker_ctx.lorawan_version.major,
                            tracker_ctx.lorawan_version.minor, tracker_ctx.lorawan_version.patch,
                            tracker_ctx.lorawan_version.revision );
    }

    modem_response_code = smtc_modem_get_modem_version( &tracker_ctx.firmware_version );
    if( modem_response_code == SMTC_MODEM_RC_OK )
    {
        HAL_DBG_TRACE_INFO( "LoRa Basics Modem version: %.2x.%.2x.%.2x\n", tracker_ctx.firmware_version.major,
                            tracker_ctx.firmware_version.minor, tracker_ctx.firmware_version.patch );
    }
}

static void tracker_app_set_adr( void )
{
    /* SF9 = DR3 for EU868 / IN865 / RU864 / AU915 / CN470 /AS923 / KR920 */
    uint8_t adr_custom_list_freq_SF9_DR3[16] = { 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03,
                                                 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03 }; /* 125kHz - SF9 */
    /* SF9 = DR1 for US915 */
    uint8_t adr_custom_list_SF9_DR1[16] = { 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
                                            0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01 }; /* 125kHz - SF9 */
    uint8_t custom_nb_trans             = 3;

    /* Set the ADR profile once joined */
    switch( tracker_ctx.lorawan_region )
    {
    case SMTC_MODEM_REGION_EU_868:
    case SMTC_MODEM_REGION_IN_865:
    case SMTC_MODEM_REGION_RU_864:
    case SMTC_MODEM_REGION_AU_915:
    case SMTC_MODEM_REGION_AS_923_GRP1:
    case SMTC_MODEM_REGION_AS_923_GRP2:
    case SMTC_MODEM_REGION_AS_923_GRP3:
    case SMTC_MODEM_REGION_CN_470:
    case SMTC_MODEM_REGION_CN_470_RP_1_0:
    case SMTC_MODEM_REGION_KR_920:
        ASSERT_SMTC_MODEM_RC(
            smtc_modem_adr_set_profile( stack_id, SMTC_MODEM_ADR_PROFILE_CUSTOM, adr_custom_list_freq_SF9_DR3 ) );
        ASSERT_SMTC_MODEM_RC( smtc_modem_set_nb_trans( stack_id, custom_nb_trans ) );
        break;
    case SMTC_MODEM_REGION_US_915:
        ASSERT_SMTC_MODEM_RC(
            smtc_modem_adr_set_profile( stack_id, SMTC_MODEM_ADR_PROFILE_CUSTOM, adr_custom_list_SF9_DR1 ) );
        ASSERT_SMTC_MODEM_RC( smtc_modem_set_nb_trans( stack_id, custom_nb_trans ) );
        break;
    default:
        HAL_DBG_TRACE_ERROR( "Region not supported in this example, could not set custom ADR profile\n" );
        break;
    }
}

static void tracker_app_set_max_tx_power_supported( smtc_modem_region_t region, smtc_modem_sub_region_t sub_region )
{
    int8_t max_tx_power_supported = 22;

    switch( region )
    {
    case SMTC_MODEM_REGION_IN_865:
    case SMTC_MODEM_REGION_AU_915:
        max_tx_power_supported = 14;
        break;
    case SMTC_MODEM_REGION_AS_923_GRP1:
        if( sub_region == SMTC_MODEM_SUB_REGION_JAPAN )
        {
            max_tx_power_supported = 9;
        }
        break;
    case SMTC_MODEM_REGION_KR_920:
        max_tx_power_supported = 8;
        break;
    default:
        break;
    }

    smtc_board_set_max_tx_power_supported( max_tx_power_supported );
}

/*
 * -----------------------------------------------------------------------------
 * --- TRACKER LORA BASICS MODEM EVENT FUNCTION TYPES --------------------------
 */

/*!
 * @brief LoRa Basics Modem event callbacks called by smtc_event_process function
 */

static void on_modem_reset( uint16_t reset_count )
{
    HAL_DBG_TRACE_INFO( "###### ===== LoRa Basics Modem reset %u ==== ######\n\n", reset_count );

    if( smtc_board_is_ready( ) == true )
    {
    printk("%s %d READY -- do reset (from %p) \n", __func__, __LINE__, __builtin_return_address(0) );
        /* System reset */
        smtc_modem_hal_reset_mcu( );
    }
    else
    {
        int32_t  voltage_drop          = 0;
        uint32_t voltage_recovery_time = 0;

        smtc_board_set_ready( true );

        /* Initialize LoRaWAN parameters */
        tracker_app_modem_configure_lorawan_params( );

        /* Start the BLE thread*/
// PHIL        start_ble_thread( TRACKER_ADV_TIMEOUT_MS );

       /* Check if the batteries are too low, if yes switch the tracker in airplane mode, \note if this test is
         * moved elsewhere in this app the TRACKER_BOARD_MAX_VOLTAGE_RECOVERY_TIME value should be evaluate again */
        smtc_board_measure_battery_drop( stack_id, tracker_ctx.lorawan_region, &voltage_drop, &voltage_recovery_time );

        if( ( voltage_recovery_time > TRACKER_BOARD_MAX_VOLTAGE_RECOVERY_TIME ) &&
            ( tracker_ctx.accumulated_charge_mAh > ( TRACKER_BOARD_BATTERY_CAPACITY * 0.8 ) ) )
        {
    printk("%s %d (from %p) \n", __func__, __LINE__, __builtin_return_address(0) );
            HAL_DBG_TRACE_ERROR( "###### ===== BATTERIES LOW, STAY IN AIRPLANE MODE ==== ######\n\n" );
            tracker_ctx.airplane_mode = true;
            smtc_board_leds_blink( smtc_board_get_led_all_mask( ), 500, 5 );
        }

        if( tracker_ctx.airplane_mode == false ) {
           /* Start the Join process */
           ASSERT_SMTC_MODEM_RC( smtc_modem_join_network( stack_id ) );

           HAL_DBG_TRACE_INFO( "###### ===== JOINING ==== ######\n\n" );

           /* Notify user with leds for join process */
           smtc_board_start_periodic_led_pulse( smtc_board_get_led_rx_mask( ) | smtc_board_get_led_tx_mask( ),
                                                LED_JOIN_PULSE_MS, LED_JOIN_PERIOD_MS );
        }
        else
        {
            HAL_DBG_TRACE_MSG( "TRACKER IN AIRPLANE MODE\n\n" );

            /* Stop Hall Effect sensors while the tracker is static in airplane mode */
            smtc_board_hall_effect_enable( false );

            /* Reset accelerometer IRQ if any */
            is_accelerometer_detected_moved( );
        }
    }
}

extern void wifi_init (void);

static void on_modem_network_joined( void )
{
    smtc_modem_return_code_t
       rc;

    /*
     * Request a time sync every hour.
     */
    smtc_modem_time_set_sync_interval_s(60 * 60);

    /*
     * Start a clock sync service
     */
    rc = smtc_modem_time_start_sync_service(stack_id, SMTC_MODEM_TIME_ALC_SYNC);

    if (rc != SMTC_MODEM_RC_OK) {
       printk("%s: Error while attempting to start clock services.  rc: %d  (from %p) \n", __func__, rc, __builtin_return_address(0) );
    }

    /* Stop network research notification */
    smtc_board_stop_periodic_led_pulse( );

    /* Disable auto switch to network controlled after a certain amount of TX without RX */
    ASSERT_SMTC_MODEM_RC( smtc_modem_connection_timeout_set_thresholds( stack_id, 0, 0 ) );

    /* Initialize GNSS middleware */
#if (ENABLE_GNSS > 0)
    {
       mw_version_t mw_version;

       gnss_mw_get_version( &mw_version );
       HAL_DBG_TRACE_INFO( "Initializing GNSS middleware v%d.%d.%d\n", mw_version.major, mw_version.minor,
                           mw_version.patch );
       gnss_mw_init( tracker_modem_radio, stack_id );
       gnss_mw_set_constellations( GNSS_MW_CONSTELLATION_GPS_BEIDOU );
       gnss_mw_set_user_aiding_position( tracker_ctx.gnss_assistance_position_latitude,
                                         tracker_ctx.gnss_assistance_position_longitude );
    }
#endif

    wifi_init();
}

static void on_modem_down_data( int8_t rssi, int8_t snr, smtc_modem_event_downdata_window_t rx_window, uint8_t port,
                                const uint8_t* payload, uint8_t size )
{
    HAL_DBG_TRACE_INFO( "Downlink received:\n" );
    HAL_DBG_TRACE_INFO( "  - LoRaWAN Fport = %d\n", port );
    HAL_DBG_TRACE_INFO( "  - Payload size  = %d\n", size );
    HAL_DBG_TRACE_INFO( "  - RSSI          = %d dBm\n", rssi - 64 );
    HAL_DBG_TRACE_INFO( "  - SNR           = %d dB\n", snr >> 2 );

    switch( rx_window )
    {
    case SMTC_MODEM_EVENT_DOWNDATA_WINDOW_RX1:
    {
        HAL_DBG_TRACE_INFO( "  - Rx window     = %s\n", xstr( SMTC_MODEM_EVENT_DOWNDATA_WINDOW_RX1 ) );
        break;
    }
    case SMTC_MODEM_EVENT_DOWNDATA_WINDOW_RX2:
    {
        HAL_DBG_TRACE_INFO( "  - Rx window     = %s\n", xstr( SMTC_MODEM_EVENT_DOWNDATA_WINDOW_RX2 ) );
        break;
    }
    case SMTC_MODEM_EVENT_DOWNDATA_WINDOW_RXC:
    {
        HAL_DBG_TRACE_INFO( "  - Rx window     = %s\n", xstr( SMTC_MODEM_EVENT_DOWNDATA_WINDOW_RXC ) );
        break;
    }
    default:
        break;
    }

    if( size != 0 )
    {
        HAL_DBG_TRACE_ARRAY( "Payload", payload, size );
        tracker_app_parse_downlink_frame( port, payload, size );
    }

    smtc_board_led_pulse( smtc_board_get_led_rx_mask( ), true, LED_PERIOD_MS );

    /* Update the system_sanity_check bit field */
    tracker_ctx.system_sanity_check |= TRACKER_DOWNLINK_SUCCESSFUL_ONCE;
}

static void on_modem_clk_synch( smtc_modem_event_time_status_t time_status )
{
    /* Set the ADR according to the region */
    tracker_app_set_adr( );

    /* Update the system_sanity_check bit field */
    tracker_ctx.system_sanity_check |= TRACKER_DOWNLINK_SUCCESSFUL_ONCE;

#if (ENABLE_GNSS > 0)
    if( time_status != SMTC_MODEM_EVENT_TIME_NOT_VALID )
    {
        /* Start now the geoloc scan sequence */
        gnss_mw_scan_aggregate( false );
        gnss_mw_scan_start( GNSS_MW_MODE_MOBILE, 0 );
    }
#endif
}

static void on_modem_almanac_update( smtc_modem_event_almanac_update_status_t status )
{
    if( status == SMTC_MODEM_EVENT_ALMANAC_UPDATE_COMPLETED )
    {
        HAL_DBG_TRACE_INFO( "Almanac update is completed\n\n" );
    }
    else
    {
        HAL_DBG_TRACE_INFO( "Almanac update is not completed yet\n" );
    }
}

static void on_middleware_gnss_event( uint8_t pending_events )
{
    printk("%s %d GGGGGGGGGGGGGGGGG (from %p) \n", __func__, __LINE__, __builtin_return_address(0) );

    /* Parse events */
    if( gnss_mw_has_event( pending_events, GNSS_MW_EVENT_SCAN_DONE ) )
    {
        gnss_mw_event_data_scan_done_t gnss_scan_results;

        HAL_DBG_TRACE_INFO( "GNSS middleware event - SCAN DONE\n" );
        gnss_mw_get_event_data_scan_done( &gnss_scan_results );
        gnss_mw_display_results( &gnss_scan_results );

        /* Convert GPS timestamp to UTC timestamp */
        for( int i = 0; i < gnss_scan_results.nb_scans_valid; i++ )
        {
            gnss_scan_results.scans[i].timestamp =
                apps_modem_common_convert_gps_to_utc_time( gnss_scan_results.scans[i].timestamp );
        }

        /* Store the consumption */
        tracker_ctx.gnss_scan_charge_uAh += gnss_scan_results.power_consumption_uah;

        /* timestamp the beginning ot the sequence */
        if( gnss_scan_results.nb_scans_valid > 0 )
        {
            tracker_ctx.start_sequence_timestamp = gnss_scan_results.scans[0].timestamp;
        }
        else
        {
            tracker_ctx.start_sequence_timestamp = apps_modem_common_get_utc_time( );
        }

        /* Check if a new assistance position is available */
        tracker_gnss_store_new_assistance_position( &gnss_scan_results );

        /* Log results in internal memory */
        if( tracker_ctx.internal_log_enable )
        {
            tracker_store_gnss_in_internal_log( &gnss_scan_results );
        }

        if( gnss_scan_results.context.almanac_update_required )
        {
            uint8_t dm_almanac_status = SMTC_MODEM_DM_FIELD_ALMANAC_STATUS;

            HAL_DBG_TRACE_MSG( "Almanac update required, require an almanac update\n" );
            ASSERT_SMTC_MODEM_RC( smtc_modem_dm_request_single_uplink( &dm_almanac_status, 1 ) );
        }
    }

    if( gnss_mw_has_event( pending_events, GNSS_MW_EVENT_TERMINATED ) )
    {
        int32_t duty_cycle_status_ms = 0;

        HAL_DBG_TRACE_INFO( "GNSS middleware event - TERMINATED\n" );
        gnss_mw_get_event_data_terminated( &tracker_ctx.gnss_mw_event_data );
        HAL_DBG_TRACE_PRINTF( "-- number of scans sent: %u\n", tracker_ctx.gnss_mw_event_data.nb_scans_sent );
        HAL_DBG_TRACE_PRINTF( "-- aiding position check sent: %d\n",
                              tracker_ctx.gnss_mw_event_data.aiding_position_check_sent );

        ASSERT_SMTC_MODEM_RC( smtc_modem_get_duty_cycle_status( &duty_cycle_status_ms ) );
        HAL_DBG_TRACE_PRINTF( "Remaining duty cycle %d ms\n", duty_cycle_status_ms );

        /* Led start for user notification */
        smtc_board_led_pulse( smtc_board_get_led_tx_mask( ), true, LED_PERIOD_MS );
    }

    if( gnss_mw_has_event( pending_events, GNSS_MW_EVENT_SCAN_CANCELLED ) )
    {
        HAL_DBG_TRACE_INFO( "GNSS middleware event - SCAN CANCELLED\n" );

        /* Start now the GNSS scan group sequence */
        gnss_mw_scan_aggregate( false );
        gnss_mw_scan_start( GNSS_MW_MODE_MOBILE, 0 );
    }

    if( gnss_mw_has_event( pending_events, GNSS_MW_EVENT_ERROR_NO_TIME ) )
    {
        HAL_DBG_TRACE_INFO( "GNSS middleware event - ERROR NO TIME\n" );

        HAL_DBG_TRACE_MSG( "Trig a new sync request\n" );
        ASSERT_SMTC_MODEM_RC( smtc_modem_time_trigger_sync_request( stack_id ) );

        /* Force the nb_scan_sent value to 0 to launch Wi-Fi scan */
        tracker_ctx.gnss_mw_event_data.nb_scans_sent = 0;
    }

    if( gnss_mw_has_event( pending_events, GNSS_MW_EVENT_ERROR_ALMANAC_UPDATE ) )
    {
        uint8_t dm_almanac_status = SMTC_MODEM_DM_FIELD_ALMANAC_STATUS;

        HAL_DBG_TRACE_ERROR( "GNSS middleware event - ALMANAC UPDATE REQUIRED\n" );
        ASSERT_SMTC_MODEM_RC( smtc_modem_dm_request_single_uplink( &dm_almanac_status, 1 ) );
    }

    if( gnss_mw_has_event( pending_events, GNSS_MW_EVENT_ERROR_NO_AIDING_POSITION ) )
    {
        HAL_DBG_TRACE_ERROR( "GNSS middleware event - ERROR NO AIDING POSITION set\n" );

        smtc_modem_hal_reset_mcu( );
    }

    if( gnss_mw_has_event( pending_events, GNSS_MW_EVENT_ERROR_UNKNOWN ) )
    {
        HAL_DBG_TRACE_WARNING( "GNSS middleware event - GNSS MW EVENT ERROR UNKNOWN\n" );
    }

    if( gnss_mw_has_event( pending_events, GNSS_MW_EVENT_TERMINATED ) ||
        gnss_mw_has_event( pending_events, GNSS_MW_EVENT_ERROR_NO_TIME ) ||
        gnss_mw_has_event( pending_events, GNSS_MW_EVENT_ERROR_ALMANAC_UPDATE ) ||
        gnss_mw_has_event( pending_events, GNSS_MW_EVENT_ERROR_NO_AIDING_POSITION ) ||
        gnss_mw_has_event( pending_events, GNSS_MW_EVENT_ERROR_UNKNOWN ) )
    {
        /* Has tracker moved ? */
        tracker_ctx.accelerometer_move_history =
            ( tracker_ctx.accelerometer_move_history << 1 ) + is_accelerometer_detected_moved( );

        if( ( tracker_ctx.gnss_mw_event_data.nb_scans_sent == 0 ) ||
            ( tracker_ctx.scan_priority == TRACKER_NO_PRIORITY ) )
        {
            HAL_DBG_TRACE_MSG( "Start Wi-Fi scan\n" );
            wifi_mw_scan_start( 0 );
        }
        else
        {
            uint32_t sequence_duration_sec = apps_modem_common_get_utc_time( ) - tracker_ctx.start_sequence_timestamp;

            if( tracker_app_is_tracker_in_static_mode( ) == true )
            {
                if( sequence_duration_sec > tracker_ctx.static_scan_interval )
                {
                    sequence_duration_sec = tracker_ctx.static_scan_interval;
                }

                /* Stop Hall Effect sensors while the tracker is static */
                smtc_board_hall_effect_enable( false );

                HAL_DBG_TRACE_MSG( "Switch instatic mode\n" );
                gnss_mw_scan_aggregate( true );
                gnss_mw_scan_start( GNSS_MW_MODE_STATIC, tracker_ctx.static_scan_interval - sequence_duration_sec );

                /* Send sensors values in static mode */
                tracker_app_read_and_send_sensors( );
            }
            else
            {
                if( sequence_duration_sec > tracker_ctx.mobile_scan_interval )
                {
                    sequence_duration_sec = tracker_ctx.mobile_scan_interval;
                }

                HAL_DBG_TRACE_MSG( "Continue in mobile mode\n" );
                gnss_mw_scan_aggregate( false );
                gnss_mw_scan_start( GNSS_MW_MODE_MOBILE, tracker_ctx.mobile_scan_interval - sequence_duration_sec );
            }
        }
    }

    gnss_mw_clear_pending_events( );
}

/*!
 * @}
 */

/* --- EOF ------------------------------------------------------------------ */

///	Start LoRa thread.
void semtracker_thread_start(void)
{
   k_tid_t tid;

   tid = k_thread_create(&semtracker_kthread, semtracker_stack_area,
                         K_THREAD_STACK_SIZEOF(semtracker_stack_area),
                         semtracker_application,
                         NULL, NULL,	NULL,
                         SEMTRACKER_PRIORITY, 0, K_NO_WAIT);

   printk("%s %d KID: %p   thead_data: %p (from %p) \n", __func__, __LINE__, tid, &semtracker_kthread, __builtin_return_address(0) );

   k_thread_name_set(tid, "semtracker");

}

/*-------------------------------------------------------------------------
 *
 * name:        semtracker_thread_wakeup
 *
 * description:
 *
 * input:
 *
 * output:
 *
 *-------------------------------------------------------------------------*/
void semtracker_thread_wakeup (void)
{
   k_wakeup(&semtracker_kthread);
}
