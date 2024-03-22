#include "smtc_modem_hal.h"
#include "smtc_hal_flash.h"
#include "smtc_hal_gpio.h"
#include "smtc_hal_lp_timer.h"
#include "smtc_hal_rng.h"
#include "smtc_hal_rtc.h"
#include "lr1mac_defs.h"
#include "lr11xx_drv.h"
#include "settings.h"

#define ADDR_FLASH_LORAWAN_CONTEXT        ADDR_FLASH_PAGE(254)
#define ADDR_FLASH_MODEM_CONTEXT          ADDR_FLASH_PAGE(255)
#define ADDR_FLASH_DEVNONCE_CONTEXT       ADDR_FLASH_PAGE(253)
#define ADDR_FLASH_SECURE_ELEMENT_CONTEXT ADDR_FLASH_PAGE(252)

uint8_t __attribute__( ( section( ".noinit" ) ) ) saved_crashlog[CRASH_LOG_SIZE];
volatile bool __attribute__( ( section( ".noinit" ) ) ) crashlog_available;

/* ------------ Time management ------------*/

uint32_t smtc_modem_hal_get_time_in_s( void )
{
    return hal_rtc_get_time_s( );
}

uint32_t smtc_modem_hal_get_compensated_time_in_s( void )
{
    return hal_rtc_get_time_s( );
}

int32_t smtc_modem_hal_get_time_compensation_in_s( void )
{
    return 0;
}

uint32_t smtc_modem_hal_get_time_in_ms( void )
{
    return hal_rtc_get_time_ms( );
}

uint32_t smtc_modem_hal_get_time_in_100us( void )
{
    return hal_rtc_get_time_100us( );
}

uint32_t smtc_modem_hal_get_radio_irq_timestamp_in_100us( void )
{
    // in lbm current implementation the call of this function is done in radio_planner radio irq handler
    // so the current time is the irq time
    return hal_rtc_get_time_100us( );
}

/* ------------ Timer management ------------*/

void smtc_modem_hal_start_timer( const uint32_t milliseconds, void ( *callback )( void* context ), void* context )
{
    hal_lp_timer_start( milliseconds, &( hal_lp_timer_irq_t ){ .context = context, .callback = callback } );
}

void smtc_modem_hal_stop_timer( void )
{
    hal_lp_timer_stop( );
}

/* ------------ IRQ management ------------*/

void smtc_modem_hal_disable_modem_irq( void )
{
   lr11xx_drv_radio_irq_disable( );
    hal_lp_timer_irq_disable( );
}

void smtc_modem_hal_enable_modem_irq( void )
{
   lr11xx_drv_radio_irq_enable();
   hal_lp_timer_irq_enable( );
}

/* ------------ Context saving management ------------*/
void smtc_modem_hal_context_restore( const modem_context_type_t ctx_type, uint32_t offset, uint8_t* buffer, const uint32_t size )
{
   switch( ctx_type ) {
       case CONTEXT_LORAWAN_STACK:
	  settings_read(SETTINGS_ID_LORAWAN_STACK, buffer, size);
	  break;
       case CONTEXT_MODEM:
	  settings_read(SETTINGS_ID_MODEM, buffer, size);
	  break;

       default:
	  printk("%s: Invalid context: %d size: %d \n", __func__, ctx_type, size );
	  break;
   }

#if 0
    case CONTEXT_MODEM:
	hal_flash_read_buffer( ADDR_FLASH_MODEM_CONTEXT, buffer, size );
	break;
    case CONTEXT_LR1MAC:
	hal_flash_read_buffer( ADDR_FLASH_LORAWAN_CONTEXT, buffer, size );
	break;
    case CONTEXT_DEVNONCE:

	hal_flash_read_buffer( ADDR_FLASH_DEVNONCE_CONTEXT, buffer, size );
	break;
    case CONTEXT_SECURE_ELEMENT:
	hal_flash_read_buffer( ADDR_FLASH_SECURE_ELEMENT_CONTEXT, buffer, size );
	break;
#endif
}

void smtc_modem_hal_context_store( const modem_context_type_t ctx_type, uint32_t offset, const uint8_t* buffer, const uint32_t size )
{
   switch( ctx_type ) {
       case CONTEXT_LORAWAN_STACK:
	  settings_write(SETTINGS_ID_LORAWAN_STACK, buffer, size);
	  break;
       case CONTEXT_MODEM:
	  settings_write(SETTINGS_ID_MODEM, buffer, size);
	  break;

       default:
	  printk("%s: Invalid context: %d size: %d \n", __func__, ctx_type, size );
	  break;
   }
#if 0
   switch( ctx_type )
   {
       case CONTEXT_MODEM:
	  hal_flash_erase_page( ADDR_FLASH_MODEM_CONTEXT, 1 );
	  hal_flash_write_buffer( ADDR_FLASH_MODEM_CONTEXT, buffer, size );
	  break;
       case CONTEXT_LR1MAC:
	  hal_flash_erase_page( ADDR_FLASH_LORAWAN_CONTEXT, 1 );
	  hal_flash_write_buffer( ADDR_FLASH_LORAWAN_CONTEXT, buffer, size );
	  break;
       case CONTEXT_DEVNONCE:
	  hal_flash_erase_page( ADDR_FLASH_DEVNONCE_CONTEXT, 1 );
	  hal_flash_write_buffer( ADDR_FLASH_DEVNONCE_CONTEXT, buffer, size );
	  break;
       case CONTEXT_SECURE_ELEMENT:
	  hal_flash_erase_page( ADDR_FLASH_SECURE_ELEMENT_CONTEXT, 1 );
	  hal_flash_write_buffer( ADDR_FLASH_SECURE_ELEMENT_CONTEXT, buffer, size );
	  break;
       default:
	  mcu_panic( );
	  break;
   }
#endif
}




/* ------------ Crashlog management ------------*/
void smtc_modem_hal_crashlog_store( const uint8_t* crash_string, uint8_t crash_string_length )
{
//    memcpy( &saved_crashlog, crashlog, CRASH_LOG_SIZE );
   strncpy(saved_crashlog, crash_string, CRASH_LOG_SIZE);
}

void smtc_modem_hal_crashlog_restore( uint8_t* crash_string, uint8_t* crash_string_length )
{
   uint32_t
      len = strlen(saved_crashlog);

   memcpy( crash_string, &saved_crashlog, len);

   *crash_string_length = (uint8_t) len;

}

void smtc_modem_hal_set_crashlog_status( bool available )
{
    crashlog_available = available;
}

bool smtc_modem_hal_get_crashlog_status( void )
{
    return crashlog_available;
}



/* ------------ assert management ------------*/
void smtc_modem_hal_on_panic( uint8_t* func, uint32_t line, const char* fmt, ... )

{
    uint8_t out_buff[255] = { 0 };
    uint8_t out_len       = snprintf( ( char* ) out_buff, sizeof( out_buff ), "%s:%u ", func, line );

    va_list args;
    va_start( args, fmt );
    out_len += sprintf( ( char* ) &out_buff[out_len], fmt, args );
    va_end( args );

    smtc_modem_hal_crashlog_store( out_buff, out_len );

    smtc_modem_hal_print_trace(
	"\x1B[0;31m"  // red color
	"Modem panic :%s:%u\n"
	"\x1B[0m",  // revert default color
	func, line );
    smtc_modem_hal_reset_mcu( );
}

void smtc_modem_hal_assert_fail( uint8_t* func, uint32_t line )
{
   smtc_modem_hal_crashlog_store( ( uint8_t* ) func, strlen(func) );
    smtc_modem_hal_set_crashlog_status( true );
    smtc_modem_hal_print_trace(
	"\x1B[0;31m"  // red color
	"crash log :%s:%u\n"
	"\x1B[0m",  // revert default color
	func, line );
    smtc_modem_hal_reset_mcu( );
}

uint32_t smtc_modem_hal_get_random_nb_in_range( const uint32_t val_1, const uint32_t val_2 )
{
    return hal_rng_get_random_in_range( val_1, val_2 );
}

int32_t smtc_modem_hal_get_signed_random_nb_in_range( const int32_t val_1, const int32_t val_2 )
{
    return hal_rng_get_signed_random_in_range( val_1, val_2 );
}




/* ------------ Radio env management ------------*/
void smtc_modem_hal_irq_config_radio_irq( void ( *callback )( void* context ), void* context )
{
   lr11xx_drv_radio_irq_config(callback, context);
}

void smtc_modem_hal_start_radio_tcxo( void )
{
    // put here the code that will start the tcxo if needed
}

void smtc_modem_hal_stop_radio_tcxo( void )
{
    // put here the code that will stop the tcxo if needed
}

uint32_t smtc_modem_hal_get_radio_tcxo_startup_delay_ms( void )
{
#if defined( LR11XX )
    return 5;
#else
#error need to define LR11XX
    return 0;
#endif
}


/*-------------------------------------------------------------------------
 *
 * name:         smtc_modem_hal_set_ant_switch
 *
 * description:  No antenna switch needed.  Just a stub.
 *
 * input:
 *
 * output:
 *
 *-------------------------------------------------------------------------*/
void smtc_modem_hal_set_ant_switch( bool is_tx_on ) { ; }





/* ------------ Environment management ------------*/

uint8_t smtc_modem_hal_get_battery_level( void )
{
    return 254;
}

int8_t smtc_modem_hal_get_temperature( void )
{
    int8_t temperature = 0;
    printk("%s %d (from %p) \n", __func__, __LINE__, __builtin_return_address(0) );

#if PHIL
    hal_adc_init( );
    temperature = hal_adc_get_temp( );
    hal_adc_deinit( );
#endif
    return temperature;
}

uint8_t smtc_modem_hal_get_voltage( void )
{
   printk("%s %d (from %p) \n", __func__, __LINE__, __builtin_return_address(0) );
#if PHIL
   uint16_t measure_vref_mv = 0;
   hal_adc_init( );
   measure_vref_mv = hal_adc_get_vref_int( );
   hal_adc_deinit( );

   // convert voltage from mv to cloud readable (1/50V = 20mv)
   return ( uint8_t )( measure_vref_mv / 20 );
#endif
   return 0;
}

int8_t smtc_modem_hal_get_board_delay_ms( void )
{
    return 30;
}




/* ------------ Trace management ------------*/

void smtc_modem_hal_print_trace( const char* fmt, ... )
{
    va_list args;
    va_start( args, fmt );
    hal_trace_print( fmt, args );
    va_end( args );
}

extern void semtracker_thread_wakeup(void);

/*-------------------------------------------------------------------------
 *
 * name:        smtc_modem_hal_user_lbm_irq
 *
 * description: A timer or LR1110 interrupt has occurred.  Wakeup the LBM
 *              thread.
 *
 * input:
 *
 * output:
 *
 *-------------------------------------------------------------------------*/
void smtc_modem_hal_user_lbm_irq( void )
{
   semtracker_thread_wakeup();
}
