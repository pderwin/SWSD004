#include <zephyr/kernel.h>
#include <time.h>
#include "smtc_hal_rtc.h"
#include "smtc_hal_mcu.h"

// sub-second number of bits
#define N_PREDIV_S                     10U

// Synchronous prediv
#define PREDIV_S                       ( ( 1U << N_PREDIV_S ) - 1U )

// Asynchronous prediv
#define PREDIV_A                       ( ( 1U << ( 15U - N_PREDIV_S ) ) - 1U )

// Sub-second mask definition
#define ALARM_SUBSECOND_MASK           ( N_PREDIV_S << RTC_ALRMASSR_MASKSS_Pos )

// RTC Time base in us
#define USEC_NUMBER                    1000000U
#define MSEC_NUMBER                    ( USEC_NUMBER / 1000 )

#define COMMON_FACTOR                  3U
#define CONV_NUMER                     ( MSEC_NUMBER >> COMMON_FACTOR )
#define CONV_DENOM                     ( 1U << ( N_PREDIV_S - COMMON_FACTOR ) )

/*!
 * Days, Hours, Minutes and seconds
 */
#define DAYS_IN_LEAP_YEAR              ( ( uint32_t ) 366U )
#define DAYS_IN_YEAR                   ( ( uint32_t ) 365U )
#define SECONDS_IN_1DAY                ( ( uint32_t ) 86400U )
#define SECONDS_IN_1HOUR               ( ( uint32_t ) 3600U )
#define SECONDS_IN_1MINUTE             ( ( uint32_t ) 60U )
#define MINUTES_IN_1HOUR               ( ( uint32_t ) 60U )
#define HOURS_IN_1DAY                  ( ( uint32_t ) 24U )

/*!
 * Correction factors
 */
#define DAYS_IN_MONTH_CORRECTION_NORM  ( ( uint32_t ) 0x99AAA0 )
#define DAYS_IN_MONTH_CORRECTION_LEAP  ( ( uint32_t ) 0x445550 )

// clang-format on

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*!
 * RTC timer context
 */
typedef struct
{
    uint32_t        time_ref_in_ticks;  // Reference time
#if PHIL
   RTC_TimeTypeDef calendar_time;      // Reference time in calendar format
    RTC_DateTypeDef calendar_date;      // Reference date in calendar format
#endif
} rtc_context_t;

typedef struct bsp_rtc_s
{
#if PHIL
   RTC_HandleTypeDef handle;
#endif
    /*!
     * Keep the value of the RTC timer when the RTC alarm is set
     * Set with the \ref bsp_rtc_set_context function
     * Value is kept as a Reference to calculate alarm
     */
    rtc_context_t context;
} bsp_rtc_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

// PHIL static bsp_rtc_t bsp_rtc;

static volatile bool wut_timer_irq_happened = false;

/*!
 * Get the elapsed time in seconds and milliseconds since RTC initialization
 *
 * \param [OUT] milliseconds Number of milliseconds elapsed since RTC
 *                           initialization
 * \retval seconds           Number of seconds elapsed since RTC initialization
 */
static uint32_t rtc_get_calendar_time( uint16_t* milliseconds );

/*-------------------------------------------------------------------------
 *
 * name:        hal_rtc_get_time_s
 *
 * description:
 *
 * input:
 *
 * output:
 *
 *-------------------------------------------------------------------------*/
uint32_t hal_rtc_get_time_s( void )
{
    uint16_t milliseconds = 0;
    return rtc_get_calendar_time( &milliseconds );
}

/*-------------------------------------------------------------------------
 *
 * name:        hal_rtc_get_time_100us
 *
 * description:
 *
 * input:
 *
 * output:
 *
 *-------------------------------------------------------------------------*/
uint32_t hal_rtc_get_time_100us( void )
{
    uint32_t seconds             = 0;
    uint16_t milliseconds_div_10 = 0;

    seconds = rtc_get_calendar_time( &milliseconds_div_10 );

    return seconds * 10000 + milliseconds_div_10;
}

/*-------------------------------------------------------------------------
 *
 * name:        hal_rtc_get_time_ms
 *
 * description:
 *
 * input:
 *
 * output:
 *
 *-------------------------------------------------------------------------*/
uint32_t hal_rtc_get_time_ms( void )
{
    uint32_t seconds             = 0;
    uint16_t milliseconds_div_10 = 0;

    seconds = rtc_get_calendar_time( &milliseconds_div_10 );

    return seconds * 1000 + ( milliseconds_div_10 / 10 );
}

/*-------------------------------------------------------------------------
 *
 * name:        get uptime with resolution to 100 uSecs
 *
 * description:
 *
 * input:
 *
 * output:      seconds:            full seconds
 *              milliseconds_di_10: number of 100 uSec ticks within the second
 *
 *-------------------------------------------------------------------------*/
static uint32_t rtc_get_calendar_time( uint16_t* msec_div_10_p )
{
    uint64_t
       msec_div_10,
       seconds,
       uptime_ticks,
       uptime_100_usecs;

    /*
     * Get number of seconds of uptime, as well as msecs
     */
    uptime_ticks = k_uptime_ticks();

    /*
     * The counter always runs with an input clock of 32768 Hz.  Convert
     * the tick value to time in terms of 100 uSecs by multiplying by
     * 10000, and the divide by 32768.
     */
    uptime_100_usecs = (uptime_ticks * 10000) / 32768;

    seconds     = uptime_100_usecs / 10000;
    msec_div_10 = uptime_100_usecs % 10000;

    /*
     * This return value seems to be the current partial
     * second in terms of 100 uSec units.
     */
    *msec_div_10_p = msec_div_10;

    return seconds;
}

void RTC_WKUP_IRQHandler( void )
{
    printk("%s %d (from %p) \n", __func__, __LINE__, __builtin_return_address(0) );
#if PHIL
    HAL_RTCEx_WakeUpTimerIRQHandler( &bsp_rtc.handle );
    wut_timer_irq_happened = true;
#endif
}
