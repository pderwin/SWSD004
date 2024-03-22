#include <zephyr/kernel.h>
#include <time.h>
#include "smtc_hal_rtc.h"
#include "smtc_hal_mcu.h"

static uint32_t rtc_get_calendar_time( uint16_t* msec_div_10_p );

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
