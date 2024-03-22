#include <stdint.h>

#define CRASH_LOG_SIZE (1024)

// __attribute__( ( section( ".noinit" ) ) ) static uint8_t          crashlog_buff_noinit[CRASH_LOG_SIZE];
__attribute__( ( section( ".noinit" ) ) ) static volatile uint8_t crashlog_length_noinit;
__attribute__( ( section( ".noinit" ) ) ) static volatile uint8_t crashlog_available_noinit;

uint32_t smtc_modem_hal_crashlog_get_status (void)
{
   return 0;
}
