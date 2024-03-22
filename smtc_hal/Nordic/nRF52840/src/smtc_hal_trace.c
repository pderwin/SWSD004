#include <zephyr/kernel.h>

void hal_trace_print( const char* fmt, va_list argp )
{
   vprintk(fmt, argp);
}
