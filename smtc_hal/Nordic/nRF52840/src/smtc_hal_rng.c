#include <zephyr/kernel.h>
#include <zephyr/drivers/entropy.h>
#include "smtc_hal_rng.h"

const static struct device *entropy_device = DEVICE_DT_GET(DT_ALIAS(rng0));

/*-------------------------------------------------------------------------
 *
 * name:        hal_rng_get_random
 *
 * description:
 *
 * input:
 *
 * output:
 *
 *-------------------------------------------------------------------------*/
uint32_t hal_rng_get_random( void )
{
   int
      rc;
   uint32_t
      rand_nb = -1;

   if (entropy_device == NULL) {
      printk("%s: Error opening entropy device\n", __func__);
      return 0;
   }

   rc = entropy_get_entropy(entropy_device, (uint8_t *) &rand_nb, sizeof(rand_nb));
   if (rc) {
      printk("%s: Error getting random number\n", __func__);
      return 0;
   }

   return rand_nb;
}

/*-------------------------------------------------------------------------
 *
 * name:        hal_rng_get_random_in_range
 *
 * description:
 *
 * input:
 *
 * output:
 *
 *-------------------------------------------------------------------------*/
uint32_t hal_rng_get_random_in_range( const uint32_t val_1, const uint32_t val_2 )
{
   if ( val_1 <= val_2 ) {
      return ( ( hal_rng_get_random( ) % ( val_2 - val_1 + 1 ) ) + val_1 );
   }
   else {
      return ( ( hal_rng_get_random( ) % ( val_1 - val_2 + 1 ) ) + val_2 );
   }
}
