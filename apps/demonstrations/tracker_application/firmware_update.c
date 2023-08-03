#include <zephyr/kernel.h>
#include "firmware_update.h"
// #include "lr1110_transceiver_0303.h"
#include "lr1110_transceiver_0308.h"

lr11xx_fw_update_status_t firmware_update (const void *radio)
{
   lr11xx_fw_update_status_t status;

   status = lr11xx_update_firmware( radio, LR11XX_FIRMWARE_UPDATE_TO, LR11XX_FIRMWARE_VERSION,
                                    lr11xx_firmware_image, ( uint32_t ) LR11XX_FIRMWARE_IMAGE_SIZE );

   switch( status )
   {
       case LR11XX_FW_UPDATE_OK:
          printk( "Expected firmware running!\n" );
          printk( "Please flash another application (like EVK Demo App).\n" );
          break;
       case LR11XX_FW_UPDATE_WRONG_CHIP_TYPE:
          printk( "Wrong chip type!\n" );
          break;

       case LR11XX_FW_UPDATE_ERROR:
          printk( "Error! Wrong firmware version - please retry.\n" );
          break;
   }

   return status;
}
