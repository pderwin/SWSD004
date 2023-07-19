/*!
 * \file      smtc_hal_rng.c
 *
 * \brief     Random Number Generator Hardware Abstraction Layer implementation
 *
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
 */

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */
#include <zephyr/kernel.h>
#include <zephyr/drivers/entropy.h>
#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

// PHIL #include "stm32l4xx_hal.h"
#include "smtc_hal_rng.h"

#include "smtc_hal_mcu.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

// PHIL static RNG_HandleTypeDef rng_handle;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */
const static struct device *entropy_device;

uint32_t hal_rng_get_random( void )
{
   uint32_t rand_nb = -1;

   if (entropy_device == NULL) {
      entropy_device = device_get_binding(DT_N_S_entropy_bt_hci_FULL_NAME);

      if (entropy_device == NULL) {
         printk("%s: Error opening entropy device\n", __func__);
      }
   }

   if (entropy_device) {
      entropy_get_entropy(entropy_device, (uint8_t *) &rand_nb, sizeof(rand_nb));
   }

   return rand_nb;
}

uint32_t hal_rng_get_random_in_range( const uint32_t val_1, const uint32_t val_2 )
{
    if( val_1 <= val_2 )
    {
        return ( uint32_t )( ( hal_rng_get_random( ) % ( val_2 - val_1 + 1 ) ) + val_1 );
    }
    else
    {
        return ( uint32_t )( ( hal_rng_get_random( ) % ( val_1 - val_2 + 1 ) ) + val_2 );
    }
}

int32_t hal_rng_get_signed_random_in_range( const int32_t val_1, const int32_t val_2 )
{
    uint32_t tmp_range = 0;  // ( val_1 <= val_2 ) ? ( val_2 - val_1 ) : ( val_1 - val_2 );

    if( val_1 <= val_2 )
    {
        tmp_range = ( val_2 - val_1 );
        return ( int32_t )( ( val_1 + hal_rng_get_random_in_range( 0, tmp_range ) ) );
    }
    else
    {
        tmp_range = ( val_1 - val_2 );
        return ( int32_t )( ( val_2 + hal_rng_get_random_in_range( 0, tmp_range ) ) );
    }
}

void HAL_RNG_MspInit( void /* RNG_HandleTypeDef* hrng */ )
{
    printk("%s %d (from %p) \n", __func__, __LINE__, __builtin_return_address(0) );
#if PHIL
   RCC_PeriphCLKInitTypeDef periph_clk_init = { 0 };

    // must reconfigure PLLSAI1 for RNG
    HAL_RCCEx_GetPeriphCLKConfig( &periph_clk_init );
    periph_clk_init.PeriphClockSelection = RCC_PERIPHCLK_RNG;
    if( HAL_RCCEx_PeriphCLKConfig( &periph_clk_init ) != HAL_OK )
    {
        mcu_panic( );  // Initialization Error
    }

    // RNG Peripheral clock enable
    __HAL_RCC_RNG_CLK_ENABLE( );
#endif
}

void HAL_RNG_MspDeInit( void /* RNG_HandleTypeDef* hrng */ )
{
    printk("%s %d (from %p) \n", __func__, __LINE__, __builtin_return_address(0) );
#if PHIL
    // Enable RNG reset state
    __HAL_RCC_RNG_FORCE_RESET( );

    // Release RNG from reset state
    __RNG_RELEASE_RESET( );

    __HAL_RCC_RNG_CLK_DISABLE( );
#endif
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
