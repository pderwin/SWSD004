/*!
 * \file      smtc_hal_lp_timer.c
 *
 * \brief     Implements Low Power Timer utilities functions.
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
#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

#include "smtc_hal_lp_timer.h"
#include "smtc_hal_mcu.h"
#include "lr11xx_hal.h"
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

// PHIL static LPTIM_HandleTypeDef lptim_handle;

// PHIL static hal_lp_timer_irq_t lptim_tmr_irq = { .context = NULL, .callback = NULL };

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */
void hal_lp_timer_init( void )
{
    printk("%s %d (from %p) \n", __func__, __LINE__, __builtin_return_address(0) );
    lr11xx_hal_timer_init();
}

void hal_lp_timer_start( const uint32_t milliseconds, const hal_lp_timer_irq_t* tmr_irq )
{
    lr11xx_hal_timer_start(milliseconds, tmr_irq->callback, tmr_irq->context);
}

void hal_lp_timer_stop( void )
{
    lr11xx_hal_timer_stop();

#if PHIL
    HAL_LPTIM_TimeOut_Stop_IT( &lptim_handle );
#endif
}

void hal_lp_timer_irq_enable( void )
{
//    printk("%s %d (from %p) \n", __func__, __LINE__, __builtin_return_address(0) );
#if PHIL
    HAL_NVIC_EnableIRQ( LPTIM1_IRQn );
#endif
}

void hal_lp_timer_irq_disable( void )
{
//    printk("%s %d (from %p) \n", __func__, __LINE__, __builtin_return_address(0) );
#if PHIL
    HAL_NVIC_DisableIRQ( LPTIM1_IRQn );
#endif
}

void LPTIM1_IRQHandler( void )
{
    printk("%s %d (from %p) \n", __func__, __LINE__, __builtin_return_address(0) );
#if PHIL
    HAL_LPTIM_IRQHandler( &lptim_handle );
    HAL_LPTIM_TimeOut_Stop( &lptim_handle );

    if( lptim_tmr_irq.callback != NULL )
    {
        lptim_tmr_irq.callback( lptim_tmr_irq.context );
    }
#endif
}

void HAL_LPTIM_MspInit( void /* LPTIM_HandleTypeDef* lptimhandle */ )
{
    printk("%s %d (from %p) \n", __func__, __LINE__, __builtin_return_address(0) );
#if PHIL
    if( lptimhandle->Instance == LPTIM1 )
    {
        __HAL_RCC_LPTIM1_CLK_ENABLE( );
        HAL_NVIC_SetPriority( LPTIM1_IRQn, 0, 0 );
        HAL_NVIC_EnableIRQ( LPTIM1_IRQn );
    }
#endif
}

void HAL_LPTIM_MspDeInit( void /* LPTIM_HandleTypeDef* lptimhandle */ )
{
    printk("%s %d (from %p) \n", __func__, __LINE__, __builtin_return_address(0) );
#if PHIL
    if( lptimhandle->Instance == LPTIM1 )
    {
        __HAL_RCC_LPTIM1_CLK_DISABLE( );
        HAL_NVIC_DisableIRQ( LPTIM1_IRQn );
    }
#endif
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
