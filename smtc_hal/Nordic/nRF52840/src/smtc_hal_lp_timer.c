#include <zephyr/kernel.h>
#include "smtc_hal_lp_timer.h"
#include "lr11xx_drv.h"

void hal_lp_timer_init( void )
{
   /* nothing to do. */
}

void hal_lp_timer_start( const uint32_t milliseconds, const hal_lp_timer_irq_t* tmr_irq )
{
    lr11xx_drv_timer_start(milliseconds, tmr_irq->callback, tmr_irq->context);
}

void hal_lp_timer_stop( void )
{
    lr11xx_drv_timer_stop();
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
