#include <zephyr/kernel.h>
#include "smtc_hal_gpio.h"

/*!
 * GPIO setup data structure
 */
typedef struct bsp_gpio_s
{
    hal_gpio_pin_names_t pin;
    uint32_t             mode;
    uint32_t             pull;
    uint32_t             speed;
    uint32_t             alternate;
} gpio_t;



/*!
 * Array holding attached IRQ gpio data context
 */
static hal_gpio_irq_t const* gpio_irq[16];

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*!
 * Generic gpio initialization
 *
 * \param [in/out] gpio  Holds MCU gpio parameters
 * \param [in]     value Initial MCU pit value
 * \param [in/out] irq   Pointer to IRQ data context.
 *                         NULL when setting gpio as output
 */
// PHIL static void gpio_init( const gpio_t* gpio, const uint32_t value, const hal_gpio_irq_t* irq );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */
extern void lr11xx_hal_dio_irq_register( void (*callback)(void *context), void *context);

void hal_gpio_irq_attach( const hal_gpio_irq_t* irq )
{
    lr11xx_hal_dio_irq_register(irq->callback, irq->context);
}

void hal_gpio_irq_deatach( const hal_gpio_irq_t* irq )
{
    if( irq != NULL )
    {
	gpio_irq[( irq->pin ) & 0x0F] = NULL;
    }
}

void hal_gpio_irq_enable( void )
{
   printk("%s %d (from %p) \n", __func__, __LINE__, __builtin_return_address(0) );
#if PHIL
   HAL_NVIC_EnableIRQ( EXTI0_IRQn );
    HAL_NVIC_EnableIRQ( EXTI1_IRQn );
    HAL_NVIC_EnableIRQ( EXTI2_IRQn );
    HAL_NVIC_EnableIRQ( EXTI3_IRQn );
    HAL_NVIC_EnableIRQ( EXTI4_IRQn );
    HAL_NVIC_EnableIRQ( EXTI9_5_IRQn );
    HAL_NVIC_EnableIRQ( EXTI15_10_IRQn );
#endif
}

void hal_gpio_irq_disable( void )
{
    printk("%s %d (from %p) \n", __func__, __LINE__, __builtin_return_address(0) );
#if PHIL
   HAL_NVIC_DisableIRQ( EXTI0_IRQn );
    HAL_NVIC_DisableIRQ( EXTI1_IRQn );
    HAL_NVIC_DisableIRQ( EXTI2_IRQn );
    HAL_NVIC_DisableIRQ( EXTI3_IRQn );
    HAL_NVIC_DisableIRQ( EXTI4_IRQn );
    HAL_NVIC_DisableIRQ( EXTI9_5_IRQn );
    HAL_NVIC_DisableIRQ( EXTI15_10_IRQn );
#endif
}

//
// MCU pin state control
//

void hal_gpio_set_value( const hal_gpio_pin_names_t pin, const hal_gpio_state_t value )
{
    printk("%s %d (from %p) \n", __func__, __LINE__, __builtin_return_address(0) );
#if PHIL
    GPIO_TypeDef* gpio_port = ( GPIO_TypeDef* ) ( AHB2PERIPH_BASE + ( ( pin & 0xF0 ) << 6 ) );

    HAL_GPIO_WritePin( gpio_port, ( 1 << ( pin & 0x0F ) ), ( value != 0 ) ? GPIO_PIN_SET : GPIO_PIN_RESET );
#endif
}
