> ![MikroE](http://www.mikroe.com/img/designs/beta/logo_small.png)
> #[Magneto click](http://www.mikroe.com/click/magneto/)#
> ##By [MikroElektronika](http://www.mikroe.com)
---

## Installation
Package manager required to install the package to your IDE.  

###Example
```
#include "magneto.h"

sbit MAGNETO_CS at GPIOD_ODR.B13;

// Initialize all system peripherals
void system_init()
{
    DisableInterrupts();

    GPIO_Digital_Output( &GPIOD_BASE, _GPIO_PINMASK_13 );
    
    UART1_Init( 57600 );

    /* SPI Init - NOTE* Magneto samples bits on the second clock transition */
    SPI3_Init_Advanced( _SPI_FPCLK_DIV16,
                        _SPI_MASTER | _SPI_8_BIT | _SPI_CLK_IDLE_LOW |
                        _SPI_SECOND_CLK_EDGE_TRANSITION | _SPI_MSB_FIRST |
                        _SPI_SS_DISABLE | _SPI_SSM_ENABLE | _SPI_SSI_1,
                        &_GPIO_MODULE_SPI3_PC10_11_12 );
    
    // Initialize magneto and check for success or failure
    if( magneto_init( MAGNETO_SPI, 0, 1 ) )
        return;
}


void main()
{
    float *read_angle;

    system_init();

    while( 1 )
    {
        read_angle = magneto_get_angles( 1 );
        
        if( magneto_error() )
        {
            magneto_error_t *error = magneto_get_errors( 1 );
            
            if( *error == MAGNETO_WRONG_NUM_OF_CLOCKS )
                UART_Write_Text( "Too fast of SPI clock\r\n" );
        } else {
            display_angle( *read_angle );
        }

        ...
    }
}
```
