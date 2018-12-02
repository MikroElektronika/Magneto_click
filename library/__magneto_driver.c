/*
    __magneto_driver.c

-----------------------------------------------------------------------------

  This file is part of mikroSDK.

  Copyright (c) 2017, MikroElektonika - http://www.mikroe.com

  All rights reserved.

----------------------------------------------------------------------------- */

#include "__magneto_driver.h"
#include "__magneto_hal.c"

/* ------------------------------------------------------------------- MACROS */



/* ---------------------------------------------------------------- VARIABLES */

#ifdef   __MAGNETO_DRV_I2C__
static uint8_t _slaveAddress;
#endif

const uint16_t _MAGNETO_CLEAR_ERROR_FLAG                               = 0x0001;
const uint16_t _MAGNETO_PROGRAMMING_CONTROL                            = 0x0003;
const uint16_t _MAGNETO_OTP_REGISTER_ZERO_POS_HIGH                     = 0x0016;
const uint16_t _MAGNETO_OTP_REGISTER_ZERO_POS_LOW                      = 0x0017;
const uint16_t _MAGNETO_DIAG_AGC                                       = 0x3FFD;
const uint16_t _MAGNETO_MAGNITUDE                                      = 0x3FFF;
const uint16_t _MAGNETO_ANGLE                                          = 0x3FFF;

const uint16_t _MAGNETO_RESET_VALUE                                    = 0x0000;
const uint16_t _MAGNETO_SET_VALUE                                      = 0xFFFE;
const uint16_t _MAGNETO_MAGNITUDE_VALUE                                = 0x1FFE;
const uint16_t _MAGNETO_PARITY_VALUE                                   = 0x000F;
const uint16_t _MAGNETO_READ_COMMAND_BIT                               = 0x4000;
const uint16_t _MAGNETO_HIGH_BYTE                                      = 0xFF00;
const uint16_t _MAGNETO_LOW_BYTE                                       = 0x00FF;

/* -------------------------------------------- PRIVATE FUNCTION DECLARATIONS */

static uint8_t _calcParity( uint16_t parity );

/* --------------------------------------------- PRIVATE FUNCTION DEFINITIONS */

/* Utility function used to calculate even parity of data */
static uint8_t _calcParity( uint16_t parity )
{
    uint8_t counter;
    uint16_t result;
    
    result = 0;

    for ( counter = 0; counter < 15; counter++ )
    {
        result += ( parity >> counter ) & 0x01;
    }
    
    result %= 2;
    
    return result;
}


/* --------------------------------------------------------- PUBLIC FUNCTIONS */

#ifdef   __MAGNETO_DRV_SPI__

void magneto_spiDriverInit(T_MAGNETO_P gpioObj, T_MAGNETO_P spiObj)
{
    hal_spiMap( (T_HAL_P)spiObj );
    hal_gpioMap( (T_HAL_P)gpioObj );

    hal_gpio_csSet(1);
    hal_gpio_pwmSet(0);
}

#endif
#ifdef   __MAGNETO_DRV_I2C__

void magneto_i2cDriverInit(T_MAGNETO_P gpioObj, T_MAGNETO_P i2cObj, uint8_t slave)
{
    _slaveAddress = slave;
    hal_i2cMap( (T_HAL_P)i2cObj );
    hal_gpioMap( (T_HAL_P)gpioObj );

    // ... power ON
    // ... configure CHIP
}

#endif
#ifdef   __MAGNETO_DRV_UART__

void magneto_uartDriverInit(T_MAGNETO_P gpioObj, T_MAGNETO_P uartObj)
{
    hal_uartMap( (T_HAL_P)uartObj );
    hal_gpioMap( (T_HAL_P)gpioObj );

    // ... power ON
    // ... configure CHIP
}

#endif



/* ----------------------------------------------------------- IMPLEMENTATION */


/* Generic read - 16-bit command  function */
uint16_t magneto_readData( uint16_t addressCommand )
{
    uint8_t writeReg[ 2 ];
    uint8_t readReg[ 2 ];
    uint16_t result;
    
    addressCommand &= _MAGNETO_ANGLE;
    addressCommand |= _MAGNETO_READ_COMMAND_BIT;
    addressCommand |= ( _calcParity( addressCommand ) << _MAGNETO_PARITY_VALUE );
    
    writeReg[ 0 ] = ( uint8_t ) ( ( addressCommand & _MAGNETO_HIGH_BYTE ) >> 8 );
    writeReg[ 1 ] = ( uint8_t ) ( addressCommand & _MAGNETO_LOW_BYTE );

    hal_gpio_csSet( 0 );
    Delay_1us();
    hal_spiWrite( writeReg, 2 );
    hal_gpio_csSet( 1 );

    hal_gpio_csSet( 0 );
    Delay_1us();
    hal_spiRead( readReg, 2 );
    hal_gpio_csSet( 1 );
    
    result = readReg[0];
    result <<= 8;
    result |= readReg[1];
    
    result &= _MAGNETO_MAGNITUDE_VALUE;

    return result;
}

/* Generic write function - 16-bit command & write data  */
void magneto_writeData( uint16_t addressCommand, uint16_t writeData )
{
    uint8_t bufferCommand[ 2 ];
    uint8_t bufferData[ 2 ];

    addressCommand &= _MAGNETO_ANGLE;
    addressCommand |= ( _calcParity( addressCommand ) << _MAGNETO_PARITY_VALUE );
    
    bufferCommand[ 0 ] = ( uint8_t ) ( ( addressCommand & _MAGNETO_HIGH_BYTE ) >> 8 );
    bufferCommand[ 1 ] = ( uint8_t ) ( addressCommand & _MAGNETO_LOW_BYTE );
    
    bufferData[ 0 ] = ( uint8_t ) ( ( addressCommand & _MAGNETO_HIGH_BYTE ) >> 8 );
    bufferData[ 1 ] = ( uint8_t ) ( writeData & _MAGNETO_LOW_BYTE );

    hal_gpio_csSet( 0 );
    Delay_1us();
    hal_spiWrite( bufferCommand, 2 );
    hal_gpio_csSet( 1 );
    
    hal_gpio_csSet( 0 );
    Delay_1us();
    hal_spiWrite( bufferData, 2 );
    hal_gpio_csSet( 1 );
}

/* Get state function */
uint16_t magneto_getState()
{
    return magneto_readData( _MAGNETO_DIAG_AGC );
}

/* Check state function */
uint8_t magneto_checkState()
{
    if ( magneto_getState() != 0 )
        return 1;
    else
        return 0;
}

/* Calculate angle function  */
float magneto_calculateAngle()
{
    uint16_t buffer;
    uint8_t counter;
    float angle;
    angle = 0.0;
    
    magneto_readData( _MAGNETO_CLEAR_ERROR_FLAG );
    
    magneto_writeData( _MAGNETO_OTP_REGISTER_ZERO_POS_HIGH, _MAGNETO_RESET_VALUE );
    magneto_writeData( _MAGNETO_OTP_REGISTER_ZERO_POS_LOW, _MAGNETO_RESET_VALUE );
    
    buffer = magneto_readData( _MAGNETO_MAGNITUDE );
    
    magneto_writeData( _MAGNETO_OTP_REGISTER_ZERO_POS_HIGH, ( buffer >> 6 ) & _MAGNETO_SET_VALUE );
    magneto_writeData( _MAGNETO_OTP_REGISTER_ZERO_POS_LOW, buffer & _MAGNETO_ANGLE );
    
    buffer = magneto_getState();

    for ( counter = 0; counter < 10; counter++ )
    {
        buffer = magneto_readData( _MAGNETO_MAGNITUDE );
        angle += ( float ) ( buffer  * 0.0219 );
    }

    angle /= 10.0;
    
    return angle;
}





/* -------------------------------------------------------------------------- */
/*
  __magneto_driver.c

  Copyright (c) 2017, MikroElektonika - http://www.mikroe.com

  All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

3. All advertising materials mentioning features or use of this software
   must display the following acknowledgement:
   This product includes software developed by the MikroElektonika.

4. Neither the name of the MikroElektonika nor the
   names of its contributors may be used to endorse or promote products
   derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY MIKROELEKTRONIKA ''AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL MIKROELEKTRONIKA BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------------- */