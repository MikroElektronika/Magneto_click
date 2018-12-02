/*
    __magneto_driver.h

-----------------------------------------------------------------------------

  This file is part of mikroSDK.
  
  Copyright (c) 2017, MikroElektonika - http://www.mikroe.com

  All rights reserved.

----------------------------------------------------------------------------- */

/**
@file   __magneto_driver.h
@brief    Magneto Driver
@mainpage Magneto Click
@{

@image html libstock_fb_view.jpg

@}

@defgroup   MAGNETO
@brief      Magneto Click Driver
@{

| Global Library Prefix | **MAGNETO** |
|:---------------------:|:-----------------:|
| Version               | **1.0.0**    |
| Date                  | **Oct 2018.**      |
| Developer             | **Nenad Filipovic**     |

*/
/* -------------------------------------------------------------------------- */

#include "stdint.h"

#ifndef _MAGNETO_H_
#define _MAGNETO_H_

/** 
 * @macro T_MAGNETO_P
 * @brief Driver Abstract type 
 */
#define T_MAGNETO_P    const uint8_t*

/** @defgroup MAGNETO_COMPILE Compilation Config */              /** @{ */

   #define   __MAGNETO_DRV_SPI__                            /**<     @macro __MAGNETO_DRV_SPI__  @brief SPI driver selector */
//  #define   __MAGNETO_DRV_I2C__                            /**<     @macro __MAGNETO_DRV_I2C__  @brief I2C driver selector */                                          
// #define   __MAGNETO_DRV_UART__                           /**<     @macro __MAGNETO_DRV_UART__ @brief UART driver selector */ 

                                                                       /** @} */
/** @defgroup MAGNETO_VAR Variables */                           /** @{ */

extern const uint16_t _MAGNETO_CLEAR_ERROR_FLAG;
extern const uint16_t _MAGNETO_PROGRAMMING_CONTROL;
extern const uint16_t _MAGNETO_OTP_REGISTER_ZERO_POS_HIGH;
extern const uint16_t _MAGNETO_OTP_REGISTER_ZERO_POS_LOW;
extern const uint16_t _MAGNETO_DIAG_AGC;
extern const uint16_t _MAGNETO_MAGNITUDE;
extern const uint16_t _MAGNETO_ANGLE;

                                                                       /** @} */
/** @defgroup MAGNETO_TYPES Types */                             /** @{ */



                                                                       /** @} */
#ifdef __cplusplus
extern "C"{
#endif

/** @defgroup MAGNETO_INIT Driver Initialization */              /** @{ */

#ifdef   __MAGNETO_DRV_SPI__
void magneto_spiDriverInit(T_MAGNETO_P gpioObj, T_MAGNETO_P spiObj);
#endif
#ifdef   __MAGNETO_DRV_I2C__
void magneto_i2cDriverInit(T_MAGNETO_P gpioObj, T_MAGNETO_P i2cObj, uint8_t slave);
#endif
#ifdef   __MAGNETO_DRV_UART__
void magneto_uartDriverInit(T_MAGNETO_P gpioObj, T_MAGNETO_P uartObj);
#endif


/** @defgroup MAGNETO_FUNC Driver Functions */                   /** @{ */


/**
 * @brief Generic read - 16-bit command  function
 *
 * @param[in] addressCommand         14-bit command address
 *
 * @return result                    14-bit read data
 *
 * Function takes the 16-bit address commnad of the target register
 * and the 16-bit write data to be written to that register.
 */
uint16_t magneto_readData( uint16_t addressCommand );

/**
 * @brief Generic write function - 16-bit command & write data
 *
 * @param[in] addressCommand         14-bit command address
 *
 * @param[in] writeData              14-bit data to write
 *
 * Function takes the 16-bit address commnad of the target register
 * and the 16-bit write data to be written to that register.
 */
void magneto_writeData( uint16_t addressCommand, uint16_t writeData );

 /**
 * @brief Get state function
 *
 * @return state                     16-bit data containing flags
 * - 0 - OK
 * - 1 - ERROR
 *
 * Function read and returns the value of the state register.
 */
uint16_t magneto_getState();

 /**
 * @brief Check state function
 *
 * @return state                      8-bit data containing flags state
 * - 0 - OK
 * - 1 - ERROR
 *
 * Function check and returns 0 or 1.
 */
uint8_t magneto_checkState();

/**
 * @brief Calculate angle function
 *
 * @return angle                     float angle value
 *
 * Function read the 16-bit data from register,
 * calculate and convert to float angle value from 0° to 360°.
 */
float magneto_calculateAngle();




                                                                       /** @} */
#ifdef __cplusplus
} // extern "C"
#endif
#endif

/**
    @example Click_Magneto_STM.c
    @example Click_Magneto_TIVA.c
    @example Click_Magneto_CEC.c
    @example Click_Magneto_KINETIS.c
    @example Click_Magneto_MSP.c
    @example Click_Magneto_PIC.c
    @example Click_Magneto_PIC32.c
    @example Click_Magneto_DSPIC.c
    @example Click_Magneto_AVR.c
    @example Click_Magneto_FT90x.c
    @example Click_Magneto_STM.mbas
    @example Click_Magneto_TIVA.mbas
    @example Click_Magneto_CEC.mbas
    @example Click_Magneto_KINETIS.mbas
    @example Click_Magneto_MSP.mbas
    @example Click_Magneto_PIC.mbas
    @example Click_Magneto_PIC32.mbas
    @example Click_Magneto_DSPIC.mbas
    @example Click_Magneto_AVR.mbas
    @example Click_Magneto_FT90x.mbas
    @example Click_Magneto_STM.mpas
    @example Click_Magneto_TIVA.mpas
    @example Click_Magneto_CEC.mpas
    @example Click_Magneto_KINETIS.mpas
    @example Click_Magneto_MSP.mpas
    @example Click_Magneto_PIC.mpas
    @example Click_Magneto_PIC32.mpas
    @example Click_Magneto_DSPIC.mpas
    @example Click_Magneto_AVR.mpas
    @example Click_Magneto_FT90x.mpas
*/                                                                     /** @} */
/* -------------------------------------------------------------------------- */
/*
  __magneto_driver.h

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