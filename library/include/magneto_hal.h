/****************************************************************************
* Title                 :   HAL for Magneto click
* Filename              :   magneto_hal.h
* Author                :   RBL
* Origin Date           :   07/12/2015
* Notes                 :   None
*****************************************************************************/
/**************************CHANGE LIST **************************************
*
*    Date    Software Version    Initials   Description
*  07/12/15         .1           RBL      Interface Created.
*
*****************************************************************************/
/**
 * @file magneto_hal.h
 * @brief Hardware interface from sensor to MCU
 *
 * @date 05 Dec 2015
 * @author Richard Lowe
 * @copyright GNU Public License
 *
 * @version .1 - Initial testing and verification
 *
 * @note Test configuration:
 *   MCU:             STM32F107VC
 *   Dev.Board:       EasyMx Pro v7
 *   Oscillator:      72 Mhz internal
 *   Ext. Modules:    Magneto click
 *   SW:              ARM 4.7
 *
 */
#ifndef MAGNETO_HAL_H_
#define MAGNETO_HAL_H_

/******************************************************************************
* Includes
*******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include "magneto_defs.h"

/******************************************************************************
* Preprocessor Constants
*******************************************************************************/



/******************************************************************************
* Configuration Constants
*******************************************************************************/


/******************************************************************************
* Macros
*******************************************************************************/



/******************************************************************************
* Typedefs
*******************************************************************************/


/******************************************************************************
* Variables
*******************************************************************************/


/******************************************************************************
* Function Prototypes
*******************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Magneto HAL Initialization
 *
 * @param mode - mode of operation
 * @param address_id - address of slave if I2C mode selected
 *
 * @return result of initialization
 * @retval -1 : error
 * @retval 0  : success
 *
 * @code
 * if( magneto_hal_init( MAGNETO_SPI, 0x00 ) < 0 )
 *     return ERROR;
 * @endcode
 */
int magneto_hal_init( magneto_mode_t mode, uint8_t address_id,
                      uint8_t num_devices );

/**
 * @brief Set slave address used in multiple I2C
 *
 * @param[IN] slave_address
 */
void magneto_hal_set_slave( uint8_t slave_address );

/**
 * @brief magneto_hal_write
 *
 * @param[IN] address
 * @param[IN] buff
 *
 * @return returns previous command as a reflection to success
 * or failure of instruction.
 */
int16_t magneto_hal_write( uint16_t address, int16_t value );

/**
 * @brief magneto_hal_read
 *
 * @param[IN] address
 * @param[IN] buff
 *
 * @return true on success / false on failure
 */
bool magneto_hal_read( uint16_t address, int16_t *buff );


#ifdef __cplusplus
} // extern "C"
#endif

#endif /*File_H_*/

/*** End of File **************************************************************/
