/****************************************************************************
* Title                 :   Magneto click library
* Filename              :   magneto.h
* Author                :   RBL
* Origin Date           :   05/12/2015
* Notes                 :   None
*****************************************************************************/
/**************************CHANGE LIST **************************************
*
*    Date    Software Version    Initials   Description
*  05/12/15         0.1          RBL        Interface Created.
*  23/12/15         0.2          RBL        Correction of angle reporting
*  14/01/16         0.3          RBL        Added multiple I2C support
*
*****************************************************************************/
/**
 * @file magneto.h
 * @brief Angle measurement with magnetic fields.  All with 14 bit resolution.
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
 *   Ext. Modules:    Magneto Click
 *   SW:              ARM 4.5
 *
 * @mainpage
 * @section General Description
 * @par
 * The AS5048 is an easy to use 360° angle position sensor with a
 * 14-bit high resolution output. The maximum system accuracy
 * is 0.05° assuming linearization and averaging is done by the
 * external microcontroller.
 * @par
 * The IC measures the absolute position of the magnet’s rotation
 * angle and consists of Hall sensors, analog digital converter and
 * digital signal processing. The absolute position information of
 * the magnet is directly accessible over a PWM output and can
 * be read out over a standard SPI or a high speed I2C interface.
 * AS5048A has a SPI interface, AS5048B I2C interface. Both devices
 * offer a PWM output.
 * @par
 * The zero position can be programmed via SPI or I2C command.
 * This simplifies the assembly of the complete system because
 * the zero position of the magnet does not need to be
 * mechanically aligned. The sensor tolerates misalignment, air
 * gap variations, temperature variations and as well external
 * magnetic fields. This robustness and wide temperature range
 * (-40°C up to +150°C) of the AS5048 makes the IC ideal for
 * rotation angle sensing in harsh industrial and medical
 * environments. Several AS5048 ICs can be connected in daisy
 * chain for serial data read out. An internal voltage regulator
 * allows the AS5048 to operate at either 3.3 V or 5 V supplies.
 *
 * @section Specifications
 * @ref specifications
 * @section Example
 * @ref example
 */
#ifndef MAGNETO_H_
#define MAGNETO_H_

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
/**
 * @struct Magneto State
 *
 */
typedef struct
{
    uint8_t ocf       : 1;
    uint8_t cof       : 1;
    uint8_t comp_low  : 1;
    uint8_t comp_high : 1;
} magneto_state_t;

/**
 * @enum Error types
 *
 */
typedef enum
{
    MAGNETO_WRONG_FRAMING_ERROR = 0, /**< Usually caused by SPI timing */
    MAGNETO_WRONG_COMMAND_INVAILD,   /**< Invalid command specified */
    MAGNETO_WRONG_PARITY,            /**< Parity of data did not pass CRC */
    MAGNETO_WRONG_NUM_OF_CLOCKS      /**< Too rapid of input clock */
} magneto_error_t;

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
 * @brief Initializes the magneto IC
 *
 * @param mode[IN] - Modes of operation can be either SPI or I2C
 * @param address[IN] - Only used in case of I2C mode
 * @param num_of_devices[IN] - How many devices are chained on the bus
 *
 * @return int
 * @retval 0 Success
 * @retval -1 failure on bus
 *
 * @code
 * if( magneto_init( MAGNETO_SPI, 0, 1 ) )
 *     return error_code;
 * @endcode
 */
int magneto_init( magneto_mode_t mode, uint8_t *address,
                  uint8_t num_of_devices );

/**
 * @brief Returns angles of all attached devices
 *
 * You may select individual devices or 0 for all.
 *
 * @param[IN] device_num - 1 through 10 for individual
 * selection or 0 for all.
 *
 * @return Array of upto 10 angles 0.. 16384 steps
 */
int16_t *magneto_get_angles( uint8_t device_num );

/**
 * @brief Get degrees
 *
 * @param[IN] device_num
 * @return degrees in float precision
 */
float *magneto_get_degrees( uint8_t device_num );

/**
 * @brief Get radians
 *
 * @param[IN] device_num - 1 through 10 for individual
 * selection or 0 for all.
 *
 * @return radians in float precision
 */
float *magneto_get_radians( uint8_t device_num );

/**
 * @brief Gets the value of the state register
 *
 * @param[IN] device_num - 1 through 10 for individual
 * selection or 0 for all.
 *
 * @return flags
 */
magneto_state_t *magneto_get_states( uint8_t device_num );

/**
 * @brief Get gain of selected device
 *
 * @param[IN] device_num - 1 through 10 for individual
 * selection or 0 for all.
 *
 * @return
 */
uint8_t *magneto_get_gains( uint8_t device_num );

/**
 * @brief Print the diagnostic register of the sensor
 *
 * @param[IN] function pointer to print char function
 */
void magneto_print_state( void( *print_char )( char c ) );

/**
 * @brief Get and clear the error register by reading it
 *
 * @param[IN] device_num - 1 through 10 for individual
 * selection or 0 for all.
 *
 */
magneto_error_t *magneto_get_errors( uint8_t device_num );

/**
 * @brief Get zero positions
 *
 * @param[IN] device_num - 1 through 10 for individual
 * selection or 0 for all.
 *
 * @return OTP positions
 */
int16_t *magneto_get_zero_positions( uint8_t device_num );

/**
 * @brief Check if an error has been encountered.
 *
 * @return error true if error occurred in last operation false if
 * operation was successful.
 */
bool magneto_error( void );

#ifdef __cplusplus
} // extern "C"
#endif

#endif /*File_H_*/

/**
 * @page example Example
 * @section Code Example
 * @code
 * // Initialize all system peripherals
 * static void system_init()
 * {
 *    DisableInterrupts();
 *
 *     GPIO_Digital_Output( &GPIOD_BASE, _GPIO_PINMASK_13 );
 *     // SPI Init - NOTE* Magneto samples bits on the second clock transition
 *     SPI3_Init_Advanced( _SPI_FPCLK_DIV16,
 *                        _SPI_MASTER | _SPI_8_BIT | _SPI_CLK_IDLE_LOW |
 *                        _SPI_SECOND_CLK_EDGE_TRANSITION | _SPI_MSB_FIRST |
 *                        _SPI_SS_DISABLE | _SPI_SSM_ENABLE | _SPI_SSI_1,
 *                        &_GPIO_MODULE_SPI3_PC10_11_12 );
 *    // Initialize display
 *    tft_initialize();
 *    // Start timer
 *    init_timer2();
 *
 *    // Initialize magneto and check for success or failure
 *    if( magneto_init( MAGNETO_SPI, 0, 1 ) ) // SPI mode, 0 address, 1 device
 *        return;
 *}
 *
 * void main( )
 * {
 *     float *read_angle;
 *     int i;
 *
 *     for( i = 0; i < 10; i++ )
 *     {
 *         read_angle = magneto_get_degrees( 1 ); // Get degrees from device 1
 *         ...
 *     }
 * ...
 * }
 * @endcode
 *
 *
 */

/**
 * @page specifications Specifications
 * @par
 * <a href="http://www.mouser.com/ds/2/588/AS5048-EK-AB_Operation-Manual_Rev.1.3-263495.pdf">Datasheet</a>
 *
 * @section FEATURES
 *
 * @par
 * Programmable Fan Control circuit
 * <ul>
 * <li>Contactless rotary position sensor over 360°</li>
 * <li>Immune to external magnetic stray fields</li>
 * <li>14-bit resolution (0.0219°/LSB)</li>
 * <li>Standard SPI or I 2 C interface and PWM</li>
 * <li>Zero position programmable via SPI or I 2 C</li>
 * <li>Temperature range: -40°C to +150°C</li>
 * </ul>
 *
 *
 * @par
 * Applications
 * <ul>
 * <li>Angle Sensor</li>
 * <li>Replacement of potentiometers</li>
 * <li>Factory and process automation</li>
 * <li>Robotics</li>
 * </ul>
 */

/*** End of File **************************************************************/
