/****************************************************************************
* Title                 :   Magneto Definitions
* Filename              :   magneto_defs.h
* Author                :   RBL
* Origin Date           :   06/12/2015
* Notes                 :   None
*****************************************************************************/
/**************************CHANGE LIST **************************************
*
*    Date    Software Version    Initials   Description
*  06/12/15         .1           RBL      Interface Created.
*
*****************************************************************************/
/**
 * @file magneto_defs.h
 * @brief Defines shared registry entries for the magneto click
 *
 * @date 06 Dec 2015
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
#ifndef MAGNETO_DEFS_H
#define MAGNETO_DEFS_H

/******************************************************************************
* Includes
*******************************************************************************/


/******************************************************************************
* Preprocessor Constants
*******************************************************************************/


/******************************************************************************
* Configuration Constants
*******************************************************************************/
/**
 * @def SPI Registry
 */
#define AS5048A_NOP                         0x0000

#define AS5048A_CLEAR_ERROR_FLAG            0x0001
#define AS5048A_CLEAR_PARITY_BIT            2
#define AS5048A_CLEAR_PARITY_MASK           ( 1 << AS5048A_CLEAR_PARITY_BIT )
#define AS5048A_CLEAR_COMMAND_INVALID_BIT   1
#define AS5048A_CLEAR_COMMAND_INVALID_MASK  ( 1 << AS5048A_CLEAR_COMMAND_INVALID_BIT )
#define AS5048A_CLEAR_FRAMING_ERROR_BIT     0
#define AS5048A_CLEAR_FRAMING_ERROR_MASK    ( 1 << AS5048A_CLEAR_FRAMING_ERROR_BIT )

#define AS5048A_PROGRAMMING_CONTROL              0x0003
#define AS5048A_PROGRAMMING_CONTROL_VERIFY_BIT   6
#define AS5048A_PROGRAMMING_CONTROL_VERIFY_MASK  ( 1 << AS5048A_PROGRAMMING_CONTROL_VERIFY_BIT )
#define AS5048A_PROGRAMMING_CONTROL_BURN_BIT     3
#define AS5048A_PROGRAMMING_CONTROL_BURN_MASK    ( 1 << AS5048A_PROGRAMMING_CONTROL_BURN_BIT )
#define AS5048A_PROGRAMMING_CONTROL_PROG_EN_BIT  0
#define AS5048A_PROGRAMMING_CONTROL_PROG_EN_MASK ( 1 << AS5048A_PROGRAMMING_CONTROL_PROG_EN_BIT )

#define AS5048A_OTP_REGISTER_ZERO_POS_HIGH  0x0016
#define AS5048A_OTP_REGISTER_ZERO_POS_LOW   0x0017

#define AS5048A_DIAG_AGC                    0x3FFD
#define AS5048A_DIAG_AGC_COMP_HIGH_BIT      11
#define AS5048A_DIAG_AGC_COMP_HIGH_MASK     ( 1 << AS5048A_DIAG_AGC_COMP_HIGH_BIT )
#define AS5048A_DIAG_AGC_COMP_LOW_BIT       10
#define AS5048A_DIAG_AGC_COMP_LOW_MASK      ( 1 << AS5048A_DIAG_AGC_COMP_LOW_BIT )
#define AS5048A_DIAG_AGC_COF_BIT            9
#define AS5048A_DIAG_AGC_COF_MASK           ( 1 << AS5048A_DIAG_AGC_COF_BIT )
#define AS5048A_DIAG_AGC_OCF_BIT            8
#define AS5048A_DIAG_AGC_OCF_MASK           ( 1 << AS5048A_DIAG_AGC_OCF_BIT )

#define AS5048A_MAGNITUDE                   0x3FFE
#define AS5048A_ANGLE                       0x3FFF

/**
 * @def I2C Registry
 */
#define AS5048A_I2C_PROGRAMMING_CONTROL     0x3
#define AS5048A_I2C_SLAVE_ADDRESS           0x15
#define AS5048A_I2C_OTP_REGISTER_ZERO_POS_HIGH 0x16
#define AS5048A_I2C_OTP_REGISTER_ZERO_POS_LOW 0x17
#define AS5048A_I2C_AUTOMATIC_GAIN_CONTROL  0xFA
#define AS5048A_I2C_DIAGNOSTICS             0xFB
#define AS5048A_I2C_MAGNITUDE_HIGH          0xFC
#define AS5048A_I2C_MAGNITUDE_LOW           0xFD
#define AS5048A_I2C_ANGLE_HIGH              0xFE
#define AS5048A_I2C_ANGLE_LOW               0xFF

#define AS5048A_MAX_READ_SIZE               2

#define AS5048A_PARITY_BIT                  15
#define AS5048A_READWRITE                   14
#define AS5048A_ERRORFLAG_BIT               14
#define AS5048A_ERRORFLAG_MASK              ( 1 << AS5048A_ERRORFLAG_BIT )

/******************************************************************************
* Macros
*******************************************************************************/



/******************************************************************************
* Typedefs
*******************************************************************************/
/**
 * @enum Modes available to MAGNETO
 */
typedef enum
{
    MAGNETO_I2C,  /**< I2C Mode */
    MAGNETO_SPI,  /**< Chainable SPI bus mode */
    MAGNETO_PWM   /**< PWM Output */
} magneto_mode_t;

/******************************************************************************
* Variables
*******************************************************************************/


/******************************************************************************
* Function Prototypes
*******************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif



#ifdef __cplusplus
} // extern "C"
#endif

#endif

/*** End of File **************************************************************/
