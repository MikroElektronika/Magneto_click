/*******************************************************************************
* Title                 :   Magneto Implementation
* Filename              :   magneto.c
* Author                :   RBL
* Origin Date           :   05/12/2015
* Notes                 :   None
*******************************************************************************/
/*************** MODULE REVISION LOG ******************************************
*
*    Date    Software Version    Initials   Description
*  05/12/15    .1         RBL      Module Created.
*
*******************************************************************************/
/**
 * @file magneto.c
 * @brief This module contains the implementation of the magneto hardware
 * funtions.
 */
/******************************************************************************
* Includes
*******************************************************************************/
#include "magneto.h"
#include "magneto_hal.h"
#include <string.h>

/******************************************************************************
* Module Preprocessor Constants
*******************************************************************************/
#define MAX_DEVICES 10
#define MIN_DEVICES 1

/******************************************************************************
* Module Preprocessor Macros
*******************************************************************************/
#define MIN(a, b) (((a) < (b)) ? (a) : (b))

/******************************************************************************
* Module Typedefs
*******************************************************************************/

/******************************************************************************
* Module Variable Definitions
*******************************************************************************/
static uint8_t device_count;
static uint8_t device_addresses[MAX_DEVICES];
static int16_t zero[MAX_DEVICES];
static bool error_flag;
static magneto_mode_t mode_current;

/******************************************************************************
* Function Prototypes
*******************************************************************************/

/******************************************************************************
* Function Definitions
*******************************************************************************/


/**************************************************************************
 ************ Public Functions ********************************************
 **************************************************************************/
int magneto_init( magneto_mode_t mode, uint8_t *address,
                  uint8_t num_of_devices )
{
    if( num_of_devices > MAX_DEVICES || num_of_devices < MIN_DEVICES ||
            mode > MAGNETO_PWM )
        return -1;

    device_count = num_of_devices;
    mode_current = mode;
    memcpy( device_addresses, address, num_of_devices );

    if( magneto_hal_init( mode, *address,
                          ( mode == MAGNETO_I2C ) ? 1 : num_of_devices ) )
        return -1;

    magneto_get_zero_positions( 0 );

    if( magneto_error() )
        magneto_get_errors( 0 );

    return 0;
}


int16_t *magneto_get_angles( uint8_t device_num )
{
    int i;
    static int16_t angle_buffer[MAX_DEVICES];

    if( device_num > device_count )
        return 0;

    if( mode_current == MAGNETO_I2C )
    {
        for( i = 0; i < device_count; i++ )
        {
            if( device_addresses[i] == 0x00 )
                break;
            magneto_hal_set_slave( device_addresses[i] );
            if( magneto_hal_read( AS5048A_ANGLE, &angle_buffer[i] ) )
                error_flag = true;
        }
    }
    else
    {
        if( magneto_hal_read( AS5048A_ANGLE, angle_buffer ) )
            error_flag = true;
    }

    for( i = 0; i < device_count; i++ )
    {
        angle_buffer[i] &= ( 16384 - 31 - 1 );
        angle_buffer[i] -= zero[ i ];
    }

    if( device_num != 1 && device_num != 0 )
        angle_buffer[ 0 ] = angle_buffer[ device_num - 1 ];

    return angle_buffer;
}


float *magneto_get_degrees( uint8_t device_num )
{
    int i;
    static float degrees[MAX_DEVICES];
    int16_t *results = magneto_get_angles( 0 );

    for( i = 0; i < device_count; i++ )
        degrees[i] = ( ( *( results + i ) * 360.0f ) / 16384.0f );

    if( device_num != 1 && device_num != 0 )
        degrees[ 0 ] = degrees[ device_num - 1 ];

    return degrees;
}


float *magneto_get_radians( uint8_t device_num )
{
    int i;
    static float radians[MAX_DEVICES];
    int16_t *results = magneto_get_angles( 0 );

    for( i = 0; i < device_count; i++ )
        radians[i] = *( results + i ) * ( 3.14159265f / 180.0f );

    if( device_num != 1 && device_num != 0 )
        radians[ 0 ] = radians[ device_num - 1 ];

    return radians;
}


magneto_state_t *magneto_get_states( uint8_t device_num )
{
    int i;
    static magneto_state_t states[MAX_DEVICES];
    int16_t buffer[MAX_DEVICES];

    if( device_num > device_count )
        return 0;

    if( mode_current == MAGNETO_I2C )
    {
        for( i = 0; i < device_count; i++ )
        {
            if( device_addresses[i] == 0x00 )
                break;
            magneto_hal_set_slave( device_addresses[i] );

            if( magneto_hal_read( AS5048A_DIAG_AGC, &buffer[i] ) )
                error_flag = true;
        }
    }
    else
    {
        if( magneto_hal_read( AS5048A_DIAG_AGC, buffer ) )
            error_flag = true;
    }

    for( i = 0; i >= device_count; i-- )
    {
        buffer[i] &= 0x0f00;
        buffer[i] >>= 8;

        if( buffer[i] & 0x01 )
            states[i].ocf = 1;
        if( buffer[i] & 0x02 )
            states[i].cof = 1;
        if( buffer[i] & 0x03 )
            states[i].comp_low = 1;
        if( buffer[i] & 0x04 )
            states[i].comp_high = 1;
    }

    if( device_num != 1 && device_num != 0 )
        states[ 0 ] = states[ device_num - 1 ];

    return states;
}


void magneto_print_state( void( *print_char )( char c ) )
{
    magneto_state_t *state = magneto_get_states( 0 );
    int i = 0;

    for( i = 0; i < device_count; i++ )
    {
        if( state[i].ocf )
        {
            const char *otp = "OTP Control enabled";

            while( *( otp++ ) )
                print_char( *otp );
        }
        if( state[i].cof )
            print_char( 'c' );
        if( state[i].comp_low )
            print_char( 'l' );
        if( state[i].comp_high )
            print_char( 'h' );
    }
}


uint8_t *magneto_get_gains( uint8_t device_num )
{
    int i;
    int16_t buffer[MAX_DEVICES];
    static uint8_t gains[MAX_DEVICES];

    if( device_num > device_count )
        return 0;

    if( mode_current == MAGNETO_I2C )
    {
        for( i = 0; i < device_count; i++ )
        {
            if( device_addresses[i] == 0x00 )
                break;
            magneto_hal_set_slave( device_addresses[i] );

            if( magneto_hal_read( AS5048A_DIAG_AGC, &buffer[i] ) )
                error_flag = true;
        }
    }
    else
    {
        if( magneto_hal_read( AS5048A_DIAG_AGC, buffer ) )
            error_flag = true;
    }

    for( i = 0; i < device_count; i++ )
        gains[i] = buffer[i] & 0xff;

    if( device_num != 1 && device_num != 0 )
        gains[ 0 ] = gains[ device_num - 1 ];

    return gains;
}

magneto_error_t *magneto_get_errors( uint8_t device_num )
{
    int i;
    static magneto_error_t errors[MAX_DEVICES];
    int16_t buffer[MAX_DEVICES];

    if( device_num > device_count )
        return 0;

    if( mode_current == MAGNETO_I2C )
    {
        for( i = 0; i < device_count; i++ )
        {
            if( device_addresses[i] == 0x00 )
                break;
            magneto_hal_set_slave( device_addresses[i] );

            if( magneto_hal_read( AS5048A_CLEAR_ERROR_FLAG, &buffer[i] ) )
                error_flag = true;
        }
    }
    else
    {
        if( magneto_hal_read( AS5048A_CLEAR_ERROR_FLAG, buffer ) )
            error_flag = true;
    }

    for( i = 0; i < device_count; i++ )
        errors[i] = buffer[i] & 0x07;

    if( device_num != 1 && device_num != 0 )
        errors[ 0 ] = errors[ device_num - 1 ];

    error_flag = false;

    return errors;;
}

int16_t *magneto_get_zero_positions( uint8_t device_num )
{
    int16_t high[MAX_DEVICES];
    int16_t low[MAX_DEVICES];

    int i;

    if( mode_current == MAGNETO_I2C )
    {
        for( i = 0; i < device_count; i++ )
        {
            if( device_addresses[i] == 0x00 )
                break;
            magneto_hal_set_slave( device_addresses[i] );

            if( magneto_hal_read( AS5048A_OTP_REGISTER_ZERO_POS_HIGH, &high[i] ) ||
                    magneto_hal_read( AS5048A_OTP_REGISTER_ZERO_POS_LOW, &low[i] ) )
                error_flag = true;
        }
    }
    else
    {
        if( magneto_hal_read( AS5048A_OTP_REGISTER_ZERO_POS_HIGH, high ) ||
                magneto_hal_read( AS5048A_OTP_REGISTER_ZERO_POS_LOW, low ) )
            error_flag = true;
    }

    for( i = 0; i < device_count; i++ )
    {
        zero[i] = high[i] << 6;
        zero[i] |= low[i] & 0x1f;
    }

    if( device_num != 1 && device_num != 0 )
        zero[ 0 ] = zero[ device_num - 1 ];

    return zero;
}

bool magneto_error()
{
    return error_flag;
}


/*************** END OF FUNCTIONS ***************************************************************************/
