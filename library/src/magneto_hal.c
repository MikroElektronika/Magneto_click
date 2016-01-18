/*******************************************************************************
* Title                 :   Hal I2C
* Filename              :   hal_i2c.c
* Author                :   MSV
* Origin Date           :   02/11/2015
* Notes                 :   None
*******************************************************************************/
/*************** MODULE REVISION LOG ******************************************
*
*    Date    Software Version    Initials   Description
*  02/11/15    XXXXXXXXXXX         MSV      Module Created.
*
*******************************************************************************/
/**
 * @file magneto_hal.c
 *
 * @brief Hardware Access Layer for magneto click.
 */
/******************************************************************************
* Includes
*******************************************************************************/
#include "magneto_hal.h"
#include <stddef.h>
#include <string.h>
#include <stdio.h>

/******************************************************************************
* Module Preprocessor Constants
*******************************************************************************/
#define WRITE 0
#define READ 1
/******************************************************************************
* Module Preprocessor Macros
*******************************************************************************/

/******************************************************************************
* Module Typedefs
*******************************************************************************/

/******************************************************************************
* Module Variable Definitions
*******************************************************************************/
#if defined( __MIKROC_PRO_FOR_ARM__ )
#if defined( STM32F107VC ) || defined( STM32F407VG ) || \
    defined( STM32F030C6 ) || defined( STM32F746VG )
static unsigned int( *i2c_start_p )( void );
static unsigned int( *i2c_write_p )( unsigned char slave_address,
                                     unsigned char *buffer,
                                     unsigned long count,
                                     unsigned long end_mode );
static void( *i2c_read_p )( unsigned char slave_address,
                            unsigned char *buffer,
                            unsigned long count,
                            unsigned long end_mode );
static unsigned int ( *spi_read_p )( unsigned int buffer );
static void ( *spi_write_p )( unsigned int data_out );

#elif defined( LM3S1165 ) || defined( TM4C129ENCZAD )
static void ( *i2c_set_slave_address_p )( unsigned char slave_address,
        unsigned char dir );
static unsigned char ( *i2c_write_p )( unsigned char data_out,
                              unsigned char mode );
static unsigned char ( *i2c_read_p )( char *data_in,
                             char mode );

static unsigned int ( *spi_read_p )( unsigned int buffer );
static void ( *spi_write_p )( unsigned int data_out );
#endif

#elif defined( __MIKROC_PRO_FOR_AVR__ )
static unsigned char( *i2c_busy_p )( void );
static unsigned char( *i2c_status_p )( void );
static unsigned char( *i2c_start_p )( void );
static void( *i2c_stop_p )( void );
static void( *i2c_close_p )( void );
static void( *i2c_write_p )( unsigned char data_out );
static unsigned char( *i2c_read_p )( unsigned char ack );
static unsigned char( *spi_read_p )( unsigned char data_in );
static void( *spi_write_p )( unsigned char data_out );

#elif defined( __MIKROC_PRO_FOR_PIC__ )
static unsigned char( *i2c_is_idle_p )( void );
static unsigned char( *i2c_start_p )( void );
static void( *i2c_stop_p )( void );
static void( *i2c_restart_p )( void );
static unsigned char( *i2c_write_p )( unsigned char data_out );
static unsigned char( *i2c_read_p )( unsigned char ack );
static unsigned char( *spi_read_p )( unsigned char _data );
static void ( *spi_write_p )( unsigned char data_out );

#elif defined( __MIKROC_PRO_FOR_PIC32__ )
static unsigned int( *i2c_is_idle_p )( void );
static unsigned int( *i2c_start_p )( void );
static void( *i2c_stop_p )( void );
static unsigned int( *i2c_restart_p )( void );
static unsigned int( *i2c_write_p )( unsigned char data_out );
static unsigned char( *i2c_read_p )( unsigned int ack );
static unsigned long( *spi_read_p )( unsigned long buffer );
static void ( *spi_write_p )( unsigned long data_out );

#elif defined( __MIKROC_PRO_FOR_DSPIC__ )
static unsigned int( *i2c_is_idle_p )( void );
static unsigned int( *i2c_start_p )( void );
static void( *i2c_stop_p )( void );
static void( *i2c_restart_p )( void );
static unsigned int( *i2c_write_p )( unsigned char data_out );
static unsigned char( *i2c_read_p )( unsigned int ack );
static unsigned int( *spi_read_p )( unsigned int buffer );
static void ( *spi_write_p )( unsigned int data_out );

#elif defined( __MIKROC_PRO_FOR_8051__ )
static unsigned char( *i2c_busy_p )( void );
static unsigned char ( *i2c_status_p )( void );
static unsigned char( *i2c_start_p )( void );
static void( *i2c_stop_p )( void );
static void( *i2c_close_p )( void );
static void( *i2c_write_p )( unsigned char data_out );
static unsigned char( *i2c_read_p )( unsigned char ack );

#elif defined( __MIKROC_PRO_FOR_FT90x__ )
static void( *i2c_soft_reset_p )( void );
static void( *i2c_set_slave_address_p )( unsigned char slave_address );
static unsigned char( *i2c_write_p )( unsigned char data_out );
static unsigned char( *i2c_read_p )( unsigned char *data_in );
static unsigned char( *i2c_write_bytes_p )( unsigned char *buffer,
        unsigned int count );
static unsigned char( *i2c_read_bytes_p )( unsigned char *buffer,
        unsigned int count );
static unsigned char( *i2c_write_10bit_p )( unsigned char data_out,
        unsigned int address_10bit );
static unsigned char( *i2c_read_10bit_p )( unsigned char *data_in,
        unsigned int address_10bit );
static unsigned char( *spi_read_p )( unsigned char dummy );
static void( *spi_write_p )( unsigned char dataOut );
#elif defined ( __GNUC__ )

#define spi_write_p
#define spi_read_p

#endif

#if defined( __MIKROC_PRO_FOR_ARM__ )   || \
    defined( __MIKROC_PRO_FOR_AVR__ )   || \
    defined( __MIKROC_PRO_FOR_PIC__ )   || \
    defined( __MIKROC_PRO_FOR_PIC32__ ) || \
    defined( __MIKROC_PRO_FOR_DSPIC__ ) || \
    defined( __MIKROC_PRO_FOR_8051__ )  || \
    defined( __MIKROC_PRO_FOR_FT90x__ )
extern sfr sbit MAGNETO_CS;
#endif

static magneto_mode_t current_mode;
static uint8_t i2c_address;
static uint8_t device_count;

/******************************************************************************
* Function Prototypes
*******************************************************************************/
static void cs_low( void );
static void cs_high( void );
static uint8_t parity_check( uint16_t data_in );

#if defined( __GNUC__ )
const char *byte_to_binary( int x );

const char *byte_to_binary( int x )
{
    static char b[9];
    b[0] = '\0';

    int z;

    for ( z = 128; z > 0; z >>= 1 )
        strcat( b, ( ( x & z ) == z ) ? "1" : "0" );

    return b;
}
#endif


/******************************************************************************
* Function Definitions
*******************************************************************************/
static void cs_low()
{
#ifdef __GNUC__
    printf( "CS_LOW\n" );
#else
    MAGNETO_CS = 0;
    Delay_1us();  // tL Time between CS falling edge and CLK rising edge 350ns
#endif
}

static void cs_high()
{
#ifdef __GNUC__
    printf( "CS_HIGH\n" );
#else
    //Delay_1us();  // tH Time between last falling edge of CLK and rising edge of CS 50ns
    MAGNETO_CS = 1;
    Delay_1us();  // TcsnH High time of CS bwtween two transmissions 350ns
#endif
}

static uint8_t parity_check( uint16_t data_in )
{
    uint8_t count = 0;
    int i;

    for( i = 0; i < 16; i++ )
    {
        if( data_in & 0x1 )
            count++;

        data_in >>= 1;
    }

    return count & 0x1;
}

int magneto_hal_init( magneto_mode_t mode, uint8_t address_id,
                      uint8_t num_devices )
{
    if( mode > MAGNETO_PWM )
        return -1;

    current_mode = mode;
    device_count = num_devices;

    switch( current_mode )
    {
        case MAGNETO_I2C:
#if defined( __MIKROC_PRO_FOR_ARM__ ) || defined(__MIKROC_PRO_FOR_FT90x__)
            i2c_address = address_id;
#else
            i2c_address = ( address_id << 1 );
#endif

#if defined( __MIKROC_PRO_FOR_ARM__ )
#if defined( STM32F107VC ) || defined( STM32F407VG ) || \
    defined( STM32F030C6 ) || defined( STM32F746VG )
            i2c_start_p = I2C_Start_Ptr;
            i2c_write_p = I2C_Write_Ptr;
            i2c_read_p = I2C_Read_Ptr;
#elif defined( LM3S1165 ) || defined( TM4C129ENCZAD )
            i2c_set_slave_address_p = I2C_Master_Slave_Addr_Set_Ptr;
            i2c_write_p = I2C_Write_Ptr;
            i2c_read_p = I2C_Read_Ptr;
#endif
#elif defined( __MIKROC_PRO_FOR_AVR__ )
   #if defined( ATMEGA32 )
            i2c_busy_p = TWI_Busy;
            i2c_status_p = TWI_Status;
            i2c_close_p = TWI_Close;
            i2c_start_p = TWI_Start;
            i2c_stop_p = TWI_Stop;
            i2c_write_p = TWI_Write;
            i2c_read_p = TWI_Read;
   #elif defined ( ATXMEGA128A1 )
            i2c_busy_p = TWIC_Busy;
            i2c_status_p = TWIC_Status;
            i2c_close_p = TWIC_Close;
            i2c_start_p = TWIC_Start;
            i2c_stop_p = TWIC_Stop;
            i2c_write_p = TWIC_Write;
            i2c_read_p = TWIC_Read;
   #endif
#elif defined( __MIKROC_PRO_FOR_PIC__ )
            i2c_is_idle_p = I2C1_Is_Idle;
            i2c_start_p = I2C1_Start;
            i2c_stop_p = I2C1_Stop;
            i2c_restart_p = I2C1_Repeated_Start;
            i2c_write_p = I2C1_Wr;
            i2c_read_p = I2C1_Rd;
#elif defined( __MIKROC_PRO_FOR_PIC32__ )
            i2c_is_idle_p = I2C_Is_Idle_Ptr;
            i2c_start_p = I2C_Start_Ptr;
            i2c_stop_p = I2C_Stop_Ptr;
            i2c_restart_p = I2C_Restart_Ptr;
            i2c_write_p = I2C_Write_Ptr;
            i2c_read_p = I2C_Read_Ptr;
#elif defined( __MIKROC_PRO_FOR_DSPIC__ )
            i2c_is_idle_p = I2C1_Is_Idle;
            i2c_start_p = I2C1_Start;
            i2c_stop_p = I2C1_Stop;
            i2c_restart_p = I2C1_Restart;
            i2c_write_p = I2C1_Write;
            i2c_read_p = I2C1_Read;
#elif defined( __MIKROC_PRO_FOR_8051__ )
            i2c_busy_p = TWI_Busy;
            i2c_status_p = TWI_Status;
            i2c_close_p = TWI_Close;
            i2c_start_p = TWI_Start;
            i2c_stop_p = TWI_Stop;
            i2c_write_p = TWI_Write;
            i2c_read_p = TWI_Read;
#elif defined( __MIKROC_PRO_FOR_FT90x__ )
            i2c_soft_reset_p = I2CM_Soft_Reset_Ptr;
            i2c_set_slave_address_p = I2CM_Set_Slave_Address_Ptr;
            i2c_write_p = I2CM_Write_Ptr;
            i2c_read_p = I2CM_Read_Ptr;
            i2c_write_bytes_p = I2CM_Write_Bytes_Ptr;
            i2c_read_bytes_p = I2CM_Read_Bytes_Ptr;
            i2c_write_10bit_p = I2CM_Write_10Bit_Ptr;
            i2c_read_10bit_p = I2CM_Read_10Bit_Ptr;
#endif
            break;
        case MAGNETO_SPI:
#if defined( __GNUC__ )

#else
#if defined( __MIKROC_PRO_FOR_ARM__ )   || \
            defined( __MIKROC_PRO_FOR_AVR__ )   || \
            defined( __MIKROC_PRO_FOR_PIC__ )   || \
            defined( __MIKROC_PRO_FOR_PIC32__ ) || \
            defined( __MIKROC_PRO_FOR_DSPIC__ ) || \
            defined( __MIKROC_PRO_FOR_8051__ )
            spi_read_p = SPI_Rd_Ptr;
            spi_write_p = SPI_Wr_Ptr;
            cs_high();
#elif defined( __MIKROC_PRO_FOR_FT90x__ )
            spi_read_p = SPIM_Rd_Ptr;
            spi_write_p = SPIM_Wr_Ptr;
            cs_high();
#endif
#endif
            break;
        case MAGNETO_PWM:

            break;
    };

#ifdef __GNUC__

#else
#if defined( __MIKROC_PRO_FOR_ARM__ )   || \
        defined( __MIKROC_PRO_FOR_AVR__ )   || \
        defined( __MIKROC_PRO_FOR_PIC__ )   || \
        defined( __MIKROC_PRO_FOR_PIC32__ ) || \
        defined( __MIKROC_PRO_FOR_DSPIC__ ) || \
        defined( __MIKROC_PRO_FOR_8051__ )  || \
        defined( __MIKROC_PRO_FOR_FT90x__ )
    Delay_10ms();
#endif
#endif
    return 0;
}

void magneto_hal_set_slave( uint8_t slave_address )
{
#if defined( __MIKROC_PRO_FOR_ARM__ ) || defined(__MIKROC_PRO_FOR_FT90x__)
    i2c_address = slave_address;
#else
    i2c_address = ( slave_address << 1 );
#endif
}

int16_t magneto_hal_write( uint16_t address, int16_t value )
{
    uint16_t command;

    switch( current_mode )
    {
        case MAGNETO_I2C:
        {
            uint8_t buffer[ AS5048A_MAX_READ_SIZE ];
            buffer[1] = ( uint8_t )value & 0xff;

            switch( address )
            {
                case AS5048A_CLEAR_ERROR_FLAG:

                    break;
                case AS5048A_PROGRAMMING_CONTROL:
                    buffer[0] = 0x03;
                    break;
                case AS5048A_OTP_REGISTER_ZERO_POS_HIGH:
                    buffer[0] = 0x16;
                    break;
                case AS5048A_OTP_REGISTER_ZERO_POS_LOW:
                    buffer[0] = 0x17;
                    break;
            }
#if defined( __MIKROC_PRO_FOR_ARM__ )
#if defined( STM32F107VC ) || defined( STM32F407VG ) || \
    defined( STM32F030C6 ) || defined( STM32F746VG )
            i2c_start_p();
            i2c_write_p( i2c_address,
                         buffer,
                         2,
                         END_MODE_STOP );
#elif defined( LM3S1165 ) || defined( TM4C129ENCZAD )
      i2c_set_slave_address_p( i2c_address, _I2C_DIR_MASTER_TRANSMIT );
      i2c_write_p( buffer[0], _I2C_MASTER_MODE_BURST_SEND_START );
      i2c_write_p( buffer[1], _I2C_MASTER_MODE_BURST_SEND_FINISH );
#endif
#elif defined(__MIKROC_PRO_FOR_FT90x__)
            i2c_set_slave_address_p( i2c_address );
            //TODO: Send bytes
            i2c_write_p( buffer[0] );
            i2c_write_p( buffer[1] );

#elif defined(__MIKROC_PRO_FOR_AVR__)   || \
  defined(__MIKROC_PRO_FOR_8051__)  || \
  defined(__MIKROC_PRO_FOR_DSPIC__) || \
  defined(__MIKROC_PRO_FOR_PIC32__) || \
  defined(__MIKROC_PRO_FOR_PIC__)
            i2c_start_p();
            i2c_write_p( i2c_address | WRITE );
            i2c_write_p( buffer[0] );
            i2c_write_p( buffer[1] );
            i2c_stop_p();

#elif defined( __GNUC__)
            printf( "Start\n" );
            printf( "Address: 0x%02x\n", address );
            printf( "\tData: 0x%02x\n", value );
#endif
        }
        break;
        case MAGNETO_SPI:

#if defined ( __GNUC__ )
            cs_low();
            printf( "%s\n", byte_to_binary( command ) ); // Write command
            cs_high();
            cs_low();
            printf( "%s\n", byte_to_binary( value ) );
            cs_high();
#else
            command = 0x00 | ( address & 0x3FFF );
            command |= ( ( int16_t )parity_check( command ) << AS5048A_PARITY_BIT );

            if( device_count == 1 )
            {
                cs_low();
                spi_write_p( ( command >> 8 ) & 0xff );
                spi_write_p( command & 0xff );
                cs_high();

                value = 0x00 | ( value & 0x3FFF );
                value |= ( ( int16_t )parity_check( value ) << AS5048A_PARITY_BIT );

                cs_low();
                spi_write_p( ( value >> 8 ) & 0xff );
                spi_write_p( value & 0xff );
                cs_high();

                cs_low();
                command = ( spi_read_p( 0x00 ) << 8 );
                command |= spi_read_p( 0x00 );
                cs_high();
            }
            else
            {
                uint8_t i;

                cs_low();
                for( i = 0; i < device_count; i++ )
                {
                    spi_write_p( ( command >> 8 ) & 0xff );
                    spi_write_p( command & 0xff );
                }
                cs_high();

                value = 0x00 | ( value & 0x3FFF );
                value |= ( ( int16_t )parity_check( value ) << AS5048A_PARITY_BIT );

                cs_low();
                for( i = 0; i < device_count; i++ )
                {
                    spi_write_p( ( value >> 8 ) & 0xff );
                    spi_write_p( value & 0xff );
                }
                cs_high();

                cs_low();
                for( i = 0; i < device_count; i++ )
                {
                    command = ( spi_read_p( 0x00 ) << 8 );
                    command |= spi_read_p( 0x00 );
                }
                cs_high();
            }
#endif
            break;
        case MAGNETO_PWM:

            break;
    };

    return command & 0x3FFF;
}


bool magneto_hal_read( uint16_t address, int16_t *buff )
{
    uint16_t command;
    bool error = false;

    switch( current_mode )
    {
        case MAGNETO_I2C:
        {
            uint8_t buffer[2];

            switch( address )
            {
                case AS5048A_CLEAR_ERROR_FLAG:

                    break;
                case AS5048A_PROGRAMMING_CONTROL:
                    buffer[0] = 0x03;
                    buffer[1] = 0x00;
                    break;
                case AS5048A_OTP_REGISTER_ZERO_POS_HIGH:
                    buffer[0] = 0x16;
                    buffer[1] = 0x00;
                    break;
                case AS5048A_OTP_REGISTER_ZERO_POS_LOW:
                    buffer[0] = 0x17;
                    buffer[1] = 0x00;
                    break;
                case AS5048A_DIAG_AGC:
                    buffer[0] = 0xFA; // AGC
                    buffer[1] = 0xFB;
                    break;
                case AS5048A_MAGNITUDE:
                    buffer[0] = 0xFC;
                    buffer[1] = 0xFD;
                    break;
                case AS5048A_ANGLE:
                    buffer[0] = 0xFE;
                    buffer[1] = 0xFF;
                    break;
            }

#if defined( __MIKROC_PRO_FOR_ARM__ )
#if defined( STM32F107VC ) || defined( STM32F407VG ) || \
    defined( STM32F030C6 ) || defined( STM32F746VG )
            i2c_start_p();              // issue I2C start signal
            i2c_write_p( i2c_address,
                         buffer,
                         1,
                         END_MODE_RESTART );

            i2c_read_p( i2c_address,
                        ( uint8_t * )buff,
                        ( buffer[1] ) ? 2 : 1,
                        END_MODE_STOP );
    #elif defined( LM3S1165 ) || defined( TM4C129ENCZAD )
          i2c_set_slave_address_p( i2c_address, _I2C_DIR_MASTER_RECEIVE );
          i2c_write_p( buffer[0], _I2C_MASTER_MODE_SINGLE_SEND );
          i2c_read_p( buff, _I2C_MASTER_MODE_BURST_RECEIVE_START );
    #endif
#elif defined(__MIKROC_PRO_FOR_FT90x__)
            i2c_set_slave_address_p( i2c_address );

            i2c_write_p( buffer[0] );
            i2c_read_bytes_p( ( uint8_t * )buff,
                              ( buffer[1] != 0x00 ) ? 2 : 1 );

#elif defined(__MIKROC_PRO_FOR_AVR__) || defined(__MIKROC_PRO_FOR_PIC32__)
            i2c_start_p();
            i2c_write_p( i2c_address | WRITE );
            i2c_write_p( buffer[0] );
            i2c_start_p();

            i2c_write_p( i2c_address | READ );

            if( buffer[1] )
                *( buff++ ) = i2c_read_p( 1 );

            *buff = i2c_read_p( 0 );
            i2c_stop_p();

#elif defined(__MIKROC_PRO_FOR_PIC__)  || defined(__MIKROC_PRO_FOR_DSPIC__)
            i2c_start_p();
            i2c_write_p( i2c_address | WRITE );
            i2c_write_p( buffer[0] );
            i2c_restart_p();
            i2c_write_p( i2c_address | READ );

            if( buffer[1] )
                *( buff++ ) = i2c_read_p( 1 );

            *buff = i2c_read_p( 0 );
            i2c_stop_p();

#elif defined( __GNUC__ )
            int input;

            printf( "Read from address 0x%02x\n", buffer[0] );

            scanf( "%d", &input );
            *( buff ) = input;
            *( buff ) <<= 8;
            scanf( "%d", &input );
            *( ++buff ) |= input;
#endif
        }
        break;
        case MAGNETO_SPI:
            command = 0x4000 | ( address & 0x3FFF );
            command |= ( ( uint16_t )parity_check( command ) << AS5048A_PARITY_BIT );
#if defined( __GNUC__ )
            cs_low();
            printf( "%s\n", byte_to_binary( command ) ); // Write command
            cs_high();
            printf( "NOP\n" );
#else
            if( device_count == 1 )
            {
                cs_low();
                spi_write_p( ( command >> 8 ) & 0xff );
                spi_write_p( command & 0xff );
                cs_high();

                cs_low();
                command = ( spi_read_p( AS5048A_NOP ) << 8 );
                command |= ( spi_read_p( AS5048A_NOP ) & 0xff );
                cs_high();

                if( command & AS5048A_ERRORFLAG_MASK )
                    error = true;

                *buff = command & 0x3FFF;
            }
            else
            {
                uint8_t devices = device_count;

                cs_low();

                while( devices-- )
                {
                    spi_write_p( ( command >> 8 ) & 0xff );
                    spi_write_p( command & 0xff );
                }

                cs_high();

                devices = device_count;
                cs_low();
                while( devices-- )
                {
                    command = ( spi_read_p( AS5048A_NOP ) << 8 );
                    command |= ( spi_read_p( AS5048A_NOP ) & 0xff );
                    *( buff + ( devices - 1 ) ) = command & 0x3FFF;

                    if( command & AS5048A_ERRORFLAG_MASK )
                        error = true;
                }
                cs_high();
            }
#endif
            break;
        case MAGNETO_PWM:

            break;
    };

    return error;
}



/*************** END OF FUNCTIONS *********************************************/