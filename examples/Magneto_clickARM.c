/*****************************************************************************
* Title                 :   Magneto click Example
* Filename              :   Magneto_clickARM.c
* Author                :   RBL
* Origin Date           :   29/12/2015
* Notes                 :   None
*****************************************************************************/
/*************** MODULE REVISION LOG *****************************************
*
*    Date    Software Version    Initials   Description
*  29/12/15           .1         RBL        Module Created.
*
*****************************************************************************/
/**
 * @file Magneto_clickARM.c
 *
 * @brief This module contains the example for the Magneto click and various
 * functions contained in the Magneto library.
 */
/*****************************************************************************
 * Includes
 *******************************************************************************/
#include "magneto.h"
#include "built_in.h"
#include "resources.h"
#include <stdint.h>

/******************************************************************************
* Module Preprocessor Constants
*******************************************************************************/
#define INNER_RADIUS 20.0f
#define OUTER_RADIUS 60.0f
#define X_CENTER     160.0f
#define Y_CENTER     110.0f
#define PI           3.14159265f

/******************************************************************************
* Module Preprocessor Macros
*******************************************************************************/

/******************************************************************************
* Module Typedefs
*******************************************************************************/
/**
 * @struct Line beginning and end points in X and Y
 */
typedef struct
{
    int x1;
    int y1;
    int x2;
    int y2;
} line_t;

/*****************************************************************************
 * Module Variable Definitions
 ****************************************************************************/
sbit MAGNETO_CS at GPIOD_ODR.B13;

// TFT module connections
unsigned int TFT_DataPort at GPIOE_ODR;
sbit TFT_RST at GPIOE_ODR.B8;
sbit TFT_RS at GPIOE_ODR.B12;
sbit TFT_CS at GPIOE_ODR.B15;
sbit TFT_RD at GPIOE_ODR.B10;
sbit TFT_WR at GPIOE_ODR.B11;
sbit TFT_BLED at GPIOE_ODR.B9;
// End TFT module connections

static bool volatile update_flag;

/*****************************************************************************
 * Function Prototypes
 ****************************************************************************/
static void init_timer2( void );
static void system_init( void );
static void tft_initialize( void );
static void tft_update( void );

/*****************************************************************************
 * Function Implementations
 ****************************************************************************/

static void init_timer2()
{
    RCC_APB1ENR.TIM2EN = 1;
    TIM2_CR1.CEN = 0;
    TIM2_PSC = 239;  // 100ms
    TIM2_ARR = 62499;
    NVIC_IntEnable( IVT_INT_TIM2 );
    TIM2_DIER.UIE = 1;
    TIM2_CR1.CEN = 1;
}

static void tft_initialize()
{
    TFT_Init_ILI9341_8bit( 320, 280 );
    TFT_BLED = 1;
    TFT_Set_Default_Mode();
    TFT_Set_Pen( CL_WHITE, 1 );
    TFT_Set_Brush( 1, CL_WHITE, 0, 0, 0, 0 );
    TFT_Set_Font( TFT_defaultFont, CL_BLACK, FO_HORIZONTAL );

    TFT_Fill_Screen( CL_WHITE );
    TFT_Set_Pen( CL_BLACK, 1 );
    TFT_Line( 20, 222, 300, 222 );
    TFT_Set_Font( &Verdana12x13_Regular, CL_BLACK, FO_HORIZONTAL );
    TFT_Write_Text( "EasyMx PRO v7 for STM32", 19, 223 );
    TFT_Set_Font( &Verdana12x13_Regular, CL_RED, FO_HORIZONTAL );
    TFT_Write_Text( "www.mikroe.com", 200, 223 );
    TFT_Image( ( 320 - 220 ) / 2, 0, rudderangle_bmp, 1 );
    TFT_Set_Font( TFT_defaultFont, CL_BLACK, FO_HORIZONTAL );
}

static void tft_update()
{
    static float last_angle;
    static line_t last_line;
    float *read_angle;
    line_t current_line;
    float total = 0;
    char tmp_txt[20];
    int i;

    for( i = 0; i < 10; i++ )
    {
        read_angle = magneto_get_degrees( 1 );

        if( magneto_error() )
        {
            magneto_error_t *error = magneto_get_errors( 1 );
            TFT_Set_Font( TFT_defaultFont, CL_RED, FO_HORIZONTAL );
            TFT_Write_Text( "ERROR", 138, 25 );
        }
        total += *read_angle;
    }

    total /= 10.0f;

    TFT_Set_Font( TFT_defaultFont, CL_WHITE, FO_HORIZONTAL );
    TFT_Write_Text( "ERROR", 138, 25 );
    sprintf( tmp_txt, "%3.1f", last_angle );
    TFT_Write_Text( tmp_txt, 10, 25 );
    TFT_Set_Font( TFT_defaultFont, CL_BLACK, FO_HORIZONTAL );
    sprintf( tmp_txt, "%3.1f", total );
    TFT_Write_Text( tmp_txt, 10, 25 );

    last_angle = total;

    TFT_Set_Pen( CL_WHITE, 4 );
    TFT_Line( last_line.x1, last_line.y1, last_line.x2, last_line.y2 );
    TFT_Set_Pen( CL_RED, 4 );

    if( total <= 90.0f )
    {
        current_line.x1 = ( int )( X_CENTER + ( INNER_RADIUS * sin( 90.0f *
                                                ( PI / 180.0f ) ) ) );
        current_line.y1 = ( int )( Y_CENTER + ( INNER_RADIUS * -cos( 90.0f *
                                                ( PI  / 180.0f ) ) ) );
        current_line.x2 = ( int )( X_CENTER + ( OUTER_RADIUS * sin( 90.0f *
                                                ( PI / 180.0f ) ) ) );
        current_line.y2 = ( int )( Y_CENTER + ( OUTER_RADIUS * -cos( 90.0f *
                                                ( PI  / 180.0f ) ) ) );
    }
    else if( total >= 270.0f )
    {
        current_line.x1 = ( int )( X_CENTER + ( INNER_RADIUS * sin( 270.0f *
                                                ( PI / 180.0f ) ) ) );
        current_line.y1 = ( int )( Y_CENTER + ( INNER_RADIUS * -cos( 270.0f *
                                                ( PI  / 180.0f ) ) ) );
        current_line.x2 = ( int )( X_CENTER + ( OUTER_RADIUS * sin( 270.0f *
                                                ( PI / 180.0f ) ) ) );
        current_line.y2 = ( int )( Y_CENTER + ( OUTER_RADIUS * -cos( 270.0f *
                                                ( PI  / 180.0f ) ) ) );
    }
    else
    {
        current_line.x1 = ( int )( X_CENTER + ( INNER_RADIUS * sin( total *
                                                ( PI / 180.0f ) ) ) );
        current_line.y1 = ( int )( Y_CENTER + ( INNER_RADIUS * -cos( total *
                                                ( PI  / 180.0f ) ) ) );
        current_line.x2 = ( int )( X_CENTER + ( OUTER_RADIUS * sin( total *
                                                ( PI / 180.0f ) ) ) );
        current_line.y2 = ( int )( Y_CENTER + ( OUTER_RADIUS * -cos( total *
                                                ( PI  / 180.0f ) ) ) );
    }

    TFT_Line( current_line.x1, current_line.y1, current_line.x2,
              current_line.y2 );
    memcpy( &last_line, &current_line, sizeof( line_t ) );

    update_flag = false;
}

// Initialize all system peripherals
static void system_init()
{
    DisableInterrupts();

    GPIO_Digital_Output( &GPIOD_BASE, _GPIO_PINMASK_13 );
    /* SPI Init - NOTE* Magneto samples bits on the second clock transition */
    SPI3_Init_Advanced( _SPI_FPCLK_DIV16,
                        _SPI_MASTER | _SPI_8_BIT | _SPI_CLK_IDLE_LOW |
                        _SPI_SECOND_CLK_EDGE_TRANSITION | _SPI_MSB_FIRST |
                        _SPI_SS_DISABLE | _SPI_SSM_ENABLE | _SPI_SSI_1,
                        &_GPIO_MODULE_SPI3_PC10_11_12 );
    // Initialize display
    tft_initialize();
    // Start timer
    init_timer2();
    // Initialize magneto and check for success or failure
    if( magneto_init( MAGNETO_SPI, 0, 1 ) )
        return;
}

void main()
{
    system_init();
    EnableInterrupts();

    while( 1 )
    {
        if( update_flag )
            tft_update();
    }
}

// ISR timer for signaling
void timer2_interrupt() iv IVT_INT_TIM2
{
    TIM2_SR.UIF = 0;
    update_flag = true;
}

/*************** END OF FUNCTIONS ***************************************/
