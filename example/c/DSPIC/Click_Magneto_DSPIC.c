/*
Example for Magneto Click

    Date          : Oct 2018.
    Author        : Nenad Filipovic

Test configuration dsPIC :
    
    MCU                : P33FJ256GP710A
    Dev. Board         : EasyPIC Fusion v7
    dsPIC Compiler ver : v7.1.0.0

---

Description :

The application is composed of three sections :

- System Initialization - Initializes SPI, GPIO and LOG structures, set CS and PWM pins as output.
- Application Initialization - Initialization driver enable's - SPI,
     check state register and start write log.
- Application Task - (code snippet) This is a example which demonstrates the use of Magneto Click board.
     Magneto Click communicates with register via SPI by write and read from register
     and calculate float angle value.
     Results are being sent to the Usart Terminal where you can track their changes.
     All data logs on usb uart for aproximetly every 3 sec.

Additional Functions :

- UART
- Conversions

*/

#include "Click_Magneto_types.h"
#include "Click_Magneto_config.h"


float angleValue;
char logText[50];

void systemInit()
{
    mikrobus_gpioInit( _MIKROBUS1, _MIKROBUS_CS_PIN, _GPIO_OUTPUT );
    mikrobus_gpioInit( _MIKROBUS1, _MIKROBUS_PWM_PIN, _GPIO_OUTPUT );

    mikrobus_spiInit( _MIKROBUS1, &_MAGNETO_SPI_CFG[0] );

    mikrobus_logInit( _MIKROBUS2, 9600 );

    Delay_100ms();
}

void applicationInit()
{
    magneto_spiDriverInit( (T_MAGNETO_P)&_MIKROBUS1_GPIO, (T_MAGNETO_P)&_MIKROBUS1_SPI );

    mikrobus_logWrite( "-------------------------", _LOG_LINE );

    if ( magneto_getState() != 1 )
    {
        mikrobus_logWrite( "      Magneto Click      ", _LOG_LINE );
    }
    else
    {
        mikrobus_logWrite( "          ERROR          ", _LOG_LINE );
    }

    mikrobus_logWrite( "-------------------------", _LOG_LINE );
    Delay_100ms();
}

void applicationTask()
{
    angleValue = magneto_calculateAngle();

    FloatToStr( angleValue, logText );
    mikrobus_logWrite( "    Angle : ", _LOG_TEXT );
    mikrobus_logWrite( logText, _LOG_TEXT );
    mikrobus_logWrite( "°", _LOG_LINE );
    mikrobus_logWrite( "-------------------------", _LOG_LINE );

    Delay_1sec();
    Delay_1sec();
    Delay_1sec();
}

void main()
{
    systemInit();
    applicationInit();

    while (1)
    {
            applicationTask();
    }
}