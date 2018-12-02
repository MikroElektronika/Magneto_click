![MikroE](http://www.mikroe.com/img/designs/beta/logo_small.png)

---

# Magneto Click

- **CIC Prefix**  : MAGNETO
- **Author**      : Nenad Filipovic
- **Verison**     : 1.0.0
- **Date**        : Oct 2018.

---

### Software Support

We provide a library for the Magneto Click on our [LibStock](https://libstock.mikroe.com/projects/view/1681/magneto-click) 
page, as well as a demo application (example), developed using MikroElektronika 
[compilers](http://shop.mikroe.com/compilers). The demo can run on all the main 
MikroElektronika [development boards](http://shop.mikroe.com/development-boards).

**Library Description**

The library covers all the necessary functions to control Magneto Click board. 
Library performs the communication with the device via SPI driver by writting to registers and by reading from registers.
Library measures 14-bit value from register and calculate the absolute position of the magnet’s rotation angle degrees [ ° ].

Key functions :

- ``` uint16_t magneto_readData( uint16_t addressCommand ) ``` - Generic read - 16-bit command  function
- ``` void magneto_writeData( uint16_t addressCommand, uint16_t writeData ) ``` - Generic write function - 16-bit command & write data
- ``` float magneto_calculateAngle() ``` - Calculate angle function

**Examples Description**

The application is composed of three sections :

- System Initialization - Initializes SPI, GPIO and LOG structures, set CS and PWM pins as output.
- Application Initialization - Initialization driver enable's - SPI,
     check state register and start write log.
- Application Task - (code snippet) This is a example which demonstrates the use of Magneto Click board.
     Magneto Click communicates with register via SPI by write and read from register
     and calculate float angle value.
     Results are being sent to the Usart Terminal where you can track their changes.
     All data logs on usb uart for aproximetly every 3 sec.


```.c

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

```



The full application code, and ready to use projects can be found on our 
[LibStock](https://libstock.mikroe.com/projects/view/1681/magneto-click) page.

Other mikroE Libraries used in the example:

- SPI
- UART
- Conversions


**Additional notes and informations**

Depending on the development board you are using, you may need 
[USB UART click](http://shop.mikroe.com/usb-uart-click), 
[USB UART 2 Click](http://shop.mikroe.com/usb-uart-2-click) or 
[RS232 Click](http://shop.mikroe.com/rs232-click) to connect to your PC, for 
development systems with no UART to USB interface available on the board. The 
terminal available in all Mikroelektronika 
[compilers](http://shop.mikroe.com/compilers), or any other terminal application 
of your choice, can be used to read the message.

---
### Architectures Supported

#### mikroC

| STM | KIN | CEC | MSP | TIVA | PIC | PIC32 | DSPIC | AVR | FT90x |
|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|
| x | x | x | x | x | x | x | x | x | x |

#### mikroBasic

| STM | KIN | CEC | MSP | TIVA | PIC | PIC32 | DSPIC | AVR | FT90x |
|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|
| x | x | x | x | x | x | x | x | x | x |

#### mikroPascal

| STM | KIN | CEC | MSP | TIVA | PIC | PIC32 | DSPIC | AVR | FT90x |
|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|
| x | x | x | x | x | x | x | x | x | x |

---
---
