//Easy MX PRO v7 for STM32 ARM
//STM32F107VG 120MHz
//MAGNETO click on socket 1

#include "AS5048A_Driver.h"

// TFT module connections
unsigned int TFT_DataPort at GPIOE_ODR;
sbit TFT_RST at GPIOE_ODR.B8;
sbit TFT_RS at GPIOE_ODR.B12;
sbit TFT_CS at GPIOE_ODR.B15;
sbit TFT_RD at GPIOE_ODR.B10;
sbit TFT_WR at GPIOE_ODR.B11;
sbit TFT_BLED at GPIOE_ODR.B9;
// End TFT module connections


sbit AS5048A_CS at GPIOD_ODR.B13;
sbit AS5048A_OUT at GPIOD_ODR.B10;

char text[30];
unsigned int temp = 0x00;
unsigned int temp1 = 0x00;
float tempAngle = 0x00;
float angle = 0x00;
unsigned char i = 0;

void Display_Init(){
  TFT_Init_ILI9341_8bit(320, 240);
  TFT_BLED = 1;
  TFT_Set_Pen(CL_BLUE, 1);
  TFT_Set_Font(TFT_defaultFont, CL_WHITE, FO_HORIZONTAL);
  TFT_Set_Brush(1,CL_BLUE,0,0,0,0);
  TFT_Fill_Screen(CL_BLUE);
}

void MCU_Init() {
  GPIO_Digital_Output ( &GPIOD_BASE, _GPIO_PINMASK_10  );
  GPIO_Digital_Output( &GPIOD_BASE, _GPIO_PINMASK_13 );
  AS5048A_OUT = 0;
  AS5048A_CS = 1;
  // Set SPI3 to the Master Mode, data length is 8-bit, clock = Peripheral/64, clock IDLE state low and data transmitted at the first clock edge transition, MSB transferred first, Slave Select Disabled :
  SPI3_Init_Advanced(_SPI_FPCLK_DIV16, _SPI_MASTER | _SPI_8_BIT | _SPI_CLK_IDLE_LOW | _SPI_FIRST_CLK_EDGE_TRANSITION | _SPI_MSB_FIRST | _SPI_SS_DISABLE | _SPI_SSM_ENABLE | _SPI_SSI_1, &_GPIO_MODULE_SPI3_PC10_11_12);
}

void main() {
  MCU_Init();
  Display_Init();
  TFT_Write_Text("Ready...",10,10);
  //
  AS5048A_readCmd(0x0001);
  AS5048A_writeCmd(0x0016,0x00);
  AS5048A_writeCmd(0x0017,0x00);
  temp = AS5048A_readCmd(0x3FFF);
  AS5048A_writeCmd(0x0016,((temp >> 6) & 0xFF));
  AS5048A_writeCmd(0x0017,(temp & 0x3F));
  temp = AS5048A_readCmd(0x3FFD);
  
  while(1) {
        i = 0;
        tempAngle = 0;
        while(i < 10){
            temp =  AS5048A_readCmd(0x3FFF);
            if((temp & 0x4000) == 0x0000){
                tempAngle += (float)((temp & 0x3FFF) * 0.022);
                i++;
            }else{
                AS5048A_readCmd(0x0001);
            }
        }
        
        //tempAngle = (float)(temp & 0x3FFF) * 0.022;
        angle = tempAngle / 10;
        FloatToStr(angle,text);
        TFT_Fill_Screen(CL_BLUE);
        TFT_Write_Text(text,10,30);
        temp =  AS5048A_readCmd(0x3FFD) & 0x3FFF;
        WordToStr(temp,text);
        TFT_Write_Text(text,10,60);
        Delay_ms(100);
  }
}