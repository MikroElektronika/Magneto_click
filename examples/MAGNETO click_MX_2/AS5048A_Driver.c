

#define AS5048A_CLEAR_ERROR_FLAG 0x0001
#define AS5048A_PROGRAMMING_CONTROL 0x0003
#define AS5048A_OTP_REGISTER_ZERO_POS_HIGH 0x0016
#define AS5048A_OTP_REGISTER_ZERO_POS_LOW 0x0017
#define AS5048A_DIAG_AGC 0x3FFD
#define AS5048A_MAGNITUDE 0x3FFE
#define AS5048A_ANGLE 0x3FFF


extern sfr sbit AS5048A_CS;
extern sfr sbit AS5048A_OUT;


unsigned command = 0x00;
unsigned int readValue = 0xFF;
unsigned char highByte;
unsigned char lowByte;


static void pauseSpi(){
    AS5048A_CS = 1;
    Delay_us(1);
    AS5048A_CS = 0;
    //delay_us(1);
}

/**
 * Utility function used to calculate even parity of word
 */
static unsigned char calcParity(unsigned int par){

    unsigned char i =0;
    unsigned char ret = 0;

    for (i = 0; i<15; i++)
        ret += (par >> i) & 0x01;
    return ret % 2;
}


/*
 * Read a register from the sensor
 * Takes the address of the register as a 16 bit word
 * Returns the value of the register
 */
unsigned int AS5048A_readCmd(unsigned int address){

    char i =0;
    unsigned char highData;
    unsigned char lowData;

    readValue = 0xFF;
    command = ( address & 0x3FFF) ;
    command |= 0x4000;
    command |= (calcParity(command) << 0x0F);
    highByte = ((command >> 8) & 0xFF);
    lowByte =  (command & 0xFF);
    pauseSpi();
    highData = SPI3_Read(highByte);
    lowData  = SPI3_Read(lowByte);

    pauseSpi();
    highData = SPI3_Read(0x00);
    lowData  = SPI3_Read(0x00);
    AS5048A_CS = 1;
    readValue = highData;
    readValue <<= 8;
    readValue |= lowData;
    return readValue;
}

/*
 * Write to a register
 * Takes the 16-bit  address of the target register and the 16 bit word of data
 * to be written to that register
 * Returns the value of the register after the write has been performed. This
 * is read back from the sensor to ensure a sucessful write.
 */

void AS5048A_writeCmd(unsigned int address, unsigned int wrData){

    unsigned char hData = 0x00;
    unsigned char lData = 0x00;
    unsigned int tmpData = 0x00;
    unsigned char highData;
    unsigned char lowData;

    command = ( address & 0x3FFF) ;
    command |= (calcParity(command) << 0x0F);
    highByte = ((command >> 8) & 0xFF);
    lowByte = (command & 0xFF);
    tmpData = wrData & 0x3FFF;
    tmpData |= (calcParity(tmpData) << 0x0F);
    hData = ((tmpData >> 8 ) & 0xFF);
    lData = (tmpData & 0xFF);
    pauseSpi();
    highData = SPI3_Read(highByte);
    lowData  = SPI3_Read (lowByte);
    pauseSpi();
    highData = SPI3_Read(hData);
    lowData  = SPI3_Read (lData);
    pauseSpi();
    highData = SPI3_Read(0x00);
    lowData  = SPI3_Read(0x00);
    AS5048A_CS = 1;
}

/*
unsigned int AS5048A_GetAngle() {
  return AS5048A_Read(AS5048A_MAGNITUDE);
} */