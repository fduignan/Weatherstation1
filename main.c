// This is a test program for use with a low energy weather station based
// around the STM32F030 and an NRF905 radio module.  The sensing module is
// a GY-652 (HMC5983 + BMP180) which uses an I2C interface.
// The readings for this sensor will be transmitted periodically over
// the radio link
// The STM32F030 runs at the default speed of 8MHz on its internal oscillator.
// A serial interface is provided for debugging purposes.
// This version (0.1) does not take much heed of power saving - that will come later.
//
// This test program was used to develop the code for the BMP180 pressure/temperature sensor
//
/* Wiring : This needs to change due to io conflicts
 
 STM32F030          NRF905
 PA1                DR
 PF0                Pwr  
 PF1                CE   
 PA4                CSN
 PA5                SPI SCLK
 PA6                SPI MISO
 PA7                SPI MOSI
 PB1                TXEn
 
 UART Interface
 PA2                UART TX  
 PA3                UART RX  
 
 BMP180/HMC5983 I2C interface wiring
 I2C SDA   PA10
 I2C SCL   PA9 
 
 
 */ 
#include "stm32f030xx.h"
#include "serial.h"
#include "i2c.h"
#include "spi.h"
#include "nrf905.h"
#include "time.h"
const uint8_t BaseStationAddr[]={0xde,0xad,0xb0,0x55};
const uint8_t NodeAddr[]={0xfd,0x00,0x00,0x00};
// Uncomment the line below to use test data and calibration values from the BMP180 datasheet
//#define TEST_DATA 1
char Msg[32]="                                ";
uint8_t Buffer[32];
void delay(int dly)
{
  while( dly--);
}
void configPins()
{
  // Power up PORTA
  RCC_AHBENR |= BIT17;	
}	

void dumpI2C1Registers()
{
// Used for debugging only    
    eputs("\r\nGPIOA_MODER:");
    printHex(GPIOA_MODER);
    drainUART();
    eputs("\r\nGPIOA_AFRH:");
    printHex(GPIOA_AFRH);
    drainUART();    
    eputs("\r\nCR1:");
    printHex(I2C1_CR1);
    drainUART();
    eputs("\r\nCR2:");
    printHex(I2C1_CR2);
    drainUART();
    eputs("\r\nOAR1:");    
    printHex(I2C1_OAR1);
    drainUART();
    eputs("\r\nOAR2:");    
    printHex(I2C1_OAR2);
    drainUART();
    eputs("\r\nTIMINGR:");    
    printHex(I2C1_TIMINGR);
    drainUART();
    eputs("\r\nTIMEOUTR:");    
    printHex(I2C1_TIMEOUTR);
    drainUART();
    eputs("\r\nISR:");    
    printHex(I2C1_ISR);
    drainUART();
    eputs("\r\nICR:");    
    printHex(I2C1_ICR);
    drainUART();
    eputs("\r\nPECR:");    
    printHex(I2C1_PECR);
    drainUART();
    eputs("\r\nRXDR:");    
    printHex(I2C1_RXDR);
    drainUART();
    eputs("\r\nTXDR:");    
    printHex(I2C1_TXDR);
    drainUART();
}
void printByteArray(uint8_t *Ary,int len)
{
    eputs("\r\n");
    while (len--)
    {
        printByte(*Ary++);
        eputs(" ");
    }
}
uint8_t readBMPRegister(uint8_t RegNum)
{    
  I2CTransaction Transaction;    
  Transaction.Mode = 'r';
  Transaction.SlaveAddress = 0x77;
  Transaction.TXCount = 1;
  Transaction.RXCount = 1;
  Transaction.TXData[0] = RegNum;  
  I2CDoTransaction(&Transaction);
  return Transaction.RXData[0];
}
typedef struct {
    int16_t AC1;
    int16_t AC2;
    int16_t AC3;
    uint16_t AC4;
    uint16_t AC5;
    uint16_t AC6;
    int16_t B1;
    int16_t B2;
    int16_t MB;
    int16_t MC;
    int16_t MD;
} CalibrationData_t;
CalibrationData_t CalibrationData;

uint8_t writeBMPRegister(uint8_t RegNum,uint8_t Value)
{    
  I2CTransaction Transaction;    
  Transaction.Mode = 'w';
  Transaction.SlaveAddress = 0x77;
  Transaction.TXCount = 2;
  Transaction.RXCount = 0;
  Transaction.TXData[0] = RegNum;
  Transaction.TXData[1] = Value;    
  return I2CDoTransaction(&Transaction);  
}
void readBMPRegisters(uint8_t * Registers)
{
    Registers[0]=readBMPRegister(0xf8);
    Registers[1]=readBMPRegister(0xf7);
    Registers[2]=readBMPRegister(0xf6);
    Registers[3]=readBMPRegister(0xf4);
    Registers[4]=readBMPRegister(0xe0);
    Registers[5]=readBMPRegister(0xd0);
}
void readBMPCalibrationData()
{
    CalibrationData.AC1 = ((int16_t)readBMPRegister(0xaa)<<8)+(int16_t)readBMPRegister(0xab);
    CalibrationData.AC2 = ((int16_t)readBMPRegister(0xac)<<8)+(int16_t)readBMPRegister(0xad);
    CalibrationData.AC3 = ((int16_t)readBMPRegister(0xae)<<8)+(int16_t)readBMPRegister(0xaf);
    CalibrationData.AC4 = ((uint16_t)readBMPRegister(0xb0)<<8)+(uint16_t)readBMPRegister(0xb1);
    CalibrationData.AC5 = ((uint16_t)readBMPRegister(0xb2)<<8)+(uint16_t)readBMPRegister(0xb3);
    CalibrationData.AC6 = ((uint16_t)readBMPRegister(0xb4)<<8)+(uint16_t)readBMPRegister(0xb5);
    CalibrationData.B1 = ((int16_t)readBMPRegister(0xb6)<<8)+(int16_t)readBMPRegister(0xb7);
    CalibrationData.B2 = ((int16_t)readBMPRegister(0xb8)<<8)+(int16_t)readBMPRegister(0xb9);
    CalibrationData.MB = ((int16_t)readBMPRegister(0xba)<<8)+(int16_t)readBMPRegister(0xbb);
    CalibrationData.MC = ((int16_t)readBMPRegister(0xbc)<<8)+(int16_t)readBMPRegister(0xbd);
    CalibrationData.MD = ((int16_t)readBMPRegister(0xbe)<<8)+(int16_t)readBMPRegister(0xbf);
#ifdef TEST_DATA  
  // Test data from data sheet
    CalibrationData.AC1 = 408;
    CalibrationData.AC2 = -72;
    CalibrationData.AC3 = -14383;
    CalibrationData.AC4 = 32741;
    CalibrationData.AC5 = 32757;
    CalibrationData.AC6 = 23153;
    CalibrationData.B1 = 6190;
    CalibrationData.B2 = 4;
    CalibrationData.MB = -32768;
    CalibrationData.MC = -8711;
    CalibrationData.MD = 2686;
#endif    
}    
int readTemperature()
{
    long RawTemp;
    long Temperature,X1,X2,B5;
    writeBMPRegister(0xf4,0x2e);
    // short delay for conversion
    int TimeOut=0x100;
    while((TimeOut--) && (readBMPRegister(0xf4)&0x20) );    
    
    RawTemp=readBMPRegister(0xf6);
    RawTemp <<= 8;
    RawTemp+=readBMPRegister(0xf7); 
#ifdef TEST_DATA
    RawTemp = 27898;
#endif         
    X1 = (((long)RawTemp-(long)CalibrationData.AC6)*(long)CalibrationData.AC5)>>15;
	X2 = ((long)CalibrationData.MC << 11)/((long)X1 + (long)CalibrationData.MD);
	B5 = X1 + X2;
	Temperature = (B5+8)>>4;    
    return Temperature;    
}
int readPressure()
{
    // Need to read the temperature and then the pressure.
    long RawTemp;
    long Temperature,X1,X2,X3,B3,B5,B6,Pressure;
    unsigned long B4,B7;
    writeBMPRegister(0xf4,0x2e);
    delay(1000); // short delay for conversion
    int TimeOut=0x100;
    while((TimeOut--) && (readBMPRegister(0xf4)&0x20));        
    RawTemp=readBMPRegister(0xf6);
    RawTemp <<= 8;
    RawTemp+=readBMPRegister(0xf7);        
#ifdef TEST_DATA
    RawTemp = 27898;
#endif     
    X1 = (((long)RawTemp-(long)CalibrationData.AC6)*(long)CalibrationData.AC5)>>15;
	X2 = ((long)CalibrationData.MC << 11)/((long)X1 + (long)CalibrationData.MD);
	B5 = X1 + X2;   
    B6 = B5 - 4000;    
    // Now read the raw pressure.  Will use oversampling of 0 to save power    
    writeBMPRegister(0xf4,0x34);
    delay(1000);  // short delay for conversion
    TimeOut=0x100;    
    while((TimeOut--) && (readBMPRegister(0xf4)&0x20) );    
    long RawPressure = ((long)readBMPRegister(0xf6))<<16;    
    RawPressure += ((long)readBMPRegister(0xf7)) << 8;
    RawPressure += ((long)readBMPRegister(0xf8));
    RawPressure >>= 8;
#ifdef TEST_DATA
     RawPressure = 23843; // test data from data sheet
#endif       
    X1 =((long)CalibrationData.B2 * (B6 * B6) >> 12 ) >> 11;        
    X2 = ((long)CalibrationData.AC2 * B6) >> 11;    
    X3 = X1 + X2;    
    B3 = (((long)CalibrationData.AC1*4+X3)+2) >> 2;    
    X1 = ((long)CalibrationData.AC3*B6) >> 13;
    X2 = ((long)CalibrationData.B1*(B6*B6 >> 12))>>16;        
    X3 = ((X1+X2)+2) >> 2;    
    B4 = ((CalibrationData.AC4*(unsigned long)(X3+32768)))>>15;    
    B7 = ((unsigned long)RawPressure-B3)*50000UL;
    if (B7 < 0x80000000)
    {
        Pressure = (B7 * 2)/B4;        
    }
    else
    {
        Pressure = (B7 / B4) << 1;        
    }    
    X1 = (Pressure >> 8) * (Pressure >> 8);    
    X1 = (X1 * 3038)>>16;    
    X2 = (-7357*Pressure) >> 16;    
    Pressure = Pressure + ( (X1+X2+3791)  >> 4 );
    return Pressure;    
    
}
void ShowCalibrationData()
{
    // Used during debugging only - check to see if the 
    // calibration data were read correctly
    int i=0;    
    eputs("\r\n");
    // print the individual 8 bit registers
    uint8_t Addr = 0xaa;
    for (i=0;i<22;i++)
    {
        printByte(readBMPRegister(Addr++));
        eputs(" ");
        drainUART();
    }
    eputs("\r\n");
    // print the 16 bit signed/unsigned co-efficients
    printDecimal(CalibrationData.AC1);
    eputs("\r\n");
    drainUART();
    printDecimal(CalibrationData.AC2);
    eputs("\r\n");
    drainUART();
    printDecimal(CalibrationData.AC3);
    eputs("\r\n");
    drainUART();
    printDecimal(CalibrationData.AC4);
    eputs("\r\n");
    drainUART();
    printDecimal(CalibrationData.AC5);
    eputs("\r\n");
    drainUART();
    printDecimal(CalibrationData.AC6);
    eputs("\r\n");
    drainUART();
    printDecimal(CalibrationData.B1);
    eputs("\r\n");
    drainUART();
    printDecimal(CalibrationData.B2);
    eputs("\r\n");
    drainUART();
    printDecimal(CalibrationData.MB);
    eputs("\r\n");
    drainUART();
    printDecimal(CalibrationData.MC);
    eputs("\r\n");
    drainUART();
    printDecimal(CalibrationData.MD);
    eputs("\r\n");
    drainUART();

}
void low_power_mode()
{					
    PwrLow(); // Put NRF905 into low power mode
    // Turn off GPIO B,A and F		
	RCC_AHBENR &= ~(BIT17+BIT18+BIT22);			
    RCC_APB1ENR &= ~BIT21;   // Turn off clock for I2C1
    RCC_APB2ENR &= ~BIT12;		// turn off SPI1 	
	RCC_CFGR |= 0xf0; // drop bus speed by a factor of 512
	cpu_sleep();      // stop cpu
}
void resume_from_low_power()
{	
	RCC_CFGR &= ~0xf0; // speed up to 8MHz		
	// Turn on GPIO B,A and F
	RCC_AHBENR |= BIT18+BIT17+BIT22;
    RCC_APB1ENR |= BIT21;   // Turn on clock for I2C1
    RCC_APB2ENR |= BIT12;		// turn on SPI1 	
    PwrHigh(); // power up the radio

}
void TxPacket(uint8_t *Pkt,int len)
{
    unsigned timeout=100;
    TXEnHigh();
    writeTXPayload(Pkt,len);
    setTXAddress(BaseStationAddr);
    CEHigh();
    delay(10); // wait for tx to go
    CELow();
    while( ( (getStatus()&0x20) == 0) && (timeout--)); // wait for tx to complete (with eventual timeout) 
    TXEnLow();  // clear the DR bit
}
void Int2String(unsigned int x,char *Str)
{
    int index=0;
    // Can have up to 4billion so 10 digits
    Str[10]=0; // terminate the string 
    while(index < 10)
    {
        Str[9-index]= x%10+'0';
        x = x / 10;
        index++;
    }
}
int main()
{  
  int Count = 0;  
  configPins();  
  SCR |= BIT2;  // enable Deep Sleep mode
  PWR_CR |= BIT0; // Put internal voltage regulator into low power mode when cpu is stopped  
#ifdef DEBUG  
  initUART(9600);    
#endif  
  initI2C();
  initRTC();  // RTC will generate a periodic interrupt at a rate ot 1Hz
  initSPI();
  initNRF905();
  PwrHigh(); // turn on the radio
  CELow();
  TXEnLow();  
// Set Frequency to 434.2MHz 
  
  setRXPower(1); // low RX sensitivity
  setTXPower(2); // Set TX power (+6dBm) 
  setChannel(0x76); // Channel number  
  setRange(0);      // 400Mhz range 
  setAutoRetran(0);  // Don't bother with auto-retransmit (actually seems to work better without)
    
  writeRegister(9,0x5f); // 8 bit CRC, CRC Enable, 16MHz external crystal, not using clock out
  writeRegister(2,0x44);
  writeRegister(3,0x20);
  setTXAddress(BaseStationAddr);
  setRXAddress(BaseStationAddr);
    
  enable_interrupts();
  readBMPCalibrationData();            

#ifdef DEBUG
  eputs("Starting\r\n");
  ShowCalibrationData();
#endif
  
  while(1) {            
    Int2String(readTemperature(),&Msg[0]);    
    Msg[10]=',';
    Int2String(readPressure(),&Msg[11]);       
    TxPacket(Msg,0x20);
    low_power_mode(); // Sleep and wait for RTC interrupt
    resume_from_low_power();    
    
  }
  return 0;
}
