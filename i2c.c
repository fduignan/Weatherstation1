#include <stdint.h>	
#include "stm32f030xx.h"
#include "i2c.h"
void I2CStop(void);
void I2CStart();
volatile I2CTransaction * pTransaction;
//#define DEBUG 1
#ifdef DEBUG
#include "serial.h"
#define MAX_DEBUG_TRACE 32
int DebugTraceData[MAX_DEBUG_TRACE];
int DebugIndex=0;
void DebugTrace(int Value)
{
	
	if (DebugIndex < MAX_DEBUG_TRACE)
	{
		DebugTraceData[DebugIndex++]=Value;
	}
}
#endif
int initI2C()
{	
	unsigned Divisor;
    RCC_AHBENR |= BIT17;    // Turn on port A    
    RCC_APB1ENR |= BIT21;   // Turn on clock for I2C1
    // PA9 and PA10 will be used for I2C
    GPIOA_MODER |= BIT21 + BIT19;
    GPIOA_MODER &= ~(BIT20+BIT18);   
    // Alternate function 4 is I2C on PA9/PA10
    GPIOA_AFRH &= 0xfffff00f;
    GPIOA_AFRH |= BIT10+BIT6;
	RCC_APB1RSTR &= ~BIT21; // Ensure I2C1 is out of reset        
	// Timing figures taken from Table 66 in Reference guide Document ID: DM00091010 
	// Assuming an 8MHz clock and a desired I2C rate of 100kHz	
	I2C1_TIMINGR = 0x10420f13;        
	ISER |= BIT23; // enable I2C1 interrupts in NVIC
	I2C1_ICR = BIT13+BIT12+BIT11+BIT10+BIT9+BIT8+BIT5+BIT4+BIT3; // clear all pending interrupts
	I2C1_CR1 = BIT6+BIT2+BIT1; // enable the various interrupts
	return 0;

}
int I2CDoTransaction(I2CTransaction *pTrans)
{   // Assuming MASTER mode
    // Blocking call for I2C comms.  Power save sleep while waiting to be included.
    
    unsigned TempVal;
	int Timeout=0xfffff;
    I2C1_CR1 &= ~BIT0;   // disable the I2C Interface
	pTransaction = pTrans;
	pTransaction->TXIndex=0;
    pTransaction->RXIndex=0;
	pTransaction->Status=0;
    pTransaction->Complete = 0;
    // Must program length of outgoing transfer into NBYTES in I2C_CR2
    TempVal = I2C1_CR2 & 0xff00ffff;   // read current value of CR2
    TempVal |= (pTransaction->TXCount & 0xff) << 16; // assuming transfer length of < 256
    TempVal &= 0xffffff00;
    TempVal |= (pTransaction->SlaveAddress & 0x7f) << 1;
    TempVal &= ~BIT11; // 7 bit addressing
    pTrans->Mode |= 32; // enforce lower case    
    TempVal &= ~BIT10; // set WRN=0 =>Write : all transactions start with a write
        
    // Will do software end mode (i.e. software will send the stop signal)
    TempVal &= ~BIT25;
    I2C1_CR2 = TempVal;                // Write new value out    
    I2C1_CR1 |= BIT0;                  // enable the I2C Interface
    I2CStart();                        // Begin transmission
    
	// Now need to wait for the transaction to complete.      
	while ( (pTransaction->Complete ==  0) && (Timeout--) ); 	
        //cpu_sleep();      // Can't sleep when using deep-sleep mode of CPU - I2C can't wake CPU back up.
 
    
	#ifdef DEBUG
	int Index;
	eputs("dbg\r\n");
    drainUART();
	for (Index = 0; Index < MAX_DEBUG_TRACE; Index++)
	{
		printHex(DebugTraceData[Index]);
		DebugTraceData[Index] = 0; // clear out debug trace for next time around
		eputs("\r\n");
		drainUART();
	}
	DebugIndex = 0;        
	eputs("\r\n");
	#endif
    if (Timeout <= 0)
        return I2C_ERROR; 
    else
        return pTransaction->Status;
	
}

void I2CISR()
{
    static unsigned ISRReg;
    ISRReg=I2C1_ISR;
    // Assuming master mode read write only (+faults)
    #ifdef DEBUG
	DebugTrace(ISRReg);
    #endif
    if (ISRReg & BIT1)  // TXIS Interrupt
    { 
        // Write next byte to TXDR
        I2C1_TXDR = pTransaction->TXData[pTransaction->TXIndex];
        if (pTransaction->TXIndex < pTransaction->TXCount)
            pTransaction->TXIndex++;
        
        
    }
    if (ISRReg & BIT2)  // RXNE Interrupt
    {
        // Read next byte from RXDR        
        pTransaction->RXData[pTransaction->RXIndex++]=I2C1_RXDR;
        
        if (pTransaction->RXIndex >= pTransaction->RXCount)            
        {
            pTransaction->RXIndex = 0; // prevent run past buffer end (precaution)
            I2CStop();            
            pTransaction->Status = I2C_READ_COMPLETE;
            pTransaction->Complete = 1;
        }
        
    }

    if (ISRReg & BIT6)  // TC Interrupt 
    { 
        // Just finished sending a block of bytes out.
        // Next action depends on mode        
        if (pTransaction->Mode=='w')
        {               
            pTransaction->Status = I2C_WRITE_COMPLETE; // write complete
            // Simple write complete so issue stop
            I2CStop();             
            pTransaction->Complete = 1;
        } 
        else
        {
            // This is part of a read operation so set up registers for 
            // reception of the expected number of bytes and issue repeat start.
            unsigned TempVal = I2C1_CR2 & 0xff00ffff;   // read current value of CR2
            TempVal |= (pTransaction->RXCount & 0xff) << 16; // assuming transfer length of < 256
            // A read is due so set read flag
            TempVal |= BIT10; // set WRN=1 =>Read
            I2C1_CR2 = TempVal; // write register value out
            
            I2CStart(); // send repeat start
        }        

    }

    if (ISRReg & BIT7)  // TCR Interrupt
    {
        // Receive complete
        pTransaction->Status = I2C_READ_COMPLETE; // read complete
        I2CStop();
        pTransaction->Complete = 1;
    }
    if (ISRReg & (BIT11+BIT10+BIT9+BIT8))  // Error Interrupt
    {
        // Something has gone wrong, flag an error and finish up
        pTransaction->Status = I2C_ERROR; // Flag an error
        pTransaction->Complete = 1;
    }   
}

void I2CStart()
{
	I2C1_CR2 |= BIT13; // send a start signal
}
void I2CStop(void)
{
	I2C1_CR2 |= BIT14; // send a stop signal
}
