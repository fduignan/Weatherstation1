
// i2c.h
// API for the NXP STM32F030 I2C interface
#include <stdint.h>

#ifndef __I2C_H
#define __I2C_H
#define MAX_I2C_DATA 64

#define I2C_WRITE_COMPLETE 1
#define I2C_READ_COMPLETE  2 
#define I2C_ERROR          -1

typedef struct {
	char Mode; // can be 'r' or 'w'	
	uint8_t SlaveAddress;  // Use the 7 bit address here
	unsigned TXIndex;
    unsigned RXIndex;
	unsigned TXCount;
    unsigned RXCount;
    volatile int Status;
    volatile int Complete;
	uint8_t TXData[MAX_I2C_DATA];
    uint8_t RXData[MAX_I2C_DATA];
    
} I2CTransaction;


int initI2C();	
int I2CDoTransaction(I2CTransaction *pTransaction);

#endif
