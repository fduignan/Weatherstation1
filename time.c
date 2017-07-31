#include "stm32f030xx.h"

void RTCISR(void)
{
	RTC_ISR &= ~BIT8;
	EXTI_PR |= BIT17;
}
void initRTC()
{	
	// Turn on power control circuit clock
	RCC_APB1ENR |= BIT28;
	// Turn on LSI
	RCC_CSR |= BIT0;
	// Wait for LSI to be ready
	while ( (RCC_CSR & BIT1) == 0);
	// Unlock the RTC domain
	PWR_CR |= BIT8; // set DBP in PWR_CR
	RCC_BDCR |=  BIT16; // put RTC power domain into reset
	RCC_BDCR &=  ~BIT16; // take it back out of reset
	
	// Turn the RTC on, use LSI
	RCC_BDCR |= BIT15 + BIT9;
	RCC_BDCR &= ~BIT8;
	// Unlock the RTC
	RTC_WPR = 0xca;
	RTC_WPR = 0x53;	
	// RTC Initialization procedure (see reference manual)
	RTC_ISR |= BIT7; // set INIT bit
	while ((RTC_ISR & BIT6)==0); // wait for init to start	
	ISER |= BIT2;   // enable RTC IRQ in NVIC
	
// Alarm interrupt configuration	
	RTC_CR = 0;
	RTC_ALRMAR = BIT31+BIT23+BIT15; // ignore hour and minute fields -> alarm every minute (approx)
	RTC_ALRMASSR = 0x0f000000;  // match all sub second bits
	RTC_CR |= BIT12+BIT8;		// Enable alarm and alarm interrupt
	RTC_ISR &= ~BIT7; 			// clear INIT bit
	
// RTC alarm is an EXTI interrupt in fact so need to configure this too
	EXTI_IMR |= BIT17;  // RTC is triggered via EXTI17,18 or 19
	EXTI_RTSR |= BIT17;
}
