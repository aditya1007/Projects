#include "LPTimer.h"
#include "MKL25Z4.h"
#include "LEDs.h"
#include "GPIO_defs.h"

volatile int32_t LPT_ticks=0;

void Init_LPTMR(uint32_t freq) {
	SIM->SCGC5 |=  SIM_SCGC5_LPTMR_MASK;

	// Configure LPTMR
	// select 1 kHz LPO clock with prescale factor 0, dividing clock by 2
	// resulting in 500 Hz clock
	LPTMR0->PSR = /* LPTMR_PSR_PBYP_MASK | */ LPTMR_PSR_PCS(1) | LPTMR_PSR_PRESCALE(0); 
	LPTMR0->CSR = LPTMR_CSR_TIE_MASK;
	LPTMR0->CMR = (FREQ_LPO/(2*freq))-1; // Period - 1

#if 1
	// Configure NVIC 
	NVIC_SetPriority(LPTimer_IRQn, 3); 
	NVIC_ClearPendingIRQ(LPTimer_IRQn); 
	NVIC_EnableIRQ(LPTimer_IRQn);	
#endif
}

void Start_LPTMR(void) {
	LPTMR0->CSR |= LPTMR_CSR_TEN_MASK;
}

void Stop_LPTMR(void) {
	LPTMR0->CSR &= ~LPTMR_CSR_TEN_MASK;
}

void LPTimer_IRQHandler(void) {
	LPTMR0->CSR |= LPTMR_CSR_TCF_MASK;
	LPT_ticks++;
}