#ifndef LPTIMER_H
#define LPTIMER_H
#include "MKL25Z4.h"

#define FREQ_LPO (1000)

void Init_LPTMR(uint32_t freq);
void Start_LPTMR(void);
void Stop_LPTMR(void);

extern volatile int32_t LPT_ticks;

#endif
