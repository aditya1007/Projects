#ifndef GPIO_DEFS_H
#define GPIO_DEFS_H

#define MASK(x) (1UL << (x))

#define DEBUG1_I2C_exec (1) 	// on port B
#define DEBUG1_I2C_msg (2) 	// on port B
#define DEBUG1_I2C_busy (3) 	// on port B

#define CONFIG_SW_3 (3) // on port E
#define CONFIG_SW_4 (4) // on port E
#define CONFIG_SW_5 (5) // on port E

#define SWITCH_PRESSED(x) (!(PTE->PDIR & (MASK(x)))) //switch pressed

//#define V1 0
//#define V2 0
//#define V3 1

#define SET_BIT(x) {PTB->PSOR = MASK(x);}
#define CLEAR_BIT(x) {PTB->PCOR = MASK(x);}

#endif
