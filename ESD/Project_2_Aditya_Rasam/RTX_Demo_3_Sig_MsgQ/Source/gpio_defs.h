#ifndef GPIO_DEFS_H
#define GPIO_DEFS_H

#define SW1_POS (7)		// on port D
#define SW2_POS (6)		// on port D

#define MASK(x) (1UL << (x))

#define SWITCH_PRESSED(x) (!(PTE->PDIR & (MASK(x))))

//-----Defined Aditya-----Proj2-------
#define DEBUG1_I2C_exec (1) 	// on port B
#define DEBUG1_I2C_msg (2) 	// on port B
#define DEBUG1_I2C_busy (3) 	// on port B

#define CONFIG_SW_3 (3) // on port E
#define CONFIG_SW_4 (4) // on port E
#define CONFIG_SW_5 (5) // on port E
//------------------------------------

// Freedom KL25Z LEDs
#define RED_LED_POS (18)		// on port B
#define GREEN_LED_POS (19)	// on port B
#define BLUE_LED_POS (1)		// on port D

#define ON_TIME (4000)
#define FLASH_TIME (600)

#endif
// *******************************ARM University Program Copyright © ARM Ltd 2013*************************************   
