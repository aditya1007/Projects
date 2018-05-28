/*----------------------------------------------------------------------------
 *----------------------------------------------------------------------------*/
#include <MKL25Z4.H>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "gpio_defs.h"
#include "LEDs.h"
#include "i2c.h"
#include "mma8451.h"
#include "delay.h"

#define FLASH_DELAY 10
#define ACC_SENSITIVITY 40

void Init_Debug_Signals(void) {
		//------Edited by Aditya------------
	SIM->SCGC5 |=  SIM_SCGC5_PORTB_MASK;;
	
	PORTB->PCR[DEBUG1_I2C_exec] &= ~PORT_PCR_MUX_MASK;          
	PORTB->PCR[DEBUG1_I2C_exec] |= PORT_PCR_MUX(1);
	
	PORTB->PCR[DEBUG1_I2C_msg] &= ~PORT_PCR_MUX_MASK;          
	PORTB->PCR[DEBUG1_I2C_msg] |= PORT_PCR_MUX(1);
	
	PORTB->PCR[DEBUG1_I2C_busy] &= ~PORT_PCR_MUX_MASK;          
	PORTB->PCR[DEBUG1_I2C_busy] |= PORT_PCR_MUX(1);
	
	
	PTB->PDDR |= MASK(DEBUG1_I2C_exec)| MASK(DEBUG1_I2C_msg)| MASK(DEBUG1_I2C_busy);

}

void Init_Config_Signals(void) {

	SIM->SCGC5 |=  SIM_SCGC5_PORTE_MASK;;
	//Select as GPIO
	PORTE->PCR[3] &= ~PORT_PCR_MUX_MASK;          
	PORTE->PCR[3] |= PORT_PCR_MUX(1); 
	
	PORTE->PCR[4] &= ~PORT_PCR_MUX_MASK;          
	PORTE->PCR[4] |= PORT_PCR_MUX(1); 
	
	PORTE->PCR[5] &= ~PORT_PCR_MUX_MASK;          
	PORTE->PCR[5] |= PORT_PCR_MUX(1); 
	
		// Select port E on pin mux, enable pull-up resistors
	PORTE->PCR[3] = PORT_PCR_MUX(1) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK;
	PORTE->PCR[4] = PORT_PCR_MUX(1) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK;
	PORTE->PCR[5] = PORT_PCR_MUX(1) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK;
	
	// Clear switch bits to input
		PTE->PDDR &= ~MASK(3); 
		PTE->PDDR &= ~MASK(4); 
		PTE->PDDR &= ~MASK(5); 
	
	
	/*
		SIM->SCGC5 |=  SIM_SCGC5_PORTE_MASK;;
	//Select as GPIO
	PORTE->PCR[CONFIG_SW_1] &= ~PORT_PCR_MUX_MASK;          
	PORTE->PCR[CONFIG_SW_1] |= PORT_PCR_MUX(1); 
	
	PORTE->PCR[CONFIG_SW_2] &= ~PORT_PCR_MUX_MASK;          
	PORTE->PCR[CONFIG_SW_2] |= PORT_PCR_MUX(1); 
	
	PORTE->PCR[CONFIG_SW_3] &= ~PORT_PCR_MUX_MASK;          
	PORTE->PCR[CONFIG_SW_3] |= PORT_PCR_MUX(1); 
	
		// Select port E on pin mux, enable pull-up resistors
	PORTE->PCR[CONFIG_SW_1] = PORT_PCR_MUX(1) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK;
	PORTE->PCR[CONFIG_SW_2] = PORT_PCR_MUX(1) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK;
	PORTE->PCR[CONFIG_SW_3] = PORT_PCR_MUX(1) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK;
	
	// Clear switch bits to input
		PTE->PDDR &= ~MASK(CONFIG_SW_1); 
		PTE->PDDR &= ~MASK(CONFIG_SW_2); 
		PTE->PDDR &= ~MASK(CONFIG_SW_3); 
	*/
}
void Mode_selection (void){
	
	if ((SWITCH_PRESSED(5)) & (!(SWITCH_PRESSED(CONFIG_SW_4))) & (!(SWITCH_PRESSED(CONFIG_SW_3)))){
Mode_block = 1;
Mode_FSM_Poll = 0;
Mode_ISR = 0;
}
else
{
	Mode_block = 0;
}
	
if (!(SWITCH_PRESSED(CONFIG_SW_5)) & ((SWITCH_PRESSED(4))) & (!(SWITCH_PRESSED(CONFIG_SW_3)))){
Mode_block = 0;
Mode_FSM_Poll = 1;
Mode_ISR = 0;
}
else
{
	Mode_FSM_Poll = 0;
}

if (!(SWITCH_PRESSED(CONFIG_SW_5))&(!(SWITCH_PRESSED(CONFIG_SW_4)))&((SWITCH_PRESSED(3)))){
Mode_block = 0;
Mode_FSM_Poll = 0;
Mode_ISR = 1;
}
else { 
Mode_ISR = 0;
}
}



/*----------------------------------------------------------------------------
  MAIN function
 *----------------------------------------------------------------------------*/
int main (void) {
	int16_t prev_acc_X, prev_acc_Y, prev_acc_Z;
	int n;
	
	Init_RGB_LEDs();
	Init_Debug_Signals();
	Init_Config_Signals();
	Mode_selection();
	Control_RGB_LEDs(1, 1, 0);								/* yellow: starting up */
	i2c_init();																/* init i2c	*/
	Delay(200);
	

	if (!init_mma()) {												/* init mma peripheral */
		Control_RGB_LEDs(1, 0, 0);							/* Light red error LED */
		while (1)																/* not able to initialize mma */
			;
	}
	Control_RGB_LEDs(0, 0, 0);							

	Delay(50);

	while (1) {
		Delay(50);
		prev_acc_X = acc_X;
		prev_acc_Y = acc_Y;
		prev_acc_Z = acc_Z;
		
	//	Mode_selection();
		read_full_xyz();
		
		if ((abs(prev_acc_X - acc_X) > ACC_SENSITIVITY) || 
			(abs(prev_acc_Y - acc_Y) > ACC_SENSITIVITY) || 
			(abs(prev_acc_Z - acc_Z) > ACC_SENSITIVITY)) {
			// Flash LEDs
				for (n=0; n<2; n++) {
					Control_RGB_LEDs(1, 1, 1);
					Delay(FLASH_DELAY);
					Control_RGB_LEDs(0, 0, 0);							
					Delay(FLASH_DELAY*2);		
				}
			}		
	}
}

