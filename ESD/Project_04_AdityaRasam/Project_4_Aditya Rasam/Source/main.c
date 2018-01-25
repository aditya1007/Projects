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
#include "LPTimer.h"
#include "Delay.h"

int Variable_Delay_Green=0,Variable_Delay_Yellow=0,Variable_Delay_Red=0;

void Init_Accel(void) {
	Delay(50);
	if (!init_mma()) {												/* init mma peripheral */
		Control_RGB_LEDs(1, 0, 0);							/* light red error LED */
		while (1)																/* not able to initialize mma */
			;
	}
	Delay(50);
}

void Tilt( void ) {
	int16_t accel[3];
	float roll, pitch;
	//adi888888888888888
	
			// Allow low leakage stop mode
		SMC->PMPROT = SMC_PMPROT_ALLS_MASK; // 
		// Enable low-leakage stop mode and regular run mode
		SMC->PMCTRL = SMC_PMCTRL_STOPM(3) | SMC_PMCTRL_RUNM(2);
		SMC->STOPCTRL = SMC_STOPCTRL_PSTOPO(0) | SMC_STOPCTRL_VLLSM(3);
	
		// Enable LLWU
		// allow LPTMR0 to wake LLWU
		LLWU->ME |= LLWU_ME_WUME0_MASK;
	
	// Enable stop mode (deep sleep)
		SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

	//8888888888888888888888
	// Start LPTimer for future use
	Init_LPTMR(2);
	Start_LPTMR();
	__enable_irq();
	

	while (1) {
					__wfi() ; // then go to sleep	

		read_full_xyz(accel);
		convert_xyz_to_roll_pitch(accel, &roll, &pitch);
		
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Red	<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<	
		if ((fabs(roll) > 30) || (fabs(pitch) > 30))				//Red
		{
		{Control_RGB_LEDs(1, 0, 0);
						Control_RGB_LEDs(0, 0, 0);
							}
		}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Yellow	<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<	

		else if ((fabs(roll) > 15) || (fabs(pitch) > 15))		//Yellow
		{
			if(Variable_Delay_Yellow<=39){
				Control_RGB_LEDs(0, 1, 0);
				 						Delay(6);
				Control_RGB_LEDs(1, 0, 0);
						Control_RGB_LEDs(0, 0, 0);
							}
			else if((39<Variable_Delay_Yellow)&(Variable_Delay_Yellow<78)){
				Control_RGB_LEDs(0, 1, 0);
				 						Delay(8);
				Control_RGB_LEDs(1, 0, 0);
						Control_RGB_LEDs(0, 0, 0);
							}
			 else if((78<Variable_Delay_Yellow)&(Variable_Delay_Yellow<117)){
				Control_RGB_LEDs(0, 1, 0);
				 						Delay(20);
				Control_RGB_LEDs(1, 0, 0);
						Control_RGB_LEDs(0, 0, 0);
							}
			 else if((117<Variable_Delay_Yellow)&(Variable_Delay_Yellow<156)){
				Control_RGB_LEDs(0, 1, 0);
				 						Delay(35);//35
				Control_RGB_LEDs(1, 0, 0);
						Control_RGB_LEDs(0, 0, 0);
							}
			 else if((156<Variable_Delay_Yellow)&(Variable_Delay_Yellow<195)){
				Control_RGB_LEDs(0, 1, 0);
				 						Delay(70);//--70
				Control_RGB_LEDs(1, 0, 0);
						Control_RGB_LEDs(0, 0, 0);
							}

			 else if((195<Variable_Delay_Yellow)&(Variable_Delay_Yellow<234)){
				Control_RGB_LEDs(0, 1, 0);
				 						Delay(500);//25---200
				Control_RGB_LEDs(1, 0, 0);
						Control_RGB_LEDs(0, 0, 0);
							}
			 else if((234<Variable_Delay_Yellow)&(Variable_Delay_Yellow<273)){
				Control_RGB_LEDs(0, 1, 0);
				 						Delay(600);
				Control_RGB_LEDs(1, 0, 0);
						Control_RGB_LEDs(0, 0, 0);
							}
			 else if((273<Variable_Delay_Yellow)&(Variable_Delay_Yellow<312)){
				Control_RGB_LEDs(0, 1, 0);
				 						Delay(500);
				Control_RGB_LEDs(1, 0, 0);
						Control_RGB_LEDs(0, 0, 0);
							}

				else {
				Control_RGB_LEDs(0, 1, 0);
				 						Delay(50);
				Control_RGB_LEDs(1, 0, 0);
						Control_RGB_LEDs(0, 0, 0);
					if (Variable_Delay_Yellow>1500)
								{Variable_Delay_Yellow=0;}
							}
		Variable_Delay_Yellow = Variable_Delay_Yellow+1;

		}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Green	<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<	
		else																								
		{Control_RGB_LEDs(0, 1, 0);
			if(Variable_Delay_Green<=39){		//1
						Delay(3);//100
						Control_RGB_LEDs(0, 0, 0);
							}
		 else if((39<Variable_Delay_Green)&(Variable_Delay_Green<=78)){//2
						Delay(10);//-10
						Control_RGB_LEDs(0, 0, 0);
							}
			 else if((78<Variable_Delay_Green)&(Variable_Delay_Green<=117)){//3
						Delay(30);//100------------50-----30
						Control_RGB_LEDs(0, 0, 0);
							}
			 else if((117<Variable_Delay_Green)&(Variable_Delay_Green<=156)){//4
						Delay(70);//350-----100----80----70if
						Control_RGB_LEDs(0, 0, 0);
							}
			 else if((156<Variable_Delay_Green)&(Variable_Delay_Green<=195)){//5
						Delay(200);//515777777777--------300--260-----245 i y
						Control_RGB_LEDs(0, 0, 0);
							}

			 else if((195<Variable_Delay_Green)&(Variable_Delay_Green<=234)){//6
						Delay(600);
						Control_RGB_LEDs(0, 0, 0);
							}
			 else if((234<Variable_Delay_Green)&(Variable_Delay_Green<=273)){//7
						Delay(1100);
						Control_RGB_LEDs(0, 0, 0);
							}
			 else if((273<Variable_Delay_Green)&(Variable_Delay_Green<=312)){//8
						Delay(2010);
						Control_RGB_LEDs(0, 0, 0);
							}
			 else if((312<Variable_Delay_Green)&(Variable_Delay_Green<=351)){
						Delay(1310);
						Control_RGB_LEDs(0, 0, 0);
						 }
				else {
							Delay(1110);
							Control_RGB_LEDs(0, 0, 0);
					if (Variable_Delay_Green>1500)
								{Variable_Delay_Green=0;}
							}
		Variable_Delay_Green = Variable_Delay_Green+1;
		}
		
	}
}

int main (void) {
	Init_RGB_LEDs();
	Control_RGB_LEDs(0, 0, 0);			// yellow: starting up 
	i2c_init();											// init i2c
	Init_Accel();										// init accelerometer
	Tilt();
}
