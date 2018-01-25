#include <MKL25Z4.H>
#include "mma8451.h"
#include "gpio_defs.h"
#include "i2c.h"
#include "delay.h"


int16_t acc_X=0, acc_Y=0, acc_Z=0;
uint8_t x=0, msg_done=0, m;
//int8_t V1=1, V2=0, V3 =0;
//initializes mma8451 sensor
//i2c has to already be enabled


int init_mma()
{
	uint8_t data[1];

	//set active mode, 14 bit samples, 2g full scale and 800 Hz ODR 
	data[0] = 0x01;
	i2c_write_bytes(MMA_ADDR, REG_CTRL1, data, 1);
	return 1;
}

void read_full_xyz()
{
	int i;
	uint8_t data[6];
	int16_t temp[3];
	
	
		
if (Mode_block){
			i2c_read_bytes(MMA_ADDR, REG_XHI, data, 6);;
				for ( i=0; i<3; i++ ) {
								temp[i] = (int16_t) ((data[2*i]<<8) | data[2*i+1]);
							}
		}
else if (Mode_FSM_Poll){
					while(x==0){
								i2c_read_bytes_fsm(MMA_ADDR, REG_XHI, data, 6);
								ShortDelay(8);	
								}
					x=0;
					for ( i=0; i<3; i++ ) {
								temp[i] = (int16_t) ((data[2*i]<<8) | data[2*i+1]);
							}
					}
		else if (Mode_ISR){
				msg_done = 0;
			//	Prog_flow = 1;
					while(msg_done==0){
						

							
					i2c_read_bytes_ISR(MMA_ADDR, REG_XHI, data, 6);
						while (transfer_complete==0){
						;
						}
							
//					}
					}
					//msg_done=0;
					for ( i=0; i<3; i++ ) {
								temp[i] = (int16_t) ((data[2*i]<<8) | data[2*i+1]);
									
							}
						}

	// Align for 14 bits
	acc_X = temp[0]/4;
	acc_Y = temp[1]/4;
	acc_Z = temp[2]/4;
}
