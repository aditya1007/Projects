#include <MKL25Z4.H>
#include <math.h>
#include "mma8451.h"
#include "gpio_defs.h"
#include "i2c.h"



//initializes mma8451 sensor
//i2c has to already be enabled
int init_mma()
{
	uint8_t data[2];

	//set active mode, low noise, 14 bit samples, 2g full scale and 800 Hz ODR 
	data[0] = 0x05;
	// only 2x oversampling at 800 Hz ODR
	data[1] = 0x00;

	i2c_write_bytes(MMA_ADDR, REG_CTRL1, data, 2);
	return 1;
}

void read_full_xyz(int16_t * acc)
{
	int i;
	uint8_t data[6];
	int16_t temp[3];
	
	i2c_read_bytes(MMA_ADDR, REG_XHI, data, 6);

	for ( i=0; i<3; i++ ) {
		temp[i] = (int16_t) ((data[2*i]<<8) | data[2*i+1]);
		acc[i] = temp[i]/4; 	// Align for 14 bits
	}
}

void convert_xyz_to_roll_pitch(int16_t * acc,
	float * roll, float * pitch) {
	float ax = acc[0]/COUNTS_PER_G,
				ay = acc[1]/COUNTS_PER_G,
				az = acc[2]/COUNTS_PER_G;
	
	*roll = atan2f(ay, az)*57.2957795;//180/M_PI;
	*pitch = atan2f(ax, sqrtf(ay*ay + az*az))*57.2957795;//180/M_PI;
}
	