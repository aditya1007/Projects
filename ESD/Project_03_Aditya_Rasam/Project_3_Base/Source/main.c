/*----------------------------------------------------------------------------
 *----------------------------------------------------------------------------*/
#include <MKL25Z4.H>
#include <stdio.h>
#include <math.h>
#include <ctype.h>
#include <string.h>

#include "gpio_defs.h"
#include "UART.h"
#include "LEDs.h"
#include "timers.h"		
#include "delay.h"
#include "vector.h"
#include "profile.h"
#include "region.h"
#include "Drift_Calculation.h"

#define NUM_TESTS 100
#define MAX_MAG_ERROR 0.1
#define MAX_ANGLE_ERROR 1.0
#define Oneeighty_over_PI (57.2958f)
#define PI_over_180 (0.017453f)
#define PI (3.14159265f)


typedef struct {
	VECTOR_T BtW; // Boat motion relative to water
	VECTOR_T BtG;	// Boat motion relative to ground
	VECTOR_T WtG; // Water motion relative to ground
} TEST_CASE;

TEST_CASE Tests[] = { 									//--------Updated table to give values in radians instead of degress
{{	1.000	,	0.000	},	{	1.414	,	0.7854	},	{	1.000	,	1.5708	}},
{{	1.000	,	0.000	},	{	1.414	,	5.4978	},	{	1.000	,	4.71239	}},
{{	1.000	,	0.000	},	{	0.000	,	0.000	},	{	1.000	,	3.14159	}},
{{	1.000	,	0.000	},	{	2.000	,	0.000	},	{	1.000	,	0.000	}},
{{	1.000	,	1.5708	},	{	1.414	,	0.7854	},	{	1.000	,	0.000	}},
{{	1.000	,	-1.5708	},	{	1.414	,	-0.7854	},	{	1.000	,	0.000	}},
{{	1.000	,	3.14159	},	{	1.414	,	3.927	},	{	1.000	,	4.71239	}},
{{	1.000	,	0.7854	},	{	1.414	,	0.000	},	{	1.000	,	5.4978	}},
{{	1.000	,	2.3562	},	{	1.414	,	3.14159	},	{	1.000	,	3.927	}},
{{	4.000	,	0.000	},	{	5.000	,	-0.6435	},	{	3.000	,	4.71239	}},
{{	4.000	,	2.0944	},	{	5.000	,	2.7379	},	{	3.000	,	3.6652	}},
{{	3.000	,	2.3562	},	{	3.200	,	1.3487	},	{	2.998	,	0.34011	}},
{{	3.000	,	3.14159	},	{	2.300	,	1.9897	},	{	2.946	,	0.794195}}
};

int Get_Data(float * STW, float * HDG, float * TRK, float * SOG) {
	char id[4], buffer[100];
	char * p;
	int i,n;
	char debug = 0;
	float value=0.0;
	
	printf("\r\nEnter the input data with this format: \r\nSTW:1.2345,HDG:124.23,SOG:1.3525,TRK:155.33\r\n");
	// Order doesn't matter
	scanf("%100[^\r]", buffer);
	printf("\r\nReceived: %s\r\n", buffer);

	p = buffer;
	while (!isalpha(*p)) // advance to start of buffer 
			p++;

	for (i=0; i<4; i++) {
		sscanf(p, "%3s:%f,%n", id, &value, &n);
		if (debug)
			printf("\r\nGot it: %s and %f. Read %d chars.\r\n", id, value, n);
		p += n;
		if (debug)
			printf("Remainder is %s\r\n", p);
		if (!strcmp(id, "STW")) {
			*STW = value;
		} else if (!strcmp(id, "HDG")) {
			*HDG = value;
		} else if (!strcmp(id, "TRK")) {
			*TRK = value;
		} else if (!strcmp(id, "SOG")) {
			*SOG = value;
		} else {
			printf("\r\nUnknown identifier: %s\r\n", id);
			return 0;
		}		
	}
	return 1;
}

/*----------------------------------------------------------------------------
  MAIN function
 *----------------------------------------------------------------------------*/
int main (void) {
	float stw = 0;
	float sog = 0;
	float hdg = 0;
	float trk = 0;
	float cspd = 0;
	float cang = 0;
	int i, t, print_approx_results=1;
	char name[13]="ADITYA RASAM";									//%%%%%%%%%%%%%%%%%%%--Modified by Aditya--%%%%%%%%%%%%%%%%%
	char u_id[7]="arasam";												//%%%%%%%%%%%%%%%%%%%--Modified by Aditya--%%%%%%%%%%%%%%%%%

	
	// Phase 1: initialization
	Init_Profiling();
	Init_RGB_LEDs();
	__disable_irq();
	Init_UART0(115200);
	__enable_irq();
	printf("\r\n\nProject 3 Base Code\r\n");
	printf("\r\nName: %s", name);					//%%%%%%%%%%%%%%%%%%%--Modified by Aditya--%%%%%%%%%%%%%%%%%
	printf("\r\nUnity_id: %s", u_id);			//%%%%%%%%%%%%%%%%%%%--Modified by Aditya--%%%%%%%%%%%%%%%%%

	
	Control_RGB_LEDs(1,1,0);

	// Phase 2: Known test cases for profiling and optimizing Compute_Current
	Control_RGB_LEDs(0,1,0);
	for (t=0; t<NUM_TESTS; t++) {
		for (i=0; i<13; i++) {
			stw = Tests[i].BtW.magnitude;
			hdg = Tests[i].BtW.angle;
			sog = Tests[i].BtG.magnitude;
			trk = Tests[i].BtG.angle;
	
			Enable_Profiling();
			TOGGLE_BLUE_LED
			Compute_Current(stw, hdg, sog, trk, &cspd, &cang);
			TOGGLE_BLUE_LED
			Disable_Profiling();
		
			if (print_approx_results) {
				
				printf("\r\nTest %d", i);
				printf("\r\nCorrect Speed: %f kts, Angle: %f deg", Tests[i].WtG.magnitude, Tests[i].WtG.angle*Oneeighty_over_PI);	
				printf("\r\nCalculated Speed: %f kts, Angle: %f deg", cspd, cang*Oneeighty_over_PI);
				if (fabs(cspd-Tests[i].WtG.magnitude) > MAX_MAG_ERROR)
					printf("\r\nXXX Magnitude error!!! XXX");
				if (fabs(cang-Tests[i].WtG.angle) > MAX_ANGLE_ERROR)
					printf("\r\n                XXX Angle error!!! XXX");
			
			}
		}
		print_approx_results = 0; // Don't print out approximation results for any tests after the first set
	}
	printf("\r\n");
	Sort_Profile_Regions();
	Print_Sorted_Profile();
	Control_RGB_LEDs(0,1,0);

	// Phase 3: Process test cases from serial port
	// This code allows testing with input data from the serial port (115.200 kbaud, 8N1)
	while (1) {
		if (Get_Data(&stw, &hdg, &trk, &sog)) {
			printf("STW:%f, HDG:%f, TRK:%f, SOG:%f\r\n", stw, hdg, trk, sog);
			cspd = 0;
			cang = 0;
			//%%%%%%%%%%%%%%%%%%%--Modified by Aditya--%%%%%%%%%%%%%%%%%
			hdg=hdg*PI_over_180;				//Conversion from degrees to radians
			trk=trk*PI_over_180;
			
			TOGGLE_BLUE_LED // Do not delete - used for grading
			Compute_Current(stw, hdg, sog, trk, &cspd, &cang);
			TOGGLE_BLUE_LED	// Do not delete - used for grading
			printf("Current speed: %f, Current direction: %f\r\n", cspd, cang*Oneeighty_over_PI);
		} else {
			printf("Input data format error.\r\n"); 
		}
	}
}
