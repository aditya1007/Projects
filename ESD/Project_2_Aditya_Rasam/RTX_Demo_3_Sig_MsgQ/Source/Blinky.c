#include <cmsis_os.h>
#include <MKL25Z4.H>
#include "gpio_defs.h"
#include "system_MKL25Z4.h"             // Keil::Device:Startup

/*
	Wiring Needed:
			Switch 1: SW1_POS PTD7	(J2-19)
			Switch 2: SW2_POS PTD6	(J2-17)
*/

#define NUM_Q_ENT 16
#define MMA_ADDR 0x3A
#define REG_CTRL1  0x2A
#define REG_XHI 0x01
#define ACC_SENSITIVITY 60
#define FLASH_DELAY 10

typedef struct{
	uint8_t 	RD_WR;
	uint8_t 	DEV_ADDR;
	uint8_t 	REG_ADDR;	
	uint8_t 	NO_OF_BYTES;	
	uint8_t 	DATA[7];
	osSemaphoreId 	SEMA_TRIG;
}I2C_data;
static uint8_t acq_data[6];
int16_t acc_X=0, acc_Y=0, acc_Z=0;
uint8_t Fls=0, Icomm=0;
uint8_t I2C_MODE=1;
static uint8_t rd_wr, dev_addr, reg_addr, no_of_bytes, data[6];
uint8_t Mode_block = 0, Mode_ISR_PerByte = 0, Mode_ISR_PerTranc = 0;

extern int counter;
int count_value;

osThreadId t_F;  
osThreadId t_I2C;


void Thread_Flash(void const * arg);
void Thread_I2C(void const * arg);												//---Edit_adi_Prj2


osThreadDef(Thread_Flash, osPriorityNormal, 1, 0);
osThreadDef(Thread_I2C, osPriorityNormal, 1, 0);					//---Edit_adi_Prj2

// Synchronization
osSemaphoreId RGB_sem;
osSemaphoreDef(RGB_sem);

osSemaphoreId RGB_sem1;
osSemaphoreDef(RGB_sem1);

void i2c_init(void);
void i2c_wait( void );


/*
struct I2C_data  {
  uint8_t                 dev_addr;    ///< number of items (elements) in the pool
  uint8_t                 reg_addr;    ///< size of an item
	uint8_t                 no_of_bytes; 
	uint8_t                 data[6]; 
	//uint8_t                 dev_addr; 
  //void                       *pool;    ///< pointer to memory for pool
} I2C_data;

*/


osPoolDef(mpool, NUM_Q_ENT, I2C_data);                    // Define memory pool
osPoolId  mpool;
osMessageQId I2C_msgq;																	//---Edit_adi_Prj2
osMessageQDef(I2C_msgq, NUM_Q_ENT, &I2C_data);						//---Edit_adi_Prj2


uint32_t g_RGB_delay=700; 	// delay for RGB sequence

void Initialize_Ports(void) {
	// Enable clock to ports A, B and D
	SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTD_MASK;;
	
	// Make 3 pins GPIO
	PORTB->PCR[RED_LED_POS] &= ~PORT_PCR_MUX_MASK;          
	PORTB->PCR[RED_LED_POS] |= PORT_PCR_MUX(1);          
	PORTB->PCR[GREEN_LED_POS] &= ~PORT_PCR_MUX_MASK;          
	PORTB->PCR[GREEN_LED_POS] |= PORT_PCR_MUX(1);          
	PORTD->PCR[BLUE_LED_POS] &= ~PORT_PCR_MUX_MASK;          
	PORTD->PCR[BLUE_LED_POS] |= PORT_PCR_MUX(1);          
	
	// Set LED port bits to outputs
	PTB->PDDR |= MASK(RED_LED_POS) | MASK(GREEN_LED_POS);
	PTD->PDDR |= MASK(BLUE_LED_POS);

	// Select port D on pin mux, enable pull-up resistors
	PORTD->PCR[SW1_POS] = PORT_PCR_MUX(1) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK;
	PORTD->PCR[SW2_POS] = PORT_PCR_MUX(1) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK;

	// Clear switch bits to input
	PTD->PDDR &= ~MASK(SW1_POS); 
	PTD->PDDR &= ~MASK(SW2_POS); 
	
	// Turn off LEDs
	PTB->PSOR |= MASK(RED_LED_POS) | MASK(GREEN_LED_POS);
	PTD->PSOR |= MASK(BLUE_LED_POS);
}





void Init_Debug_Signals(void) {
		//------Edited by Aditya------------
	SIM->SCGC5 |=  SIM_SCGC5_PORTB_MASK;;
	
	PORTB->PCR[DEBUG1_I2C_exec] &= ~PORT_PCR_MUX_MASK;          
	PORTB->PCR[DEBUG1_I2C_exec] |= PORT_PCR_MUX(1);
	
	PORTB->PCR[DEBUG1_I2C_msg] &= ~PORT_PCR_MUX_MASK;          
	PORTB->PCR[DEBUG1_I2C_msg] |= PORT_PCR_MUX(1);
	
	PORTB->PCR[3] &= ~PORT_PCR_MUX_MASK;          
	PORTB->PCR[3] |= PORT_PCR_MUX(1);
	
	
	PTB->PDDR |= MASK(DEBUG1_I2C_exec)| MASK(DEBUG1_I2C_msg)| MASK(3);

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
Mode_ISR_PerByte = 0;
Mode_ISR_PerTranc = 0;
}
else
{
	Mode_block = 0;
}
	
if (!(SWITCH_PRESSED(CONFIG_SW_5)) & ((SWITCH_PRESSED(4))) & (!(SWITCH_PRESSED(CONFIG_SW_3)))){
Mode_block = 0;
Mode_ISR_PerByte = 1;
Mode_ISR_PerTranc = 0;
}
else
{
	Mode_ISR_PerByte = 0;
}

if (!(SWITCH_PRESSED(CONFIG_SW_5))&(!(SWITCH_PRESSED(CONFIG_SW_4)))&((SWITCH_PRESSED(3)))){
Mode_block = 0;
Mode_ISR_PerByte = 0;
Mode_ISR_PerTranc = 1;
}
else { 
Mode_ISR_PerTranc = 0;
}
}
void Initialize_Interrupts(void) {
	/* Configure PORT peripheral. Select GPIO and enable pull-up 
	resistors and interrupts on all edges for pins connected to switches */


	NVIC_SetPriority(I2C0_IRQn, 8);
	NVIC_ClearPendingIRQ(I2C0_IRQn); 
	NVIC_EnableIRQ(I2C0_IRQn);
		__enable_irq();
	I2C0->C1 |= I2C_C1_IICIE_MASK;

}



void I2C0_IRQHandler (void){
	static uint8_t dummy, num_bytes_read=0, is_last_read=0, T=0;
	static enum {SRA, RSTART, DataCHECK, DataREAD} next_state_1 = SRA;

	if (Mode_ISR_PerByte){

		PTB->PSOR = MASK(DEBUG1_I2C_exec);	//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
		I2C0->S |= I2C_S_IICIF_MASK;
		//PTB->PSOR = MASK(DEBUG1_I2C_exec);	//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
		osSignalSet(t_I2C, 1);
		PTB->PCOR = MASK(DEBUG1_I2C_exec);										//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
	

									}
	else if (Mode_ISR_PerTranc){
					PTB->PSOR = MASK(DEBUG1_I2C_exec);	//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

				switch(next_state_1){

		case SRA:

					I2C0->S |= I2C_S_IICIF_MASK;
					next_state_1 = RSTART;
					I2C0->D = reg_addr;								//	send register address	
			break;

			case RSTART:
					I2C0->S |= I2C_S_IICIF_MASK;
					next_state_1 = DataCHECK;
					I2C_M_RSTART;											//	repeated start									
					I2C0->D = dev_addr | 0x01 ;				//	send dev address (read)							
			break;

			case DataCHECK:

					I2C0->S |= I2C_S_IICIF_MASK;
					next_state_1 = DataREAD;
					I2C_REC;													//	set to receive mode								
					if (num_bytes_read < no_of_bytes) {
							is_last_read = (num_bytes_read == no_of_bytes-1)? 1: 0;
							if (is_last_read){
									NACK;													// tell HW to send NACK after read	
													} 
							else {
										ACK;													// tell HW to send ACK after read								
										}
									}
							dummy = I2C0->D;								//	dummy read	
		
			break;

			case DataREAD:

					if (is_last_read){
					I2C_M_STOP;										//	send stop		
					PTB->PCOR = MASK(DEBUG1_I2C_msg);							//-------- Edit ADITYA--------------------

					//osSignalSet(t_I2C, 1);	
					next_state_1 = SRA;
					I2C0->S |= I2C_S_IICIF_MASK;
					count_value = counter;
					count_value = count_value;

	//	//			PTB->PCOR = MASK(DEBUG1_I2C_exec);	//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

														} 
							else {
										next_state_1 = DataCHECK;
										}
								data[num_bytes_read++] = I2C0->D; //	read data	
					if (is_last_read){
						num_bytes_read = 0;
						osSignalSet(t_I2C, 1);	
															} 						

			break;

			
			/*case Final_State:
					I2C0->S |= I2C_S_IICIF_MASK;
					next_state_1 = SRA;
					I2C_M_STOP;										//	send stop		
					osSignalSet(t_I2C, 1);
					PTB->PCOR = MASK(DEBUG1_I2C_exec);										//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

			break;*/

	}

				PTB->PCOR = MASK(DEBUG1_I2C_exec);

	}
}

void Init_RGB_LEDs(void) {
	// Enable clock to ports B and D
	SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTD_MASK;
	
	// Make 3 pins GPIO
	PORTB->PCR[RED_LED_POS] &= ~PORT_PCR_MUX_MASK;          
	PORTB->PCR[RED_LED_POS] |= PORT_PCR_MUX(1);          
	PORTB->PCR[GREEN_LED_POS] &= ~PORT_PCR_MUX_MASK;          
	PORTB->PCR[GREEN_LED_POS] |= PORT_PCR_MUX(1);          
	PORTD->PCR[BLUE_LED_POS] &= ~PORT_PCR_MUX_MASK;          
	PORTD->PCR[BLUE_LED_POS] |= PORT_PCR_MUX(1);          
	
	// Set ports to outputs
	PTB->PDDR |= MASK(RED_LED_POS) | MASK(GREEN_LED_POS);
	PTD->PDDR |= MASK(BLUE_LED_POS);
}




void Control_RGB_LEDs(int r_on, int g_on, int b_on) {
	if (r_on)
		PTB->PCOR = MASK(RED_LED_POS);
	else
		PTB->PSOR = MASK(RED_LED_POS);
	if (g_on)
		PTB->PCOR = MASK(GREEN_LED_POS);
	else
		PTB->PSOR = MASK(GREEN_LED_POS);
	if (b_on)
		PTD->PCOR = MASK(BLUE_LED_POS);
	else
		PTD->PSOR = MASK(BLUE_LED_POS);
}


void i2c_init( void )
{
 //clock i2c peripheral and port E
	SIM->SCGC4		 |= SIM_SCGC4_I2C0_MASK;
	SIM->SCGC5		 |= SIM_SCGC5_PORTE_MASK;

	//set pins to I2C function
	PORTE->PCR[ 24 ] |= PORT_PCR_MUX( 5 );
	PORTE->PCR[ 25 ] |= PORT_PCR_MUX( 5 );

	//set baud rate
	//baud = bus freq/(scl_div+mul)
	I2C0->F				= ( I2C_F_ICR( 0x11 ) | I2C_F_MULT( 0 ) );

	//enable i2c and set to master mode
	I2C0->C1		 |= ( I2C_C1_IICEN_MASK );

	// Select high drive mode
	I2C0->C2		 |= ( I2C_C2_HDRS_MASK );
}


void i2c_wait( void )
{
//	PTB->PSOR = MASK(3);							//-------- Edit Aditya--------------------------
	while( ( I2C0->S & I2C_S_IICIF_MASK ) == 0 ) {
		;
	}
	I2C0->S |= I2C_S_IICIF_MASK;
//	PTB->PCOR = MASK(3);							//-------- Edit Aditya--------------------------
}


int i2c_read_bytes(uint8_t dev_adx, uint8_t reg_adx, uint8_t * data, uint8_t data_count) {
	
	uint8_t dummy, num_bytes_read=0, is_last_read=0;
	
	
	I2C_TRAN;													//	set to transmit mode	
	PTB->PSOR = MASK(DEBUG1_I2C_msg);												//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
	I2C_M_START;											//	send start										
	I2C0->D = dev_adx;								//	send dev address (write)							
	i2c_wait();												//	wait for completion								

	I2C0->D = reg_adx;								//	send register address								
	i2c_wait();												//	wait for completion								

	I2C_M_RSTART;											//	repeated start									
	I2C0->D = dev_adx | 0x01 ;				//	send dev address (read)							
	i2c_wait();												//	wait for completion								

	I2C_REC;													//	set to receive mode								
	while (num_bytes_read < data_count) {
		is_last_read = (num_bytes_read == data_count-1)? 1: 0;
		if (is_last_read){
			NACK;													// tell HW to send NACK after read							
		} else {
			ACK;													// tell HW to send ACK after read								
		}

		dummy = I2C0->D;								//	dummy read										
		i2c_wait();											//	wait for completion								

		if (is_last_read){
			I2C_M_STOP;										//	send stop		
		PTB->PCOR = MASK(DEBUG1_I2C_msg);											//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
			
		}
		data[num_bytes_read++] = I2C0->D; //	read data										
	}

	return 1;

}


int i2c_read_bytes_mode_2(uint8_t dev_adx, uint8_t reg_adx, uint8_t * data, uint8_t data_count) {
	osEvent result; 
	uint8_t dummy, num_bytes_read=0, is_last_read=0;
	
//	PTB->PSOR = MASK(DEBUG1_I2C_exec);							//-------- Edit Aditya--------------------------
	
	I2C_TRAN;													//	set to transmit mode	
	PTB->PSOR = MASK(DEBUG1_I2C_msg);												//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
	counter = 0;																						// CLearing the counter				
	I2C_M_START;											//	send start										
	I2C0->D = dev_adx;								//	send dev address (write)		
	result = osSignalWait(1, osWaitForever);															//SSSSSSSSSSSSSSSS
//	PTB->PSOR = MASK(DEBUG1_I2C_msg);												//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

	//i2c_wait();												//	wait for completion								

	I2C0->D = reg_adx;								//	send register address								
	result = osSignalWait(1, osWaitForever);															//SSSSSSSSSSSSSSSS
	//i2c_wait();												//	wait for completion								

	I2C_M_RSTART;											//	repeated start									
	I2C0->D = dev_adx | 0x01 ;				//	send dev address (read)							
	result = osSignalWait(1, osWaitForever);															//SSSSSSSSSSSSSSSS

	//i2c_wait();												//	wait for completion								

	I2C_REC;													//	set to receive mode								
	while (num_bytes_read < data_count) {
		is_last_read = (num_bytes_read == data_count-1)? 1: 0;
		if (is_last_read){
			NACK;													// tell HW to send NACK after read							
		} else {
			ACK;													// tell HW to send ACK after read								
		}

		dummy = I2C0->D;								//	dummy read				
		result = osSignalWait(1, osWaitForever);														//SSSSSSSSSSSSSSSS
		//i2c_wait();											//	wait for completion								

		if (is_last_read){
			I2C_M_STOP;										//	send stop		
		PTB->PCOR = MASK(DEBUG1_I2C_msg);											//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
			count_value = counter;
			count_value = count_value;
		}
		data[num_bytes_read++] = I2C0->D; //	read data										
	}
						if (is_last_read){
						num_bytes_read = 0;
															} 						
	//	PTB->PCOR = MASK(DEBUG1_I2C_exec);							//-------- Edit Aditya

	return 1;

}


int i2c_read_bytes_mode_3(uint8_t dev_adx, uint8_t reg_adx, uint8_t * data, uint8_t data_count) {
	osEvent result1; 
//	uint8_t dummy, num_bytes_read=0, is_last_read=0;
	
	
	I2C_TRAN;													//	set to transmit mode	
	PTB->PSOR = MASK(DEBUG1_I2C_msg);												//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
	counter = 0;
	I2C_M_START;											//	send start										
	I2C0->D = dev_adx;								//	send dev address (write)		
		
	result1 = osSignalWait(1, osWaitForever);															//SSSSSSSSSSSSSSSS
//	PTB->PSOR = MASK(DEBUG1_I2C_msg);												//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//	PTB->PCOR = MASK(DEBUG1_I2C_msg);												//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

	return 1;

}

int i2c_write_bytes(uint8_t dev_adx, uint8_t reg_adx, uint8_t * data, uint8_t data_count) {
	uint8_t num_bytes_written=0;
	
	I2C_TRAN;													//	set to transmit mode	

	PTB->PSOR = MASK(DEBUG1_I2C_msg);							//-------- Edit ADITYA--------------------
	
	I2C_M_START;											//	send start										
	I2C0->D = dev_adx;								//	send dev address (write)							
	i2c_wait();												//	wait for completion								

	I2C0->D = reg_adx;								//	send register address								
	i2c_wait();												//	wait for completion								

	while (num_bytes_written < data_count) {
		I2C0->D = data[num_bytes_written++]; //	write data										
		i2c_wait();											//	wait for completion								
	}
	I2C_M_STOP;												//		send stop	
	PTB->PCOR = MASK(DEBUG1_I2C_msg);							//-------- Edit ADITYA--------------------
	
	
	return 1;
}







int init_mma()
{
	uint8_t data[1];

	//set active mode, 14 bit samples, 2g full scale and 800 Hz ODR 
	data[0] = 0x01;
	i2c_write_bytes(MMA_ADDR, REG_CTRL1, data, 1);
	return 1;
}


void Thread_Flash(void const * arg){
	I2C_data *mptr;
	int n=0;
	static int16_t prev_acc_X=0, prev_acc_Y=0, prev_acc_Z=0;
	volatile static int32_t F=0;

	
				mptr = osPoolAlloc(mpool);            // Allocate memory for the message
				mptr->RD_WR= 1;                        // Set the message content
				mptr->DEV_ADDR = MMA_ADDR;
				mptr->REG_ADDR = REG_XHI;
				mptr->NO_OF_BYTES= 6;                        
				prev_acc_X = acc_X;
				prev_acc_Y = acc_Y;
				prev_acc_Z = acc_Z;
	
	
	//osThreadYield();
	while(1){
		
	
					//	osSemaphoreRelease(RGB_sem);
					mptr->SEMA_TRIG = RGB_sem;
					osMessagePut(I2C_msgq,(uint32_t)mptr, osWaitForever);

					F = osSemaphoreWait(RGB_sem, osWaitForever);	
																	//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

//----------------------------ACC PROCESSING--------------------------------------------
				if (mptr->RD_WR == 1){
					if ((abs(prev_acc_X - acc_X) > ACC_SENSITIVITY) || 
						(abs(prev_acc_Y - acc_Y) > ACC_SENSITIVITY) || 
						(abs(prev_acc_Z - acc_Z) > ACC_SENSITIVITY)) {

					//				PTB->PSOR = MASK(3);													//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

			// Flash LEDs				
				for (n=0; n<2; n++) {
					Control_RGB_LEDs(1, 1, 1);
					osDelay(FLASH_DELAY);
					Control_RGB_LEDs(0, 0, 0);							
					osDelay(FLASH_DELAY*2);		
				}

				prev_acc_X = acc_X;
				prev_acc_Y = acc_Y;
				prev_acc_Z = acc_Z;
		//PTB->PCOR = MASK(3);												//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
		osDelay(100);
			}		
		}
		
					}
																		}

void Thread_I2C(void const * arg)
	{
		osEvent result; 
		I2C_data *rptr;
		static int16_t temp_data[3];
		int i=0;

	
	volatile static int32_t t=0;
		
	while (1) {
		
		result = osMessageGet(I2C_msgq, osWaitForever);
				if (result.status == osEventMessage) {

					
						rptr = (I2C_data*) result.value.p;
					
					
					//t = osSemaphoreWait(rptr->SEMA_TRIG, osWaitForever);

					rd_wr=rptr->RD_WR;
					dev_addr=rptr->DEV_ADDR;
					dev_addr=dev_addr;
					reg_addr=rptr->REG_ADDR;
					no_of_bytes=rptr->NO_OF_BYTES;
			
					if (rd_wr==1){
						if (Mode_block){
								i2c_read_bytes(dev_addr, reg_addr, data, no_of_bytes);;
															}
						else if (Mode_ISR_PerByte){
							Initialize_Interrupts();
						i2c_read_bytes_mode_2(dev_addr, reg_addr, data, no_of_bytes);
																		}
						else if (Mode_ISR_PerTranc){
							Initialize_Interrupts();
						i2c_read_bytes_mode_3(dev_addr, reg_addr, data, no_of_bytes);
																		}
							for ( i=0; i<3; i++ ) {
								temp_data[i] = (int16_t) ((data[2*i]<<8) | data[2*i+1]);
															}
								acc_X = temp_data[0]/4;
							//	acc_X = acc_X;										
								acc_Y = temp_data[1]/4;
							//	acc_Y=acc_Y;
								acc_Z = temp_data[2]/4;
							//	acc_Z=acc_Z;
														}

								osSemaphoreRelease(rptr->SEMA_TRIG);															

														//	PTB->PCOR = MASK(DEBUG1_I2C_msg);												//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
										
						//		osPoolFree(mpool, rptr);
																							}
					
						}
	
	}


	






int main (void) {
	
	Init_RGB_LEDs();
	Init_Debug_Signals();
	Init_Config_Signals();
Mode_selection ();
	Control_RGB_LEDs(1, 1, 0);								/* yellow: starting up */
	i2c_init();																/* init i2c	*/
	osDelay(200);
	
	if (!init_mma()) {												/* init mma peripheral */
		Control_RGB_LEDs(1, 0, 0);							/* Light red error LED */
		while (1)																/* not able to initialize mma */
			;
	}
	Control_RGB_LEDs(0, 0, 0);							

	osDelay(50);

	osKernelInitialize();
//	Initialize_Ports();
		RGB_sem = osSemaphoreCreate(osSemaphore(RGB_sem), 0);
	
		RGB_sem1 = osSemaphoreCreate(osSemaphore(RGB_sem1), 0);
	mpool = osPoolCreate(osPool(mpool)); 
	I2C_msgq = osMessageCreate(osMessageQ(I2C_msgq), NULL);
	t_F = osThreadCreate(osThread(Thread_Flash), NULL);
	t_I2C = osThreadCreate(osThread(Thread_I2C), NULL);
	osKernelStart(); 
}

