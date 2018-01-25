#include	 <MKL25Z4.H>
#include	 "i2c.h"
#include 	"mma8451.h" 
#include 	"gpio_defs.h"
#include "LEDs.h"
#include "delay.h"

//init i2c0
//uint8_t new_data = 0;


volatile uint8_t 
				dummy, 
				irq_data[7],
				Prog_flow=1, 
				ready_to_read = 0, 
				last_byte_read=0,
				start_to_read=0, 
				ready_to_transmit=1,
				data_up=0;

static uint8_t is_last_read_1=0;
volatile uint8_t transfer_complete = 1;

//uint8_t tmp_dev_adx, tmp_reg_adx, tmp_data[6], tmp_data_count,p=0, second_last_byte_read;
uint8_t Mode_block = 0, Mode_FSM_Poll = 0, Mode_ISR = 0;


static uint8_t num_bytes_read = 0;

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
	PTB->PSOR = MASK(DEBUG1_I2C_busy);							//-------- Edit Aditya--------------------------
	while( ( I2C0->S & I2C_S_IICIF_MASK ) == 0 ) {
		;
	}
	I2C0->S |= I2C_S_IICIF_MASK;
	PTB->PCOR = MASK(DEBUG1_I2C_busy);							//-------- Edit Aditya--------------------------
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
	


		I2C0->S |= I2C_S_IICIF_MASK;
		
			if (is_last_read_1){
					I2C_M_STOP;										//	send stop		
					PTB->PCOR = MASK(DEBUG1_I2C_msg);		
				
					}			
	
		irq_data[num_bytes_read++] = I2C0->D; //	read data

	
		
	
transfer_complete = 1;
}

int i2c_read_bytes_ISR(uint8_t dev_adx, uint8_t reg_adx, uint8_t * data, uint8_t data_count) {
	
		static uint8_t dummy, seq;//, num_bytes_read=0; 
		

		static enum {START_TRAN, SRA, RSTART, DataCHECK, Final_State, POLL_STATUS} next_state_1 = START_TRAN;

		

		PTB->PSOR = MASK(DEBUG1_I2C_exec);							//-------- Edit Aditya--------------------------
		
		
		switch(next_state_1){
		
		case START_TRAN:
					seq = 1;
					next_state_1 = POLL_STATUS;
					num_bytes_read = 0;
					I2C_TRAN;													//	set to transmit mode	
					PTB->PSOR = MASK(DEBUG1_I2C_msg);							//-------- Edit ADITYA--------------------	
					I2C_M_START;											//	send start			
					I2C0->D = dev_adx;								//	send dev address (write)
		
			
		
			break;
		
		case POLL_STATUS:
		
		if( ( I2C0->S & I2C_S_IICIF_MASK ) == 0 ) {
			next_state_1 = POLL_STATUS;
		}
		else
		{
		I2C0->S |= I2C_S_IICIF_MASK;         //set iicif bit

					if (seq==1){
				next_state_1 = SRA;
					}
					if (seq==2){
				next_state_1 = RSTART;
					}
					if (seq==3){
				next_state_1 = DataCHECK;
					}
				}
			break;
		
		
		
		case SRA:
			seq = 2;
			next_state_1 = POLL_STATUS;
			I2C0->D = reg_adx;								//	send register address

		
			
			
			break;

		
		case RSTART:
			seq = 3;
			next_state_1 = POLL_STATUS;
		
			I2C_M_RSTART;											//	repeated start
			I2C0->D = dev_adx | 0x01 ;				//	send dev address (read)

				
			
	
			break;	
	
		
	//}
		
		case DataCHECK:
			transfer_complete = 0;
					I2C_REC;													//	set to receive mode	


		Initialize_Interrupts();
			if (num_bytes_read < data_count) {	
				
					is_last_read_1 = (num_bytes_read == data_count-1)? 1: 0; // setting last read whn num_bytes_read = 5 which is last read

				if(is_last_read_1 == 1){
						next_state_1 = Final_State;
						NACK;
						}
				else{
						next_state_1 = DataCHECK;	
						ACK;
						}
			dummy = I2C0->D;									// start recieve operation	

				
					
					
			}
		break;
			
			
			
		//x=0;
		case Final_State:
				
						next_state_1 = START_TRAN;
						msg_done = 1;
						transfer_complete = 1;
						num_bytes_read=0;
						__disable_irq();
						data[1] = irq_data[1];
						data[2] = irq_data[2];
						data[3] = irq_data[3];
						data[4] = irq_data[4];
						data[5] = irq_data[5];
						data[6] = irq_data[6];
		
		
		
		
		break;
}
	
PTB->PCOR = MASK(DEBUG1_I2C_exec);
return 1;
}
 


int i2c_read_bytes_fsm(uint8_t dev_adx, uint8_t reg_adx, uint8_t * data, uint8_t data_count) {
	
		static uint8_t dummy, is_last_read=0, seq; 
		static uint8_t num_bytes_read=0;
	
//		static uint8_t ; 
	
		static enum {START_TRAN, POLL_STATUS, SRA, RSTART, DUPDATE, REC, DCHECK} next_state = START_TRAN;
		PTB->PSOR = MASK(DEBUG1_I2C_exec);							//-------- Edit Aditya--------------------------

		switch(next_state){
		
		case START_TRAN:
			seq = 1;
			//new_data = 0;
			I2C_TRAN;													//	set to transmit mode	
			PTB->PSOR = MASK(DEBUG1_I2C_msg);							//-------- Edit ADITYA--------------------	
			I2C_M_START;											//	send start			
			I2C0->D = dev_adx;								//	send dev address (write)
			next_state = POLL_STATUS;

			break;
		
		case POLL_STATUS:
				//		new_data = 0;
		//cont_poll= ( ( I2C0->S & I2C_S_IICIF_MASK ) == 0 )?1:0;
		
		if( ( I2C0->S & I2C_S_IICIF_MASK ) == 0 ) {
			next_state = POLL_STATUS;
			//new_data = 0;
		}
		else
		{
		I2C0->S |= I2C_S_IICIF_MASK;         //set iicif bit

					if (seq==1){
				next_state = SRA;
					}
					if (seq==2){
				next_state = RSTART;
					}
					if (seq==3){
						next_state = REC;
					}
					if (seq==4){
						next_state = DUPDATE;
					}
				}
			break;
	
		
//-------------------------------------------------------		
//			i2c_wait();												//	wait for completion								
//-------------------------------------------------------
		
		case SRA:
			//new_data = 0;
			seq = 2;
			I2C0->D = reg_adx;								//	send register address	
			next_state = POLL_STATUS;
			break;
//------------------------------------------------------------		
//	i2c_wait();												//	wait for completion								
//------------------------------------------------------------
//---------THIS STATE CALLED FROM STATUS CHK AFTER SRA		
		case RSTART:
			//new_data = 0;
			seq =3;
			I2C_M_RSTART;											//	repeated start
			I2C0->D = dev_adx | 0x01 ;				//	send dev address (read)	
		
			next_state = POLL_STATUS;
			break;	
	
		
		case REC:
			//new_data = 0;
			I2C_REC;													//	set to receive mode					
			next_state = DCHECK;
			break;
		
		case DCHECK:
			//			new_data = 0;
			seq=4;
			if (num_bytes_read < data_count) {
				is_last_read = (num_bytes_read == data_count-1)? 1: 0;
				
			if (is_last_read){
					NACK;													// tell HW to send NACK after read							
				} 
			else {
					ACK;													// tell HW to send ACK after read								
				}
				dummy = I2C0->D;								//	dummy read	
					next_state = POLL_STATUS;
				
				}
			else{
					x=1;
					next_state = START_TRAN;
					num_bytes_read = 0;
					is_last_read = 0;
			}		
		break;
				

		case DUPDATE:
		if (is_last_read){
					I2C_M_STOP;										//	send stop		
					PTB->PCOR = MASK(DEBUG1_I2C_msg);			
		}			
    next_state = DCHECK;
		data[num_bytes_read++] = I2C0->D; //	read data
			break;
			default:
				next_state = START_TRAN;
			break;

}
PTB->PCOR = MASK(DEBUG1_I2C_exec);							//-------- Edit Aditya
return 1;
}





	int i2c_read_bytes(uint8_t dev_adx, uint8_t reg_adx, uint8_t * data, uint8_t data_count) {
	
	uint8_t dummy, num_bytes_read=0, is_last_read=0;
	
	PTB->PSOR = MASK(DEBUG1_I2C_exec);							//-------- Edit Aditya--------------------------
	
	I2C_TRAN;													//	set to transmit mode	
	PTB->PSOR = MASK(DEBUG1_I2C_msg);							//-------- Edit ADITYA--------------------	
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
	PTB->PCOR = MASK(DEBUG1_I2C_msg);							//-------- Edit ADITYA--------------------
			
		}
		data[num_bytes_read++] = I2C0->D; //	read data										
	}
		PTB->PCOR = MASK(DEBUG1_I2C_exec);							//-------- Edit Aditya

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


//------------------------------------Edit_Aditya_Mode2_FSM-----------------------------------------


