#include	 <MKL25Z4.H>
#include	 "i2c.h"
#include 	"gpio_defs.h"


//init i2c0
void i2c_init( void )
{
	// Peripheral initialization
	
 //clock i2c peripheral and port E
	SIM->SCGC4		 |= SIM_SCGC4_I2C0_MASK;
	SIM->SCGC5		 |= SIM_SCGC5_PORTE_MASK;

	//set pins to I2C function
	PORTE->PCR[ 24 ] |= PORT_PCR_MUX( 5 );
	PORTE->PCR[ 25 ] |= PORT_PCR_MUX( 5 );

	//set baud rate
	//baud = bus freq/(scl_div+mul)
	//I2C0->F				= ( I2C_F_ICR( 0x11 ) | I2C_F_MULT( 0 ) ); // 0x11
		I2C0->F				= ( I2C_F_ICR( 0x0 ) | I2C_F_MULT( 0 ) ); // 0x11			edit adi

	//enable i2c and set to master mode
	I2C0->C1		 |= ( I2C_C1_IICEN_MASK );

	// Select high drive mode
	I2C0->C2		 |= ( I2C_C2_HDRS_MASK );
	
	// Enable interrupt with NVIC
	// NVIC_EnableIRQ(I2C0_IRQn);
}


void i2c_busy(void){

	// Start Signal
	I2C0->C1 &= ~I2C_C1_IICEN_MASK;
	I2C_TRAN;
	I2C_M_START;
	I2C0->C1 |=  I2C_C1_IICEN_MASK;
	// Write to clear line
	I2C0->C1 |= I2C_C1_MST_MASK; /* set MASTER mode */  
	I2C0->C1 |= I2C_C1_TX_MASK; /* Set transmit (TX) mode */  
	I2C0->D = 0xFF;
	while ((I2C0->S & I2C_S_IICIF_MASK) == 0U) {
	} /* wait interrupt */  
	I2C0->S |= I2C_S_IICIF_MASK; /* clear interrupt bit */  
		
	/* Clear arbitration error flag*/  
	I2C0->S |= I2C_S_ARBL_MASK;
		
	/* Send start */  
	I2C0->C1 &= ~I2C_C1_IICEN_MASK;
	I2C0->C1 |= I2C_C1_TX_MASK; /* Set transmit (TX) mode */  
	I2C0->C1 |= I2C_C1_MST_MASK; /* START signal generated */  
		
	I2C0->C1 |= I2C_C1_IICEN_MASK;
	/*Wait until start is send*/  

	/* Send stop */  
	I2C0->C1 &= ~I2C_C1_IICEN_MASK;
	I2C0->C1 |= I2C_C1_MST_MASK;
	I2C0->C1 &= ~I2C_C1_MST_MASK; /* set SLAVE mode */  
	I2C0->C1 &= ~I2C_C1_TX_MASK; /* Set Rx */  
	I2C0->C1 |= I2C_C1_IICEN_MASK;
	
	/* wait */  
	/* Clear arbitration error & interrupt flag*/  
	I2C0->S |= I2C_S_IICIF_MASK;
	I2C0->S |= I2C_S_ARBL_MASK;
}

void i2c_wait(void) {
	int lock_detect = 75;		//200
	
	while(((I2C0->S & I2C_S_IICIF_MASK)==0) & (lock_detect > 0)) {
		lock_detect--;
	} 
	if (lock_detect <= 0)
		i2c_busy();
	
  I2C0->S |= I2C_S_IICIF_MASK;
}

#if 0 // Version 2: ISR signals i2c_read_bytes to continue in each i2c_wait_nb call
void i2c_wait_nb(void) {
	osSignalWait(1, osWaitForever); // Wait for ISR, which will signal this thread
}

void I2C0_IRQHandler(void) {
	TOGGLE_BIT(DEBUG1_POS);
  I2C0->S |= I2C_S_IICIF_MASK; // Clear flag
	osSignalSet(t_I2C, 1);
	TOGGLE_BIT(DEBUG1_POS);
}

int i2c_read_bytes(uint8_t dev_adx, uint8_t reg_adx, uint8_t * data, uint8_t data_count) {
	uint8_t dummy, num_bytes_read=0;
	
	I2C0->C1 |= I2C_C1_IICIE_MASK; 		// Enable I2C interrupts
	
 	SET_BIT(DEBUG2_POS);
	I2C_TRAN;													//	set to transmit mode							
	I2C_M_START;											//	send start										
	I2C0->D = dev_adx;								//	send dev address (write)							
	i2c_wait_nb();												//	wait for completion								

	I2C0->D = reg_adx;								//	send register address								
	i2c_wait_nb();												//	wait for completion								

	I2C_M_RSTART;											//	repeated start									
	I2C0->D = dev_adx | 0x01 ;				//	send dev address (read)							
	i2c_wait_nb();												//	wait for completion								

	I2C_REC;													//	set to receive mode								

	ACK;										
	dummy = I2C0->D;								//	dummy read to start Rx of first byte										
	i2c_wait_nb();											//	wait for completion								
	
	do {
		if (num_bytes_read == data_count-2) { // last read
			NACK;
			data[num_bytes_read++] = I2C0->D; //	read data										
			i2c_wait_nb();										//	wait for completion				
			I2C_M_STOP;												//	send stop										
		} else {
			ACK;
			data[num_bytes_read++] = I2C0->D; //	read data										
			i2c_wait_nb();											//	wait for completion								
		}
	} while (num_bytes_read < data_count-1);
	I2C0->C1 &= ~I2C_C1_IICIE_MASK; 	// Disable I2C interrupts
	CLEAR_BIT(DEBUG2_POS);
	return 1;
}

#endif

#if 0 // state machine in ISR
// Variables for ISR to use for current transaction
uint8_t i_dev_adx;
uint8_t i_reg_adx;
uint8_t * i_data;
uint8_t i_data_count;
uint8_t i_num_bytes_read;

// State machine version of ISR to reduce OS overhead
void I2C0_IRQHandler(void) {
	static int next_state=0;
	uint8_t dummy;
	
	SET_BIT(DEBUG1_POS);
  I2C0->S |= I2C_S_IICIF_MASK; // Clear flag

	switch (next_state) {
		case 0:
			I2C0->D = i_reg_adx;								//	send register address								
			next_state = 1;
			break;
		case 1:
			I2C_M_RSTART;											//	repeated start									
			I2C0->D = i_dev_adx | 0x01 ;				//	send dev address (read)							
			next_state = 2;
			break;
		case 2:
			I2C_REC;													//	set to receive mode								
			ACK;										
			dummy = I2C0->D;								//	dummy read to start Rx of first byte										
			next_state = 3;
			break;
		case 3:
			if (i_num_bytes_read == i_data_count-2) { // last read
				NACK;
				i_data[i_num_bytes_read++] = I2C0->D; //	read data										
				next_state = 4;
			} else {
				ACK;
				i_data[i_num_bytes_read++] = I2C0->D; //	read data										
				if (i_num_bytes_read < i_data_count-1) {
					next_state = 3;
				} else { 
					next_state = 4;
				}
			}
			break;
		case 4:
			I2C_M_STOP;												//	send stop										
			I2C0->C1 &= ~I2C_C1_IICIE_MASK; 	// Disable I2C interrupts
			next_state = 0;
			break;
		default:
			next_state = 0;
			break;
	}
	CLEAR_BIT(DEBUG1_POS);
}

int i2c_read_bytes(uint8_t dev_adx, uint8_t reg_adx, uint8_t * data, uint8_t data_count) {
	// Load up ISR's copy of relevant variables
	static int n_idle;
	
	SET_BIT(DEBUG2_POS);
	idle_counter = 0;
	i_dev_adx = dev_adx;
	i_reg_adx = reg_adx;
	i_data = data;
	i_data_count = data_count;
	i_num_bytes_read=0;
	I2C0->C1 |= I2C_C1_IICIE_MASK; 		// Enable I2C interrupts
	I2C_TRAN;													//	set to transmit mode							
	I2C_M_START;											//	send start										
	I2C0->D = dev_adx;								//	send dev address (write)							
	
	n_idle = idle_counter;
	CLEAR_BIT(DEBUG2_POS);
	return 1;
}
#endif


int i2c_read_bytes(uint8_t dev_adx, uint8_t reg_adx, uint8_t * data, uint8_t data_count) {
	uint8_t dummy, num_bytes_read=0;
	
	I2C_TRAN;													//	set to transmit mode							
	SET_BIT(DEBUG2_POS);
	I2C_M_START;											//	send start										
	I2C0->D = dev_adx;								//	send dev address (write)							
	i2c_wait();												//	wait for completion								

	I2C0->D = reg_adx;								//	send register address								
	i2c_wait();												//	wait for completion								

	I2C_M_RSTART;											//	repeated start									
	I2C0->D = dev_adx | 0x01 ;				//	send dev address (read)							
	i2c_wait();												//	wait for completion								

	I2C_REC;													//	set to receive mode								

	ACK;										
	dummy = I2C0->D;								//	dummy read to start Rx of first byte										

	do {
		i2c_wait();											//	wait for completion								
		if (num_bytes_read == data_count-2) { // last read
			NACK;
			data[num_bytes_read++] = I2C0->D; //	read data										
			i2c_wait();												//	wait for completion				
			I2C_M_STOP;										//	send stop										
		} else {
			ACK;
			data[num_bytes_read++] = I2C0->D; //	read data										
		}
	} while (num_bytes_read < data_count);

	CLEAR_BIT(DEBUG2_POS);
	return 1;
}

int i2c_write_bytes(uint8_t dev_adx, uint8_t reg_adx, uint8_t * data, uint8_t data_count) {
	uint8_t num_bytes_written=0;
	
	I2C_TRAN;													//	set to transmit mode							
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
	
	return 1;
}
