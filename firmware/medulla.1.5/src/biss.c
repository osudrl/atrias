#include "biss_bang.h"

void initBiSS_bang(PORT_t * port, int8_t CLK_bm, int8_t DAT_bm) {
	port->OUTSET = CLK_bm;
	port->DIRSET = CLK_bm;
	port->DIRCLR = DAT_bm;
}

uint8_t readBiSS_bang(uint8_t *data, PORT_t * port, uint8_t CLK_bm, uint8_t DAT_bm) {

	int8_t i;
	int8_t pin = 7;
	uint8_t tmp[4] = {0,0,0,0};
	uint8_t status = 0xFF;

	if ((port->IN & DAT_bm) == 0) return status;								// the device isn't ready - bail
	
	status = 0;
	
//	PORTH.OUTCLR = (1<<7);
	
	// knock twice...
	port->OUTTGL	= CLK_bm;	// down edge
//	__asm__ volatile ( "nop" );
//	__asm__ volatile ( "nop" );
//	__asm__ volatile ( "nop" );
//	__asm__ volatile ( "nop" );
	_delay_us(1);
	port->OUTTGL	= CLK_bm;	// up edge
//	__asm__ volatile ( "nop" );
//	__asm__ volatile ( "nop" );
//	__asm__ volatile ( "nop" );
//	__asm__ volatile ( "nop" );
	_delay_us(1);
	port->OUTTGL	= CLK_bm;	// down edge
//	__asm__ volatile ( "nop" );
//	__asm__ volatile ( "nop" );
//	__asm__ volatile ( "nop" );
//	__asm__ volatile ( "nop" );
	_delay_us(1);
	port->OUTTGL	= CLK_bm;	// up edge
//	__asm__ volatile ( "nop" );
//	__asm__ volatile ( "nop" );
//	__asm__ volatile ( "nop" );
//	__asm__ volatile ( "nop" );
	_delay_us(1);
	port->OUTTGL	= CLK_bm;	// down edge
	
	// Clock until we get the start bit
	while ((port->IN & DAT_bm) == 0) {
		//	__asm__ volatile ( "nop" );
		//	__asm__ volatile ( "nop" );
		//	__asm__ volatile ( "nop" );
		//	__asm__ volatile ( "nop" );
		_delay_us(1);
		port->OUTTGL	= CLK_bm;	// up edge
		//	__asm__ volatile ( "nop" );
		//	__asm__ volatile ( "nop" );
		//	__asm__ volatile ( "nop" );
		//	__asm__ volatile ( "nop" );
		_delay_us(1);
		port->OUTTGL	= CLK_bm;	// down edge
		__asm__ volatile ( "nop" );
		__asm__ volatile ( "nop" );
		__asm__ volatile ( "nop" );
	}
//	PORTH.OUTTGL = (1<<7);
	
//	__asm__ volatile ( "nop" );
//	__asm__ volatile ( "nop" );
//	__asm__ volatile ( "nop" );
//	__asm__ volatile ( "nop" );
	_delay_us(1);
	port->OUTTGL	= CLK_bm;	// up edge
//	__asm__ volatile ( "nop" );
//	__asm__ volatile ( "nop" );
//	__asm__ volatile ( "nop" );
//	__asm__ volatile ( "nop" );
	_delay_us(1);
	port->OUTTGL	= CLK_bm;	// down edge
	
//	PORTH.OUTTGL = (1<<7);
	// now dat should be 0
//	__asm__ volatile ( "nop" );
//	__asm__ volatile ( "nop" );
//	__asm__ volatile ( "nop" );
//	__asm__ volatile ( "nop" );
	_delay_us(1);
	port->OUTTGL	= CLK_bm;	// up edge
//	__asm__ volatile ( "nop" );
//	__asm__ volatile ( "nop" );
//	__asm__ volatile ( "nop" );
//	__asm__ volatile ( "nop" );
	_delay_us(1);
	port->OUTTGL	= CLK_bm;	// down edge



	// next up is the actual data - sent MSb
//	PORTH.OUTTGL = (1<<7);
	for(i=7;i>=0;i--) {

		// sample the port
			__asm__ volatile ( "nop" );
			__asm__ volatile ( "nop" );
			__asm__ volatile ( "nop" );
		tmp[3]	|= BISS_SAMPLE << i;
		
		
		port->OUTTGL	= CLK_bm;	// up edge
//			__asm__ volatile ( "nop" );
//			__asm__ volatile ( "nop" );
//			__asm__ volatile ( "nop" );
//			__asm__ volatile ( "nop" );
//			__asm__ volatile ( "nop" );
//			__asm__ volatile ( "nop" );
//			__asm__ volatile ( "nop" );
		_delay_us(1);
		port->OUTTGL	= CLK_bm;	// down edge
		
	}

//	PORTH.OUTTGL = (1<<7);
	for(i=7;i>=0;i--) {

		// sample the port
			__asm__ volatile ( "nop" );
			__asm__ volatile ( "nop" );
			__asm__ volatile ( "nop" );
		tmp[2]	|= BISS_SAMPLE << i;
		
		port->OUTTGL	= CLK_bm;	// up edge
//			__asm__ volatile ( "nop" );
//			__asm__ volatile ( "nop" );
//			__asm__ volatile ( "nop" );
//			__asm__ volatile ( "nop" );
//			__asm__ volatile ( "nop" );
//			__asm__ volatile ( "nop" );
//			__asm__ volatile ( "nop" );
		_delay_us(1);
		port->OUTTGL	= CLK_bm;	// down edge
		
	}

//	PORTH.OUTTGL = (1<<7);
	for(i=7;i>=0;i--) {

		// sample the port
			__asm__ volatile ( "nop" );
			__asm__ volatile ( "nop" );
			__asm__ volatile ( "nop" );
		tmp[1]	|= BISS_SAMPLE << i;
		
		port->OUTTGL	= CLK_bm;	// up edge
//			__asm__ volatile ( "nop" );
//			__asm__ volatile ( "nop" );
//			__asm__ volatile ( "nop" );
//			__asm__ volatile ( "nop" );
//			__asm__ volatile ( "nop" );
//			__asm__ volatile ( "nop" );
//			__asm__ volatile ( "nop" );
		_delay_us(1);
		port->OUTTGL	= CLK_bm;	// down edge
		
	}

//	PORTH.OUTTGL = (1<<7);
	for(i=7;i>=0;i--) {

		// sample the port
			__asm__ volatile ( "nop" );
			__asm__ volatile ( "nop" );
			__asm__ volatile ( "nop" );
		tmp[0]	|= BISS_SAMPLE << i;
		
		port->OUTTGL	= CLK_bm;	// up edge
//			__asm__ volatile ( "nop" );
//			__asm__ volatile ( "nop" );
//			__asm__ volatile ( "nop" );
//			__asm__ volatile ( "nop" );
//			__asm__ volatile ( "nop" );
//			__asm__ volatile ( "nop" );
//			__asm__ volatile ( "nop" );
		_delay_us(1);
		port->OUTTGL	= CLK_bm;	// down edge
		
	}


	// sample the error bit...
	__asm__ volatile ( "nop" );
	__asm__ volatile ( "nop" );
	__asm__ volatile ( "nop" );
	status	|= BISS_SAMPLE << BISS_ERROR_bp;
//	PORTH.OUTTGL = (1<<7);
	
	port->OUTTGL	= CLK_bm;	// up edge
//	__asm__ volatile ( "nop" );
//	__asm__ volatile ( "nop" );
//	__asm__ volatile ( "nop" );
//	__asm__ volatile ( "nop" );
	_delay_us(1);
	port->OUTTGL	= CLK_bm;	// down edge


	// sample the warning bit...
	__asm__ volatile ( "nop" );
	__asm__ volatile ( "nop" );
	__asm__ volatile ( "nop" );
	status	|= BISS_SAMPLE << BISS_WARN_bp;
//	PORTH.OUTTGL = (1<<7);
	
	port->OUTTGL	= CLK_bm;	// up edge
//	__asm__ volatile ( "nop" );
//	__asm__ volatile ( "nop" );
//	__asm__ volatile ( "nop" );
//	__asm__ volatile ( "nop" );
	_delay_us(1);
	port->OUTTGL	= CLK_bm;	// down edge

//	PORTH.OUTTGL = (1<<7);
	// hammer out the CRC - 6 bits
	for(i=5;i>=0;i--) {

		// sample the port
		__asm__ volatile ( "nop" );
		__asm__ volatile ( "nop" );
		__asm__ volatile ( "nop" );
		status	|= BISS_SAMPLE << i;
		
		port->OUTTGL	= CLK_bm;	// up edge
//		__asm__ volatile ( "nop" );
//		__asm__ volatile ( "nop" );
//		__asm__ volatile ( "nop" );
//		__asm__ volatile ( "nop" );
		_delay_us(1);
		port->OUTTGL	= CLK_bm;	// down edge
		
	}
	
//	PORTH.OUTSET = (1<<7);
	port->OUTSET	= CLK_bm;	// down edge
	
	// pack it up
	data[3] = tmp[3];
	data[2] = tmp[2];
	data[1] = tmp[1];
	data[0] = tmp[0];
	
	return status;
}

uint8_t readBiSS_bang_motor(uint8_t *data, PORT_t * port, uint8_t CLK_bm, uint8_t DAT_bm) {

	int8_t i;
	int8_t pin = 6; 
	uint8_t tmp[4] = {0,0,0,0};
	uint8_t status = 0xFF;

	if ((port->IN & DAT_bm) == 0) return status;								// the device isn't ready - bail
	
	status = 0;
	
//	PORTH.OUTCLR = (1<<7);
	
	// knock twice...
	port->OUTTGL	= CLK_bm;	// down edge
//	__asm__ volatile ( "nop" );
//	__asm__ volatile ( "nop" );
//	__asm__ volatile ( "nop" );
//	__asm__ volatile ( "nop" );
	_delay_us(1);
	port->OUTTGL	= CLK_bm;	// up edge
//	__asm__ volatile ( "nop" );
//	__asm__ volatile ( "nop" );
//	__asm__ volatile ( "nop" );
//	__asm__ volatile ( "nop" );
	_delay_us(1);
	port->OUTTGL	= CLK_bm;	// down edge
//	__asm__ volatile ( "nop" );
//	__asm__ volatile ( "nop" );
//	__asm__ volatile ( "nop" );
//	__asm__ volatile ( "nop" );
	_delay_us(1);
	port->OUTTGL	= CLK_bm;	// up edge
//	__asm__ volatile ( "nop" );
//	__asm__ volatile ( "nop" );
//	__asm__ volatile ( "nop" );
//	__asm__ volatile ( "nop" );
	_delay_us(1);
	port->OUTTGL	= CLK_bm;	// down edge
	
	// Clock until we get the start bit
	while ((port->IN & DAT_bm) == 0) {
		//	__asm__ volatile ( "nop" );
		//	__asm__ volatile ( "nop" );
		//	__asm__ volatile ( "nop" );
		//	__asm__ volatile ( "nop" );
		_delay_us(1);
		port->OUTTGL	= CLK_bm;	// up edge
		//	__asm__ volatile ( "nop" );
		//	__asm__ volatile ( "nop" );
		//	__asm__ volatile ( "nop" );
		//	__asm__ volatile ( "nop" );
		_delay_us(1);
		port->OUTTGL	= CLK_bm;	// down edge
	}
//	PORTH.OUTTGL = (1<<7);
	
//	__asm__ volatile ( "nop" );
//	__asm__ volatile ( "nop" );
//	__asm__ volatile ( "nop" );
//	__asm__ volatile ( "nop" );
	_delay_us(1);
	port->OUTTGL	= CLK_bm;	// up edge
//	__asm__ volatile ( "nop" );
//	__asm__ volatile ( "nop" );
//	__asm__ volatile ( "nop" );
//	__asm__ volatile ( "nop" );
	_delay_us(1);
	port->OUTTGL	= CLK_bm;	// down edge
	
//	PORTH.OUTTGL = (1<<7);
	// now dat should be 0
//	__asm__ volatile ( "nop" );
//	__asm__ volatile ( "nop" );
//	__asm__ volatile ( "nop" );
//	__asm__ volatile ( "nop" );
	_delay_us(1);
	port->OUTTGL	= CLK_bm;	// up edge
//	__asm__ volatile ( "nop" );
//	__asm__ volatile ( "nop" );
//	__asm__ volatile ( "nop" );
//	__asm__ volatile ( "nop" );
	_delay_us(1);
	port->OUTTGL	= CLK_bm;	// down edge



	// next up is the actual data - sent MSb
//	PORTH.OUTTGL = (1<<7);
	for(i=7;i>=0;i--) {

		// sample the port
			__asm__ volatile ( "nop" );
			__asm__ volatile ( "nop" );
			__asm__ volatile ( "nop" );
		tmp[3]	|= BISS_SAMPLE << i;
		
		
		port->OUTTGL	= CLK_bm;	// up edge
//			__asm__ volatile ( "nop" );
//			__asm__ volatile ( "nop" );
//			__asm__ volatile ( "nop" );
//			__asm__ volatile ( "nop" );
//			__asm__ volatile ( "nop" );
//			__asm__ volatile ( "nop" );
//			__asm__ volatile ( "nop" );
		_delay_us(1);
		port->OUTTGL	= CLK_bm;	// down edge
		
	}

//	PORTH.OUTTGL = (1<<7);
	for(i=7;i>=0;i--) {

		// sample the port
			__asm__ volatile ( "nop" );
			__asm__ volatile ( "nop" );
			__asm__ volatile ( "nop" );
		tmp[2]	|= BISS_SAMPLE << i;
		
		port->OUTTGL	= CLK_bm;	// up edge
//			__asm__ volatile ( "nop" );
//			__asm__ volatile ( "nop" );
//			__asm__ volatile ( "nop" );
//			__asm__ volatile ( "nop" );
//			__asm__ volatile ( "nop" );
//			__asm__ volatile ( "nop" );
//			__asm__ volatile ( "nop" );
		_delay_us(1);
		port->OUTTGL	= CLK_bm;	// down edge
		
	}

//	PORTH.OUTTGL = (1<<7);
	for(i=7;i>=0;i--) {

		// sample the port
			__asm__ volatile ( "nop" );
			__asm__ volatile ( "nop" );
			__asm__ volatile ( "nop" );
		tmp[1]	|= BISS_SAMPLE << i;
		
		port->OUTTGL	= CLK_bm;	// up edge
//			__asm__ volatile ( "nop" );
//			__asm__ volatile ( "nop" );
//			__asm__ volatile ( "nop" );
//			__asm__ volatile ( "nop" );
//			__asm__ volatile ( "nop" );
//			__asm__ volatile ( "nop" );
//			__asm__ volatile ( "nop" );
		_delay_us(1);
		port->OUTTGL	= CLK_bm;	// down edge
		
	}

//	PORTH.OUTTGL = (1<<7);
	for(i=7;i>=0;i--) {

		// sample the port
			__asm__ volatile ( "nop" );
			__asm__ volatile ( "nop" );
			__asm__ volatile ( "nop" );
		tmp[0]	|= BISS_SAMPLE << i;
		
		port->OUTTGL	= CLK_bm;	// up edge
//			__asm__ volatile ( "nop" );
//			__asm__ volatile ( "nop" );
//			__asm__ volatile ( "nop" );
//			__asm__ volatile ( "nop" );
//			__asm__ volatile ( "nop" );
//			__asm__ volatile ( "nop" );
//			__asm__ volatile ( "nop" );
		_delay_us(1);
		port->OUTTGL	= CLK_bm;	// down edge
		
	}


	// sample the error bit...
	__asm__ volatile ( "nop" );
	__asm__ volatile ( "nop" );
	__asm__ volatile ( "nop" );
	status	|= BISS_SAMPLE << BISS_ERROR_bp;
//	PORTH.OUTTGL = (1<<7);
	
	port->OUTTGL	= CLK_bm;	// up edge
//	__asm__ volatile ( "nop" );
//	__asm__ volatile ( "nop" );
//	__asm__ volatile ( "nop" );
//	__asm__ volatile ( "nop" );
	_delay_us(1);
	port->OUTTGL	= CLK_bm;	// down edge


	// sample the warning bit...
	__asm__ volatile ( "nop" );
	__asm__ volatile ( "nop" );
	__asm__ volatile ( "nop" );
	status	|= BISS_SAMPLE << BISS_WARN_bp;
//	PORTH.OUTTGL = (1<<7);
	
	port->OUTTGL	= CLK_bm;	// up edge
//	__asm__ volatile ( "nop" );
//	__asm__ volatile ( "nop" );
//	__asm__ volatile ( "nop" );
//	__asm__ volatile ( "nop" );
	_delay_us(1);
	port->OUTTGL	= CLK_bm;	// down edge

//	PORTH.OUTTGL = (1<<7);
	// hammer out the CRC - 6 bits
	for(i=5;i>=0;i--) {

		// sample the port
		__asm__ volatile ( "nop" );
		__asm__ volatile ( "nop" );
		__asm__ volatile ( "nop" );
		status	|= BISS_SAMPLE << i;
		
		port->OUTTGL	= CLK_bm;	// up edge
//		__asm__ volatile ( "nop" );
//		__asm__ volatile ( "nop" );
//		__asm__ volatile ( "nop" );
//		__asm__ volatile ( "nop" );
		_delay_us(1);
		port->OUTTGL	= CLK_bm;	// down edge
		
	}
	
//	PORTH.OUTSET = (1<<7);
	port->OUTSET	= CLK_bm;	// down edge
	
	// pack it up
	data[3] = tmp[3];
	data[2] = tmp[2];
	data[1] = tmp[1];
	data[0] = tmp[0];
	
	return status;
}