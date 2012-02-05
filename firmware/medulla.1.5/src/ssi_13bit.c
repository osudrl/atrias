#include "ssi_13bit.h"

#define NBITS	13

// Define ports and things
#define PORT_SSI0 PORTC
#define PORT_SSI1 PORTF
#define SSI0_DAT 6
#define SSI0_DAT_bm 1<<6
#define SSI0_CLK_bm 1<<5
#define SSI1_DAT 7
#define SSI1_DAT_bm 1<<7
#define SSI1_CLK_bm 1<<5

#define SSI0_SAMPLE	((PORT_SSI0.IN & SSI0_DAT_bm) >> SSI0_DAT)
#define SSI1_SAMPLE	((PORT_SSI1.IN & SSI1_DAT_bm) >> SSI1_DAT)

void initSSI_13() {

	PORT_SSI0.OUTSET	= SSI0_CLK_bm;				// clocks high...
	PORT_SSI1.OUTSET	= SSI1_CLK_bm;
	
	PORT_SSI0.DIRSET	= SSI0_CLK_bm;				// clocks -> output
	PORT_SSI1.DIRSET	= SSI1_CLK_bm;

	PORT_SSI0.DIRCLR	= SSI0_DAT_bm;				// data -> input
	PORT_SSI1.DIRCLR	= SSI1_DAT_bm;

}


// Bang in the 13bit encoder values
//	*data is a pointer to the container where the encoder values will be placed
void readSSI_13(uint16_t *data) {

	int8_t i;
	uint8_t tmp0H = 0;
	uint8_t tmp0L = 0;
	uint8_t tmp1H = 0;
	uint8_t tmp1L = 0;

//	data[0] = 0;
//	data[1] = 0;
//	data[2] = 0;
//	data[3] = 0;

	// clocks high
//	PORT_SSI0.OUTSET	= SSI0_CLK_bm;
//	PORT_SSI1.OUTSET	= SSI1_CLK_bm;
//	PORT_SSI2.OUTSET	= SSI2_CLK_bm;
//	PORT_SSI3.OUTSET	= SSI3_CLK_bm;

	// High bits
	for(i=4;i>=0;i--) {

		// toggle clock low
		PORT_SSI0.OUTTGL	= SSI0_CLK_bm;
		PORT_SSI1.OUTTGL	= SSI1_CLK_bm;


//		__asm__ volatile ( "nop" );								// wait some cycles...
//		__asm__ volatile ( "nop" );

		// toggle clock high
		PORT_SSI0.OUTTGL	= SSI0_CLK_bm;
		PORT_SSI1.OUTTGL	= SSI1_CLK_bm;

//		__asm__ volatile ( "nop" );								// wait some cycles...
//		__asm__ volatile ( "nop" );
//		__asm__ volatile ( "nop" );
//		__asm__ volatile ( "nop" );

		// sample the ports
//		temp[i] = (SSI3_SAMPLE<<3) | (SSI2_SAMPLE<<2) | (SSI1_SAMPLE<<1) | (SSI0_SAMPLE);
		tmp0H	|= SSI0_SAMPLE << i;
		tmp1H	|= SSI1_SAMPLE << i;
	}
	
	for(i=7;i>=0;i--) {

		// toggle clock low
		PORT_SSI0.OUTTGL	= SSI0_CLK_bm;
		PORT_SSI1.OUTTGL	= SSI1_CLK_bm;


//		__asm__ volatile ( "nop" );								// wait some cycles...
//		__asm__ volatile ( "nop" );

		// toggle clock high
		PORT_SSI0.OUTTGL	= SSI0_CLK_bm;
		PORT_SSI1.OUTTGL	= SSI1_CLK_bm;

//		__asm__ volatile ( "nop" );								// wait some cycles...
//		__asm__ volatile ( "nop" );
//		__asm__ volatile ( "nop" );
//		__asm__ volatile ( "nop" );

		// sample the ports
//		temp[i] = (SSI3_SAMPLE<<3) | (SSI2_SAMPLE<<2) | (SSI1_SAMPLE<<1) | (SSI0_SAMPLE);
		tmp0L	|= SSI0_SAMPLE << i;
		tmp1L	|= SSI1_SAMPLE << i;

	}
	

	//one more toggle...
	PORT_SSI0.OUTTGL	= SSI0_CLK_bm;
	PORT_SSI1.OUTTGL	= SSI1_CLK_bm;
	
	//TODO: this will probably need some casting
	data[0] = (tmp0H<<8) | (tmp0L);
	data[1] = (tmp1H<<8) | (tmp1L);
	
	
	//XXX
	_delay_us(15);
	
	//clks high
	PORT_SSI0.OUTSET	= SSI0_CLK_bm;
	PORT_SSI1.OUTSET	= SSI1_CLK_bm;
	
/*
	for(i=0;i<NBITS;i++) {

		// pack the values into data
		data[0] |= ( (temp[i] & (1<<0)) >> 0) << i;
		data[1] |= ( (temp[i] & (1<<1)) >> 1) << i;
		data[2] |= ( (temp[i] & (1<<2)) >> 2) << i;
		data[3] |= ( (temp[i] & (1<<3)) >> 3) << i;

	}
*/

}
