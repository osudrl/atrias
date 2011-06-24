// Kevin Kemper
//
//
////////////////////////////////////////////////////////////////////////////////
#ifndef	BISS_SPI_H
#define BISS_SPI_H

#include <stdio.h>
#include <avr/io.h>

#include "./menial_io.h"

#define BISS_ERROR_bp	7
#define BISS_ERROR_bm	(1<<BISS_ERROR_bp)

#define BISS_WARN_bp	6
#define BISS_WARN_bm	(1<<BISS_WARN_bp)

#define MAX_BISS_CNT	20

inline void space() {

	__asm__ volatile ( "nop" );
	__asm__ volatile ( "nop" );
	__asm__ volatile ( "nop" );
	__asm__ volatile ( "nop" );
	__asm__ volatile ( "nop" );
	__asm__ volatile ( "nop" );
	__asm__ volatile ( "nop" );
	__asm__ volatile ( "nop" );
	__asm__ volatile ( "nop" );
	__asm__ volatile ( "nop" );
	__asm__ volatile ( "nop" );
	__asm__ volatile ( "nop" );

}

uint8_t readBiSS_spi(uint8_t *data, uint8_t *status) {

	uint8_t tmp[5] = {0,0,0,0,0};
	uint8_t cnt = 0;
//	uint8_t status = 0;
	
//	data[3] = tmp[3];
//	data[2] = tmp[2];
//	data[1] = tmp[1];
//	data[0] = tmp[0];

//	while ((PORT_BISS.IN & BISS_DAT_bm) == 0) ;									// the device isn't ready

//	*status = 0;
	
	if ((PORT_BISS.IN & BISS_DAT_bm) == 0)										// the device isn't ready
		return 0xFF;
	
	// knock twice...

	PORT_BISS.OUTTGL	= BISS_CLK_bm;	// down edge
	space();
//	_delay_us(1);
	PORT_BISS.OUTTGL	= BISS_CLK_bm;	// up edge
	space();
//	_delay_us(1);
	PORT_BISS.OUTTGL	= BISS_CLK_bm;	// down edge
	space();
//	_delay_us(1);
	PORT_BISS.OUTTGL	= BISS_CLK_bm;	// up edge
	space();
//	_delay_us(1);
	PORT_BISS.OUTTGL	= BISS_CLK_bm;	// down edge
	space();

	if ((PORT_BISS.IN & BISS_DAT_bm) != 0) {
		PORT_BISS.OUTTGL	= BISS_CLK_bm;	// up edge
		return 0xFF;
	}
	
	// Clock until we get the start bit
	while ( ((PORT_BISS.IN & BISS_DAT_bm) == 0) && (cnt<MAX_BISS_CNT)) {
		cnt++;
		
		space();
//		_delay_us(1);
		PORT_BISS.OUTTGL	= BISS_CLK_bm;	// up edge
		space();
//		_delay_us(1);
		PORT_BISS.OUTTGL	= BISS_CLK_bm;	// down edge
	}
	
	if ( cnt>=MAX_BISS_CNT ) {													// took too long
		return cnt;
	}
	
	space();
//	_delay_us(1);
	PORT_BISS.OUTTGL	= BISS_CLK_bm;	// up edge
	space();
//	_delay_us(1);
	PORT_BISS.OUTTGL	= BISS_CLK_bm;	// down edge
	
	// now dat should be 0
	space();
//	_delay_us(1);
	PORT_BISS.OUTTGL	= BISS_CLK_bm;	// up edge
	
//	__asm__ volatile ( "nop" );
//	_delay_us(1);
//	PORT_BISS.OUTTGL	= BISS_CLK_bm;	// down edge
	
	
//	PORTH.OUTTGL = (1<<7);
	
	BISS_SPI.CTRL = SPI_ENABLE_bm | SPI_MASTER_bm | SPI_MODE_2_gc | SPI_PRESCALER_DIV16_gc;

//	PORT_BISS.OUTCLR = (1<<4);
	// MSB
	//  Send clk
	BISS_SPI.DATA = 0x00;      // initiate write
	// wait for transfer complete
	while((BISS_SPI.STATUS & SPI_IF_bm) == 0);
	tmp[3] = BISS_SPI.DATA;

	BISS_SPI.DATA = 0x00;      // initiate write
	// wait for transfer complete
	while((BISS_SPI.STATUS & SPI_IF_bm) == 0);
	tmp[2] = BISS_SPI.DATA;

	BISS_SPI.DATA = 0x00;      // initiate write
	// wait for transfer complete
	while((BISS_SPI.STATUS & SPI_IF_bm) == 0);
	tmp[1] = BISS_SPI.DATA;

	BISS_SPI.DATA = 0x00;      // initiate write
	// wait for transfer complete
	while((BISS_SPI.STATUS & SPI_IF_bm) == 0);
	tmp[0] = BISS_SPI.DATA;


	BISS_SPI.DATA = 0x00;      // initiate write
	// wait for transfer complete
	while((BISS_SPI.STATUS & SPI_IF_bm) == 0);
	tmp[4] = BISS_SPI.DATA;
	
//	PORT_BISS.OUTSET = (1<<4);
//	printf("%.2X\t%.2X\t%.2X\t%.2X\n", tmp[3], tmp[2], tmp[1], tmp[0]);
	
	// pack it up
	data[3] = tmp[3];
	data[2] = tmp[2];
	data[1] = tmp[1];
	data[0] = tmp[0];
	
	*status = tmp[4];

	BISS_SPI.CTRL &= ~SPI_ENABLE_bm;

//	if ((status & BISS_ERROR_bm) == 0)
//		printf("BiSS %.2X: error - the position data is junk.\n",status);	
	
//	printf("(>\")> : %.2X  %.2X      %u\n",tmp1H, tmp1L, data[0]);

	return 0;
} // end readBiSS



void initBiSS_spi() {

	PORT_BISS.OUTSET	= BISS_CLK_bm | (1<<4);
	PORT_BISS.DIRSET	= BISS_CLK_bm | (1<<4); //TODO check the 1<<4
	PORT_BISS.DIRCLR	= BISS_DAT_bm;

}

#endif // !BISS_SPI_H
