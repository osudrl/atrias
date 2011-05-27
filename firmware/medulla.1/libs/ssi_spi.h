// Kevin Kemper

#define F_CPU 32000000UL

#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>

#include "./menial_io.h"

void initSSI_spi() {


	PORT_SSI1.OUTSET	= SSI1_CLK_bm;
	PORT_SSI1.DIRSET	= SSI1_CLK_bm | (1<<4); //TODO check the 1<<4
	PORT_SSI1.DIRCLR	= SSI1_DAT_bm;

	SSI1_SPI.CTRL = SPI_ENABLE_bm | SPI_MASTER_bm | SPI_MODE_1_gc | SPI_PRESCALER_DIV16_gc;
	
	SSI1_SPI.DATA = 0x00;
	while((SSI1_SPI.STATUS & SPI_IF_bm) == 0);
	
	SSI1_SPI.DATA = 0x00;
	while((SSI1_SPI.STATUS & SPI_IF_bm) == 0);
	_delay_ms(1);
	SSI1_SPI.CTRL &= ~SPI_ENABLE_bm;
	PORT_SSI1.OUTSET	= SSI1_CLK_bm;



}
uint8_t readSSI_spi(uint16_t *data) {

	uint8_t tmp1H = 0;
	uint8_t tmp1L = 0;

	if ((PORT_SSI1.IN & SSI1_DAT_bm) == 0)										// the device isn't ready
		return 0xFF;
		
	
	SSI1_SPI.CTRL = SPI_ENABLE_bm | SPI_MASTER_bm | SPI_MODE_1_gc | SPI_PRESCALER_DIV16_gc;

	// x-12-11-10-9-8-7-6 bits
	//  Send clk
	SSI1_SPI.DATA = 0x00;      // initiate write

	// wait for transfer complete
	while((SSI1_SPI.STATUS & SPI_IF_bm) == 0);

	tmp1H = SSI1_SPI.DATA & 0x7F;


	// 5-4-3-2-1-0-x-x bits
	//  Send clk
	SSI1_SPI.DATA = 0x00;      // initiate write
	// wait for transfer complete
	while((SSI1_SPI.STATUS & SPI_IF_bm) == 0);

	tmp1L = SSI1_SPI.DATA;



	SSI1_SPI.CTRL &= ~SPI_ENABLE_bm;

	PORT_SSI1.OUTSET	= SSI1_CLK_bm;

	data[1] = (tmp1H<<6) | (tmp1L>>2);

//	printf("%.2X %.2X  %u\n",tmp1H, tmp1L, data[1]);
	
	return 0x00;

} // end readSSI_uart


