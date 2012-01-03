#ifndef BISS_H
#define BISS_H

// Kevin Kemper

#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>


#define NBITS	32

#define BISS_ERROR_bp	7
#define BISS_ERROR_bm	(1<<7)

#define BISS_WARN_bp	6
#define BISS_WARN_bm	(1<<6)

#define BISS_SAMPLE	((port->IN & DAT_bm) >> pin)

// Notes:
//  - data changes state on rising clock edges
//	- sample when clk is low

void initBiSS_bang(PORT_t * port, int8_t CLK_bm, int8_t DAT_bm);


// Bang in the 32bit encoder values
//	data is a 32 bit container where the encoder values will be placed
uint8_t readBiSS_bang(uint8_t *data, PORT_t * port, uint8_t CLK_bm, uint8_t DAT_bm);
uint8_t readBiSS_bang_motor(uint8_t *data, PORT_t * port, uint8_t CLK_bm, uint8_t DAT_bm);

#endif