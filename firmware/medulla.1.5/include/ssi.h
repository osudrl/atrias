// Kevin Kemper
// Modifications by Kit Morton

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>

void initSSI_bang();

// Bang an arbitrary number of bits from the encoder
void readSSI_bang(uint32_t *data);
