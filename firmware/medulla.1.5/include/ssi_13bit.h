// Kevin Kemper
// Modifications by Kit Morton

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>

void initSSI_13();

// Bang an arbitrary number of bits from the encoder
void readSSI_13(uint16_t *data);
