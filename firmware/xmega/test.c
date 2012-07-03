#include "uart.h"
#include "adc.h"
#include "avr_compiler.h"
#include <stdlib.h>
#include <stdio.h>
#include <util/delay.h>

USART_data_t serial;

int main(void){

	CLK.CTRL = 0b00000000;
	//CLK.PSCTRL = 0b00010100;

	PMIC.CTRL |= PMIC_LOLVLEX_bm | PMIC_MEDLVLEX_bm | PMIC_HILVLEX_bm |
		PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;
	sei();

/*
	PORTD.DIR = 1;
	TCD0.CTRLA = TC_CLKSEL_DIV1_gc;
	TCD0.CTRLB = TC_WGMODE_SS_gc | TC0_CCCEN_bm |  TC0_CCAEN_bm |TC0_CCBEN_bm | TC0_CCDEN_bm;
	TCD0.PER = 8000;
	TCD0.CCA = 4000;
*/


	PORTF.DIR = 0b00001000;//Set Tx to output, Rx to input
	uartInit(&serial, &USARTF0, 9600);

	adcInit(&ADCA);

	int data[4];
	char serialbuffer[50];
	
	while(1){
		_delay_ms(500);
		adcRead(&ADCA, data);
		sprintf(serialbuffer, "%4d %4d %4d %4d\n\r", data[0], data[1], data[2], data[3]);
		//sprintf(serialbuffer, "a\n\r");
		sendstring(&serial, serialbuffer);
	}

}
ISR(USARTF0_RXC_vect){
	USART_RXComplete(&serial);
}

/*Usart module interrupt to inform data has been properly sent*/
ISR(USARTF0_DRE_vect){
	USART_DataRegEmpty(&serial);
}


