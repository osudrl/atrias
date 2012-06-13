#include "uart.h"

void initUART(PORT_t *port, USART_t *usart, short baud) {

	if ( ((unsigned short)(usart) & 0x0010) > 0) {								//port 0
		port->OUTSET = (1<<7);													//	set tx hi
		port->DIRSET = (1<<7);													//	tx pin as output
		port->DIRCLR = (1<<6);													//	rx pin as input
	}
	else {																		// port 1
		port->OUTSET = (1<<3);        											// 	set tx hi
		port->DIRSET = (1<<3);													// 	tx pin as output
		port->DIRCLR = (1<<2);													//	rx pin as input
	}


	switch (baud) {

		case 9600:
			usart->BAUDCTRLA = 207; // 9600b  (BSCALE=207,BSEL=0)
			break;

		case 19200:
			usart->BAUDCTRLA = 103; // 19200b  (BSCALE=103,BSEL=0)
			break;

		case 57600:
			usart->BAUDCTRLA = 34;  // 57600b  (BSCALE=34,BSEL=0)
			break;

		case 1152:
			usart->BAUDCTRLA = 33; usart->BAUDCTRLB = (-1<<4); // 115.2kb (BSCALE=33,BSEL=-1)
			break;

		case 2304:
			usart->BAUDCTRLA = 31; usart->BAUDCTRLB = (-2<<4); // 230.4kb (BSCALE=31,BSEL=-2)
			break;

		case 4608:
			usart->BAUDCTRLA = 27; usart->BAUDCTRLB = (-3<<4); // 460.8kb (BSCALE=27,BSEL=-3)
			break;
		
		case 9216:
			usart->BAUDCTRLA = 19; usart->BAUDCTRLB = (-4<<4); // 921.6kb (BSCALE=19,BSEL=-4)	
			break;

		case 500:
			usart->BAUDCTRLA = 1; usart->BAUDCTRLB = (1<<4); // 500kb (BSCALE=19,BSEL=-4)
			break;

		case 1:
			usart->BAUDCTRLA = 1;   // 1Mb (BSCALE=1,BSEL=0)
			break;

		default:
			usart->BAUDCTRLA = 33; usart->BAUDCTRLB = (-1<<4); // 115.2kb (BSCALE=33,BSEL=-1)
			break;
	}			

	usart->CTRLB = USART_TXEN_bm | USART_RXEN_bm;	// enable tx and rx on USART
	
	usart->CTRLC = USART_CHSIZE_8BIT_gc;
}


void UARTWriteChar(USART_t *usart, uint8_t data) {

	while ( !( usart->STATUS & USART_DREIF_bm) );

	// Put our character into the transmit buffer
	usart->DATA = data; 										// clear TX interrupt flag
}


uint8_t UARTReadChar(USART_t *usart) {

	while(!(usart->STATUS&USART_RXCIF_bm));										// wait for RX complete

	return usart->DATA;
}
