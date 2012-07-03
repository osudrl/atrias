/*
Xmega USART Drivers Source
Uses atmel application note drivers

Daniel Sidlauskas Miller

Make sure to link usart_driver.c when compiling
*/
#include "uart.h"

/*UART Initiate*/
/*
Note - You still need to set the Tx to output and Rx to input to use the USART

	When adding baudrate support, the 2x bit is set in this function only for 38400
		check your baudrate caclulations to see if it should be set for new rates

Params-
	title - pointer to uart struct from atmel driver which will be referred to in 
		other parts or the program for use of the intitiated module

	interface - pointer to specific uart module to be used (eg - USARTE0

	baudrate - baudrate for communication, make sure the baudrate selected is defined with
		good values
*/
void uartInit(USART_data_t * title,USART_t * interface, unsigned int baudrate){
	USART_InterruptDriver_Initialize(title, interface, USART_DREINTLVL_LO_gc);
	USART_Format_Set(title->usart, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, false);
	USART_RxdInterruptLevel_Set(title->usart, USART_RXCINTLVL_LO_gc);
	if(baudrate == 38400){
		title->usart->CTRLB |= 0x04; //Set 2x bit
		USART_Baudrate_Set(interface, 22 , -2);
	}
	else if(baudrate == 9600){
		USART_Baudrate_Set(interface, 12 , 0);
	}

	USART_Rx_Enable(title->usart);
	USART_Tx_Enable(title->usart);
	
}

/*Send Character*/
/*
Params-
	uart - pointer to uart struct, same as title used in initiation

	buffer - single byte to be transmitted
*/
void sendchar(USART_data_t * uart, char buffer){
	char bytetobuffer;
	do{
		bytetobuffer = USART_TXBuffer_PutByte(uart, buffer);
	}while(!bytetobuffer);

}

/*Send String*/
/*
Note - sprintf() is helpful for creating strings to send over the UART that you want to
read without intelligence in the terminal

Params
	uart - pointer to uart struct, same as title used in initiation

	buffer - pointer to character string to be send, sends until null character is reached
*/
	
void sendstring(USART_data_t * uart, char *buffer){
	int i;
	for(i=0;buffer[i]!=0;i++){
		sendchar(uart, buffer[i]);

	}
	
}


