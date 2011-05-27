
#ifndef HBA4_H
#define HBA4_H

#define F_CPU 32000000UL

#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>

#include "uart.h"


#define EN_ENC0
#define EN_ENC1
//#define EN_ENC2

#define MAX_CNT	500

uint8_t readChar(USART_t *usart, uint8_t *data) {
	uint16_t cnt = 0;
	
	while( !(usart->STATUS & USART_RXCIF_bm) && (cnt < MAX_CNT) ) {					// wait for RX complete
		cnt++;
	}

	if (cnt >= MAX_CNT)
		return 1;

	*data = usart->DATA;
	return 0;
}



void initHBA4() {
	uint8_t tmp;

	////////////////////////////////////////////////////////////////////////////
	// Encoder 0
#ifdef EN_ENC0
	PORTCFG.MPCMASK		= ENC0_RX_bm;
	PORT_ENC0.PIN0CTRL	= PORT_OPC_PULLUP_gc;

	PORT_ENC0.OUTSET 	= ENC0_RTS_bm;
	PORT_ENC0.DIRSET 	= ENC0_RTS_bm;

	initUART(&PORT_ENC0,&ENC0_USART,9600);
	_delay_ms(2000);

	UARTWriteChar(&ENC0_USART, 0xFF);	// multi-byte command
	UARTWriteChar(&ENC0_USART, 0x0F);	// change baud rate
	ENC0_USART.STATUS  |= USART_TXCIF_bm;
	UARTWriteChar(&ENC0_USART, 0x00);	// 115200
	while ( !( ENC0_USART.STATUS & USART_TXCIF_bm) );
	
	ENC0_USART.STATUS  |= USART_TXCIF_bm;
	
	initUART(&PORT_ENC0,&ENC0_USART,1152);
	PORT_ENC0.OUTSET = ENC0_RTS_bm;

#endif
	////////////////////////////////////////////////////////////////////////////
	// Encoder 1
#ifdef EN_ENC1
	PORTCFG.MPCMASK		= ENC1_RX_bm;
	PORT_ENC1.PIN0CTRL	= PORT_OPC_PULLUP_gc;

	PORT_ENC1.OUTSET 	= ENC1_RTS_bm;
	PORT_ENC1.DIRSET 	= ENC1_RTS_bm;
	

	initUART(&PORT_ENC1,&ENC1_USART,9600);
	_delay_ms(2000);

	UARTWriteChar(&ENC1_USART, 0xFF);	// multi-byte command
	UARTWriteChar(&ENC1_USART, 0x0F);	// change baud rate
	ENC1_USART.STATUS  |= USART_TXCIF_bm;
	UARTWriteChar(&ENC1_USART, 0x00);	// 115200
	while ( !( ENC1_USART.STATUS & USART_TXCIF_bm) );
	
	ENC1_USART.STATUS  |= USART_TXCIF_bm;
	
	initUART(&PORT_ENC1,&ENC1_USART,1152);
	PORT_ENC1.OUTSET = ENC1_RTS_bm;


#endif
	////////////////////////////////////////////////////////////////////////////
	// Encoder 2
#ifdef EN_ENC2
	PORTCFG.MPCMASK		= ENC2_RX_bm;
	PORT_ENC2.PIN0CTRL	= PORT_OPC_PULLUP_gc;

	PORT_ENC2.OUTSET 	= ENC2_RTS_bm;
	PORT_ENC2.DIRSET 	= ENC2_RTS_bm;

	initUART(&PORT_ENC2,&ENC2_USART,9600);
	_delay_ms(2000);

	UARTWriteChar(&ENC2_USART, 0xFF);	// multi-byte command
	UARTWriteChar(&ENC2_USART, 0x0F);	// change baud rate
	ENC2_USART.STATUS  |= USART_TXCIF_bm;
	UARTWriteChar(&ENC2_USART, 0x00);	// 115200
	while ( !( ENC2_USART.STATUS & USART_TXCIF_bm) );
	
	ENC2_USART.STATUS  |= USART_TXCIF_bm;
	
	initUART(&PORT_ENC2,&ENC2_USART,1152);
	PORT_ENC2.OUTSET = ENC2_RTS_bm;
#endif

}

uint8_t getHBA4pos0(uint16_t *data) {
	uint8_t status = 0;
	uint16_t cnt = 0;
	uint8_t tmp0[2] = {0,0};

#ifdef EN_ENC0
	PORT_ENC0.OUTSET = ENC0_RTS_bm;

	ENC0_USART.STATUS  |= USART_TXCIF_bm;										// Clear the Tx flag
	ENC0_USART.CTRLB = USART_TXEN_bm;											// Enable the Tx
	
	UARTWriteChar(&ENC0_USART, 0x1F);											// Ask for position data with the broadcast address

	while ( !( ENC0_USART.STATUS & USART_TXCIF_bm) );							// Wait for the Tx to complete
	_delay_us(5);

	PORT_ENC0.OUTCLR = ENC0_RTS_bm;												// Set the RTS pin to allow incoming data
	ENC0_USART.CTRLB = 0;														// Disable the UART
	
	cnt = 0;
	while ( (( PORT_ENC0.IN & ENC0_RX_bm) == 0) && (cnt<MAX_CNT) ) cnt++;		// Wait unitl the Rx line ready (high)

	if (cnt >= MAX_CNT)
		return 1;

	ENC0_USART.CTRLB = USART_RXEN_bm;											// Enable the Rx
	
	status |= readChar(&ENC0_USART, &tmp0[1]);									// MSB
	status |= readChar(&ENC0_USART, &tmp0[0]);									// LSB
	
	PORT_ENC0.OUTSET = ENC0_RTS_bm;												// Set RTS for Tx
	ENC0_USART.CTRLB = USART_TXEN_bm;
#endif


	((uint8_t*)data)[0] = tmp0[0];
	((uint8_t*)data)[1] = tmp0[1];


	return status;
//	printf("0x%.2X %.2X		0x%.2X %.2X		0x%.2X %.2X\n",tmp0[1],tmp0[0],tmp1[1],tmp1[0],tmp2[1],tmp2[0]);

}


uint8_t getHBA4pos1(uint16_t *data) {
	uint8_t status = 0;
	uint16_t cnt = 0;
	uint8_t tmp1[2] = {0,0};
		

#ifdef EN_ENC1
	PORT_ENC1.OUTSET = ENC1_RTS_bm;
	
	ENC1_USART.STATUS  |= USART_TXCIF_bm;										// Clear the Tx flag
	ENC1_USART.CTRLB = USART_TXEN_bm;											// Enable the Tx
	
	UARTWriteChar(&ENC1_USART, 0x1F);											// Ask for position data with the broadcast address

	while ( !( ENC1_USART.STATUS & USART_TXCIF_bm) );							// Wait for the Tx to complete
	_delay_us(5);

	PORT_ENC1.OUTCLR = ENC1_RTS_bm;												// Set the RTS pin to allow incoming data
	ENC1_USART.CTRLB = 0;														// Disable the UART
	
	cnt = 0;
	while ( (( PORT_ENC1.IN & ENC1_RX_bm) == 0) && (cnt<MAX_CNT) ) cnt++;		// Wait unitl the Rx line ready (high)

	if (cnt >= MAX_CNT)
		return 1;

	ENC1_USART.CTRLB = USART_RXEN_bm;											// Enable the Rx
	
	status |= readChar(&ENC1_USART, &tmp1[1]);									// MSB
	status |= readChar(&ENC1_USART, &tmp1[0]);									// LSB
	
	PORT_ENC1.OUTSET = ENC1_RTS_bm;
	ENC1_USART.CTRLB = USART_TXEN_bm;
#endif

	
	((uint8_t*)data)[0] = tmp1[0];
	((uint8_t*)data)[1] = tmp1[1];


	return status;
//	printf("0x%.2X %.2X		0x%.2X %.2X		0x%.2X %.2X\n",tmp0[1],tmp0[0],tmp1[1],tmp1[0],tmp2[1],tmp2[0]);

}



uint8_t getHBA4pos2(uint16_t *data) {
	uint8_t status = 0;
	uint16_t cnt = 0;
	uint8_t tmp2[2] = {0,0};
		

#ifdef EN_ENC2
	PORT_ENC2.OUTSET = ENC2_RTS_bm;

	ENC2_USART.STATUS  |= USART_TXCIF_bm;										// Clear the Tx flag
	ENC2_USART.CTRLB = USART_TXEN_bm;											// Enable the Tx
	
	UARTWriteChar(&ENC2_USART, 0x1F);											// Ask for position data with the broadcast address

	while ( !( ENC2_USART.STATUS & USART_TXCIF_bm) );							// Wait for the Tx to complete
	_delay_us(7);

	PORT_ENC2.OUTCLR = ENC2_RTS_bm;												// Set the RTS pin to allow incoming data
	ENC2_USART.CTRLB = 0;														// Disable the UART

	
	cnt = 0;
	while ( (( PORT_ENC2.IN & ENC2_RX_bm) == 0) && (cnt<MAX_CNT) ) cnt++;		// Wait unitl the Rx line ready (high)

	if (cnt >= MAX_CNT)
		return 1;

	ENC2_USART.CTRLB = USART_RXEN_bm;											// Enable the Rx
	
	status |= readChar(&ENC2_USART, &tmp2[1]);									// MSB
	status |= readChar(&ENC2_USART, &tmp2[0]);									// LSB
	
	PORT_ENC2 .OUTSET = ENC2_RTS_bm;
	ENC2_USART.CTRLB = USART_TXEN_bm;
#endif


	((uint8_t*)data)[0] = tmp2[0];
	((uint8_t*)data)[1] = tmp2[1];

	return status;
//	printf("0x%.2X %.2X		0x%.2X %.2X		0x%.2X %.2X\n",tmp0[1],tmp0[0],tmp1[1],tmp1[0],tmp2[1],tmp2[0]);

}





#endif

