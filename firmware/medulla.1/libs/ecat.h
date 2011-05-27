// Kevin Kemper

#include <stdio.h>
#include <avr/io.h>


// SPI pins (port E)
#define SPI_ECAT	SPIE
#define PORT_ECAT	PORTE

#define ECAT_EEPROM_LOADED	0
#define ECAT_SPI_IRQ		1

#define SEL		4
#define MOSI	5
#define MISO	6
#define SPICLK	7


// eCAT commands
#define E_NOP		0
#define E_READ		2
#define E_READ_WS	3
#define E_WRITE		4
#define E_ADDREX	6

// Register locations
#define DL_STATUS	0x0110 // 2 bytes
#define AL_CONTROL	0x0120
#define AL_STATUS	0x0130
#define PDI_ERROR	0x030D

#define SM0_BASE	0x0800
#define SM1_BASE	0x0800+8
#define SM2_BASE	0x0800+16
#define SM3_BASE	0x0800+24

// Offsets
#define SM_PHY_ADDR		0x0	// 2 bytes
#define SM_LENGTH		0x2	// 2 bytes
#define SM_CONTROL		0x4 // 1 byte
#define SM_STATUS		0x5 // 1 byte
#define SM_ACTIVE		0x6	// 1 byte
#define SM_PDI_CTRL		0x7	// 1 byte


#define AL_STATUS_INIT_bm	(1<<0)
#define AL_STATUS_PREOP_bm	(1<<1)
#define AL_STATUS_SAFEOP_bm	(1<<2)
#define AL_STATUS_OP_bm		(1<<3)
#define AL_STATUS_ERR_bm	(1<<4)




////////////////////////////////////////////////////////////////////////////////
// SPI stuffs

void spiWrite(uint8_t data) {

	SPI_ECAT.DATA = data;      // initiate write
	// wait for transfer complete
	while((SPI_ECAT.STATUS & SPI_IF_bm) == 0);			//TODO: move this to the start of the function.  This would probably speed up the writes
	
}

uint8_t spiRead(void) {

	// write 0x00 and read back results clocked into data buffer
	spiWrite(0);
	return SPI_ECAT.DATA;

}

// initialize SPI interface to talk to the ET1100
// PORTC:4 - SS   (active low)
// PORTC:5 - MOSI
// PORTC:6 - MISO
// PORTC:7 - SCK
void spiInit(void) {
	PORT_ECAT.DIRSET	= (1<<SEL) | (1<<MOSI) | (1<<SPICLK);	// configure MOSI, SS, CLK as outputs on PORTC
	PORT_ECAT.DIRCLR	= (1<<MISO);							//  MISO as input
	PORT_ECAT.OUTSET	= (1<<SEL);								// set SEL high
	
	// enable SPI master mode, CLK/4 -> 8 MHz
	SPI_ECAT.CTRL = SPI_ENABLE_bm | SPI_MASTER_bm | SPI_MODE_0_gc | SPI_PRESCALER_DIV4_gc;
	
	spiRead();
}


uint8_t spiWriteRead(uint8_t data) {
	spiWrite(data);
	return SPI_ECAT.DATA;
}


////////////////////////////////////////////////////////////////////////////////
// eCAT specific stuffs

void initeCAT () {
	spiInit();
	
	PORT_ECAT.DIRCLR	= (1<<ECAT_EEPROM_LOADED);
	PORT_ECAT.PIN0CTRL	|= (3<<3);							// set internal pullup
}



uint16_t readAddr(uint16_t addr, uint8_t * data, uint8_t len) {
	uint16_t al_event;
	uint8_t i=0;

	PORT_ECAT.OUTCLR = (1<<SEL);
	
	al_event = spiWriteRead((uint8_t)(addr >> 5));
		
	if ( addr > 0xFFF ) {
		al_event |= spiWriteRead( ((uint8_t)(addr&0x1F)<<3) | E_ADDREX ) << 8;
					spiWriteRead( ((uint8_t)(addr>>8)&0xE0) | (E_READ_WS<<2) );
	}
	else {
		al_event |= spiWriteRead( ((uint8_t)(addr&0x1F)<<3) | E_READ_WS ) << 8;
	}


	spiWrite(0xFF);
	
	for (i=0;i<len-1;i++) {
		*(data+i) = spiWriteRead(0x00);
	}		
	*(data+i) = spiWriteRead(0xFF);		// read termination byte

	PORT_ECAT.OUTSET = (1<<SEL);
	
	return al_event;
}


uint16_t writeAddr(uint16_t addr, uint8_t * data, uint8_t len) {
	uint16_t al_event;
	uint8_t i=0;

	PORT_ECAT.OUT &= ~(1<<SEL);
	
	al_event = spiWriteRead((uint8_t)(addr >> 5));

	if ( addr > 0xFFF ) {
		al_event |= spiWriteRead( ((uint8_t)(addr&0x1F)<<3) | E_ADDREX ) << 8;
					spiWriteRead( ((uint8_t)(addr>>8)&0xE0) | (E_WRITE<<2) );
	}
	else {
		al_event |= spiWriteRead( ((uint8_t)(addr&0x1F)<<3) | E_WRITE ) << 8;
	}


	for (i=0;i<len;i++) {
		spiWrite(*(data+i));
	}		
//	spiWrite(*(data+i));

	PORT_ECAT.OUT |= (1<<SEL);
	
	return al_event;
}



uint16_t eCATnop() {
	uint16_t al_event;
	uint16_t addr = 0x0000;

	PORT_ECAT.OUTCLR = (1<<SEL);
	
	// addr0
	al_event = spiWriteRead((uint8_t)(addr >> 5));

	// addr1/cmd
	al_event |= spiWriteRead( ((uint8_t)(addr&0x1F)<<3) | E_NOP ) << 8;


	PORT_ECAT.OUTSET = (1<<SEL);
	
	return al_event;
}



