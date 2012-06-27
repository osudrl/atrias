#ifndef ECAT_H
#define ECAT_H

// Kevin Kemper
// Modifications by Kit Morton

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

void spiWrite(uint8_t data);
uint8_t spiRead(void);

// initialize SPI interface to talk to the ET1100
// PORTC:4 - SS   (active low)
// PORTC:5 - MOSI
// PORTC:6 - MISO
// PORTC:7 - SCK
void spiInit(void);
uint8_t spiWriteRead(uint8_t data);

////////////////////////////////////////////////////////////////////////////////
// eCAT specific stuffs

void initeCAT (void);
uint16_t readAddr(uint16_t addr, uint8_t * data, uint8_t len);
uint16_t writeAddr(uint16_t addr, uint8_t * data, uint8_t len);
uint16_t eCATnop(void);

#endif

