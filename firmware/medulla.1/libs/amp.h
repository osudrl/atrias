//Kevin Kemper

#include <stdio.h>
#include <avr/io.h>

#include "./menial_io.h"
//#include "./uart.h"

// TODO:
//	- use drive heatbeat?




// Master Command struture
//	SOF(0xAH)	Addr	Ctrl	Idx		Offset	Length	CRC		Data...	CRC
//	8			8		8		8		8		8		16				16

// NOTES:
//	- All bytes are sent Little Endian (LSB first).
//	- The CRC is sent upperbyte first, then lower byte.


#define SOF		0xA5		// Start of frame
#define ADDR	63		// Broadcast mesage (only one amp connected anyway)

//NOTE: for now we're going to ignore the sequence bits in the ctrl byte.
#define CTRL_R	0x01
#define CTRL_W	0x02


// Slave reply struture
//	SOF(0xAH)	Addr(0xFF)	Ctrl	Status 1	Status 2	Length	CRC		Data...		CRC
//	8			8			8		8			8			8		16					16

// NOTES:
//	- Status 2 is undefined (ignore it)
//	- All bytes are sent Little Endian (LSB first).
//	- The CRC is sent upperbyte first, then lower byte.

#define STAT1_COMPLETE		0x01		// Command complete
#define STAT1_INCOMPLETE	0x02		// Command incomplete
#define STAT1_INVALID		0x04		// Invalid command
#define STAT1_NO_WRITE		0x06		// Do not have write access
#define STAT1_CRC_ERR		0x08		// Frame or CRC error





#define IDX_PWM				0x1B
#define OFF_APPLIED_PWM		0x00




static uint16_t crctable[256] =
{ 	0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
	0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
	0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
	0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
	0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
	0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
	0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
	0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
	0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
	0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
	0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
	0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
	0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
	0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
	0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
	0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
	0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
	0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
	0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
	0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
	0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
	0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
	0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
	0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
	0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
	0xA9AB, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
	0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
	0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
	0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
	0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
	0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
	0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0 };



inline void crccheck (uint8_t value, uint16_t *accumulator) {
	*accumulator = ( *accumulator<<8 ) ^ crctable[ (*accumulator>>8) ^ value ];
}



void initAmp() {
	// init the uart
	initUART(&PORT_AMP_UART,&AMP_RS232,AMP_BAUD);

	PORT_AMP_CTL.DIRSET = DIRECTION_bm;
	
	
	// set: Resolver Resolution?
	
	// set: Torque Current Loop Proportional Gain?
	
	// set: Torque Current Loop Integral Gain?
	
	// set: Peak Current Limit
	// set: Event Recovery Time: Current Limiting
	
	// set: Peak Current Hold Time
	
	// set: Continuous Current Limit
	// set: Event Recovery Time: Continuous Current Limiting
	
	// set: Peak to Continuous Current Transition Time?
	
	
	// set: Motor Type
	
	// set: Encoder Direction
	
	// set: Synchronization Mode
	
	// set: Encoder Counts Per Electrical Cycel
	
	// set: Under-Voltage Limit
	// set: Event Recovery Time: User Under Voltage
	
	// set: Digital Input Mask: Active Level
	
	// set: Digital Input Mask: Positve Limit
	// set: Event Action: Commanded Positive Limit
	// set: Event Recovery Time: Positive Limit

	// set: Digital Input Mask: Negative Limit
	// set: Event Action: Commanded Negative Limit
	// set: Event Recovery Time: Negative Limit
		
		
	
}



// Master Command struture
//	SOF(0xAH)	Addr	Ctrl	Idx		Offset	Length	CRC		Data...	CRC
//	8			8		8		8		8		8		16				16

void readTest () {
	
	uint8_t		i;
	uint16_t	crc;
	
	uint8_t		tx_header[8];
	uint8_t		rx_data[14];
	
	tx_header[0] = 0xA5;
	tx_header[1] = 63;
	tx_header[2] = 0x01;
	tx_header[3] = 0x45;
	tx_header[4] = 0x02;
	tx_header[5] = 0x02;
	
//	header[0] = 0x5A;
//	header[1] = 0xC0;
//	header[2] = 0xFE;
//	header[3] = 0xBA;
//	header[4] = 0xFD;
//	header[5] = 0xFD;
	
	crc = 0;
	for (i=0;i<6;i++)
		crccheck(tx_header[i], &crc);

	tx_header[6] = (uint8_t)(crc>>8);
	tx_header[7] = (uint8_t)(crc);

	printf("CRC: 0x%.4X    CRCh: 0x%.2X   CRCl: 0x%.2X\n", (uint16_t)crc, (uint8_t)(crc>>8), (uint8_t)(crc));
//	printf("should be      CRCh: 0x0D   CRCl: 0xF7\n");

	for (i=0;i<8;i++) {
//		_delay_us(100);
		PORTH.OUTTGL = 1<<1;
		UARTWriteChar(&AMP_RS232, tx_header[i]);
//		while ( ( AMP_RS232.STATUS & USART_TXCIF_bm) == 0);
//		AMP_RS232.STATUS |= USART_TXCIF_bm;
//		PORTH.OUTTGL = 1<<1;
		
		
	}

	for (i=0;i<14;i++)
		rx_data[i] = UARTReadChar(&AMP_RS232);


	printf("Amp response:\n");
	for (i=0;i<14;i++)
		printf("0x%.2X\t",rx_data[i]);
	printf("\n");	
}


// Useful commands
//	Applied PWM Duty Cycle
//	Primary Encoder Counts
//	Current Target - Torque
//	Current Measured - Torque
//	Velocity Measured Post-Filter
//	Position Measured
//	Digital Input Values



void ampWrite(uint8_t idx, uint8_t off, uint8_t length, uint16_t *tx_data){
	
	uint8_t		i;
	uint8_t		crc[2];
	
	uint8_t		tx_header[8];
	uint8_t		rx_header[8];

	
	tx_header[0] = SOF;
	tx_header[1] = ADDR;
	tx_header[2] = CTRL_W;
	tx_header[3] = idx;
	tx_header[4] = off;
	tx_header[5] = length;
	
	// header crc
	crc[0] = 0;
	crc[1] = 0;
	for (i=0;i<6;i++)
		crccheck(tx_header[i], (uint16_t*)crc);

	tx_header[6] = crc[1];
	tx_header[7] = crc[0];

	// data crc
	crc[0] = 0;
	crc[1] = 0;
	for (i=0;i<(length<<1);i++) {
//		crccheck(((uint8_t*)tx_data)[i+1],	(uint16_t*)crc);
		crccheck(((uint8_t*)tx_data)[i],	(uint16_t*)crc);
	}

	// write header
	for (i=0;i<8;i++) {
		UARTWriteChar(&AMP_RS232, tx_header[i]);
	}
	
	// write data
	for (i=0;i<(length<<1);i++) {
		UARTWriteChar(&AMP_RS232, ((uint8_t*)tx_data)[i]);
//		UARTWriteChar(&AMP_RS232, ((uint8_t*)tx_data)[i+1]);
	}
	UARTWriteChar(&AMP_RS232, crc[1]);
	UARTWriteChar(&AMP_RS232, crc[0]);


	// response
	for (i=0;i<8;i++)
		rx_header[i] = UARTReadChar(&AMP_RS232);

	printf("\nWrote:\n");
	for (i=0;i<8;i++)
		printf("0x%.2X\t",tx_header[i]);
	for (i=0;i<(length<<1);i++) {
		printf("0x%.2X\t",((uint8_t*)tx_data)[i]);
//		printf("0x%.2X\t",((uint8_t*)tx_data)[i+1]);	
	}
	printf("0x%.2X\t",crc[1]);
	printf("0x%.2X\t",crc[0]);
	printf("\n");
	
	printf("Amp write response:\n");
	printf("header:\n");
	for (i=0;i<8;i++)
		printf("0x%.2X\t",rx_header[i]);
	printf("\n\n");
	
}



void ampRead(uint8_t idx, uint8_t off, uint8_t length, uint8_t *rx_data){
	
	uint8_t		i;
	uint16_t	crc;
	
	uint8_t		tx_header[8];
	uint8_t		rx_header[8];
//	uint8_t		rx_data[length<<1];

	
	tx_header[0] = SOF;
	tx_header[1] = ADDR;
	tx_header[2] = CTRL_R;
	tx_header[3] = idx;
	tx_header[4] = off;
	tx_header[5] = length;
	
	crc = 0;
	for (i=0;i<6;i++)
		crccheck(tx_header[i], &crc);

	tx_header[6] = (uint8_t)(crc>>8);
	tx_header[7] = (uint8_t)(crc);

	for (i=0;i<8;i++) {
		UARTWriteChar(&AMP_RS232, tx_header[i]);
	}

	// response
	for (i=0;i<8;i++)
		rx_header[i] = UARTReadChar(&AMP_RS232);

	for (i=0;i<(length<<1)+2;i++)
		rx_data[i] = UARTReadChar(&AMP_RS232);

	
	printf("\nAmp read response:\n");
	printf("header:\n");
	for (i=0;i<8;i++)
		printf("0x%.2X\t",rx_header[i]);
	printf("\ndata:\n");
	for (i=0;i<(length<<1);i+=2) {
		printf("0x%.2X ",rx_data[i+1]);
		printf("0x%.2X\t",rx_data[i]);
	}
	printf("\n\n");
	
}




void botherAmp () {

	uint16_t data[16];

	// gain write access
//	data[0] = 0x000E;
//	ampWrite(0x07, 0x00, 0x01, data);
//	ampRead(0x07, 0x00, 0x01, (uint8_t*)data);
	
	// enable the bridge
//	data[0] = 0x0000;
//	ampWrite(0x01, 0x00, 0x01, data);

// current loop gains
//	data[0] = 60000;
//	ampWrite(0x34, 0x00, 0x01, data);
//	data[0] = 40000;
//	ampWrite(0x34, 0x01, 0x01, data);

	// Drive status
	ampRead(0x02, 0x00, 0x06, (uint8_t*)data);

	// Digital input values
	ampRead(0x23, 0x00, 0x01, (uint8_t*)data);

}



void readPWM () {
	uint8_t		data[2];
	
	ampRead(IDX_PWM, OFF_APPLIED_PWM, 1, data);
	
}



