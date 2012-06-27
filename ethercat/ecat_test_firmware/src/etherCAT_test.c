#include "avr/interrupt.h"
#include <stdio.h>
#include "clock.h"
#include "ecat.h"
#include "uart.h"
#include "util/delay.h"

void init_eCAT(void);
void eCATWriteData(void* out);
void eCATReadData(void* in);

static int	uart_putchar(char c, FILE *stream);
static FILE mystdout = FDEV_SETUP_STREAM (uart_putchar, NULL, _FDEV_SETUP_WRITE);
uint8_t			rs_data;

// Ethercat variables
uint8_t			data[512];
uint8_t			al_status;
uint16_t		al_event;
uint16_t		SM0_addr, SM1_addr, SM2_addr, SM3_addr, SM2_len, SM3_len;

uint16_t		command_addr, motor_current_addr, timestep_addr, encoder0_addr;
uint8_t		command;
int16_t			motor_current;
uint8_t			out_data[6];
int16_t			*timestep_pntr = out_data;
uint32_t		*encoder0_pntr = out_data+2;

uint16_t		motor_current_old;

int main(void) {
	Config32MHzClock(); // Configure clock to 32 Mhz

	stdout = &mystdout;															// attach the uart to the stdout for printf
	initUART(&PORTD,&USARTD0,1152);												// RS232 uart 115.2kb
	printf("Starting\n");
	init_eCAT();	
	
	while(1) {
		motor_current++;
		eCATReadData(&command);

		(*timestep_pntr) = command;
		(*encoder0_pntr) = ((uint32_t)command)*1000;
	
		eCATWriteData(timestep_pntr);
		printf("%d\n", command);
	}
	
    return 0;
}

void init_eCAT() {
	// Init eCAT variables
	al_status	= 0;
	al_event	= 0;
	SM0_addr	= 0;
	SM1_addr	= 0;
	SM2_addr	= 0;
	SM3_addr	= 0;
	SM2_len		= 2;
	SM3_len		= 2;

	PORTC.DIRSET  = 1;
	
	initeCAT();						// Start the SPI driver
	while(!(PORT_ECAT.IN) & (1<<ECAT_EEPROM_LOADED));							// The EEPROM_LOADED signal indicates that the SPI interface is operational.
	printf("EEPROM loaded\n");
	_delay_ms(1000);
	uint8_t DLStatus;
	readAddr(0x0031,&DLStatus,1);
	printf("%X\n",DLStatus);

	al_event |= readAddr(AL_STATUS, &al_status, 1);								// check to see what the et1100 is doing so we're on the same page
	printf("al_status: %d\n",al_status);
	if (al_status == AL_STATUS_OP_bm) {											// if the eCAT is in OP, grab and store the SM addresses:


		al_event |= readAddr(SM0_BASE, data, 2);
		SM0_addr = *((uint16_t*)(data+SM_PHY_ADDR));

		al_event |= readAddr(SM1_BASE, data, 2);
		SM1_addr = *((uint16_t*)(data+SM_PHY_ADDR));
	
		al_event |= readAddr(SM2_BASE, data, 4);
		SM2_addr	= *((uint16_t*)(data+SM_PHY_ADDR));
		//SM2_len		= *((uint16_t*)(data+SM_LENGTH));

		al_event |= readAddr(SM3_BASE, data, 4);
		SM3_addr	= *((uint16_t*)(data+SM_PHY_ADDR));
		//SM3_len		= *((uint16_t*)(data+SM_LENGTH));
	}

}

void eCATWriteData(void* out) {
	//if (al_status == AL_STATUS_OP_bm) {										// if we're in OP mode, update the txpdo on the ET1100
		al_event |= writeAddr(0x2000, out, 6);
	//}
	//else {																	// else keep updating al_event
	//	al_event |= eCATnop();
	//}	
}

void eCATReadData(void* in) {
	if (al_event & 0x01) {													// if al_control needs checking...
		al_event = readAddr(AL_CONTROL, &al_status, 1);
	//	al_status |= 0b1111;
		//printf("AL_CONTROL: %d\n",al_status);
		writeAddr(AL_STATUS, &al_status, 1);

		if (al_status == AL_STATUS_OP_bm) {									// store SM addresses:
			
			al_event |= readAddr(SM0_BASE, data, 2);
			SM0_addr = *((uint16_t*)(data+SM_PHY_ADDR));

			al_event |= readAddr(SM1_BASE, data, 2);
			SM1_addr = *((uint16_t*)(data+SM_PHY_ADDR));
				
			al_event |= readAddr(SM2_BASE, data, 4);
			SM2_addr	= *((uint16_t*)(data+SM_PHY_ADDR));
			SM2_len		= *((uint16_t*)(data+SM_LENGTH));

			al_event |= readAddr(SM3_BASE, data, 4);
			SM3_addr	= *((uint16_t*)(data+SM_PHY_ADDR));
			SM3_len		= *((uint16_t*)(data+SM_LENGTH));
		}	
	}
	
		
//	if (al_event & 0xFF00) {												// if there's a SM interrupt pending...
//		if (al_event & 0x0400) {
//			al_event |= readAddr(SM2_BASE+SM_STATUS, data, 1);
				
//			if (*data & 0x01) {												// stuff wrtten to the buffer!
				al_event = readAddr(0x1000, in,3);
//			}
//		
//		}
//		else if (al_event & 0x0800) {										// SM3...
//			al_event |= readAddr(SM3_BASE+SM_STATUS, data, 1);
//		}
//		else {
//			//ignore
//		}
//	}
}

static int uart_putchar (char c, FILE *stream) {

    if (c == '\n')
        uart_putchar('\r', stream);
 
    // Wait for the transmit buffer to be empty
    while ( !( USARTD0.STATUS & USART_DREIF_bm) );
 
    // Put our character into the transmit buffer
    USARTD0.DATA = c; 
 
    return 0;
}

