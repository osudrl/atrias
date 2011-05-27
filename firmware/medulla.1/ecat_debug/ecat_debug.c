// Devin Koepl

////////////////////////////////////////////////////////////////////////////////

// delay.h needs to know how fast the clk is rolling along at
#define F_CPU 32000000UL

#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "../libs/ecat.h"

#include "../libs/clock.h"

#include "../../../drl-sim/atrias/include/atrias/ucontroller.h"

////////////////////////////////////////////////////////////////////////////////

int main(void) 
{
	uControllerInput	in;
	uControllerOutput	out;
	
	// eCat vars Don't touch unless you know what you're doing!
	uint8_t			data[512];

	uint8_t			al_status	= 0;
	uint16_t		al_event	= 0;
	
	uint16_t		SM0_addr	= 0;
	uint16_t		SM1_addr	= 0;
	uint16_t		SM2_addr	= 0;
	uint16_t		SM3_addr	= 0;
	
	uint16_t		SM2_len		= 0;
	uint16_t		SM3_len		= 0;

	////////////////////////////////////////////////////////////////////////////
	// init stuffs
	CLK.PSCTRL = 0x00;															// no division on peripheral clock
	Config32MHzClock();
	
	cli();																		// Disable interrupts
	
	_delay_ms(1000);

	PMIC.CTRL |= PMIC_MEDLVLEN_bm;// | PMIC_RREN_bm;
	sei();

	initeCAT();	
	
	// Setup port for testing.
	PORTH.DIR 			= 0xFF;

	// init the output values
	out.id				= 0;
//	out.status		|= STATUS_DISABLED;
	out.status			= 0;
	out.enc32			= 0;
	out.enc16[0]		= 0;
	out.enc16[1]		= 0;
	out.enc16[2]		= 0;
	out.enc16[3]		= 0;
	out.timestep		= 0;
	out.therm1			= 0;
	out.therm2			= 0;
	out.therm3			= 0;
	
	in.motor			= 0;
	in.command			= CMD_DISABLE;
	
////////////////////////////////////////////////////////////////////////////////
// init eCat stuff	

	while(!(PORT_ECAT.IN) & (1<<ECAT_EEPROM_LOADED));							// The EEPROM_LOADED signal indicates that the SPI interface is operational.

	al_event |= readAddr(AL_STATUS, &al_status, 1);								// check to see what the et1100 is doing so we're on the same page
	
	if (al_status == AL_STATUS_OP) 
	{											// store SM addresses:
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
	
	_delay_ms(1);
	
	while(1) {
		///////////////////////////////////////////////////////////////////////

		/*if ((al_status == AL_STATUS_OP)) {								// Stuff to do when eCAT is in OP mode

		}
		else {															// Stuff to do when eCAT is NOT in OP mode
			
		}*/

		////////////////////////////////////////////////////////////////////////
		// Manage EtherCAT stuff

		if (al_status == AL_STATUS_OP) {										// if we're in OP mode, update the txpdo on the ET1100
			al_event |= writeAddr(SM3_addr, &out  , SM3_len);
		}
		else {																	// else keep updating al_event
			al_event |= eCATnop();
		}
		
		if (al_event & 0x01) {													// if al_control needs checking...

			al_event = readAddr(AL_CONTROL, &al_status, 1);
			writeAddr(AL_STATUS, &al_status, 1);
		
			if (al_status == AL_STATUS_OP) {									// store SM addresses:		
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

		if (al_event & 0xFF00) {												// if there's a SM interrupt pending...

			if (al_event & 0x0400) {											// SM2...
				al_event |= readAddr(SM2_BASE+SM_STATUS, data, 1);
				
				if (*data & 0x01) {												// stuff wrtten to the buffer!
					al_event = readAddr(SM2_addr, &in, SM2_len);	

					// toggle a pin to indicate new data.
//					PORTH.OUT ^= 0x01;			
				}
				
			}
			else if (al_event & 0x0800) {										// SM3...
				al_event |= readAddr(SM3_BASE+SM_STATUS, data, 1);
			
			}		
		}

		// Toggle this pin every time we look for ecat interrupts.
		PORTH.OUT = (PORTH.OUT & ~0x01) | (in.command & 0x01);
		PORTH.OUTTGL = (1<<6);
	
	} // end while
} // end main

