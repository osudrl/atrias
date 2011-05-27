// Kevin Kemper
//
// This program is designed to test basic functionality of the EtherCAT system.
//  It takes data written from an EtherCAT master, modifies it and returns it and
//  can be used to check the full loop timing of the master system by toggling
//  a pin when a new transmit request is received.
////////////////////////////////////////////////////////////////////////////////

#define DEBUG


// delay.h needs to know how fast the clk is rolling along at
#define F_CPU 32000000UL

#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "../libs/menial_io.h"

#include "../libs/uart.h" 
#include "../libs/ecat.h"
#include "../libs/clock.h"

#include "../libs/timer.h"
#include "../libs/led.h"

#include "../../../drl-sim/atrias/include/atrias/ucontroller.h"


#define ID	MEDULLA_A_ID


// map the 0th usart on port E to the RS232 keyword for readability
#define RS232	USARTE0




// local functions...
void		printMenu();
void		printState(uint8_t state);
static int	uart_putchar(char c, FILE *stream);


// global vars...

#ifdef DEBUG
static FILE mystdout = FDEV_SETUP_STREAM (uart_putchar, NULL, _FDEV_SETUP_WRITE);
#endif


////////////////////////////////////////////////////////////////////////////////

int main(void) {

	uControllerInput	in;
	uControllerOutput	out;
	
	uint8_t			rx[16];
	
	uint8_t			toggle = CMD_RUN_TOGGLE_bm;
	
	uint8_t			loopCnt = 0;
	
	// rs232 input data
	#ifdef DEBUG
	uint8_t			i;
	uint8_t			rs_data;
	#endif
	
	// eCat vars - Don't touch unless you know what you're doing!
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
//	Config32KHzRTC();
	
	
	cli();																		// Disable interrupts


	
	#ifdef DEBUG
	stdout = &mystdout;															// attach the uart to the stdout for printf
	initUART(&PORTE,&RS232,1152);												// RS232 uart 115.2kb
	#endif


	initLED();
	PMIC.CTRL |= PMIC_MEDLVLEN_bm;// | PMIC_RREN_bm;
	sei();
	
	SetDutyR(0x7FFF);
	SetDutyG(0x000F);
	SetDutyB(0x000F);
	

	initTimer();

	initeCAT();
	
	
//	#ifdef DEBUG
	PORTH.OUTCLR = (1<<1);
	PORTH.DIRSET = 1<<1;

	PORTH.OUTCLR = (1<<2);
	PORTH.DIRSET = 1<<2;

	PORTH.OUTCLR = (1<<7);
	PORTH.DIRSET = 1<<7;														// for the testing toggle
//	#endif

	// init the output values
	out.id				= ID;
	out.status_byte		|= STATUS_DISABLED;
	out.enc32			= 0;
	out.enc16[0]		= 0;
	out.enc16[1]		= 0;
	out.enc16[2]		= 0;
	out.enc16[3]		= 0;
	out.timestep		= 0;
	out.therm1			= 0;
	out.therm2			= 0;
	out.therm3			= 0;
	
	in.motor_torque		= 0;
	in.command_byte		= CMD_DISABLE;



	////////////////////////////////////////////////////////////////////////////
	// init eCat stuff

	#ifdef DEBUG
	printf("\t\tmedulla.1: eCat test\n\n");							// print what program we're running...
	#endif
	

	while(!(PORT_ECAT.IN) & (1<<ECAT_EEPROM_LOADED));							// The EEPROM_LOADED signal indicates that the SPI interface is operational.
	
	
	
	al_event |= readAddr(AL_STATUS, &al_status, 1);								// check to see what the et1100 is doing so we're on the same page
	
	if (al_status == AL_STATUS_OP) {											// store SM addresses:


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

		#ifdef DEBUG
		printf("SM2_len: 0x%X, SM3_len: 0x%X\n",SM2_len,SM3_len);
		#endif
	}
	
	#ifdef DEBUG
	printMenu();
	#endif
	

	_delay_us(100);

	tc_Start();

	solidLED();

	while(1) {

//XXX: this is a hack but should be ok now that we have a step timer...
//		_delay_us(10);



		#if 1
		if (RS232.STATUS&USART_RXCIF_bm) {										// do we have something from the RS232?
			rs_data = RS232.DATA;

			switch (rs_data) {
		
				case '~':
					printf("\n\nRESET: GOING DOWN!!!\n\n");

					_delay_ms(100);
					
					CCP			= CCP_IOREG_gc;
					RST.CTRL	= RST_SWRST_bm;
					break;
		
				case 'o':
					printf("Out:\n");
					printf("out.id:			%u\n", out.id);
					printf("out.timestep:		%u\n", out.timestep);
					
					break;


				case 'i':
					printf("In size: %u\n",sizeof(in));
					printf("in.command_byte:	0x%.2X\n", in.command_byte);
					printf("in.motor_torque:	%u\n", in.motor_torque);
					break;

				case '0':
				case '1':
				case '2':
				case '3':
					if (rs_data == '0') {
						printf("SM0: ");
						al_event |= readAddr(SM0_BASE, data, 8);
					}
					else if (rs_data == '1') {
						printf("SM1: ");
						al_event |= readAddr(SM1_BASE, data, 8);
					}
					else if (rs_data == '2') {
						printf("SM2: ");
						al_event |= readAddr(SM2_BASE, data, 8);
					}
					else if (rs_data == '3') {
						printf("SM3: ");
						al_event |= readAddr(SM3_BASE, data, 8);
					}

					printf("	Physical Start Address:	0x%.4X\n",*((uint16_t*)(data+SM_PHY_ADDR)));
					printf("	Length: 0x%.4X/%d\n",*((uint16_t*)(data+SM_LENGTH)),*((uint16_t*)(data+SM_LENGTH)));
					printf("	Control Register: 0x%.2X\n",*(data+SM_CONTROL));
					printf("	Status Register: 0x%.2X\n",*(data+SM_STATUS));
					printf("	Activate: 0x%.2X\n",*(data+SM_ACTIVE));
					printf("	PDI Control: 0x%.2X\n",*(data+SM_PDI_CTRL));

					break;
					
				case 'w':
				
					if (al_status != AL_STATUS_OP) { printf("eCAT not in OP... so no write for you!\n"); break;}
					
//					printf("\tWriting %d to SM3 at 0x%X\n",ssi[0],SM3_addr);
//					al_event |= writeAddr(SM3_addr, dummy, SM3_len);
					
					break;

				case 'R':
					if (al_status != AL_STATUS_OP) { printf("eCAT not in OP... so no read for you!\n"); break;}


					al_event = readAddr(SM0_addr, data, (uint16_t)512);

					
					printf("Read from SM0 at 0x%X:\n",SM0_addr);
					
					for(i=0;i<16;i++) {
						printf("0x%.2X\t", *(data+i));
					}
					printf("\n");
					break;
					
				case 'r':
					if (al_status != AL_STATUS_OP) { printf("eCAT not in OP... so no read for you!\n"); break;}

//					if (SM2_len <= 8)
//						al_event = readAddr(SM2_addr, data, SM2_len);
//					else
//						al_event = readAddr(SM2_addr, data, 8);
					
					printf("Read from SM2 at 0x%X:\n",SM2_addr);
					
//					printf("Ch1: 0x%.2X Ch2: 0x%.4X",in.ch1, in.ch2);
					printf("\n");

					break;
					
				default:
					printMenu();
					break;
			
			}
		}
		#endif

//		out.timestep			= TC_STEP.CNT;
		

		////////////////////////////////////////////////////////////////////////
		// Manage EtherCAT stuff

		if (al_status == AL_STATUS_OP) {										// if we're in OP mode, update the txpdo on the ET1100
			al_event |= writeAddr(SM3_addr, &out  , SM3_len);
			SetDutyR( 0x00FF );
			SetDutyG( 0x7FFF );
			SetDutyB( 0x0FFF );

		}
		else {																	// else keep updating al_event
			al_event |= eCATnop();
			SetDutyR( 0x7FF0 );
			SetDutyG( 0x00FF );
			SetDutyB( 0x0FFF );
		}
		
		
		if (al_event & 0x01) {													// if al_control needs checking...
			#ifdef DEBUG
			printf("AL_Event Request:	0x%2X\n",al_event);
			#endif

			al_event = readAddr(AL_CONTROL, &al_status, 1);
			writeAddr(AL_STATUS, &al_status, 1);

			#ifdef DEBUG
			printf("\t");
			printState(al_status);
			printf("\n");
			#endif
			
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

//				printf("SM0 addr: 0x%X\n",SM0_addr);
//				printf("SM1 addr: 0x%X\n",SM1_addr);
//				printf("SM2 addr: 0x%X\n",SM2_addr);				
//				printf("SM3 addr: 0x%X\n",SM3_addr);
			}

			
		}
		
		if (al_event & 0xFF00) {												// if there's a SM interrupt pending...

			if (al_event & 0x0400) {											// SM2...
				al_event |= readAddr(SM2_BASE+SM_STATUS, data, 1);
				
				if (*data & 0x01) {												// stuff wrtten to the buffer!
					al_event = readAddr(SM2_addr, &rx, SM2_len);

					loopCnt++;
					out.timestep = loopCnt;
					
					TC_STEP.CNT = 0;
					PORTH.OUTTGL = (1<<7);

					if ( in.command_byte & (0x01) )
						PORTH.OUTSET = (1<<1);
					else
						PORTH.OUTCLR = (1<<1);

					#ifdef DEBUG
					printf("%.2X\n", rx[2]);
					#endif
					
/*					
					if (( in.command_byte & (~CMD_RUN_TOGGLE_bm)) == CMD_RUN) {		// if we have a run command
						
						// Check the toggle bit in the command
						if (toggle == ( in.command_byte & CMD_RUN_TOGGLE_bm )) {
//							global_flags.status |= STATUS_BADCMD;
//							global_flags.state = STATE_ERROR;
						}
						else {													// toggle bit is good
							toggle = in.command_byte & CMD_RUN_TOGGLE_bm;

							// reset the time step counter
							#ifdef ENABLE_TC_STEP
							TC_STEP.CNT = 0;
							#endif
						}
						
					}
*/					
				
					
				}
				
			}
			else if (al_event & 0x0800) {										// SM3...
				al_event |= readAddr(SM3_BASE+SM_STATUS, data, 1);
			
			}
			else {
				//ignore
			}
		
		}

		PORTH.OUTTGL = (1<<2);
		
//		#ifndef ENABLE_TC_STEP
//		TC_STEP.CNT = 0;
//		#endif
	
	} // end while
} // end main

////////////////////////////////////////////////////////////////////////////////
// Helper functions...


// print the options available to the user
#ifdef DEBUG
void printMenu () {
	printf("\n");
	printf("Current state: %u\n", global_flags.state);
	printf("Menu Options:\n");
	printf("~: Software Reset\n");
	printf("o: Out values\n");
	printf("\n");
	printf("0: SyncManager0 info\n");
	printf("1: SyncManager1 info\n");
	printf("2: SyncManager2 info\n");
	printf("3: SyncManager3 info\n");
	printf("\n");
	printf("w: Write SSI0 to SM3\n");
	printf("R: Read from SM0\n");
	printf("r: Read from SM2\n");
	printf("\n");
}


// translate the byte code "state" into something meaningfull for printing
void printState(uint8_t state) {

	switch (state) {
		case AL_STATUS_INIT:
			printf("init");
			break;
		case AL_STATUS_PREOP:
			printf("pre-op");
			break;
		case AL_STATUS_SAFEOP:
			printf("safe op");
			break;
		case AL_STATUS_OP:
			printf("op");
			break;
		default:
			printf("OMG!!!");
	}
}

// function to attach to stdout to the usart
static int uart_putchar (char c, FILE *stream) {

    if (c == '\n')
        uart_putchar('\r', stream);
 
    // Wait for the transmit buffer to be empty
    while ( !( RS232.STATUS & USART_DREIF_bm) );
 
    // Put our character into the transmit buffer
    RS232.DATA = c; 
 
    return 0;
}
#endif

