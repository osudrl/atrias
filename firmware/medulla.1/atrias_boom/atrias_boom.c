// Kevin Kemper
//
// This program is designed to test basic functionality of the EtherCAT system.
//  It takes data written from an EtherCAT master, modifies it and returns it and
//  can be used to check the full loop timing of the master system by toggling
//  a pin when a new transmit request is received.
////////////////////////////////////////////////////////////////////////////////

#define DEBUG

#define ENABLE_TC_STEP

//#define ENABLE_PAN_LIMIT
//#define ENABLE_TILT_LIMIT
//#define ENABLE_ROLL_LIMIT



// delay.h needs to know how fast the clk is rolling along at
#define F_CPU 32000000UL

#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "./boom_io.h"

#include "../libs/uart.h"
#include "../libs/ecat.h"
#include "../libs/clock.h"
#include "../libs/HBA4.h"
#include "../libs/timer.h"
#include "../libs/panic.h"
#include "../libs/led.h"


#include "../../../drl-sim/atrias/include/atrias/ucontroller.h"


#define ID	MEDULLA_BOOM_ID

// state machine states
#define STATE_READY		0
#define STATE_START		1
#define STATE_RUN		2
#define STATE_ERROR		4
#define STATE_RESET		5


#define ERROR_MAX		100

#define ABS(x)	(((x) < 0) ? -(x) : (x))

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

	uint8_t i, flag = 0;
	uint8_t		tmp			= 0;
	uint16_t	tmp16		= 0;
	
	uint8_t		cnt			= 0;
	
	uint8_t		panic_cnt	= 0;
	uint8_t		enc_cnt		= 0;
	
	uint32_t	ave_tilt	= 0;
	uint32_t	ave_pan		= 0;
	
	uint8_t	toggle = CMD_RUN_TOGGLE_bm;
	
	uint16_t start_tilt = 0;
	uint16_t start_pan = 0;

	uControllerInput	in;
	uControllerOutput	out;
	

	uint16_t		hba4[4];

	
	// rs232 input data
	#ifdef DEBUG
	uint8_t			rs_data;
	#endif
	
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
	
	
	#ifdef DEBUG
	stdout = &mystdout;															// attach the uart to the stdout for printf
	initUART(&PORTE,&RS232,1152);												// RS232 uart 115.2kb
	#endif

	initLED();
	PMIC.CTRL |= PMIC_MEDLVLEN_bm;// | PMIC_RREN_bm;
	sei();
	
	PORTJ.DIRSET = LED_R_bm;
	
	blinkLED();
	SetDutyR(0x7FFF);
	SetDutyG(0x7FFF);
	SetDutyB(0x7FFF);
	
	initPanic();
	

	initHBA4();


	initeCAT();
	
	// init the Hardstop pin	
	PORTCFG.MPCMASK = HS_bm;
	PORT_LIMIT.PIN0CTRL	= PORT_OPC_PULLUP_gc;
	PORT_HS.DIRCLR = HS_bm;
	

//	#ifdef DEBUG
	PORTH.OUTCLR = (1<<1);
	PORTH.DIRSET = 1<<1;

	PORTH.OUTCLR = (1<<2);
	PORTH.DIRSET = 1<<2;

	PORTH.OUTCLR = (1<<6);
	PORTH.DIRSET = 1<<6;

	PORTH.OUTCLR = (1<<7);
	PORTH.DIRSET = 1<<7;														// for the testing toggle
//	#endif

	// init the output values
	out.id				= ID;
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
	
	in.motor		= 0;
	in.command		= CMD_DISABLE;

	// init the global struct
	global_flags.status		= 0;
	global_flags.limits		= 0;
	global_flags.state		= STATE_ERROR;
	global_flags.error_cnt	= 0;
	
	

////////////////////////////////////////////////////////////////////////////////
// init eCat stuff

	#ifdef DEBUG
	printf("\t\tmedulla.1: ATRIAS boom controller, ID %.2X : %s\n\n",ID,__DATE__);							// print what program we're running...
	#endif
	

	while(!(PORT_ECAT.IN) & (1<<ECAT_EEPROM_LOADED));							// The EEPROM_LOADED signal indicates that the SPI interface is operational.
	
	
	
	al_event |= readAddr(AL_STATUS, &al_status, 1);								// check to see what the et1100 is doing so we're on the same page
	
	if (al_status == AL_STATUS_OP_bm) {											// store SM addresses:

//		global_flags.state	= STATE_OP;

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
	
	_delay_ms(1);
	getHBA4pos0(&hba4[0]);
	getHBA4pos1(&hba4[1]);
	getHBA4pos2(&hba4[2]);
	
	while(1) {

//XXX: this is a hack but should be ok now that we have a step timer...
//		_delay_us(500);



		#ifdef DEBUG
		if (RS232.STATUS&USART_RXCIF_bm) {										// do we have something from the RS232?
			rs_data = RS232.DATA;

			switch (rs_data) {
		
				case '~':
					global_flags.state = STATE_RESET;
					break;
		
				case 'o':
//					getHBA4pos(hba4);
//					out.BOOM_TILT			= hba4[0];
//					out.BOOM_PAN		= hba4[1];
//					out.BOOM_ROLL			= hba4[2];
					printf("Out:\n");
					printf("out.status:			0x%.2X\n", out.status);
					printf("out.timestep:		%u\n", out.timestep);
					printf("out.BOOM_TILT_CNT:		%u\n", out.BOOM_TILT_CNT);
					printf("out.BOOM_ROLL_CNT:		%u\n", out.BOOM_ROLL_CNT);
					printf("out.BOOM_PAN_CNT:		%u\n", out.BOOM_PAN_CNT);
					break;


				case 'i':
					printf("In size: %u\n",sizeof(in));
					printf("in.command:	%u\n", in.command);
					printf("in.motor:	%u\n", in.motor);
					break;
				
				case 'm':
					printf("t %u	p %u\n",start_tilt, start_pan);
					break;

				case 's' :
					printf("state: %u\n", global_flags.state);
					break;
				
				case 'p':
					printf("panic %u\n", (PORT_PANIC.IN & PANIC_SENSE_bm));
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
				
					if (al_status != AL_STATUS_OP_bm) { printf("eCAT not in OP... so no write for you!\n"); break;}
					
//					printf("\tWriting %d to SM3 at 0x%X\n",ssi[0],SM3_addr);
//					al_event |= writeAddr(SM3_addr, dummy, SM3_len);
					
					break;

				case 'R':
					if (al_status != AL_STATUS_OP_bm) { printf("eCAT not in OP... so no read for you!\n"); break;}


					al_event = readAddr(SM0_addr, data, (uint16_t)512);

					
					printf("Read from SM0 at 0x%X:\n",SM0_addr);
					
					for(i=0;i<16;i++) {
						printf("0x%.2X\t", *(data+i));
					}
					printf("\n");
					break;
					
				case 'r':
					if (al_status != AL_STATUS_OP_bm) { printf("eCAT not in OP... so no read for you!\n"); break;}

//					if (SM2_len <= 8)
//						al_event = readAddr(SM2_addr, data, SM2_len);
//					else
//						al_event = readAddr(SM2_addr, data, 8);
					
					printf("Read from SM2 at 0x%X:\n",SM2_addr);
					
//					printf("Ch1: 0x%.2X Ch2: 0x%.4X",in.ch1, in.ch2);
					printf("\n");

					break;
					
				default:
//					printMenu();
					break;
			
			}
		}
		#endif


		///////////////////////////////////////////////////////////////////////

		switch (global_flags.state) {

			case STATE_READY :

				// Signal the all clear
				PanicClr();
				out.status				= 0;
				
				led_solid_red();
				
				_delay_us(100);

				// update the encoder values
				tmp = getHBA4pos0(&hba4[0]);
				while (tmp && (enc_cnt<ERROR_MAX) ) {
//					printf("O");
					enc_cnt += 10;
					_delay_us(50);
					tmp = getHBA4pos0(&hba4[0]);
				}
//				printf("\n");


				tmp = getHBA4pos1(&hba4[1]);
				while (tmp && (enc_cnt<ERROR_MAX) ) {
//					printf("1");
					enc_cnt += 10;
					_delay_us(50);
					tmp = getHBA4pos1(&hba4[1]);
				}
								

				// If the value is sane, average it
				if (( hba4[0] != 0) && ( hba4[1] != 0)) {
				
					ave_tilt	+= hba4[0];
					ave_pan		+= hba4[1];
					cnt++;
					
				}

				if (cnt > 100) {
					global_flags.state = STATE_START;
					out.BOOM_TILT_CNT	= (uint16_t)(ave_tilt/100);
					out.BOOM_PAN_CNT	= (uint16_t)(ave_pan/100);
					#ifdef DEBUG
					printf("START\n");
					#endif
				}
					
				if (enc_cnt > 0) {								// decay the error counter
					enc_cnt = 0;
				}
				
				
				
				
				
				break;

			case STATE_START :
				
				start_tilt	= out.BOOM_TILT_CNT;
				start_pan	= out.BOOM_PAN_CNT;
				
				enc_cnt					= 0;									// reset the error_cnt
				panic_cnt				= 0;
				global_flags.limits		= 0;
				
				flag		= 0;
				cnt			= 0;
				ave_tilt	= 0;
				ave_pan		= 0;
				out.status				= 0;
				global_flags.status		= 0;


				led_solid_orange();

				
				// Enable the interrupts
				PMIC.CTRL |= PMIC_HILVLEN_bm;
//				PMIC.CTRL |= PMIC_MEDLVLEN_bm;
				PMIC.CTRL |= PMIC_LOLVLEN_bm;

				global_flags.state = STATE_RUN;
				#ifdef DEBUG
				printf("RUN\n");
				#endif
				break;
			

			case STATE_RUN :
				

				// Check to see if the boom is within the limits
				#ifdef ENABLE_TILT_LIMIT
				if (out.BOOM_TILT_CNT < BOOM_TILT_LOWER) {
					global_flags.status |= STATUS_LIMITSW;
					global_flags.limits |= 1<<0;					
				}
				else if (out.BOOM_TILT_CNT > BOOM_TILT_UPPER) {
					global_flags.status |= STATUS_LIMITSW;
					global_flags.limits |= 1<<1;
				}
				#endif
				
				#ifdef ENABLE_ROLL_LIMIT
				if (out.BOOM_ROLL_CNT < BOOM_ROLL_LOWER) {
					global_flags.status |= STATUS_LIMITSW;
					global_flags.limits |= 1<<2;					
				}
				else if (out.BOOM_ROLL_CNT > BOOM_ROLL_UPPER) {
					global_flags.status |= STATUS_LIMITSW;
					global_flags.limits |= 1<<3;
				}
				#endif
				
				#ifdef ENABLE_PAN_LIMIT
				if (out.BOOM_PITCH_CNT < BOOM_PITCH_LOWER) {
					global_flags.status |= STATUS_LIMITSW;
					global_flags.limits |= 1<<4;					
				}
				else if (out.BOOM_PITCH_CNT > BOOM_PITCH_UPPER) {
					global_flags.status |= STATUS_LIMITSW;
					global_flags.limits |= 1<<5;
				}				
				#endif
				


				if ((al_status == AL_STATUS_OP_bm)) {								// Stuff to do when eCAT is in OP mode
					
					// Deal with commands from the master
					if ( ( in.command & (~CMD_RUN_TOGGLE_bm)) == CMD_RUN ) {
						tc_Start();
						out.status		&= ~STATUS_DISABLED;
					}
					else if (in.command == CMD_DISABLE) {
						
						out.status |= STATUS_DISABLED;

					}

					
					// Play with the LED colors
					SetDutyR( out.BOOM_ROLL_CNT );
					SetDutyG( out.BOOM_TILT_CNT );
					SetDutyB( out.BOOM_PAN_CNT );
					
				}
				else {															// Stuff to do when eCAT is NOT in OP mode
					
					tc_Stop();
					
					led_solid_purple();
					
				}


				// Check if the error counter is too big, if it is set the panic line and GTFO
				if ( enc_cnt >= ERROR_MAX ) {
					PanicSet();
					global_flags.status |= STATUS_ENC;
					global_flags.state	= STATE_ERROR;
				}

				if (enc_cnt > 0)									// decay the error counter
					enc_cnt-=1;
				
				break;
			
			
			case STATE_RESET :
			
				#ifdef DEBUG
				printf("\n\nRESET: GOING DOWN!!!\n\n");
				#endif
				
				_delay_ms(10);
				// TODO: reset the eCAT too maybe?
					
				CCP			= CCP_IOREG_gc;
				RST.CTRL	= RST_SWRST_bm;
				break;
			
			case STATE_ERROR :
			default :
				
				PanicSet();
				tc_Stop();
				
				// set the color based on the type of problem
				if ( out.status == STATUS_LIMITSW)
					led_blink_orange();
				else if ( out.status == STATUS_ENC )
					led_blink_yellow();
				else
					led_blink_red();

				if ((al_status == AL_STATUS_OP_bm)) {								// Stuff to do when eCAT is in OP mode
					
					// Deal with commands from the master
					if (in.command == CMD_RESTART) {
						global_flags.state	= STATE_READY;
						#ifdef DEBUG
						printf("READY\n");
						#endif
					}
				}
				
				led_blink_red();

				break;
				
		}


		// Handle the hardstop
		if ( (PORT_HS.IN & HS_bm) != 0) {	// stop is set
			global_flags.state	= STATE_ERROR;
			out.status			|= STATUS_DANGER;
			#ifdef DEBUG
			printf("Kill!\n");
			#endif
		}


		////////////////////////////////////////////////////////////////////////
		// Manage any status changes caused by interrupts
		if (global_flags.status != 0) {
			
			if (global_flags.status & STATUS_LIMITSW) {
				printf("Limit SW: %X\n",global_flags.limits);				
				global_flags.state		= STATE_ERROR;
			}
					
			if (global_flags.status & STATUS_TCOVF) {
				printf("TOV\n");
				global_flags.state		= STATE_ERROR;
			}

			if (global_flags.status & STATUS_BADPWM) {
				printf("Bad PWM\n");
				global_flags.state		= STATE_ERROR;
			}
			
			if (global_flags.status & STATUS_ENC) {
				printf("Bad encoder\n");
				global_flags.state		= STATE_ERROR;
			}
			
			if (global_flags.status & STATUS_BADCMD) {
				printf("Bad cmd: 0x%.2X\n", in.command);
				global_flags.state		= STATE_ERROR;
			}


			out.status			   |= global_flags.status;
			global_flags.status		= 0;
		}
		
		
		
		
		////////////////////////////////////////////////////////////////////////
		// Manage EtherCAT stuff
		PORTH.OUTTGL = (1<<2);

		if (al_status == AL_STATUS_OP_bm) {										// if we're in OP mode, update the txpdo on the ET1100
			al_event |= writeAddr(SM3_addr, &out  , SM3_len);
		}
		else {																	// else keep updating al_event
			al_event |= eCATnop();
		}
		
		if (al_event & 0x01) {													// if al_control needs checking...
			#ifdef DEBUG
//			printf("AL_Event Request:	0x%2X\n",al_event);
			#endif

			al_event = readAddr(AL_CONTROL, &al_status, 1);
			writeAddr(AL_STATUS, &al_status, 1);

			#ifdef DEBUG
//			printf("\t");
//			printState(al_status);
//			printf("\n");
			#endif
			
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
					al_event = readAddr(SM2_addr, &in, SM2_len);

					#ifdef DEBUG
					PORTH.OUTTGL = (1<<7);
//					printf("%.2X\n",in.command);
					#endif
					if (in.command == CMD_RESET) {
						#ifdef DEBUG
						printf("CMD_RESET\n");
						#endif
						global_flags.state = STATE_RESET;	
					}
					else if (in.command == CMD_RESTART) {
					
						#ifdef ENABLE_TC_STEP
						tc_Stop();
						#endif
						
						#ifdef DEBUG
						printf("CMD_RESTART\n");
						#endif

						toggle			= CMD_RUN_TOGGLE_bm;

					}
					else if (in.command == CMD_DISABLE) {
					
						tc_Stop();
//						out.status		|= STATUS_DISABLED;
						toggle			= CMD_RUN_TOGGLE_bm;

						#ifdef DEBUG
//						printf("CMD_DISABLE\n");
						#endif
					}
					else if (( in.command & (~CMD_RUN_TOGGLE_bm)) == CMD_RUN) {		// if we have a run command
						
						// Check the toggle bit in the command
						if (toggle != ( in.command & CMD_RUN_TOGGLE_bm )) {	// toggle bit is good
						
							toggle = in.command & CMD_RUN_TOGGLE_bm;

							// reset the time step counter
							#ifdef ENABLE_TC_STEP
							TC_STEP.CNT = 0;
							#endif
							
							// update the encoder values
							tmp = getHBA4pos0(&hba4[0]);
							while (tmp && (enc_cnt<ERROR_MAX) ) {
//								printf("O");
								enc_cnt += 10;
								_delay_us(50);
								tmp = getHBA4pos0(&hba4[0]);
							}
//							printf("\n");


							tmp = getHBA4pos1(&hba4[1]);
							while (tmp && (enc_cnt<ERROR_MAX) ) {
//								printf("1");
								enc_cnt += 10;
								_delay_us(50);
								tmp = getHBA4pos1(&hba4[1]);								
							}
//							printf("\n");

//							tmp = getHBA4pos2(&hba4[2]);
//							while (tmp && (enc_cnt<ERROR_MAX) ) {
//								printf("2");
//								enc_cnt += 20;
//								_delay_us(50);
//								tmp = getHBA4pos2(&hba4[2]);
//							}
//							printf("\n");

							if (hba4[0] - out.BOOM_TILT_CNT > 0)
								tmp16 = hba4[0] - out.BOOM_TILT_CNT;
							else
								tmp16 = out.BOOM_TILT_CNT - hba4[0];
							
							
							if ((tmp16 < 5000) || (tmp16 > 0x7FFF)) {
								out.BOOM_TILT_CNT	= hba4[0];
								flag &= ~(1<<0);
							}
							else {
								enc_cnt += 35;
								if ((flag & (1<<0)) == 0)
									printf("T %u  %u\n",out.BOOM_TILT_CNT, hba4[0]);
								flag |= (1<<0);
							}


							if (hba4[1] - out.BOOM_PAN_CNT > 0)
								tmp16 = hba4[1] - out.BOOM_PAN_CNT;
							else
								tmp16 = out.BOOM_PAN_CNT - hba4[1];
							
							
							if ((tmp16 < 5000) || (tmp16 > 0x7FFF)) {
								out.BOOM_PAN_CNT	= hba4[1];
								flag &= ~(1<<1);
							}
							else {
								enc_cnt += 35;
								if ((flag & (1<<1)) == 0)
									printf("P %u  %u\n",out.BOOM_PAN_CNT, hba4[1]);
								flag |= (1<<1);
							}
							//printf("\n");


							// update timestamp
							out.timestep			= TC_STEP.CNT;
						}
						
					}
					else {
//						#ifdef DEBUG
//						printf("CMD_BAD: 0x%.2X\n", in.command);
//						#endif
						global_flags.status |= STATUS_BADCMD;
					}
					
				
					
				}
				
			}
			else if (al_event & 0x0800) {										// SM3...
				al_event |= readAddr(SM3_BASE+SM_STATUS, data, 1);
			
			}
			else {
				//ignore
			}
		
		}

		
		
		#ifndef ENABLE_TC_STEP
		TC_STEP.CNT = 0;
		#endif
	
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
	printf("i: In values\n");
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
		case AL_STATUS_INIT_bm:
			printf("init");
			break;
		case AL_STATUS_PREOP_bm:
			printf("pre-op");
			break;
		case AL_STATUS_SAFEOP_bm:
			printf("safe op");
			break;
		case AL_STATUS_OP_bm:
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

