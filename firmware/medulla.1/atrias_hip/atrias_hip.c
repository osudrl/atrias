// Kevin Kemper
//
// This program is designed to test basic functionality of the EtherCAT system.
//  It takes data written from an EtherCAT master, modifies it and returns it and
//  can be used to check the full loop timing of the master system by toggling
//  a pin when a new transmit request is received.
////////////////////////////////////////////////////////////////////////////////

#define DEBUG

#define ENABLE_LIMITS
#define ENABLE_TC_STEP

#ifdef ENABLE_LIMITS
	#define ENABLE_PWM
#endif

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
//#include "../libs/ssi_bang.h"
#include "../libs/ssi_spi.h"
//#include "../libs/biss_bang.h"
#include "../libs/biss_spi.h"
#include "../libs/adc.h"

#ifdef ENABLE_PWM
#include "../libs/pwm.h"
#endif

#include "../libs/quadrature.h"
#include "../libs/limitSW.h"
//#include "../libs/amp.h"
#include "../libs/timer.h"
#include "../libs/panic.h"
#include "../libs/led.h"


#include "../../../drl-sim/atrias/include/atrias/ucontroller.h"


#define ID	MEDULLA_HIP_ID

// state machine states
#define STATE_READY		0
#define STATE_START		1
#define STATE_RUN		2
#define STATE_ERROR		4
#define STATE_RESET		5


#define MAX_13BIT			8192
#define ROLLOVER_THRESHOLD	4096

#define PWM_SET		10000

// map the 0th usart on port E to the RS232 keyword for readability
#define RS232	USARTE0


// Interrupt handeler for the timer overflow.
ISR(TC_VEL_OVF_vect) {

	PWMdis();
	global_flags.status	|= STATUS_TCOVF;
	printf("Vel TC OVF\n");

}

// local functions...
void tc_VelStop();
void tc_VelStart();
void initVelTimer();

void		printMenu();
void		printState(uint8_t state);
static int	uart_putchar(char c, FILE *stream);


// global vars...

#ifdef DEBUG
static FILE mystdout = FDEV_SETUP_STREAM (uart_putchar, NULL, _FDEV_SETUP_WRITE);
#endif


////////////////////////////////////////////////////////////////////////////////

int main(void) {

	uint8_t i;
	uint8_t tmp;

	uControllerInput	in;
	uControllerOutput	out;
	
	float		delta	= 0;
	float		vel		= 0;
	uint16_t	error	= 0;
	int16_t		kD		= 2500;
	float		kP		= 0.003;

	uint8_t		limit_cnt		= 0;
	uint8_t		enc_cnt			= 0;
	uint8_t		panic_cnt		= 0;

	uint16_t	spring_set		= 8850;
	uint16_t	spring_pos		= 0;
	uint16_t	last_spring_pos	= 0;
	
	uint16_t	last_spring_cnt	= 0;
	uint16_t	spring_off		= MAX_13BIT;

	uint8_t		dir;
	uint16_t	pwm;
	uint16_t	ssi[4];
	
	uint8_t		toggle = CMD_RUN_TOGGLE_bm;
	
	// rs232 input data
	#ifdef DEBUG
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
	Config32MHzClock();															// Start up the high speed clock
//	Config32KHzRTC();
	
	cli();																		// Disable interrupts

	_delay_ms(2000);
	
	#ifdef DEBUG
	stdout = &mystdout;															// attach the uart to the stdout for printf
	initUART(&PORTE,&RS232,1152);												// RS232 uart 115.2kb
	#endif


	initLED();
	PMIC.CTRL |= PMIC_MEDLVLEN_bm;// | PMIC_RREN_bm;
	sei();
	
	SetDutyR(0x7FFF);
	SetDutyG(0x7FFF);
	SetDutyB(0x7FFF);
	
	pwm = PWM_SET;
	dir = 1;																	// << Don't touch a direction of zero will drive the hip into the hardstops+
	setDirection(dir);

	PORT_AMP_CTL.DIRSET = DIRECTION_bm;
	#ifdef ENABLE_PWM
	initPWM(pwm);
	#endif

	initTimer();
	
	initVelTimer();
	
	initPanic();
	
	#ifdef ENABLE_LIMITS
	PORTCFG.MPCMASK		= 0xFF;
	PORT_LIMIT.PIN0CTRL	= PORT_OPC_PULLUP_gc;
	
	PORT_LIMIT.DIR		= 0x00;
	#endif
	
	initeCAT();
	
	initSSI_spi();

	initQuad();

	
	#ifdef DEBUG
	PORTH.OUTCLR = (1<<1);
	PORTH.DIRSET = 1<<1;

	PORTH.OUTCLR = (1<<2);
	PORTH.DIRSET = 1<<2;

	PORTH.OUTCLR = (1<<7);
	PORTH.DIRSET = 1<<7;														// for the testing toggle
	#endif


	// init the output values
	out.id				= ID;
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


	// init the global struct
	global_flags.status		= 0;
	global_flags.limits		= 0;
	global_flags.state		= STATE_READY;
	global_flags.error_cnt	= 0;
	
	_delay_ms(100);

	////////////////////////////////////////////////////////////////////////////
	// init eCat stuff

	#ifdef DEBUG
	printf("\t\tmedulla.1: ATRIAS hip controller\n\n");							// print what program we're running...
	#endif
	

	while(!(PORT_ECAT.IN) & (1<<ECAT_EEPROM_LOADED));							// The EEPROM_LOADED signal indicates that the SPI interface is operational.
	
	
	
	al_event |= readAddr(AL_STATUS, &al_status, 1);								// check to see what the et1100 is doing so we're on the same page
	
	if (al_status == AL_STATUS_OP_bm) {											// store SM addresses:


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
	
	
	#ifdef ENABLE_LIMITS
	// Check the limit switches.
	if (PORT_LIMIT.IN != 0xA0) {
		PWMdis();
		global_flags.status	|= STATUS_LIMITSW;
		global_flags.limits	= ~PORT_LIMIT.IN;
		
		#ifdef DEBUG
		printf("THIS IS NOT THE HIP!!\n");
		#endif
		
		blinkLED();
		
		while(1) {
			SetDutyR(0x0FFF);
			SetDutyG(0x00FF);
			SetDutyB(0x07FF);
		}
	}
	#endif
	
	#ifdef ENABLE_PWM
	PWMen();
	#endif

	while(1) {

//XXX: this is a hack but should be ok now that we have a step timer...
		_delay_us(10);
		
	

		#if 1
		if (RS232.STATUS&USART_RXCIF_bm) {										// do we have something from the RS232?
			rs_data = RS232.DATA;

			switch (rs_data) {
		
				case '~':
					global_flags.state = STATE_RESET;
					break;
		
				case 'o':
					printf("Out:\n");
					printf("out.status:			%u\n", out.status);
					printf("TC_VEL.CNT:			%u\n", TC_VEL.CNT);
					break;


				case 'i':
					printf("In size: %u\n",sizeof(in));
					printf("in.command:	%u\n", in.command);
					printf("in.motor:	%u\n", in.motor);
					break;

				case 'm':
					printf("pos %u	set %u\n", spring_pos, spring_set);
					break;

				case '{':
					if (kD > 10)
						kD-=10;
					printf("%u\n", kD);
					break;

				case '}':
					if (kD < 5000)
						kD+=10;
					printf("%u\n", kD);
					break;

				case '<':
					if (kP > 0)
						kP-=0.0001;
//					printf("%u\n", kP);
					break;

				case '>':
					if (kP < 1000)
						kP+=0.0001;
//					printf("%u\n", kP);
					break;

				case '-':
					if (spring_set > 10)
						spring_set-=10;
					printf("%u\n", spring_set);
					break;

				case '=':
					if (spring_set < 20000)
						spring_set+=10;
					printf("%u\n", spring_set);
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

		////////////////////////////////////////////////////////////////////////
		// Update Encoder values and sensors
		tmp = readSSI_spi(ssi);
		if (tmp == 0xFF) {
//			global_flags.error_cnt+=2;
		}
		else if (ssi[1] > 8191) {												// The ssi value is invalid (>8191)
			enc_cnt+=2;
		}
		else {
			out.TRANS_ANGLE = ssi[1];
		}

		// Transmission A rollovers.
		if ( (out.TRANS_ANGLE > last_spring_cnt) && (out.TRANS_ANGLE - last_spring_cnt > ROLLOVER_THRESHOLD) ) {
			spring_off -= MAX_13BIT;
		}
		else if ( (last_spring_cnt > out.TRANS_ANGLE) && (last_spring_cnt - out.TRANS_ANGLE > ROLLOVER_THRESHOLD) ) {
			spring_off += MAX_13BIT;
		}

		// Keep track of the last counts for sensors that could rollover.
		last_spring_cnt = out.TRANS_ANGLE;
		
		spring_pos = out.TRANS_ANGLE+spring_off;


		///////////////////////////////////////////////////////////////////////
		switch (global_flags.state) {

//////////////////////////////////
			case STATE_READY :

				// Signal the all clear
				PanicClr();

				_delay_us(10);

				led_solid_red();
	
				// Wait for the other medullas to be ready.
				//	Error_cnt is inc. when the panic line is ok. This loop will only exit if
				//	the panic line is ok for 20 consectutive checks.
				if(global_flags.error_cnt < 200) {

					if ((PORT_PANIC.IN & PANIC_SENSE_bm) != 0) {				// if the panic is ok, inc. error_cnt
						global_flags.error_cnt +=2;
					}
					else {
						if (global_flags.error_cnt > 0)
							global_flags.error_cnt --;							// when it's not ok, dec. error_cnt
					}

				}
				else {
					global_flags.state = STATE_START;
					global_flags.error_cnt	= 0;
					#ifdef ENABLE_PWM
					PWMen();
					#endif
					
				}
					
				break;

//////////////////////////////////
			case STATE_START :
			
				led_solid_orange();
				
				if (pwm < PWM_SET) {
					pwm++;
					#ifdef ENABLE_PWM
					setPWM(pwm);
					#endif
				}
				else if (pwm > PWM_SET) {
					pwm--;
					#ifdef ENABLE_PWM
					setPWM(pwm);
					#endif
				}
				else {
					
					global_flags.status		= 0;
					global_flags.error_cnt	= 0;								// reset the error_cnt
					global_flags.limits		= 0;

					limit_cnt	= 0;
					enc_cnt		= 0;
					panic_cnt	= 0;

					out.status		= STATUS_DISABLED;

					
					
					// Enable the interrupts
					PMIC.CTRL |= PMIC_HILVLEN_bm;
//					PMIC.CTRL |= PMIC_MEDLVLEN_bm;
					PMIC.CTRL |= PMIC_LOLVLEN_bm;
					
					global_flags.state = STATE_RUN;
					
					// zero out the controller vars and start the timer
					TC_VEL.CNT = 0;
					tc_VelStart();
					vel = 0;
					delta = 0;
				
				}
				
								
				
				break;
			
//////////////////////////////////
			case STATE_RUN :
				
				// update timestamp
				out.timestep			= TC_STEP.CNT;

				
				// dampy control time
				if (TC_VEL.CNT != 0) {
					vel	= ((float)(spring_pos-(float)last_spring_pos))/((float)(TC_VEL.CNT));
				}
				else {
					vel = 0.0;
//					printf("z");
				}

				last_spring_pos = spring_pos;
				
				
				delta = 1.0*((float)kD)*vel - (float)(kP*((float)spring_set-(float)spring_pos));


				if (delta > 100.0) {											// clamp down the max change in position
					delta = 100.0;
				}
				else if (delta < -100.0) {
					delta = -100.0;
				}
				

				#ifdef ENABLE_PWM

				pwm += (int16_t)delta;

				if (pwm > 19000) {
					pwm = 19000;
				}
				else if (pwm < 1000) {
					pwm = 1000;
				}
				
				#endif
				

				TC_VEL.CNT = 0;

		
				if ((al_status == AL_STATUS_OP_bm)) {								// Stuff to do when eCAT is in OP mode
					
					
					// Deal with commands from the master
					if ( ( in.command & (~CMD_RUN_TOGGLE_bm)) == CMD_RUN ) {
					
						#ifdef ENABLE_TC_STEP
						tc_Start();
						#endif
						
						out.status		&= ~STATUS_DISABLED;					// clear the disabled
						

						if (in.HIP_MTR_CMD == HIP_CMD_PIN) {					// if we should be a pin-joint, change the pwm to what the controller says
							#ifdef ENABLE_PWM
							setPWM(pwm);
							#endif
						}
						else if (in.HIP_MTR_CMD == HIP_CMD_RIGID) {				// if we should be rigid, return to the center and hold
							#ifdef ENABLE_PWM
							pwm = PWM_SET;
							setPWM(pwm);
							#endif
							printf("r\n");
						}
						SetDutyG( 0x7FFF );
						
					}
					else if (in.command == CMD_DISABLE) {
						#ifdef ENABLE_PWM
						pwm = PWM_SET;
						setPWM(pwm);
						#endif
						SetDutyG( 0x000F );
										
						out.status |= STATUS_DISABLED;

					}
					
					// Play with the LED colors
					
					SetDutyR( abs(spring_set-spring_pos)<<4 );
					SetDutyB( 0x00FF );
					
				}
				else {															// Stuff to do when eCAT is NOT in OP mode
					
					tc_Stop();
//					tc_VelStop();
					
					#ifdef ENABLE_PWM
					pwm = PWM_SET;
					setPWM(pwm);
					#endif
							
					led_solid_purple();
					
				}


				// check on the panic line
				if ((PORT_PANIC.IN & PANIC_SENSE_bm) == 0) {
					panic_cnt+=2;
				}
				
				if ( panic_cnt > 100 ) {
					PanicSet();
					global_flags.state = STATE_ERROR;
				}
				
				#ifdef ENABLE_LIMITS
				// Check the limit switches.  This must be done this way to filter
				//  the noise from the motor/amp.
				if ( (PORT_LIMIT.IN & ~0xA0) != 0) {
					limit_cnt+=2;
				}
				if ( limit_cnt > 100 ) {
					PWMdis();
					global_flags.status		|= STATUS_LIMITSW;
					global_flags.limits		= ~PORT_LIMIT.IN;
					global_flags.state		= STATE_ERROR;
				}
				#endif

				// Check if the error counter is too big, if it is set the panic line and GTFO
				if ( enc_cnt > 100 ) {
					PanicSet();
					global_flags.status |= STATUS_ENC;
					global_flags.state = STATE_ERROR;
				}

				// decay the error counters
				if (panic_cnt > 0)
					panic_cnt--;
					
				if (limit_cnt > 0)
					limit_cnt--;
					
				if (enc_cnt > 0)
					enc_cnt--;
				
				break;

//////////////////////////////////
			case STATE_RESET :
			
			
				#ifdef DEBUG
				printf("\n\nRESET: GOING DOWN!!!\n\n");
				#endif
				
				#ifdef ENABLE_PWM
				PWMdis();
				#endif
				
				_delay_ms(10);
					
				CCP			= CCP_IOREG_gc;
				RST.CTRL	= RST_SWRST_bm;
				break;
			
			case STATE_ERROR :
			default :

				PanicSet();
				tc_Stop();
				tc_VelStop();

				if ((al_status == AL_STATUS_OP_bm)) {								// Stuff to do when eCAT is in OP mode
					
					// Deal with commands from the master
					if (in.command == CMD_RESTART) {
						global_flags.state	= STATE_READY;

					}
				}
				
				#ifdef ENABLE_PWM
				PWMdis();
				#endif
				
				led_blink_red();

				break;
				
		}



		////////////////////////////////////////////////////////////////////////
		// Manage any status changes caused by interrupts
		if (global_flags.status != 0) {
			
			#ifdef DEBUG
			if (global_flags.status & STATUS_LIMITSW)
					printf("Limit SW: %X\n",global_flags.limits);
					
			if (global_flags.status & STATUS_TCOVF)
					printf("TOV\n");

			if (global_flags.status & STATUS_BADPWM)
					printf("Bad PWM\n");
					
			if (global_flags.status & STATUS_ENC)
					printf("Bad PWM\n");

//			if (global_flags.status & STATUS_PANIC)
//					printf("PANIC!!!\n");

//			if (global_flags.status & STATUS_BADCMD)
//					printf("Bad Cmd: 0x%.2X\n", in.command);

//			printf("\t\t\t0x%.2X\n", global_flags.status);
			#endif	

			global_flags.state	= STATE_ERROR;
			out.status			= global_flags.status;
			global_flags.status	= 0;
		}
		
		
		
		
		////////////////////////////////////////////////////////////////////////
		// Manage EtherCAT stuff

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

					PORTH.OUTTGL = (1<<7);

					#ifdef DEBUG
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
						}
						
					}
					else {
						#ifdef DEBUG
						printf("CMD_BAD: 0x%.2X\n", in.command);
						#endif
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

		PORTH.OUTTGL = (1<<1);
		#ifndef ENABLE_TC_STEP
		TC_STEP.CNT = 0;
		#endif
	
	} // end while
} // end main

////////////////////////////////////////////////////////////////////////////////
// Helper functions...

void tc_VelStop() {
	
	TC_VEL.CTRLA = ( TC_VEL.CTRLA & ~TC1_CLKSEL_gm ) | TC_CLKSEL_OFF_gc;
	TC_VEL.CNT = 0;
}

void tc_VelStart() {
	TC_VEL.CTRLA = ( TC_VEL.CTRLA & ~TC1_CLKSEL_gm ) | TC_CLKSEL_DIV2_gc;
}


void initVelTimer() {

	// Set period/TOP value
//	TC_SetPeriod( &TC_STEP	, 0x1000 );

	tc_VelStop();
	
	// Set the overflow interrupt as high level.
	TC_VEL.INTCTRLA = ( TC_VEL.INTCTRLA & ~TC1_OVFINTLVL_gm ) | TC_OVFINTLVL_HI_gc;
	
	
}


// print the options available to the user
#ifdef DEBUG
void printMenu () {
	printf("\n");
	printf("Current state: %u\n", global_flags.state);
	printf("Menu Options:\n");
	printf("~: Software Reset\n");
	printf("o: Out values\n");
	printf("<: Decrease PWM\n");
	printf(">: Increase PWM\n");
	printf("m: Toggle Direction\n");
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

