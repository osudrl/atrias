// Kevin Kemper
//
// This program is designed to test basic functionality of the EtherCAT system.
//  It takes data written from an EtherCAT master, modifies it and returns it and
//  can be used to check the full loop timing of the master system by toggling
//  a pin when a new transmit request is received.
////////////////////////////////////////////////////////////////////////////////



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
#include "../libs/pwm.h"
#include "../libs/quadrature.h"
#include "../libs/limitSW.h"
#include "../libs/amp.h"
#include "../libs/timer.h"
#include "../libs/panic.h"


#include "../../../drl-sim/atrias/include/atrias/ucontroller.h"





// map the 0th usart on port E to the RS232 keyword for readability
#define RS232	USARTE0




// local functions...
void		printMenu();
void		printState(uint8_t state);
static int	uart_putchar(char c, FILE *stream);


// global vars...
static FILE mystdout = FDEV_SETUP_STREAM (uart_putchar, NULL, _FDEV_SETUP_WRITE);



////////////////////////////////////////////////////////////////////////////////

int main(void) {

	uint8_t i;
	uint8_t tmp;
	uint8_t loopCnt		= 0;

	uint8_t 		duty_r		= 1;
	uint8_t 		duty_g		= 16;
	uint8_t 		duty_b		= 16;
	

//	txpdo_struct	txpdo;
//	rxpdo_struct	rxpdo;
	
	uControllerInput	in;
	uControllerOutput	out;
	
	
	uint16_t		pwm;
	uint8_t			dir;
	uint16_t		ssi[4];
	uint8_t			biss[4];
	uint8_t			adc[2];
	
	uint16_t		calcFreq     = 0;
	uint16_t		calcRPM      = 0;

	
	uint8_t			rs_data;
	
	// eCat vars
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
	Config32KHzRTC();
	
	cli();
	
	// init LEDs
	PORT_LED.OUTSET	= LED_R_bm;													// Turn on the red LED
	PORT_LED.OUTCLR = LED_G_bm | LED_B_bm;										// Make sure the other colors are off
	PORT_LED.DIRSET	= LED_R_bm | LED_G_bm | LED_B_bm;							// set LED pins to outputs

	
	stdout = &mystdout;															// attach the uart to the stdout for printf
	initUART(&PORTE,&RS232,1152);												// RS232 uart 115.2kb

	
	initADC();
	
	pwm = 10;
	dir = 0;
	SetDirection(dir);
	initPWM(pwm);
	initAmp();

	initTimer();
	
//	initPanic();
	
//	initLimitSW();

	initeCAT();
	
	initSSI_spi();
	
//	initBiSS_bang();
	initBiSS_spi();
	
	initQuad();


	PORTH.DIRSET = 1<<7;														// for the testing toggle
	


	global_flags.status	= 0;
	

	out.id			= 0xA5;
	out.status_byte = 0;



	// init the tx and rx pdos	
//	txpdo.ch1 = 0xEE;
//	txpdo.ch2 = 0xABCD;
//	txpdo.ch3 = 0x89ABCDEF;
	
//	rxpdo.ch1 = 0;
//	rxpdo.ch2 = 0;
	

////////////////////////////////////////////////////////////////////////////////

	_delay_ms(100);
	printf("\t\tmedulla.1: FCA test\n\n");										// print what program we're running...
	
	
	// The EEPROM_LOADED signal indicates that the SPI interface is operational.
	while(!(PORT_ECAT.IN) & (1<<ECAT_EEPROM_LOADED));
	
	
	
	al_event |= readAddr(AL_STATUS, &al_status, 1);		// check to see what the et1100 is doing so we're on the same page
	
	if (al_status == AL_STATUS_OP) {					// store SM addresses:

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

		printf("SM2_len: 0x%X, SM3_len: 0x%X\n",SM2_len,SM3_len);
	}
	
	
	printMenu();
	
	duty_r		= 8;
	duty_g		= 16;
	duty_b		= 15;
	
	// Fire-up the timer
//	TC_Start();
	
	// Signal the all clear
	PanicClr();
	PanicEn();
	
//	PORT_PANIC.OUTCLR = PANIC_bm;
	
	sei();																		// enable interrupts
	
	while(1) {

//XXX: this is a hack but should be ok now that we have a step timer...
		_delay_us(1);


		if (RS232.STATUS&USART_RXCIF_bm) {										// do we have something from the RS232?
			rs_data = RS232.DATA;

			switch (rs_data) {
		
				case '~':
					printf("\n\nRESET: GOING DOWN!!!\n\n");
					_delay_ms(100);
					// TODO: reset the eCAT too...
					CCP = CCP_IOREG_gc;
					RST.CTRL = RST_SWRST_bm;
					break;
		
				case 's':
					printf("SSI Values: %u\t%u\t%u\t%u\n", ssi[0],ssi[1],ssi[2],ssi[3]);
					printf("BiSS Value: %u\t\t%.2X\t%.2X\t%.2X\t%.2X\n", ((uint16_t*)biss)[1],biss[3], biss[2], biss[1], biss[0]);
					break;
				
				case 'q':
					printf("Quadrature: %u\n", TC_ENC.CNT);
					printf("Freq: %u\t\tRPM: %u\n", calcFreq, calcRPM);
					break;
					
				case 'v':
					printf("Voltages:\nLogic: %d\tPower: %d\n", adc[0],	adc[1]);
					break;
					
				case 't':
					printf("Tempratures:\n");
					printf("Therm 0: %d\tTherm 1: %d\tTherm 2: %d\n", adc[2], adc[3], adc[4]);
					break;

				case '<':
					pwm -= 100;
					setPWM(pwm);
					printf("PWM: %u\n", pwm);
					break;

				case '>':
					pwm += 100;
					setPWM(pwm);
					printf("PWM: %u\n", pwm);
					break;

				case 'm':
					dir ^= 1;
					SetDirection(dir);
					printf("Direction: %u\n", dir);
					break;
					
				case 'b':
//					printf("Ask amp for Applied PWM\n");
					botherAmp();
					break;

				case 'p':
					printf("Ask amp for Applied PWM\n");
//					readTest();
//					readPWM();
					break;


				case 'e':
					al_event |= readAddr(PDI_ERROR, data, 1);
					printf("eCAT PDI error counter: %u\n", *data );
					break;

				case 'a':
					al_event |= readAddr(AL_STATUS, data, 1);
					printf("AL_Event Request:	0x%.4X\n",al_event);
					printf("data[0]: 		0x%.2X\n",data[0]);
					printf("stored al_status:	0x%.2X\t",al_status);
					printState(al_status);
					printf("\n");
					break;
					
				case 'd':
					al_event |= readAddr(DL_STATUS, data, 2);
					printf("DL Status: 0x%.4X\n",*(uint16_t*)data);
					printf("\n");
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
/*					
				case 'r':
					if (al_status != AL_STATUS_OP) { printf("eCAT not in OP... so no read for you!\n"); break;}

//					if (SM2_len <= 8)
//						al_event = readAddr(SM2_addr, data, SM2_len);
//					else
//						al_event = readAddr(SM2_addr, data, 8);
					
					printf("Read from SM2 at 0x%X:\n",SM2_addr);
					
					printf("Ch1: 0x%.2X Ch2: 0x%.4X",rxpdo.ch1, rxpdo.ch2);
					printf("\n");

					break;
*/					
				default:
					printMenu();
					break;
			
			}
		}

		///////////////////////////////////////////////////////////////////////
		// update the PWM from eCat
//		setPWM(in.motor_torque);
		out.status_byte = in.command_byte;


		///////////////////////////////////////////////////////////////////////
		// update the ADC values...
		if (ADC_LOGIC_FLAG) {
			// read adc
			adc[0] = ADC_LOGIC_VAL;
				
			// clear flag
			ADC_LOGIC_FLAG = 1;

			// start new conversion
			ADC_LOGIC_START;
		}
		if (ADC_POW_FLAG) {
			adc[1] = ADC_POW_VAL;
			ADC_POW_FLAG = 1;
			ADC_POW_START;
		}
		
		if (THERM0_FLAG) {
			out.thermistor1 = THERM0_VAL;
			THERM0_FLAG = 1;
			THERM0_START;
		}
		if (THERM1_FLAG) {
			out.thermistor2 = THERM1_VAL;
			THERM1_FLAG = 1;
			THERM1_START;
		}
		if (THERM2_FLAG) {
			out.thermistor3 = THERM2_VAL;
			THERM2_FLAG = 1;
			THERM2_START;
		}


		////////////////////////////////////////////////////////////////////////
		// update the encoder values
		readSSI_spi(ssi);
//		PORTH.OUTTGL = (1<<7);
//		tmp = readBiSS_bang(biss);
		tmp = readBiSS_spi(biss);
//		printf("%.2X\t%.2X\t%.2X\t%.2X\n", biss[3], biss[2], biss[1], biss[0]);
		if (tmp == 0)
			printf("BiSS: not ready - come back later.\n");
//		else if ((tmp & BISS_ERROR_bm) == 0)
//			printf("BiSS %.2X: error - the position data is junk.\n",tmp);
//		else if ((tmp & BISS_WARN_bm) == 0)
//			printf("BiSS %.2X: warning - the encoder is filthy.\n",tmp);
			
			
		out.transmission_position	= ssi[1];
		out.spring_deflection		= ssi[3];

		out.rotor_position			= TC_ENC.CNT;
		
		duty_g = ssi[1]>>9;
		duty_b = ssi[3]>>9;
		duty_r = (TC_ENC.CNT>>2)>>6;

		// update quadrature velocity
//		if ((TCF1.INTFLAGS & TC1_CCAIF_bm) !=  0) {
//			calcFreq	= (uint16_t)((F_CPU / CLOCK_DIV) / ((float)(GetCaptureValue(TCF1) & 0xFFFC) * LINECOUNT ));
//			calcRPM		= calcFreq*60;
//		}


		// load channel 2 with the encoder value
//		txpdo.ch2 = ssi[0];
		

		////////////////////////////////////////////////////////////////////////
		// update timestamp
		out.timestep = TC_STEP.CNT;


		////////////////////////////////////////////////////////////////////////
		if (global_flags.status != 0) {													// if a limit was hit drop the eCAT down to preop and set the led to red
			
			switch(global_flags.status) {
				case STATUS_LIMITSW:
					printf("Limit SW: %X\n",global_flags.limits);
					break;
					
				case STATUS_TCOVF:
					printf("Timer Overflow\n");
					break;
									
				case STATUS_BADPWM:
					printf("Bad PWM\n");
					break;
					
				case STATUS_PANIC:
					printf("PANIC!!!\n");
					break;

				default :
					break;
			}

			out.status_byte = 0x10;
//			duty_r	= 1;
//			duty_g	= 16;
//			duty_b	= 16;
//			al_status = AL_STATUS_PREOP;
//			writeAddr(AL_STATUS, &al_status, 1);
			global_flags.status = 0;
		}

		////////////////////////////////////////////////////////////////////////
		// Stuff to do when eCAT is in OP mode
		if (al_status == AL_STATUS_OP) {										// if we're in OP mode, update the txpdo on the ET1100
			al_event |= writeAddr(SM3_addr, &out  , SM3_len);
			
			// update the PWM from eCat
//			setPWM(in.motor_torque);
			
			// Manage LED pwm stuff
			if (loopCnt == 16) {															// reset the all colors to on
				PORT_LED.OUTSET = LED_R_bm | LED_G_bm | LED_B_bm;
				loopCnt = 0;
			}
			else {																		// turn off each led when loopCnt == duty
	
				if (loopCnt == duty_r) {
					PORT_LED.OUTCLR	= LED_R_bm;
				}
				if (loopCnt == duty_g) {
					PORT_LED.OUTCLR	= LED_G_bm;
				}
				if (loopCnt == duty_b) {
					PORT_LED.OUTCLR	= LED_B_bm;
				}
		
			}

			loopCnt++;
		}
		////////////////////////////////////////////////////////////////////////
		// Stuff to do when eCAT is NOT in OP mode
		else {																	// else keep updating al_event
			al_event |= eCATnop();
			PORT_LED.OUTSET = LED_G_bm | LED_B_bm;
			PORT_LED.OUTCLR	= LED_R_bm;
			
			pwm = 10;
			initPWM(pwm);
		}



		////////////////////////////////////////////////////////////////////////
		// Manage EtherCAT stuf/

		// play with toggling the pin on port H to get an idea of timings
//		if (tmp != rxpdo.ch1) {
//			PORTH.OUTTGL = (1<<7);
//			tmp = rxpdo.ch1;
//		}
//		else
//			PORTE.OUT &= ~(1<<TOGGLE);
		
		
		
		if (al_event & 0x01) {													// if al_control needs checking...
			printf("AL_Event Request:	0x%2X\n",al_event);

			al_event = readAddr(AL_CONTROL, &al_status, 1);
			writeAddr(AL_STATUS, &al_status, 1);

			printf("\t");
			printState(al_status);
			printf("\n");
			
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
					al_event = readAddr(SM2_addr, &in, SM2_len);
					
//					PORTH.OUT ^= (1<<7);
				}
				
			}
			else if (al_event & 0x0800) {										// SM3...
				al_event |= readAddr(SM3_BASE+SM_STATUS, data, 1);
			
			}
			else {
				//ignore
			}
		
		}



	
	} // end while
} // end main

////////////////////////////////////////////////////////////////////////////////
// Helper functions...


// print the options available to the user
void printMenu () {
	printf("\n");
	printf("Menu Options:\n");
	printf("~: Software Reset\n");
	printf("s: Read SSIs\n");
	printf("q: Read Quadrature\n");
	printf("v: Read Voltages\n");
	printf("t: Read Tempratures\n");
	printf("<: Decrease PWM\n");
	printf(">: Increase PWM\n");
	printf("m: Toggle Direction\n");
	printf("b: Bother amp\n");
	printf("p: Read amp PWM\n");
	printf("e: Error counter\n");
	printf("a: Read eCAT AL_STATUS\n");
	printf("d: Read eCAT DL_STATUS\n");
	printf("\n");
	printf("0: SyncManager0 info\n");
	printf("1: SyncManager1 info\n");
	printf("2: SyncManager2 info\n");
	printf("3: SyncManager3 info\n");
	printf("\n");
	printf("w: Write SSI0 to SM3\n");
	printf("R: Read from SM0\n");
//	printf("r: Read from SM2\n");
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

