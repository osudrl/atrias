// Kevin Kemper
//
//	atrias_leg.c
//	This program reads in the encoders and limit switches related to one leg motor
//		on the ATRIAS.1 robot platform.
////////////////////////////////////////////////////////////////////////////////

// if DEBUG is defined, the device will print info out to the RS232
#define DEBUG

#define ENABLE_LIMITS
#define ENABLE_TC_STEP
#define ENABLE_ENCODERS

// encoders and limit switches must be enabled before using the motors
#if defined (ENABLE_LIMITS) && defined (ENABLE_ENCODERS)
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
#include "../libs/ssi_spi.h"

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


#define ID	MEDULLA_B_ID

// state machine states
#define STATE_READY		0
#define STATE_START		1
#define STATE_RUN		2
#define STATE_ERROR		4
#define STATE_RESET		5
#define STATE_DANGER	6


#define MAX_13BIT			8192
#define ROLLOVER_THRESHOLD	4096

#define ENC_MID		((ENC_TOP+1)/2)												// ENC_TOP is defined in the menial.h
#define PWM_LIMIT	9000														// The most we can deviate from the zero torque pwm


#define KD_STIFF		1000


#if ID == MEDULLA_A_ID
	#define LEG_MAX			MAX_TRAN_SEG_A_COUNT
	#define LEG_MIN			MIN_TRAN_SEG_A_COUNT
	#define TRAN_MAX		MAX_TRAN_A_COUNT
	#define TRAN_MIN		MIN_TRAN_A_COUNT
#elif ID == MEDULLA_B_ID
	#define LEG_MAX			MAX_TRAN_SEG_B_COUNT
	#define LEG_MIN			MIN_TRAN_SEG_B_COUNT
	#define TRAN_MAX		MAX_TRAN_B_COUNT
	#define TRAN_MIN		MIN_TRAN_B_COUNT
#else
	#error	"Define an ID for the device."
#endif

#define LEG_MIN16	((uint16_t)(LEG_MIN>>16))

// These define the "soft" limits where the uController will take over.
//  PERCENT_UPPER is percentag of the total range to use for the upper danger zone.
#define PERCENT_UPPER	0.20
#define PERCENT_LOWER	0.20
#define DANGER_UPPER	((uint16_t)(TRAN_MAX - (TRAN_MAX-TRAN_MIN)*PERCENT_UPPER))
#define DANGER_LOWER	((uint16_t)(TRAN_MIN + (TRAN_MAX-TRAN_MIN)*PERCENT_LOWER))


// map the 0th usart on port E to the RS232 keyword for readability
#ifdef DEBUG
#define RS232	USARTE0
#endif


// Interrupt handeler for the timer overflow of the velocity timer.
ISR(TC_VEL_OVF_vect) {

	PWMdis();
	global_flags.status	|= STATUS_TCOVF;
	#ifdef DEBUG
	printf("vel ovf\n");
	#endif
}


// local functions...
void tc_VelStop();
void tc_VelStart();
void initVelTimer();

#ifdef DEBUG
void		printMenu();
void		printState(uint8_t state);
static int	uart_putchar(char c, FILE *stream);
#endif

// global vars...
#ifdef DEBUG
static FILE mystdout = FDEV_SETUP_STREAM (uart_putchar, NULL, _FDEV_SETUP_WRITE);
#endif


////////////////////////////////////////////////////////////////////////////////

int main(void) {

	uint8_t		i;
	uint8_t		tmp8;
	uint16_t	tmp16;

	uint8_t		panic_cnt	= 0;
	uint8_t		enc_cnt		= 0;

	uint16_t	tmp_cnt16 	= 0;
	
	uint16_t	start_ssi	= 0;
	uint16_t	start_off	= 0;
	uint32_t	start_biss	= 0;

	int16_t		kD			= 0;

	uint16_t	tran_pos	= 0;												// is the absolute transmission position
	uint16_t	last_tran	= 0;												// the last measured transmission encoder value	
	uint16_t	tran_off	= 0;												// keeps track of the rollovers of the transmission encoder


	float		torque		= 0;
	float		vel			= 0;


	uControllerInput	in;
	uControllerOutput	out;
	
	uint16_t		pwm = 0;
	uint16_t		ssi[4];
	uint8_t			biss[4];
	uint8_t			status;
	uint8_t			toggle = CMD_RUN_TOGGLE_bm;
	
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
	CLK.PSCTRL = 0x00;															// no division on peripheral clocks
	Config32MHzClock();
//	Config32KHzRTC();
	
	cli();																		// Disable interrupts

	_delay_ms(1000);
	
	#ifdef DEBUG
	stdout = &mystdout;															// attach the uart to the stdout for printf
	initUART(&PORTE,&RS232,1152);												// RS232 uart 115.2kb
	#endif


	initLED();
	PMIC.CTRL |= PMIC_MEDLVLEN_bm;// | PMIC_RREN_bm;
	sei();
	
	led_solid_red();
	
	pwm = 0;

//	PORT_AMP_CTL.DIRSET = DIRECTION_bm;
	#ifdef ENABLE_PWM
	initPWM(pwm);
	#endif

	initTimer();
	
	initVelTimer();
	
	initPanic();
	
	#ifdef ENABLE_LIMITS
	initLimitSW();
	#endif

	initeCAT();
	
	initSSI_spi();
	
	initBiSS_spi();
	
	initQuad();

	initADC();
	
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
	global_flags.state		= STATE_ERROR;
	global_flags.error_cnt	= 0;
	
	_delay_ms(100);

	////////////////////////////////////////////////////////////////////////////
	// init eCat stuff

	#ifdef DEBUG
	printf("\t\tmedulla.1: ATRIAS leg controller, ID %.2X : %s\n\n",ID,__DATE__);	// print what program we're running...
	#endif
	

	while(ECAT_EEPROM_LOADED);													// The EEPROM_LOADED signal indicates that the SPI interface is operational.
	
	
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
	
	
	// Enable the interrupts
	PMIC.CTRL |= PMIC_HILVLEN_bm;
//	PMIC.CTRL |= PMIC_MEDLVLEN_bm;
	PMIC.CTRL |= PMIC_LOLVLEN_bm;

	printf("\t\t\t\tLimits: 0x%.2X\n",global_flags.limits);


	limitDis();
	
	////////////////////////////////////////////////////////////////////////////
	// Set up the transmission tracker
	tmp8 = readSSI_spi(ssi);
	while ((tmp8 == 0xFF) || (ssi[1] > 8191) || (ssi[1] == 0)) {
		tmp8 = readSSI_spi(ssi);
	}
	out.TRANS_ANGLE = ssi[1];

	tmp8 = readBiSS_spi(biss, &status);
	// Check if the BiSS read was valid
	while ((tmp8 == 0xFF) || (biss[3] > 0x22) || (biss[3] < 0x7) || ((status & BISS_ERROR_bm) == 0)) {
		tmp8 = readBiSS_spi(biss, &status);
	}
	out.LEG_SEG_ANGLE	= *((uint32_t*)biss);
	

	tran_off = (uint16_t)((out.LEG_SEG_ANGLE-LEG_MIN)*(((float)(TRAN_MAX-TRAN_MIN))/((float)(LEG_MAX-LEG_MIN)))+TRAN_MIN);

	tran_off = (tran_off / MAX_13BIT) * MAX_13BIT;

	#ifdef DEBUG
	printf("%u	%u	%u\n", out.TRANS_ANGLE, tran_off, out.TRANS_ANGLE+tran_off);
	start_off = tran_off;
	start_ssi = ssi[1];
	start_biss = out.LEG_SEG_ANGLE;
	#endif

	// Keep track of the last counts for sensors that could rollover.
	last_tran = out.TRANS_ANGLE;
	

	

	while(1) {

//XXX: this is a hack but should be ok now that we have a step timer...
		PORTH.OUTTGL = (1<<7);
		_delay_us(8);


		#if 1
		if (RS232.STATUS&USART_RXCIF_bm) {										// do we have something from the RS232?
			rs_data = RS232.DATA;

			switch (rs_data) {
		
				case '~':
					global_flags.state = STATE_RESET;
					break;
		
				case 'v':
					printf("\t\tmedulla.1: ATRIAS leg controller, ID %.2X : %s\n\n",ID,__DATE__);
					break;
		
				case 'o':
					printf("Out:\n");
					printf("out.status:			0x%.2X\n", out.status);
					printf("out.timestep:			%u\n", out.timestep);
					printf("out.TRANS_ANGLE:		%u\n", out.TRANS_ANGLE);
					printf("out.LEG_SEG_ANGLE:		%lu\n", out.LEG_SEG_ANGLE);
					printf("out.ROTOR_ANGLE:		%u\n", out.ROTOR_ANGLE);
					printf("out.THERM1:		%u\n", out.therm1);
					break;


				case 'i':
					printf("In size: %u\n",sizeof(in));
					printf("in.command:	%u\n", in.command);
					printf("in.motor:	%u\n", in.motor);
					break;

				case '<':
					kD-=10;
					break;

				case '>':
					kD+=10;
					break;

				case 'm':
					printf("D %u,	v %u,	t %u\n", kD, (uint16_t)vel, (uint16_t)torque);
					printf("%u	%u	%u\n", DANGER_UPPER, tran_pos, DANGER_LOWER);
					printf("PWM_OPEN %u\n",PWM_OPEN);
					break;
					
				case 'n':
					printf("o %u,	s %u,	b %lu\n", start_off, start_ssi, start_biss);
					break;
					
				case 'p':
					printf("panic in: %u\n", (PORT_PANIC.IN & PANIC_SENSE_bm));
					break;
					
				case 'l':
					printf("lim: 0x%.2X\n", (uint8_t)(~PORT_LIMIT.IN) );
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
		
		#ifdef ENABLE_ENCODERS

		tmp8 = readSSI_spi(ssi);
		if (tmp8 == 0xFF) {
//			enc_cnt+=2;
		}
		else if (ssi[1] > 8191) {												// The ssi value is invalid (>8191)
			enc_cnt+=10;
		}
		else {
			out.TRANS_ANGLE = ssi[1];
		}
		
		
		
		// Transmission A rollovers.
		if ( (out.TRANS_ANGLE > last_tran) && (out.TRANS_ANGLE - last_tran > ROLLOVER_THRESHOLD) ) {
			tran_off -= MAX_13BIT;
		}
		else if ( (last_tran > out.TRANS_ANGLE) && (last_tran - out.TRANS_ANGLE > ROLLOVER_THRESHOLD) ) {
			tran_off += MAX_13BIT;
		}

		// Keep track of the last counts for sensors that could rollover.
		last_tran = out.TRANS_ANGLE;
		
		tran_pos = out.TRANS_ANGLE+tran_off;
		
		
		tmp8 = readBiSS_spi(biss, &status);
		
		// Check if the BiSS read was valid
		if (tmp8 == 0x00) {
			if ( biss[3] > 0x22 ) {												// the number is too big
				enc_cnt+=10;
			}
			else if ( biss[3] < 0x7 ) {											// the number is too small
				enc_cnt+=10;
			}
			else if ((status & BISS_ERROR_bm) == 0) {							// freak out if the BiSS data is bad
				enc_cnt+=10;
			}
			else {
				out.LEG_SEG_ANGLE	= *((uint32_t*)biss);
			}
		}
		else if (tmp8 == 0xFF) {
//			enc_cnt+=4;
		}
		else {
			printf("\t\t\t\tB\n");
			enc_cnt+=10;
		}

		#endif

		out.ROTOR_ANGLE			= TC_ENC.CNT;
		
		
		if (ADC_LOGIC_FLAG) {
			out.therm1 = ADC_LOGIC_VAL;
			ADC_LOGIC_FLAG = 1;
			ADC_LOGIC_START;
		}

		// update timestamp
		out.timestep			= TC_STEP.CNT;


		////////////////////////////////////////////////////////////////////////
		switch (global_flags.state) {

////////////////////////////////////////////////////////////////////////////////
			case STATE_READY :

				// Signal the all clear
				PanicClr();

				_delay_us(100);

				led_solid_red();
				
				if ((al_status != AL_STATUS_OP_bm)) {
					global_flags.state = STATE_ERROR;	
				}

				// Wait for the other medullas to be ready.
				//	panic_cnt is inc. when the panic line is ok. This loop will only exit if
				//	the panic line is ok for some consectutive checks.
				else if(panic_cnt < 100) {

					if ((PORT_PANIC.IN & PANIC_SENSE_bm) != 0) {				// if the panic is ok, inc. cnt
						panic_cnt +=10;
					}
					else {
						if (panic_cnt > 0)
							panic_cnt --;										// when it's ok, dec. error_cnt
					}

				}
				else {
				
					global_flags.state = STATE_START;
					
					// reset stuff
					global_flags.status		= 0;
					global_flags.error_cnt	= 0;
					panic_cnt	= 0;
					enc_cnt		= 0;
					
					#ifdef ENABLE_PWM
					limitDis();
					tmp16 = tran_pos;
					setPWM(PWM_OPEN);											// apply a small torque
					PWMen();
					tmp_cnt16 = 0;
					printf("PWM_OPEN %u\n",PWM_OPEN);
					#endif

					#ifdef DEBUG
					printf("START\n");
					#endif
				}
					
				break;
				
////////////////////////////////////////////////////////////////////////////////
			case STATE_START :
			
				led_solid_orange();
				
				
				// check on the panic line
				if ((PORT_PANIC.IN & PANIC_SENSE_bm) == 0) {
					panic_cnt +=2;
				}

				// Check if the error counter is too big. If it is, set the panic line and GTFO
				if ( panic_cnt > 100 ) {
					PanicSet();
					global_flags.state	= STATE_ERROR;
					out.status		|= STATUS_DANGER;
					#ifdef DEBUG
					#ifdef ENABLE_LIMITS
					printf("PANIC\n");
					#endif
					#endif
					tmp_cnt16 = 0;
				}
				// else decay the counter
				else if (panic_cnt > 0) {
					panic_cnt--;
				}
				
				
				
				// Give the motors time to open up a bit and check to see if the
				//	transmission is good and that we're not on a hardstop.
				#ifdef ENABLE_PWM
				if ((al_status != AL_STATUS_OP_bm))
					global_flags.state = STATE_ERROR;
				else if (tmp_cnt16 > 30000) {
					#ifdef ENABLE_LIMITS
					limitEn();
					#endif
					// The encoder should get smaller after the motor moves with positve torque.
					if ((global_flags.limits != 0) || (tran_pos>=tmp16)) {		// If we hit a limit switch or the encoder didn't move right...
						PWMdis();
						limitDis();
						PanicSet();
						printf("\t\t\t\tMOTOR MOVED BAD!!!  %u\n",tmp16);
						printf("\t\t\t\tLimits: 0x%.2X\n",global_flags.limits);
	
						global_flags.state = STATE_ERROR;

						out.status	=	STATUS_BADMOTOR;
						tmp_cnt16 = 0;
					}
					else {
						printf("Motor good  %u\n",tmp16);

						global_flags.status		= 0;
						out.status				= 0;
						
						panic_cnt				= 0;
			
						// Enable the interrupts
//						PMIC.CTRL |= PMIC_HILVLEN_bm;
//						PMIC.CTRL |= PMIC_MEDLVLEN_bm;
//						PMIC.CTRL |= PMIC_LOLVLEN_bm;

						global_flags.state = STATE_RUN;
						tmp_cnt16 = 0;
			
						TC_ENC.CNT = ENC_MID;
						TC_VEL.CNT = 0;
						
						#ifdef DEBUG
						printf("RUN\n");
						#endif
					}
				}
				else{
					tmp_cnt16++;
				}
				#else
				global_flags.status		= 0;
				out.status				= 0;
						
				panic_cnt				= 0;
				global_flags.state = STATE_RUN;
				tmp_cnt16 = 0;
				
				TC_ENC.CNT = ENC_MID;
				TC_VEL.CNT = 0;
				
				#ifdef DEBUG
				printf("RUN\n");
				#endif
				
				#endif
				
				
				
				break;
			
////////////////////////////////////////////////////////////////////////////////
			case STATE_RUN :
				

				#ifdef ENABLE_ENCODERS
				// check to see if we're in the dange zone
				if ( tran_pos > DANGER_UPPER ) {
					global_flags.state = STATE_DANGER;
					out.status		|= STATUS_DANGER;
					TC_ENC.CNT = ENC_MID;
					TC_VEL.CNT = 0;
					tc_VelStart();
					vel = 0;
					torque = 0;
					tmp_cnt16 = 0;
					#ifdef DEBUG
					#ifdef ENABLE_LIMITS
					printf("DANGER\n");
					#endif
					#endif
				
				}
				else if ( tran_pos < DANGER_LOWER ) {
					global_flags.state = STATE_DANGER;
					out.status		|= STATUS_DANGER;
					TC_ENC.CNT = ENC_MID;
					TC_VEL.CNT = 0;
					tc_VelStart();
					vel = 0;
					torque = 0;
					tmp_cnt16 = 0;
					#ifdef DEBUG
					#ifdef ENABLE_LIMITS
					printf("DANGER\n");
					#endif
					#endif

				}
				else if ((al_status == AL_STATUS_OP_bm)) {								// Stuff to do when eCAT is in OP mode
				#else
				if ((al_status == AL_STATUS_OP_bm)) {
				#endif
				
					// Deal with commands from the master
					if ( ( in.command & (~CMD_RUN_TOGGLE_bm)) == CMD_RUN ) {
					
						//XXX
						#if 0
						while(1) {
							tc_Stop();
							PORT_SSI1.OUTTGL = SSI1_CLK;
						}
						#endif
					
						#ifdef ENABLE_TC_STEP
						tc_Start();
						#endif
						
						out.status		&= ~STATUS_DISABLED;
						
						pwm = in.motor;

						#ifdef ENABLE_PWM
						setPWM(pwm);
						#endif
						
						// Play with the LED colors
						tmp16 = (((uint16_t*)&out.LEG_SEG_ANGLE)[1]);
//						printf("%u\n",tmp16);
						tmp16 = tmp16-LEG_MIN16;
						tmp16 = tmp16*5 + TRAN_MIN;	// translate the leg angle into the same space as the transmission position
//						tmp16 = ((int16_t)(tran_pos - tmp16)<0)?(tmp16-tran_pos):(tran_pos-tmp16);
//						tmp16 = 0x0000;
//						printf("\t%u\n",tmp16);
						solidLED();
						SetDutyR( tmp16+0x000F );	// more red as the spring compresses
						SetDutyG( 0x0FFF );
						SetDutyB( 0x07FF );
						
						
					}
					else if (in.command == CMD_DISABLE) {
						
						out.status |= STATUS_DISABLED;
						
						#ifdef ENABLE_PWM
						pwm = PWM_OPEN;
						setPWM(pwm);
						#endif
						
						led_solid_white();

					}
					else {														//XXX if we get any other cmd then we should report bad_cmd and move to error?
//						global_flags.status |= STATUS_BADCMD;
//						out.status |= STATUS_BADCMD;
					}

					
				}
				else {															// Stuff to do when eCAT is NOT in OP mode
					
					tc_Stop();
					
					#ifdef ENABLE_PWM
					pwm = PWM_OPEN;
					setPWM(pwm);
					#endif
					
					led_solid_purple();
					
				}


				// check on the panic line
				if ((PORT_PANIC.IN & PANIC_SENSE_bm) == 0) {
					panic_cnt +=2;
				}

				// Check if the error counter is too big. If it is, set the panic line and GTFO
				if ( panic_cnt > 100 ) {
					PanicSet();
					global_flags.state	= STATE_DANGER;
					out.status		|= STATUS_DANGER;
					#ifdef DEBUG
					#ifdef ENABLE_LIMITS
					printf("DANGER\n");
					#endif
					#endif
					
					TC_ENC.CNT = ENC_MID;
					TC_VEL.CNT = 0;
					tc_VelStart();
					vel		= 0;
					torque	= 0;
					tmp_cnt16 = 0;
				}
				// else decay the counter
				else if (panic_cnt > 0) {
					panic_cnt--;
				}



				// Check if the error counter is too big, if it is set the panic line and GTFO
				if ( enc_cnt > 100 ) {
					#ifdef ENABLE_PWM
					PWMdis();
					#endif
					PanicSet();
					out.status |= STATUS_ENC;
					global_flags.state = STATE_ERROR;
					
				}

				if (enc_cnt > 0)									// decay the error counter
					enc_cnt--;
				
				break;
				
////////////////////////////////////////////////////////////////////////////////
// XXX: The following lines must happen before changing to the danger state:
//		TC_ENC.CNT = ENC_MID;
//		TC_VEL.CNT = 0;
//		tc_VelStart();
//		vel = 0;
//		torque = 0;
//		tmp_cnt16 = 0;
			case STATE_DANGER :
				
				PanicSet();
				
				#ifdef ENABLE_TC_STEP
				tc_Stop();
				#endif
					
				kD = KD_STIFF;
					
				// dampy control time
				if (TC_VEL.CNT != 0) {
					vel	= ((float)(TC_ENC.CNT-(float)ENC_MID))/((float)(TC_VEL.CNT));
				}
				else {
					vel = 0.0;
				}
				
				if (vel == 0) {
					tmp_cnt16++;
				}
				else {
					tmp_cnt16 = 0;
				}
				
				// if the system has settled down move to error state
				if (tmp_cnt16 > 500) {
					global_flags.state = STATE_ERROR;
					tmp_cnt16 = 0;
				}
				
				torque = ((float)kD)*vel;

				if (torque > MTR_MAX_TRQ) {
					torque = MTR_MAX_TRQ;
				}
				else if (torque < MTR_MIN_TRQ) {
					torque = MTR_MIN_TRQ;
				}

				#ifdef ENABLE_PWM
				pwm = (uint16_t)(torque*(((float)(MTR_MAX_CNT-MTR_MIN_CNT))/((float)(MTR_MAX_TRQ-MTR_MIN_TRQ)))+PWM_ZERO);

//					printf("%u\n",pwm);

				if (pwm > MTR_MAX_CNT) {
					pwm = MTR_MAX_CNT;
				}
				else if (pwm < MTR_MIN_CNT) {
					pwm = MTR_MIN_CNT;
				}
				
				
				if (pwm > PWM_ZERO+PWM_LIMIT) {
					pwm = PWM_ZERO+PWM_LIMIT;
				}
				else if (pwm < PWM_ZERO-PWM_LIMIT) {
					pwm = PWM_ZERO-PWM_LIMIT;
				}
				#endif
				

				TC_VEL.CNT = 0;
				TC_ENC.CNT = ENC_MID;

				
				// Play with the LED colors
				SetDutyR( out.ROTOR_ANGLE + 0x01FF);
				SetDutyG( PWM );
				SetDutyB( 0x000F );


				#ifdef ENABLE_PWM
				setPWM(pwm);
				#endif
					
					

				// Check if the encoder error counter is too big, if it is set the panic line and GTFO
				if ( enc_cnt > 100 ) {
					#ifdef ENABLE_PWM
					PWMdis();
					#endif
					PanicSet();
					out.status |= STATUS_ENC;
					global_flags.state = STATE_ERROR;
					
				}

				if (enc_cnt > 0)									// decay the error counter
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

//////////////////////////////////
			case STATE_ERROR :
			default :

				#ifdef ENABLE_PWM
				PWMdis();
				#endif
				
				limitDis();
				PanicSet();
				tc_Stop();
				tc_VelStop();

				// set the color based on the type of problem
				if ( out.status == STATUS_BADMOTOR )
					led_blink_dark_purple();
				else if ( out.status == STATUS_LIMITSW)
					led_blink_orange();
				else if ( out.status == STATUS_ENC )
					led_blink_yellow();
				else if ( out.status == STATUS_TCOVF )
					led_blink_blue();
				else
					led_blink_red();

				if ((al_status == AL_STATUS_OP_bm)) {								// Stuff to do when eCAT is in OP mode
					
					// Deal with commands from the master
					if (in.command == CMD_RESTART) {
						global_flags.state	= STATE_READY;
						panic_cnt = 0;
						
						#ifdef DEBUG
						printf("READY\n");
						#endif
					}
				}

				break;
				
		}



		////////////////////////////////////////////////////////////////////////
		// Manage any status changes caused by interrupts
		if (global_flags.status != 0) {
			
			if (global_flags.status & STATUS_LIMITSW) {
				printf("Limit SW: %X\n",global_flags.limits);
				
				// if the spring limits caused the event, move to danger state
				if ((global_flags.limits == SPRING_LIM0_bm) || (global_flags.limits == SPRING_LIM1_bm)) {
					global_flags.state		= STATE_DANGER;
					TC_ENC.CNT = ENC_MID;
					TC_VEL.CNT = 0;
					tc_VelStart();
					vel = 0;
					torque = 0;
					tmp_cnt16 = 0;
				}
				else {
					global_flags.state		= STATE_ERROR;
				}
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

		if (al_status == AL_STATUS_OP_bm) {										// if we're in OP mode, update the txpdo on the ET1100
			al_event |= writeAddr(SM3_addr, (uint8_t*)(&out)  , SM3_len);
		}
		else {																	// else keep updating al_event
			al_event |= eCATnop();
		}
		
		if (al_event & 0x01) {													// if al_control needs checking...
			#if 0
			printf("AL_Event Request:	0x%2X\n",al_event);
			#endif

			al_event = readAddr(AL_CONTROL, &al_status, 1);
			writeAddr(AL_STATUS, &al_status, 1);

			#if 0
			printf("\t");
			printState(al_status);
			printf("\n");
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
					al_event = readAddr(SM2_addr, (uint8_t*)(&in), SM2_len);


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
					
						#ifdef ENABLE_TC_STEP
						tc_Stop();
						#endif
						
						// tc_VelStop();
						
//						out.status		|= STATUS_DISABLED;
						toggle			= CMD_RUN_TOGGLE_bm;


					}
					else if (( in.command & (~CMD_RUN_TOGGLE_bm)) == CMD_RUN) {	// if we have a run command
						
//						if (TC_STEP.CNT != 0) {
//							if ((TC_STEP.CNT < 2000) || (TC_STEP.CNT > 6000))			
//								printf("%.2X\t%u\n",in.command,TC_STEP.CNT);
//						}
						
						// Check the toggle bit in the command
						if (toggle != ( in.command & CMD_RUN_TOGGLE_bm )) {		// toggle bit is good
						
							toggle = in.command & CMD_RUN_TOGGLE_bm;

							// reset the time step counter
							#ifdef ENABLE_TC_STEP
							TC_STEP.CNT = 0;
							PORTH.OUTTGL = (1<<2);
							#endif

						}
						else {
							printf("bad\t%.2X\t%u\n",in.command,TC_STEP.CNT);
//							printf("bad TGL\n");
						}
						
					}
					else {
//						#ifdef DEBUG
//						printf("CMD_BAD: 0x%.2X\n", in.command);
//						#endif
						global_flags.status |= STATUS_BADCMD;
						out.status |= STATUS_BADCMD;
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
//		TC_STEP.CNT = 0;
		#endif
	
	} // end while
} // end main

////////////////////////////////////////////////////////////////////////////////
// Helper functions...


// Stops the counter used for damping control
void tc_VelStop() {
	
	TC_VEL.CTRLA = ( TC_VEL.CTRLA & ~TC1_CLKSEL_gm ) | TC_CLKSEL_OFF_gc;
	TC_VEL.CNT = 0;
}

// Starts the counter used for damping control
void tc_VelStart() {
	TC_VEL.CTRLA = ( TC_VEL.CTRLA & ~TC1_CLKSEL_gm ) | TC_CLKSEL_DIV4_gc;	
}

// Sets up the counter used for damping control
void initVelTimer() {
	tc_VelStop();
	
	// Set the overflow interrupt as high level.
	TC_VEL.INTCTRLA = ( TC_VEL.INTCTRLA & ~TC1_OVFINTLVL_gm ) | TC_OVFINTLVL_HI_gc;	
}



#ifdef DEBUG
// print the options available to the user
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

