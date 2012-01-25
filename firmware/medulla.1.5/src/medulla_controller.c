// Kit Morton
//
//	medulla_controller.c
//	This library manages the medulla state machine and controls the E-STOP
//	and Ethercat hardware.
////////////////////////////////////////////////////////////////////////////////

#include "medulla_controller.h"
#define RS232	USARTE0

// Ethercat variables
uint8_t			data[512];
uint8_t			al_status;
uint16_t		al_event;
uint16_t		SM0_addr, SM1_addr, SM2_addr, SM3_addr, SM2_len, SM3_len;


static int	uart_putchar(char c, FILE *stream);
static FILE mystdout = FDEV_SETUP_STREAM (uart_putchar, NULL, _FDEV_SETUP_WRITE);
uint8_t			rs_data;
int8_t			eStop_count;

// Interrupt handeler for the step timer overflow.
ISR(TCD1_OVF_vect) {
	cli();
	assertEStop();
	timerOverflow();
	resetTimer(WATCHDOG_TIMER);
	sei();
}

/** This is the main event loop for the medulla program
  * It initilizes the hardware and then starts the internal
  * state machine. It assumes that the needed functions
  * init(), state_idle() state_init(), state_run(), state_stop()
  * state_error_damping(), and state_error() have are implemented
  * somewhere. INPUT_STRUCT and OUPUT_STRUCT must also be defined
  * with the name of the input and output structs.
  */
  
void medulla_run(void* in, void* out) {
	MedullaState currentState;
	
	//******** Init ********
	CLK.PSCTRL = 0x00;															// no division on peripheral clocks
	Config32MHzClock();

	stdout = &mystdout;															// attach the uart to the stdout for printf
	initUART(&PORTE,&USARTE0,1152);												// RS232 uart 115.2kb
	
	initEStop(&PORTF,0,1);
	clearEStop();
	init_eCAT();
	PMIC.CTRL |= PMIC_HILVLEN_bm;
	init();
	eStop_count = 0;
	
	
	currentState = IDLE;
	
	//******** Loop ********
	timer_Start(WATCHDOG_TIMER,TC_CLKSEL_DIV64_gc);
	TCD1.INTCTRLA =TC_OVFINTLVL_HI_gc;
	timer_Start(STEP_TIMER,TC_CLKSEL_DIV4_gc);
	sei();	// Enable interrupt
	while(1) {
		updateInput();
		
		// If the current state is init, then we want to stay in init until the estop has been deasserted
		if (!(currentState == INIT && checkEStop() == 1))
			setState(currentState);
		eCATWriteData(out);
		eCATReadData(in);
		
		// **** State Machine ****
		switch (currentState) {
			case IDLE:
				currentState = state_idle();
				// If we aren't going into and an error state and we got an INIT command,
				// change states to INIT.
				if ((currentState != ERROR) && (currentState != ERROR_DAMPING) && (getState() == INIT))
					currentState = INIT;
				break;
				
			case INIT:
				clearEStop();
				currentState = state_init();
				// If there isn't an error
				if ((currentState != ERROR) && (currentState != ERROR_DAMPING)) {
					if ((getState() == RUN) && (checkEStop() == 0))	// If we receive a run command then we go to state RUN
						currentState = RUN;
					else if (getState() != INIT)	// If we get a new state that is not RUN then we go to STOP
						currentState = STOP;
				}
				break;
				
			case RUN:
				currentState = state_run();
				if ((currentState != ERROR) && (currentState != ERROR_DAMPING)) {
					if (getState() == STOP)	// If we receive a stop command then we go to state STOP
						currentState = STOP;
					else if (getState() != RUN)	// If we get a new state that is not STOP then we go to STOP
						currentState = STOP;
				}
				break;
				
			case STOP:
				state_stop();
				currentState = IDLE;
				break;
				
			case ERROR_DAMPING:
				assertEStop();
				currentState = state_error_damping();
				break;
			case ERROR:
				assertEStop();
				currentState = state_error();
				
				// If we are commanded to go to STOP or INIT states
				// then we can go there
				if ((getState() == STOP) | (getState() == INIT))
					currentState = getState();
				break;
			default:
				break;
		}
		
		if (currentState != INIT) { // If we are in INIT, then we don't want to go to error on assertion of EStop
			if (checkEStop() == 1)
				eStop_count += 1;
			else if ((checkEStop() == 0) && (eStop_count > 0))
				eStop_count -= 1;

			if (eStop_count > 100) {
				if (currentState != ERROR)
				currentState = ERROR;
				eStop = 1;
			}
		}
		
	}
}

void init_eCAT() {
	// Init eCAT variables
	al_status	= 0;
	al_event	= 0;
	SM0_addr	= 0;
	SM1_addr	= 0;
	SM2_addr	= 0;
	SM3_addr	= 0;
	SM2_len		= 0;
	SM3_len		= 0;
	
	initeCAT();						// Start the SPI driver
	while(!(PORT_ECAT.IN) & (1<<ECAT_EEPROM_LOADED));							// The EEPROM_LOADED signal indicates that the SPI interface is operational.
	
	al_event |= readAddr(AL_STATUS, &al_status, 1);								// check to see what the et1100 is doing so we're on the same page
	
	if (al_status == AL_STATUS_OP_bm) {											// if the eCAT is in OP, grab and store the SM addresses:


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

void eCATWriteData(void* out) {
	if (al_status == AL_STATUS_OP_bm) {										// if we're in OP mode, update the txpdo on the ET1100
		al_event |= writeAddr(SM3_addr, out, SM3_len);
	}
	else {																	// else keep updating al_event
		al_event |= eCATnop();
	}
}

void eCATReadData(void* in) {
	if (al_event & 0x01) {													// if al_control needs checking...
			al_event = readAddr(AL_CONTROL, &al_status, 1);
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
		
		if (al_event & 0xFF00) {												// if there's a SM interrupt pending...

			if (al_event & 0x0400) {											// SM2...
				al_event |= readAddr(SM2_BASE+SM_STATUS, data, 1);
				
				if (*data & 0x01) {												// stuff wrtten to the buffer!
					al_event = readAddr(SM2_addr, in, SM2_len);
				}
				
			}
			else if (al_event & 0x0800) {										// SM3...
				al_event |= readAddr(SM3_BASE+SM_STATUS, data, 1);
			
			}
			else {
				//ignore
			}
		
		}
}

static int uart_putchar (char c, FILE *stream) {

    if (c == '\n')
        uart_putchar('\r', stream);
 
    // Wait for the transmit buffer to be empty
    while ( !( USARTE0.STATUS & USART_DREIF_bm) );
 
    // Put our character into the transmit buffer
    USARTE0.DATA = c; 
 
    return 0;
}
