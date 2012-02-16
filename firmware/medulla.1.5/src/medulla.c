// Kit Morton
//
//	atrias_leg.c
//	This program reads sensors and controls a motor for half of an ATRIAS 2.0
//	leg.
////////////////////////////////////////////////////////////////////////////////

#include "../../../drl-sim/atrias/include/atrias/ucontroller.h"
#include <util/delay.h>

#include "medulla_controller.h"
#include "medulla_leg.h"
#include "medulla_hip.h"
#include "medulla_boom.h"

#define ENABLE_DEBUG
//#define ENABLE_MOTOR_PASSTHROUGH

#define MEDULLA_BOOM
#define MEDULLA_HIP
#define MEDULLA_LEG

uControllerInput in;
uControllerOutput out;

void (*initilize_pntr)(void);
void (*updateInput_pntr)(uControllerInput *in, uControllerOutput *out);
void (*updateOutput_pntr)(uControllerInput *in, uControllerOutput *out);
void (*overflow_pntr)(void);

// **** State Machine ****

MedullaState (*idle_pntr)(uControllerInput *in, uControllerOutput *out);
MedullaState (*init_pntr)(uControllerInput *in, uControllerOutput *out);
MedullaState (*run_pntr)(uControllerInput *in, uControllerOutput *out);
void (*stop_pntr)(uControllerInput *in, uControllerOutput *out);
MedullaState (*error_damping_pntr)(uControllerInput *in, uControllerOutput *out);
MedullaState (*error_pntr)(void);


int main(void) {
	#ifdef ENABLE_MOTOR_PASSTHROUGH
	CLK.PSCTRL = 0x00;															// no division on peripheral clocks
	Config32MHzClock();
	initUART(&PORTE,&USARTE0,1152);
	initUART(&PORTF,&USARTF0,1152);
	sei();
	while (1) {
		
		if (USARTE0.STATUS&USART_RXCIF_bm)
			UARTWriteChar(&USARTF0, USARTE0.DATA);
		if (USARTF0.STATUS&USART_RXCIF_bm)
			UARTWriteChar(&USARTE0, USARTF0.DATA);
	}
	#endif
	
	medulla_run((void*)(&in),(void*)(&out));
	return 0;
}

void init(void) {
	//******** Init ********
	// Init DIP switches
	PORTH.PIN4CTRL = PORT_OPC_PULLUP_gc; 
	PORTH.PIN5CTRL = PORT_OPC_PULLUP_gc;
	PORTH.PIN6CTRL = PORT_OPC_PULLUP_gc;
	PORTH.PIN7CTRL = PORT_OPC_PULLUP_gc;
	printf("%X",PORTH.IN>>4);
	// link the function pointers to the functions for the current medulla
	if ((PORTH.IN>>4) == MEDULLA_HIP_ID) {
		// This is a hip medulla
		initilize_pntr = &initilize_hip;
		updateInput_pntr = &updateInput_hip;
		updateOutput_pntr = &updateOutput_hip;
		overflow_pntr = &overflow_hip;
		idle_pntr = &idle_hip;
		init_pntr = &init_hip;
		run_pntr = &run_hip;
		stop_pntr = &stop_hip;
		error_damping_pntr = &error_damping_hip;
		error_pntr = &error_hip;
		
		// set the ID in the ethercat output struct
		out.id = MEDULLA_HIP_ID;
		
		#ifdef ENABLE_DEBUG
		printf("Initilizing Hip\n");
		#endif
	}
	else if ((PORTH.IN>>4) == MEDULLA_BOOM_ID) {
		// This is a boom medulla
		initilize_pntr = &initilize_boom;
		updateInput_pntr = &updateInput_boom;
		updateOutput_pntr = &updateOutput_boom;
		overflow_pntr = &overflow_boom;
		idle_pntr = &idle_boom;
		init_pntr = &init_boom;
		run_pntr = &run_boom;
		stop_pntr = &stop_boom;
		error_damping_pntr = &error_damping_boom;
		error_pntr = &error_boom;
		
		// set the ID in the ethercat output struct
		out.id = MEDULLA_BOOM_ID;
		#ifdef ENABLE_DEBUG
		//printf("Initilizing Boom\n");
		#endif
	}
	else {
		// If it's not a hip or a boom then it must be a leg, we don't really care which one, we will
		// just send the address to the computer and have it figure it out
		initilize_pntr = &initilize_leg;
		updateInput_pntr = &updateInput_leg;
		updateOutput_pntr = &updateOutput_leg;
		overflow_pntr = &overflow_leg;
		idle_pntr = &idle_leg;
		init_pntr = &init_leg;
		run_pntr = &run_leg;
		stop_pntr = &stop_leg;
		error_damping_pntr = &error_damping_leg;
		error_pntr = &error_leg;
		
		// set the ID in the ethercat output struct
		out.id = PORTH.IN>>4;
		
		#ifdef ENABLE_DEBUG
		printf("Initilizing Leg\n");
		#endif
	}
		
	// init the input and output values
	out.state			= 0;
	out.encoder[0]		= 0;
	out.encoder[1]		= 0; 	
	out.encoder[2]		= 0;
	out.timestep		= 0;
	out.error_flags		= 0;
	out.thermistor[0]	= 0;
	out.thermistor[1]	= 0;
	out.thermistor[2]	= 0;
	out.limitSW			= 0;
	out.counter			= 0;
	
	in.enc_max			= 0;
	in.enc_min			= 0;
	in.motor_torque		= 0;
	in.command			= 0;
	
	// Init indicator LEDs
	PORTC.DIRSET = 0b111;
	PORTC.OUTSET = 0b111;
	
	// Initilize the hardware connected to this particular medulla
	initilize_pntr();
	
	#ifdef ENABLE_DEBUG
	printf("Initilized Hardware\n");
	
	curState = INIT;
	#endif
}

	
void updateInput(void) {

	// Handle heartbeat counter, reset step timer if the medulla has been read by computer
	if (in.counter != out.counter)
		out.counter = in.counter;
	
	// set the time to the curret timer value
	setTimeStep(timerValue(STEP_TIMER));
	
	// Call the updateInput functin for this particular medulla
	updateInput_pntr(&in,&out);
}

void setTimeStep(uint16_t time) {
	out.timestep = time;
}

void setState(MedullaState state) {
	out.state = state;
}

MedullaState getState(void) {
	return in.command;
}

void timerOverflow(void) {
	#ifdef ENABLE_DEBUG
	printf("Overflow\n");
	#endif
	
	overflow_pntr();
	
	// set status LED white
	PORTC.OUTCLR = 0b111;
	PORTC.OUTSET = 0b101;
	
	while(1) {		
		// We don't want to recover from this. It was scary.
	}
}

// **** State Machine ****

MedullaState state_idle(void) {
	// If E-Stop triggered return ERROR
	// If low voltage triggered return ERROR
	// If thermistors too high return ERROR
	#ifdef ENABLE_DEBUG
	if (curState != IDLE)
		printf("IDLE\n");
	curState = IDLE;
	#endif
	
	// Update status LEDs
	PORTC.OUTCLR = 0b101;
	PORTC.OUTSET = 0b010;
	
	// Reset watchdog timer
	resetTimer(WATCHDOG_TIMER);
	
	// We are idle, so let's just do nothing
	return IDLE;;
}

MedullaState state_init(void) {
	// If E-Stop triggered return ERROR
	// If Limit switch tripped return ERROR
	// If low voltage triggered return ERROR
	// If thermistors too high return ERROR
	#ifdef ENABLE_DEBUG
	if (curState != INIT)
		printf("INIT\n");
	curState = INIT;
	#endif
	
	// Change indicator LED
	PORTC.OUTCLR = 0b001;
	PORTC.OUTSET = 0b110;
	
	// Reset the watchdog timer
	resetTimer(WATCHDOG_TIMER);
	
	// Reset error flags
	out.error_flags = 0;
	
	return init_pntr(&in,&out);
}

MedullaState state_run(void) {
	//   * If E-Stop triggered return ERROR
	//	 * If Limit switch tripped return ERROR
	//	 * If low voltage triggered return ERROR
	//	 * If thermistors too high return ERROR
	//	 * If encoders out of range return ERROR_DAMPING
	
	#ifdef ENABLE_DEBUG
	if (curState != RUN)
		printf("RUN\n");
	curState = RUN;
	#endif
	
	// Set indicator LEDs
	PORTC.OUTCLR = 0b011;
	PORTC.OUTSET = 0b100;
	
	// Update outputs
	updateOutput_pntr(&in,&out);
	
	// Reset watchdog timer
	resetTimer(WATCHDOG_TIMER);
	return run_pntr(&in,&out);
}

void state_stop(void) {	
	#ifdef ENABLE_DEBUG
	if (curState != STOP)
		printf("STOP\n");
	curState = STOP;
	#endif
	
	// Update Status LEDs
	PORTC.OUTCLR = 0b001;
	PORTC.OUTSET = 0b110;
	
	// Reset Watchdog Timer
	resetTimer(WATCHDOG_TIMER);
	stop_pntr(&in,&out);
}

MedullaState state_error_damping(void) {
	// Assert E-Stop
	// If motor enabled run damping controller
	// If motor stopped return ERROR
	// Else return ERROR_DAMPING
	#ifdef ENABLE_DEBUG
	if (curState != ERROR_DAMPING)
		printf("ERROR DAMPING\n");
	curState = ERROR_DAMPING;
	#endif
	
	// Update Status LEDs
	PORTC.OUTCLR = 0b010;
	PORTC.OUTSET = 0b101;
	
	// Reset Watchdog Timer
	resetTimer(WATCHDOG_TIMER);
	
	// Set the estop error flag and the motor out of range error flag if we ever get to the error damping state
	out.error_flags |= STATUS_ESTOP;
	out.error_flags |= STATUS_MOTOR_OUT_OF_RANGE;
	
	return error_damping_pntr(&in,&out);
}

MedullaState state_error(void) {
	// - State: ERROR
	
	#ifdef ENABLE_DEBUG
	if (curState != ERROR)
		printf("ERROR\n");
	curState = ERROR;
	#endif
	
	// Reset watchdog timer
	resetTimer(WATCHDOG_TIMER);
	
	// Update Status LEDs
	PORTC.OUTCLR = 0b110;
	PORTC.OUTSET = 0b001;
	
	// Set the estop error flag if we ever get to the error state
	out.error_flags |= STATUS_ESTOP;

	return error_pntr();
}
