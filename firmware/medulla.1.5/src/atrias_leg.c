// Kit Morton
//
//	atrias_leg.c
//	This program reads sensors and controls a motor for half of an ATRIAS 2.0
//	leg.
////////////////////////////////////////////////////////////////////////////////

#include "../../../drl-sim/atrias/include/atrias/ucontroller.h"
#include <util/delay.h>

#include <stdint.h>
#include "medulla_controller.h"
#include "limitSW.h"
#include "biss_bang.h"
#include "pwm.h"

#define ENABLE_ENCODERS
#define ENABLE_LIMITSW
#define ENABLE_MOTOR_DEBUG

uControllerInput in;
uControllerOutput out;
MedullaState curState;
int8_t LimitSWCounter;

int main(void) {
	/*CLK.PSCTRL = 0x00;															// no division on peripheral clocks
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
	*/
	medulla_run((void*)(&in),(void*)(&out));
	return 0;
}

void init(void) {
	//******** Init ********
	initLimitSW(&PORTK);
	// Init analog input
	// - Toe Switch
	// - Voltage monitor
	// - Motor thermistors
	// Init Encoders
	initBiSS_bang(&PORTC,1<<5,1<<6);
	initBiSS_bang(&PORTF,1<<5,1<<7);
	// Init motor controllers for current measurement
	initUART(&PORTF,&USARTF0,1152);
	//_delay_ms(1000);
	//0xA5 0x3F 0x02 0x07 0x00 0x01 0xB3 0xE7 0x0F 0x00 0x10 0x3E
	UARTWriteChar(&USARTF0, 0xA5);
	UARTWriteChar(&USARTF0, 0x3F);
	UARTWriteChar(&USARTF0, 0x02);
	UARTWriteChar(&USARTF0, 0x07);
	UARTWriteChar(&USARTF0, 0x00);
	UARTWriteChar(&USARTF0, 0x01);
	UARTWriteChar(&USARTF0, 0xB3);
	UARTWriteChar(&USARTF0, 0xE7);
	UARTWriteChar(&USARTF0, 0x0F);
	UARTWriteChar(&USARTF0, 0x00);
	UARTWriteChar(&USARTF0, 0x10);
	UARTWriteChar(&USARTF0, 0x3E);
	_delay_ms(100);
	//0xA5 0x3F 0x02 0x01 0x00 0x01 0x01 0x47 0x00 0x00 0x00 0x00
	UARTWriteChar(&USARTF0, 0xA5);
	UARTWriteChar(&USARTF0, 0x3F);
	UARTWriteChar(&USARTF0, 0x02);
	UARTWriteChar(&USARTF0, 0x01);
	UARTWriteChar(&USARTF0, 0x00);
	UARTWriteChar(&USARTF0, 0x01);
	UARTWriteChar(&USARTF0, 0x01);
	UARTWriteChar(&USARTF0, 0x47);
	UARTWriteChar(&USARTF0, 0x00);
	UARTWriteChar(&USARTF0, 0x00);
	UARTWriteChar(&USARTF0, 0x00);
	UARTWriteChar(&USARTF0, 0x00);
	
	// init the input and output values
	out.id				= 0;
	out.state			= 0;
	out.encoder[0]		= 12;
	out.encoder[1]		= 0; 	
	out.encoder[2]		= 0;
	out.timestep		= 0;
	out.error_flags		= 0;
	out.thermistor[0]	= 0;
	out.thermistor[1]	= 0;
	out.thermistor[2]	= 0;
	out.limitSW			= 0;
	out.counter			= 0;
	
	in.motor_torque		= 0;
	in.command			= 0;
	
	LimitSWCounter = 0;
	PORTC.DIRSET = 0b111;
	PORTC.OUTSET = 0b111;
	//printf("init\n");
	curState = INIT;
	_delay_ms(1000);
}

	
void updateInput(void) {
	uint8_t tmp8;
	uint8_t	biss[4];
	uint8_t status;
	
	if (in.counter != out.counter)
	{
		setTimeStep(stepTimerValue());
		resetStepTimer();
		out.counter = in.counter;
	}
	
	#ifdef ENABLE_MOTOR_DEBUG
	if (USARTE0.STATUS&USART_RXCIF_bm)
			UARTWriteChar(&USARTF0, USARTE0.DATA);
	if (USARTF0.STATUS&USART_RXCIF_bm)
		UARTWriteChar(&USARTE0, USARTF0.DATA);
	#endif
			
	#ifdef ENABLE_LIMITSW
	if (checkLimitSW() != 0)
		LimitSWCounter++;
	else if ((checkLimitSW() == 0) && (LimitSWCounter > 0))
		LimitSWCounter--;
	if (LimitSWCounter > 100)
		out.limitSW = checkLimitSW();
	#endif
	#ifdef ENABLE_MOTOR_DEBUG
	if (USARTE0.STATUS&USART_RXCIF_bm)
			UARTWriteChar(&USARTF0, USARTE0.DATA);
	if (USARTF0.STATUS&USART_RXCIF_bm)
		UARTWriteChar(&USARTE0, USARTF0.DATA);
	#endif
	#ifdef ENABLE_ENCODERS
	// Read Motor Encoder
	status = readBiSS_bang(biss, &PORTF,1<<5,1<<7);
	#ifdef ENABLE_MOTOR_DEBUG
	if (USARTE0.STATUS&USART_RXCIF_bm)
			UARTWriteChar(&USARTF0, USARTE0.DATA);
	if (USARTF0.STATUS&USART_RXCIF_bm)
		UARTWriteChar(&USARTE0, USARTF0.DATA);
	#endif
		// Check if the BiSS read was valid
	while ((status == 0xFF) || ((status & BISS_ERROR_bm) == 0)) {
		status = readBiSS_bang(biss, &PORTF,1<<5,1<<7);
		#ifdef ENABLE_MOTOR_DEBUG
	if (USARTE0.STATUS&USART_RXCIF_bm)
			UARTWriteChar(&USARTF0, USARTE0.DATA);
	if (USARTF0.STATUS&USART_RXCIF_bm)
		UARTWriteChar(&USARTE0, USARTF0.DATA);
	#endif
	}
	out.encoder[0] = *((uint32_t*)biss);
	
	// Read Leg Encoder
	status = readBiSS_bang_motor(biss, &PORTC,1<<5,1<<6);
	#ifdef ENABLE_MOTOR_DEBUG
	if (USARTE0.STATUS&USART_RXCIF_bm)
			UARTWriteChar(&USARTF0, USARTE0.DATA);
	if (USARTF0.STATUS&USART_RXCIF_bm)
		UARTWriteChar(&USARTE0, USARTF0.DATA);
	#endif
		// Check if the BiSS read was valid
	while ((status == 0xFF) || ((status & BISS_ERROR_bm) == 0)) {
		status = readBiSS_bang_motor(biss, &PORTD,1<<5,1<<6);
		#ifdef ENABLE_MOTOR_DEBUG
	if (USARTE0.STATUS&USART_RXCIF_bm)
			UARTWriteChar(&USARTF0, USARTE0.DATA);
	if (USARTF0.STATUS&USART_RXCIF_bm)
		UARTWriteChar(&USARTE0, USARTF0.DATA);
	#endif
	}
	out.encoder[1] = *((uint32_t*)biss);
	#endif
	
	out.thermistor[0] = PORTJ.IN & (1<<3 | 1<<4);
}

void updateOutput(void) {

	if (in.motor_torque < 1)
		setPWM(in.motor_torque*-1,0,&PORTC,&TCC1,3);
	else
		setPWM(in.motor_torque,1,&PORTC,&TCC1,3);
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
	assertEStop();
	printf("Overflow\n");
	PORTC.OUTCLR = 0b110;
	PORTC.OUTSET = 0b001;
	while(1) {		
	}
}

// **** State Machine ****

MedullaState state_idle(void) {
	// If E-Stop triggered return ERROR
	// If Limit switch tripped return ERROR
	// If low voltage triggered return ERROR
	// If thermistors too high return ERROR
	//if (curState != IDLE)
		//printf("IDLE\n");
	curState = IDLE;
	PORTC.OUTCLR = 0b101;
	PORTC.OUTSET = 0b010;
	
	return IDLE;
}

MedullaState state_init(void) {
	// Reset errors
	// Enable motors
	// If E-Stop triggered return ERROR
	// If Limit switch tripped return ERROR
	// If low voltage triggered return ERROR
	// If thermistors too high return ERROR
	if (curState != INIT)
		printf("INIT\n");
	curState = INIT;
	PORTC.OUTCLR = 0b001;
	PORTC.OUTSET = 0b110;
	//0xA5 0x3F 0x02 0x07 0x00 0x01 0xB3 0xE7 0x0F 0x00 0x10 0x3E
	UARTWriteChar(&USARTF0, 0xA5);
	UARTWriteChar(&USARTF0, 0x3F);
	UARTWriteChar(&USARTF0, 0x02);
	UARTWriteChar(&USARTF0, 0x07);
	UARTWriteChar(&USARTF0, 0x00);
	UARTWriteChar(&USARTF0, 0x01);
	UARTWriteChar(&USARTF0, 0xB3);
	UARTWriteChar(&USARTF0, 0xE7);
	UARTWriteChar(&USARTF0, 0x0F);
	UARTWriteChar(&USARTF0, 0x00);
	UARTWriteChar(&USARTF0, 0x10);
	UARTWriteChar(&USARTF0, 0x3E);
	_delay_ms(100);
	//0xA5 0x3F 0x02 0x01 0x00 0x01 0x01 0x47 0x00 0x00 0x00 0x00
	UARTWriteChar(&USARTF0, 0xA5);
	UARTWriteChar(&USARTF0, 0x3F);
	UARTWriteChar(&USARTF0, 0x02);
	UARTWriteChar(&USARTF0, 0x01);
	UARTWriteChar(&USARTF0, 0x00);
	UARTWriteChar(&USARTF0, 0x01);
	UARTWriteChar(&USARTF0, 0x01);
	UARTWriteChar(&USARTF0, 0x47);
	UARTWriteChar(&USARTF0, 0x00);
	UARTWriteChar(&USARTF0, 0x00);
	UARTWriteChar(&USARTF0, 0x00);
	UARTWriteChar(&USARTF0, 0x00);
	
	initPWM(&PORTC,&TCC1,&HIRESC,4,3);
	
	// If a limit switch was hit, stop
	if (out.limitSW != 0)
		return ERROR;
	
	return INIT;
}

MedullaState state_run(void) {
	//   * Update motor torques
	//   * If E-Stop triggered return ERROR
	//	 * If Limit switch tripped return ERROR
	//	 * If low voltage triggered return ERROR
	//	 * If thermistors too high return ERROR
	//	 * If encoders out of range return ERROR_DAMPING
	if (curState != RUN)
		printf("RUN\n");
	curState = RUN;
	updateOutput();
	PORTC.OUTCLR = 0b011;
	PORTC.OUTSET = 0b100;
	
	// If a limit switch was hit, stop
	if (out.limitSW != 0) {
		printf("LIMIT SWITCH\n");
		return ERROR;
	}
	
	return RUN;
}

void state_stop(void) {	
	if (curState != STOP)
		printf("STOP\n");
	curState = STOP;
	
	disablePWM(&TCC1);
	
	PORTC.OUTCLR = 0b001;
	PORTC.OUTSET = 0b110;
	
	// Disable Motor
}

MedullaState state_error_damping(void) {
		// Assert E-Stop
		// If motor enabled run damping controller
		// If motor stopped return ERROR
		// Else return ERROR_DAMPING
	if (curState != ERROR_DAMPING)
		printf("ERROR DAMPING\n");
	curState = ERROR_DAMPING;
	PORTC.OUTCLR = 0b010;
	PORTC.OUTSET = 0b101;
	
	// If a limit switch was hit, stop
	if (out.limitSW != 0)
		return ERROR;
	
	return ERROR_DAMPING;
}

MedullaState state_error(void) {
		// - State: ERROR
		//   * Disable motors
	
	disablePWM(&TCC1);
	
	if (curState != ERROR)
		printf("ERROR\n");
	curState = ERROR;
	
	PORTC.OUTCLR = 0b110;
	PORTC.OUTSET = 0b001;

	return ERROR;
}
