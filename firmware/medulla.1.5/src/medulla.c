// Kit Morton
//
//	atrias_leg.c
//	This program reads sensors and controls a motor for half of an ATRIAS 2.0
//	leg.
////////////////////////////////////////////////////////////////////////////////

#include "../../../drl-sim/atrias/include/atrias/ucontroller.h"
#include <util/delay.h>

#include "medulla_controller.h"
#include "limitSW.h"
#include "biss_bang.h"
#include "amp.h"
#include "adc.h"

#define ENABLE_ENCODERS
#define ENABLE_LIMITSW
#define ENABLE_MOTOR
#define ENABLE_TOESW
#define ENABLE_DEBUG
#define ENABLE_THERM
//#define ENABLE_MOTOR_POWER_MONITOR
#define ENABLE_LOGIC_POWER_MONITOR
#define ENABLE_DAMPING_REGIONS
//#define ENABLE_MOTOR_PASSTHROUGH

#define ABS(x)	(((x) < 0) ? -(x) : (x))
#define DAMPING_INVERSE_P_GAIN 1

uControllerInput in;
uControllerOutput out;

#ifdef ENABLE_DEBUG
MedullaState curState;
#endif

#ifdef ENABLE_LIMITSW
int8_t LimitSWCounter;
#endif

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
	
	// Init Limit Switches
	#ifdef ENABLE_LIMITSW
	initLimitSW(&PORTK);
	LimitSWCounter = 0;
	#endif
	
	// Init Toe Switch
	#ifdef ENABLE_TOESW
	PORTA.PIN4CTRL = PORT_OPC_PULLUP_gc;
	#endif
	
	// Voltage monitor
	#ifdef ENABLE_MOTOR_POWER_MONITOR
	initADC(&ADCB);					// Init ADC
	initADC_CH(&(ADCB.CH0), 0);		// Configure channel 0 to monitor motor power input
	#endif
	
	#ifdef ENABLE_LOGIC_POWER_MONITOR
	initADC(&ADCB);					// Init ADC
	initADC_CH(&(ADCB.CH1), 1);		// Configure channel 1 to monitor logic power input
	#endif
	
	// Motor thermistors
	#ifdef ENABLE_THERM
	initADC(&ADCA);					// Init ADC
	initADC_CH(&(ADCA.CH0), 1);		// Configure channels 0, 1, and 2 for monitoring the thermistors
	initADC_CH(&(ADCA.CH1), 2);
	initADC_CH(&(ADCA.CH2), 3);
	#endif
	
	// Init Encoders
	#ifdef ENABLE_ENCODERS
	initBiSS_bang(&PORTC,1<<5,1<<6);
	initBiSS_bang(&PORTF,1<<5,1<<7);
	#endif
	
	// Init motor controllers for current measurement
	#ifdef ENABLE_MOTOR
	initAmp(&USARTF0,&PORTF,&PORTC,&TCC1,&HIRESC,4,3);
	#endif
	
	// init the input and output values
	out.id				= 0;
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
	
	#ifdef ENABLE_DEBUG
	printf("Initilized Hardware\n");
	_delay_ms(1000);
	
	curState = INIT;
	#endif
}

	
void updateInput(void) {
	#ifdef ENABLE_ENCODERS
	uint8_t	biss[4];
	uint8_t status;
	#endif
	
	// Handle heartbeat counter, reset step timer if the medulla has been read by computer
	if (in.counter != out.counter)
	{
		setTimeStep(stepTimerValue());
		resetStepTimer();
		out.counter = in.counter;
	}
	
	// Check Limit Switches
	#ifdef ENABLE_LIMITSW
	if (checkLimitSW() != 0)
		LimitSWCounter++;
	else if ((checkLimitSW() == 0) && (LimitSWCounter > 0))
		LimitSWCounter--;
	if (LimitSWCounter > 100)
		out.limitSW = checkLimitSW();
	#endif
	
	// Check Toe Switch
	#ifdef ENABLE_TOESW
	if ((PORTA.IN & 1<<4) == 0) 
		// If the to switch is not pressed
		out.toe_switch = 0;
	else
		// If the toe switch is pressed
		out.toe_switch = 1;
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
	// Check if the BiSS read was valid
	while ((status == 0xFF) || ((status & BISS_ERROR_bm) == 0)) {
		status = readBiSS_bang(biss, &PORTF,1<<5,1<<7);
	}
	out.encoder[0] = *((uint32_t*)biss);
	
	// Read Leg Encoder
	status = readBiSS_bang_motor(biss, &PORTC,1<<5,1<<6);
	// Check if the BiSS read was valid
	while ((status == 0xFF) || ((status & BISS_ERROR_bm) == 0)) {
		status = readBiSS_bang_motor(biss, &PORTD,1<<5,1<<6);
	}
	out.encoder[1] = *((uint32_t*)biss);
	#endif
	
	// Read Thermistors
	#ifdef ENABLE_THERM
	out.thermistor[0] = readADC_CH(ADCA.CH0);
	out.thermistor[1] = readADC_CH(ADCA.CH1);
	out.thermistor[2] = readADC_CH(ADCA.CH2);
	#endif
	
	// Read the power monitor ADCs
	#ifdef ENABLE_MOTOR_POWER_MONITOR
	out.motor_power = readADC_CH(ADCB.CH0);
	#endif
	
	#ifdef ENABLE_LOGIC_POWER_MONITOR
	out.logic_power = readADC_CH(ADCB.CH1);
	#endif
}

void updateOutput(void) {
	#ifdef ENABLE_MOTOR
	if (in.motor_torque < 1)
		setPWM(in.motor_torque*-1,0,&PORTC,&TCC1,3);
	else
		setPWM(in.motor_torque,1,&PORTC,&TCC1,3);
	#endif
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
	
	#ifdef ENABLE_MOTOR
	disablePWM(&TCC1);
	#endif
	
	// set status LED white
	PORTC.OUTSET = 0b111;
	
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
	
	// We are idle, so let's just do nothing
	
	return IDLE;
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
	
	// Reset errors
	#ifdef ENABLE_LIMITSW
	out.limitSW = 0;
	#endif
	
	// Enable the amplifier
	#ifdef ENABLE_MOTOR
	enableAmp(&USARTF0);
	enablePWM(&TCC1);
	#endif
	
	// If a limit switch was hit, stop
	#ifdef ENABLE_LIMITSW
	if (out.limitSW != 0)
		return ERROR;
	#endif
	
	// If the damping regions have not been set, then go back to error
	#ifdef ENABLE_DAMPING_REGIONS
	if ((in.enc_max == 0) || (in.enc_min == 0))
		return ERROR;
	#endif
	
	return INIT;
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
	updateOutput();
	
	// If a limit switch was hit, stop
	#ifdef ENABLE_LIMITSW
	if (out.limitSW != 0) {
		#ifdef ENABLE_DEBUG
		printf("LIMIT SWITCH\n");
		#endif
		return ERROR;
	}
	#endif
	
	// If any of the motor thermistors are over the max temperature, then disable
	#ifdef ENABLE_THERM
	if ((out.thermistor[0] <= THERM_MAX_VAL) ||
		(out.thermistor[1] <= THERM_MAX_VAL) ||
		(out.thermistor[2] <= THERM_MAX_VAL)) {
		#ifdef ENABLE_DEBUG
		printf("Motor Over Temp");
		#endif
		return ERROR;
	}
	#endif
	
	// Check the input voltages agains the minimums
	#ifdef ENABLE_MOTOR_POWER_MONITOR
	if (out.motor_power < MOTOR_POWER_MIN) {
		#ifdef ENABLE_DEBUG
		printf("Motor power too low");
		#endif
		return ERROR;
	}
	#endif
	
	#ifdef ENABLE_LOGIC_POWER_MONITOR
	if (out.logic_power < LOGIC_POWER_MIN) {
		#ifdef ENABLE_DEBUG
		printf("Logic power too low");
		#endif
		return ERROR;
	}
	#endif
	
	// Make sure the motor output is not inside the damping regions
	#ifdef ENABLE_DAMPING_REGIONS
	if ((out.encoder[0] < in.enc_min) || (out.encoder[0] > in.enc_max))
		return ERROR_DAMPING;
	#endif
	
	return RUN;
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
	
	// Disable Motor
	#ifdef ENABLE_MOTOR
	disablePWM(&TCC1);
	#endif

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
	
	// Run damping controller
	#ifdef ENABLE_DAMPING_REGIONS
	static uint32_t prev_encoder_val;
	
	// If this is the first call to error_damping, then do nothing because we can't calculate an velocity
	if (prev_encoder_val != 0) {
		// If the motor is still moving, then we need to run the damping controller
		if (ABS(prev_encoder_val - out.encoder[0]) > 10) {
			int torque  = (prev_encoder_val - out.encoder[0])/(DAMPING_INVERSE_P_GAIN);
			//if (torque < 0)
				//setPWM(torque*-1,0,&PORTC,&TCC1,3);
			//else
				//setPWM(toruque,1,&PORTC,&TCC1,3);
		}
		printf("%d\n",prev_encoder_val - out.encoder[0]);
	}
	
	prev_encoder_val = out.encoder[0];
	#endif
	
	// If a limit switch was hit, stop
	#ifdef ENABLE_LIMITSW
	if (out.limitSW != 0)
		return ERROR;
	#endif
	
	return ERROR_DAMPING;
}

MedullaState state_error(void) {
	// - State: ERROR
	//   * Disable motors
	
	#ifdef ENABLE_MOTOR
	disablePWM(&TCC1);
	#endif
	
	#ifdef ENABLE_DEBUG
	if (curState != ERROR)
		printf("ERROR\n");
	curState = ERROR;
	#endif
	
	// Update Status LEDss
	PORTC.OUTCLR = 0b110;
	PORTC.OUTSET = 0b001;

	return ERROR;
}
