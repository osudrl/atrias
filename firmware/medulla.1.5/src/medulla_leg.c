// Kit Morton
//
//	medulla_leg.c
//	This program reads sensors and controls a motor for half of an ATRIAS 2.0
//	leg.
////////////////////////////////////////////////////////////////////////////////

#include "medulla_leg.h"

void initilize_leg(void) {
	// Init Limit Switches
	#ifdef ENABLE_LIMITSW
	initLimitSW(&PORTK);
	LimitSWCounter = 0;
	#endif
	
	// Init Toe Switch
	#ifdef ENABLE_TOESW
	PORTA.PIN4CTRL = PORT_OPC_PULLUP_gc;
	#ifdef ENABLE_TOESW_DEBOUNCE
	toeCounter = 0;
	#endif
	#endif
	
	// Voltage monitor
	#ifdef ENABLE_MOTOR_POWER
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
	
	#ifdef ENABLE_INC_ENCODER
	initQuad();
	prev_inc_encoder = 0;
	#endif
	
	// Init motor controllers for current measurement
	#ifdef ENABLE_MOTOR
	initAmp(&USARTF0,&PORTF,&PORTC,&TCC1,&HIRESC,4,3);
	#endif
	
	// We need to wait a little while so the encoders will initilize
	_delay_ms(1000);

}

	
void updateInput_leg(uControllerInput *in, uControllerOutput *out) {
	#ifdef ENABLE_ENCODERS
	uint8_t	biss[4];
	uint8_t status;
	#endif
	
	// Check Limit Switches
	#ifdef ENABLE_LIMITSW
	if ((checkLimitSW() & 0b11101111) != 0)
		LimitSWCounter++;
	else if (((checkLimitSW() & 0b11101111) == 0) && (LimitSWCounter > 0))
		LimitSWCounter--;
	if (LimitSWCounter > 100)
		out->limitSW = (checkLimitSW() & 0b11101111);
	#endif
	
	// Check Toe Switch
	#ifdef ENABLE_TOESW
	#ifdef ENABLE_TOESW_DEBOUNCE
	if (((PORTA.IN & 1<<4) != 0)) {
		// If the to switch is pressed
		if (toeCounter < 100)
			toeCounter++;
	}
	else if (((PORTA.IN & 1<<4) == 0)) {
		// If the toe switch is not pressed
		if (toeCounter > 0)
			toeCounter--;
	}
	
	if (toeCounter == 100)
		out->toe_switch = 1;
	else if (toeCounter == 0)
		out->toe_switch = 0;
	#else
	if (((PORTA.IN & 1<<4) != 0)) {
		// If the to switch is pressed
		out->toe_switch = 1;
	}
	else {
		// If the toe switch is not pressed
		out->toe_switch = 0;
	}
	#endif
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
	//while ((status == 0xFF) || ((status & BISS_ERROR_bm) == 0)) {
	//	status = readBiSS_bang(biss, &PORTF,1<<5,1<<7);
	//}
	
	if ((status == 0xFF) || ((status & BISS_ERROR_bm) == 0)) { // Error happened
		status = readBiSS_bang(biss, &PORTF,1<<5,1<<7);
		if ((status == 0xFF) || ((status & BISS_ERROR_bm) == 0)) { // second error happened
		}
		else {
			out->encoder[0] = *((uint32_t*)biss);
		}
	}
	else {
		out->encoder[0] = *((uint32_t*)biss);
	}
	
	
	// Read Leg Encoder
	status = readBiSS_bang_motor(biss, &PORTC,1<<5,1<<6);
	// Check if the BiSS read was valid
	//while ((status == 0xFF) || ((status & BISS_ERROR_bm) == 0)) {
	//	status = readBiSS_bang_motor(biss, &PORTD,1<<5,1<<6);
	//}
	
	if ((status == 0xFF) || ((status & BISS_ERROR_bm) == 0)) { // Error happened
		status = readBiSS_bang(biss, &PORTF,1<<5,1<<7);
		if ((status == 0xFF) || ((status & BISS_ERROR_bm) == 0)) { // second error happened
		}
		else {
			out->encoder[1] = *((uint32_t*)biss);
		}
	}
	else {
		out->encoder[1] = *((uint32_t*)biss);
	}
	#endif
	
	#ifdef ENABLE_INC_ENCODER
	cur_inc_encoder = (int32_t)TC_ENC.CNT;
	if (ABS(cur_inc_encoder - prev_inc_encoder) > 1000) {
		if ((cur_inc_encoder - prev_inc_encoder) < 0)
			inc_encoder_base += 0xFFFF;
		else
			inc_encoder_base -= 0xFFFF;
	}
	out->encoder[2] = (inc_encoder_base + cur_inc_encoder);
	prev_inc_encoder = cur_inc_encoder;
	#endif
	
	// Read Thermistors
	#ifdef ENABLE_THERM
	out->thermistor[0] = readADC_CH(ADCA.CH0);
	out->thermistor[1] = readADC_CH(ADCA.CH1);
	out->thermistor[2] = readADC_CH(ADCA.CH2);
	#endif
	
	// Read the power monitor ADCs
	#ifdef ENABLE_MOTOR_POWER
	out->motor_power = readADC_CH(ADCB.CH0);
	#endif
	
	#ifdef ENABLE_LOGIC_POWER_MONITOR
	out->logic_power = readADC_CH(ADCB.CH1);
	#endif
}

void updateOutput_leg(uControllerInput *in, uControllerOutput *out) {
	#ifdef ENABLE_MOTOR
	if (in->motor_torque < 1)
		setPWM(in->motor_torque*-1,0,&PORTC,&TCC1,3);
	else
		setPWM(in->motor_torque,1,&PORTC,&TCC1,3);
	#endif
}

void overflow_leg(void) {
	#ifdef ENABLE_MOTOR
	disablePWM(&TCC1);
	#endif
}

// **** State Machine ****

MedullaState idle_leg(uControllerInput *in, uControllerOutput *out) {
	return IDLE;
}

MedullaState init_leg(uControllerInput *in, uControllerOutput *out) {
	// If Limit switch tripped return ERROR
	// If low voltage triggered return ERROR
	// If thermistors too high return ERROR
	
	// Reset error flags
	#ifdef ENABLE_LIMITSW
	out->limitSW = 0;
	#endif
	
	// Enable the amplifier
	#ifdef ENABLE_MOTOR
	enableAmp(&USARTF0);
	enablePWM(&TCC1);
	#endif
	
	// If a limit switch was hit, stop
	#ifdef ENABLE_LIMITSW
	if (out->limitSW != 0) {
		out->error_flags |= STATUS_LIMITSW;
		return ERROR;
	}
	#endif
	
	// If the damping regions have not been set, then go back to error
	#ifdef ENABLE_DAMPING_REGIONS
	if ((in->enc_max == 0) || (in->enc_min == 0))
		return ERROR;
	#endif
	
	return INIT;
}

MedullaState run_leg(uControllerInput *in, uControllerOutput *out) {
	//	 * If Limit switch tripped return ERROR
	//	 * If low voltage triggered return ERROR
	//	 * If thermistors too high return ERROR
	//	 * If encoders out of range return ERROR_DAMPING
	
	// If a limit switch was hit, stop
	#ifdef ENABLE_LIMITSW
	if (out->limitSW != 0) {
		#ifdef ENABLE_DEBUG
		printf("LIMIT SWITCH\n");
		#endif
		out->error_flags |= STATUS_LIMITSW;
		return ERROR;
	}
	#endif
	
	// If any of the motor thermistors are over the max temperature, then disable
	#ifdef ENABLE_THERM
	if ((out->thermistor[0] <= THERM_MAX_VAL) ||
		(out->thermistor[1] <= THERM_MAX_VAL) ||
		(out->thermistor[2] <= THERM_MAX_VAL)) {
		// Increment thermistor counter
		ThermistorCounter++;
	}
	else {
		// Else decrement thermistor counter if it's not already at 0
		if (ThermistorCounter > 0)
			ThermistorCounter--;
	}
	if (ThermistorCounter > 100) {
		// If the thermistors have been too hot for too long, then error out
		#ifdef ENABLE_DEBUG
		printf("Motor Over Temp");
		#endif
		out->error_flags |= STATUS_OVER_TEMP;
		return ERROR;
	}
	#endif
	
	// Check the input voltages agains the minimums
	#ifdef ENABLE_MOTOR_POWER_MONITOR
	if ((out->motor_power < MOTOR_POWER_DANGER_MAX) && (out->motor_power > MOTOR_POWER_DANGER_MAX))
		MotorPowerCounter++;
	else {
		if (MotorPowerCounter > 0)
			MotorPowerCounter--;
	}
	if (MotorPowerCounter > 1000) {
		#ifdef ENABLE_DEBUG
		printf("Motor power too low");
		#endif
		out->error_flags |= STATUS_MOTOR_VOLTAGE_LOW;
		return ERROR;
	}
	#endif
	
	#ifdef ENABLE_LOGIC_POWER_MONITOR
	if (out->logic_power < LOGIC_POWER_MIN) 
		LogicPowerCounter++;
	
	else {
		if (LogicPowerCounter > 0)
			LogicPowerCounter--;
	}
	if (LogicPowerCounter > 1000) {
		#ifdef ENABLE_DEBUG
		printf("Logic power too low");
		#endif
		out->error_flags |= STATUS_LOGIC_VOLTAGE_LOW;
		return ERROR;
	}
	#endif
	
	// Make sure the motor output is not inside the damping regions
	#ifdef ENABLE_DAMPING_REGIONS
	if ((out->encoder[0] < in->enc_min) || (out->encoder[0] > in->enc_max))
		return ERROR_DAMPING;
	#endif
	
	return RUN;
}

void stop_leg(uControllerInput *in, uControllerOutput *out) {	
	
	// Disable Motor
	#ifdef ENABLE_MOTOR
	disablePWM(&TCC1);
	#endif

}

MedullaState error_damping_leg(uControllerInput *in, uControllerOutput *out) {
	// If motor enabled run damping controller
	// If motor stopped return ERROR
	// Else return ERROR_DAMPING
		
	// Run damping controller
	#ifdef ENABLE_DAMPING_REGIONS
	static uint32_t prev_encoder_val;
	
	// If this is the first call to error_damping, then do nothing because we can't calculate an velocity
	if (prev_encoder_val != 0) {
		// If the motor is still moving, then we need to run the damping controller
		if (ABS(prev_encoder_val - out->encoder[0]) > 10) {
			int torque  = (prev_encoder_val - out->encoder[0])/(DAMPING_INVERSE_P_GAIN);
			//if (torque < 0)
				//setPWM(torque*-1,0,&PORTC,&TCC1,3);
			//else
				//setPWM(toruque,1,&PORTC,&TCC1,3);
		}
		printf("%d\n",prev_encoder_val - out->encoder[0]);
	}
	
	prev_encoder_val = out->encoder[0];
	#endif
	
	// If a limit switch was hit, stop
	#ifdef ENABLE_LIMITSW
	if (out->limitSW != 0) {
		out->error_flags |= STATUS_LIMITSW;
		return ERROR;
	}
	#endif
	
	return ERROR;
}

MedullaState error_leg(void) {
	// - State: ERROR
	//   * Disable motors
	
	#ifdef ENABLE_MOTOR
	disablePWM(&TCC1);
	#endif
	
	return ERROR;
}
