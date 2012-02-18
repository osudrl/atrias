#include "medulla_hip.h"

#ifdef ENABLE_LIMITSW
int16_t LimitSWCounter;
#endif

#ifdef ENABLE_THERM
int8_t ThermistorCounter;
#endif

#ifdef ENABLE_MOTOR_POWER_MONITOR
uint16_t	MotorPowerCounter;
#endif

#ifdef ENABLE_LOGIC_POWER_MONITOR
uint16_t	LogicPowerCounter;
#endif

#ifdef ENABLE_ENCODERS
uint16_t encoderData[2];
uint16_t encPrevious;
int32_t	encBaseValue;
#endif

void initilize_hip(void) {
	// Init Limit Switches
	#ifdef ENABLE_LIMITSW
	initLimitSW(&PORTK);
	LimitSWCounter = 0;
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
	initSSI_13();
	readSSI_13(encoderData);
	encPrevious = encoderData[1];
	#endif
	
	// Init motor controllers for current measurement
	#ifdef ENABLE_MOTOR
	initAmp(&USARTF0,&PORTF,&PORTC,&TCC1,&HIRESC,4,3);
	#endif
	
	_delay_ms(1000);
}

void updateInput_hip(uControllerInput *in, uControllerOutput *out) {
	
	// Check Limit Switches
	#ifdef ENABLE_LIMITSW
	if ((checkLimitSW() & 0b11101111) != 0)
		LimitSWCounter++;
	else if (((checkLimitSW() & 0b11101111) == 0) && (LimitSWCounter > 0))
		LimitSWCounter--;
	if (LimitSWCounter > 100)
		out->limitSW = (checkLimitSW() & 0b11101111);
	#endif
	
	#ifdef ENABLE_ENCODERS
	readSSI_13(encoderData);
	out->encoder[0] = (uint32_t)encoderData[0];
	
	// Handle Overflows
	if (ABS((int32_t)encoderData[1] - (int32_t)encPrevious) > 1000) {
		//printf("%d - %d = %d\n",encoderData[1], encPrevious,encoderData[1] - encPrevious);
		if (((int32_t)encoderData[1] - (int32_t)encPrevious) < 0)
			encBaseValue += 0x1FFF;
		else
			encBaseValue -= 0x1FFF;
	}
	out->encoder[1] = encBaseValue+(uint32_t)encoderData[1];
	encPrevious = encoderData[1];
	#endif
	
	// Read Thermistors
	#ifdef ENABLE_THERM
	out->thermistor[0] = readADC_CH(ADCA.CH0);
	out->thermistor[1] = readADC_CH(ADCA.CH1);
	out->thermistor[2] = readADC_CH(ADCA.CH2);
	#endif
	
	// Read the power monitor ADCs
	#ifdef ENABLE_MOTOR_POWER_MONITOR
	out->motor_power = readADC_CH(ADCB.CH0);
	#endif
	
	#ifdef ENABLE_LOGIC_POWER_MONITOR
	out->logic_power = readADC_CH(ADCB.CH1);
	#endif
}

void updateOutput_hip(uControllerInput *in, uControllerOutput *out) {
	#ifdef ENABLE_MOTOR
	if (in->motor_torque < 1)
		setPWM(in->motor_torque*-1,0,&PORTC,&TCC1,3);
	else
		setPWM(in->motor_torque,1,&PORTC,&TCC1,3);
	#endif
}

void overflow_hip(void) {
	#ifdef ENABLE_MOTOR
	disablePWM(&TCC1);
	#endif
}

// **** State Machine ****

MedullaState idle_hip(uControllerInput *in, uControllerOutput *out) {
	return IDLE;
}

MedullaState init_hip(uControllerInput *in, uControllerOutput *out) {
	// Reset error flags
	out->error_flags = 0;
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
	
	return INIT;
}

MedullaState run_hip(uControllerInput *in, uControllerOutput *out) {
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

	return RUN;
}

void stop_hip(uControllerInput *in, uControllerOutput *out) {
	// Disable Motor
	#ifdef ENABLE_MOTOR
	disablePWM(&TCC1);
	#endif
}

MedullaState error_damping_hip(uControllerInput *in, uControllerOutput *out) {
	return ERROR;
}

MedullaState error_hip(void) {
	#ifdef ENABLE_MOTOR
	disablePWM(&TCC1);
	#endif
	
	return ERROR;
}