#include "medulla_boom.h"

void initilize_boom(void) {
	// Init 3 SSI encoders
	#ifdef ENABLE_ENCODERS
	initSSI_bang();
	
	// Get some initial previous values
	//yawEncoderPrevVal = readSSI_bang(&PORTC,5,6);
	//rollEncoderPrevVal = readSSI_bang(&PORTD,1,2,17);

	#endif
	// Wait on other medullas
	_delay_ms(1000);
	printf("Initilizing Boom\n");
}

void updateInput_boom(uControllerInput *in, uControllerOutput *out) {
	uint32_t encoderVals[3] = {0,0,0};
	
	// Read the encoders
	#ifdef ENABLE_ENCODERS
	readSSI_bang(encoderVals);
	/*
	// Handle overflow
	if (ABS(yawEncoderCurr - yawEncoderPrevVal) > 1000) {
		if ((yawEncoderCurr - yawEncoderPrevVal) < 0)
			yawEncoderOffset += 0x1FFFF;
		else
			yawEncoderOffset -= 0x1FFFF;
	}
	
	if (ABS(rollEncoderCurr - rollEncoderPrevVal) > 1000) {
		if ((rollEncoderCurr - rollEncoderPrevVal) < 0)
			rollEncoderOffset += 0x1FFFF;
		else
			rollEncoderOffset -= 0x1FFFF;
	}
	
	if (ABS(pitchEncoderCurr - pitchEncoderPrevVal) > 1000) {
		if ((pitchEncoderCurr - pitchEncoderPrevVal) < 0)
			pitchEncoderOffset += 0x1FFFF;
		else
		pitchEncoderOffset -= 0x1FFFF;
	}
	*/
	// fill out the output variables
	out->encoder[0] = encoderVals[1];
	out->encoder[1] = encoderVals[0];
	
	#endif
}

void updateOutput_boom(uControllerInput *in, uControllerOutput *out) {
	// There is nothing for this medulla to output
	// so this is just a dummy function
}

void overflow_boom(void) {
	// When the watchdog timer overflows we don't need to do
	// anything in particular
}

// **** State Machine ****

// All these are dummy functions because there are no special errors to throw
MedullaState idle_boom(uControllerInput *in, uControllerOutput *out) {return IDLE;}
MedullaState init_boom(uControllerInput *in, uControllerOutput *out) {return INIT;}
MedullaState run_boom(uControllerInput *in, uControllerOutput *out) {return RUN;}
void stop_boom(uControllerInput *in, uControllerOutput *out) {}
MedullaState error_damping_boom(uControllerInput *in, uControllerOutput *out) {return ERROR;}
MedullaState error_boom(void) {return ERROR;}