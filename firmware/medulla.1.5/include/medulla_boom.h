#ifndef MEDULLA_BOOM_H
#define MEDULLA_BOOM_H

// Kit Morton
//
//	medulla_boom.h
//	This program reads the encoders on the boom
////////////////////////////////////////////////////////////////////////////////

#include "../../../drl-sim/atrias/include/atrias/ucontroller.h"
#include <util/delay.h>

#define _MEDULLA_BOOM

#include "medulla_controller.h"
#include "ssi.h"
#include "ssi_13bit.h"

#define ABS(x)	(((x) < 0) ? -(x) : (x))

#define ENABLE_ENCODERS
#define ENABLE_DEBUG

#ifdef ENABLE_ENCODERS
int32_t rollEncoderOffset;
uint32_t	rollEncoderPrevVal;

int32_t pitchEncoderOffset;
uint32_t	pitchEncoderPrevVal;

int32_t yawEncoderOffset;
uint32_t	yawEncoderPrevVal;
#endif


void initilize_boom(void);
void updateInput_boom(uControllerInput *in, uControllerOutput *out);
void updateOutput_boom(uControllerInput *in, uControllerOutput *out);
void overflow_boom(void);

// **** State Machine ****

MedullaState idle_boom(uControllerInput *in, uControllerOutput *out);
MedullaState init_boom(uControllerInput *in, uControllerOutput *out);
MedullaState run_boom(uControllerInput *in, uControllerOutput *out);
void stop_boom(uControllerInput *in, uControllerOutput *out);
MedullaState error_damping_boom(uControllerInput *in, uControllerOutput *out);
MedullaState error_boom(void);

#endif