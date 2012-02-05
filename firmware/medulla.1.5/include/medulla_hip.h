#ifndef MEDULLA_HIP_H
#define MEDULLA_HIP_H

// Kit Morton
//
//	medulla_hip.h
//	This program reads sensors and controls one half of an atrias 2.* hip
////////////////////////////////////////////////////////////////////////////////

#include "../../../drl-sim/atrias/include/atrias/ucontroller.h"
#include <util/delay.h>

#include "medulla_controller.h"

#include "limitSW.h"
#include "ssi_13bit.h"
#include "amp.h"
#include "adc.h"
#include "quadrature.h"

#define ENABLE_ENCODERS
#define ENABLE_LIMITSW
#define ENABLE_MOTOR
#define ENABLE_DEBUG
#define ENABLE_THERM
//#define ENABLE_MOTOR_POWER_MONITOR
#define ENABLE_LOGIC_POWER_MONITOR
//#define ENABLE_DAMPING_REGIONS
//#define ENABLE_MOTOR_PASSTHROUGH

#define ABS(x)	(((x) < 0) ? -(x) : (x))
#define DAMPING_INVERSE_P_GAIN 1

void initilize_hip(void);
void updateInput_hip(uControllerInput *in, uControllerOutput *out);
void updateOutput_hip(uControllerInput *in, uControllerOutput *out);
void overflow_hip(void);

// **** State Machine ****

MedullaState idle_hip(uControllerInput *in, uControllerOutput *out);
MedullaState init_hip(uControllerInput *in, uControllerOutput *out);
MedullaState run_hip(uControllerInput *in, uControllerOutput *out);
void stop_hip(uControllerInput *in, uControllerOutput *out);
MedullaState error_damping_hip(uControllerInput *in, uControllerOutput *out);
MedullaState error_hip(void);

#endif