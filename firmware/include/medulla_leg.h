#ifndef MEDULLA_LEG_H
#define MEDULLA_LEG_H

// Kit Morton
//
//	medulla_leg.h
//	This program reads sensors and controls a motor for half of an ATRIAS 2.0
//	leg.
////////////////////////////////////////////////////////////////////////////////

#include "../../drl-sim/atrias/include/atrias/ucontroller.h"
#include <util/delay.h>

#include "medulla_controller.h"
#include "limitSW.h"
#include "biss_bang.h"
#include "amp.h"
#include "adc.h"
#include "quadrature.h"

#define ENABLE_ENCODERS
#define ENABLE_INC_ENCODER
#define ENABLE_LIMITSW
#define ENABLE_MOTOR
#define ENABLE_TOESW
#define ENABLE_TOE_FORCE
//#define ENABLE_TOESW_DEBOUNCE
#define ENABLE_DEBUG
#define ENABLE_THERM
#define ENABLE_MOTOR_POWER
#define ENABLE_MOTOR_POWER_MONITOR
#define ENABLE_LOGIC_POWER_MONITOR
//#define ENABLE_DAMPING_REGIONS

#define ABS(x)	(((x) < 0) ? -(x) : (x))
#define DAMPING_INVERSE_P_GAIN 1

#ifdef ENABLE_DEBUG
MedullaState curState;
#endif

#ifdef ENABLE_LIMITSW
int16_t LimitSWCounter;
#endif

#ifdef ENABLE_TOESW_DEBOUNCE
uint16_t toeCounter;
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

#ifdef ENABLE_INC_ENCODER
int32_t	prev_inc_encoder;
int32_t cur_inc_encoder;
#endif

void initilize_leg(void);
void updateInput_leg(uControllerInput *in, uControllerOutput *out);
void updateOutput_leg(uControllerInput *in, uControllerOutput *out);
void overflow_leg(void);

// **** State Machine ****

MedullaState idle_leg(uControllerInput *in, uControllerOutput *out);
MedullaState init_leg(uControllerInput *in, uControllerOutput *out);
MedullaState run_leg(uControllerInput *in, uControllerOutput *out);
void stop_leg(uControllerInput *in, uControllerOutput *out);
MedullaState error_damping_leg(uControllerInput *in, uControllerOutput *out);
MedullaState error_leg(void);

#endif
