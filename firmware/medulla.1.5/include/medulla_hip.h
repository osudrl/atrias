// Kit Morton
//
//	medulla_hip.h
//	This program reads sensors and controls one half of an atrias 2.* hip
////////////////////////////////////////////////////////////////////////////////

#include "../../../drl-sim/atrias/include/atrias/ucontroller.h"
#include <util/delay.h>

#include "medulla_controller.h"

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