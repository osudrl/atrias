// Kit Morton
//
//	medulla_boom.h
//	This program reads the encoders on the boom
////////////////////////////////////////////////////////////////////////////////

#include "../../../drl-sim/atrias/include/atrias/ucontroller.h"
#include <util/delay.h>

#include "medulla_controller.h"

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