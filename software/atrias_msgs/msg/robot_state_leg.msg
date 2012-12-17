Header header

# Here to make stuff compile... this has been moved to the hip message.
uint8 hipMedullaState

robot_state_hip hip
robot_state_legHalf halfA
robot_state_legHalf halfB

# Unitless -- should be processed
uint16  toeSwitch

# Limit switches
bool    legRetractSwitch
bool    motorRetractSwitch
bool    legExtendSwitch

# Strain gauge value
int32   kneeForce
