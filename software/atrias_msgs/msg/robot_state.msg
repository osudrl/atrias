Header header

# This tells the controller what state RT Ops is in.
# Type: atrias::controllerManager::RtOpsCommand
uint8 cmState

uint8 robotConfiguration

# These are values recorded from ECat's DC
uint64 receiveDCTime
uint64 lastTransmitDCTime

# Whether or not safeties should be disabled. This will, in the future, be replaced w/ a GUI-controlled overrides system.
bool disableSafeties

robot_state_leg lLeg
robot_state_leg rLeg

robot_state_location position

uint8   boomMedullaState
float64 boomLogicVoltage
uint8   boomMedullaErrorFlags
