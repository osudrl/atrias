Header header

# This tells the controller what state RT Ops is in.
# Type: atrias::rtOps::rtOpsState
uint8 rtOpsState

uint8 robotConfiguration

# Our timing information.
robot_state_timing timing

# Whether or not safeties should be disabled. This will, in the future, be replaced w/ a GUI-controlled overrides system.
bool disableSafeties

robot_state_leg lLeg
robot_state_leg rLeg

float64 currentPositive
float64 currentNegative

robot_state_location position

uint8   boomMedullaState
float64 boomLogicVoltage
uint8   boomMedullaErrorFlags
