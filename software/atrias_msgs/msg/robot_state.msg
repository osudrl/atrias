Header header

# This tells the controller what state RT Ops is in.
# Type: atrias::controllerManager::RtOpsCommand
uint8 cmState

uint8 robotConfiguration

robot_state_leg lLeg
robot_state_leg rLeg

robot_state_location position

uint8   boomMedullaState
float64 boomLogicVoltage
float64 boomMedullaErrorFlags
