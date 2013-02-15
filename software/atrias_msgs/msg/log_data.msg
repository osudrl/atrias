# This is the main data type logged by our system at 1 kHz
# This is separate from robot_state so as to reduce the load on rtt-rosnode
# and rosbag

# These should correspond directly to robot_state values, unless
# indicated otherwise

float32 current50Amp
float32 current600Amp

int32 lKneeForce
int32 rKneeForce

float64 lALegAngle
float64 lBLegAngle
float64 rALegAngle
float64 rBLegAngle

float32 lALegVelocity
float32 lBLegVelocity
float32 rALegVelocity
float32 rBLegVelocity

float64 lAMotorAngle
float64 lBMotorAngle
float64 rAMotorAngle
float64 rBMotorAngle

float32 lAMotorVelocity
float32 lBMotorVelocity
float32 rAMotorVelocity
float32 rBMotorVelocity

float64 lARotorAngle
float64 lBRotorAngle
float64 rARotorAngle
float64 rBRotorAngle

float32 lARotorVelocity
float32 lBRotorVelocity
float32 rARotorVelocity
float32 rBRotorVelocity

float32 lLegBodyAngle
float32 rLegBodyAngle

float32 lLegBodyVelocity
float32 rLegBodyVelocity

float32 xPosition
float32 xVelocity
float32 zPosition
float32 zVelocity

uint16  lToeSwitch
uint16  rToeSwitch

uint8   rtOpsState

uint64  controllerTime

float32 lAMotorTherm0
float32 lAMotorTherm1
float32 lAMotorTherm2
float32 lAMotorTherm3
float32 lAMotorTherm4
float32 lAMotorTherm5
float32 lBMotorTherm0
float32 lBMotorTherm1
float32 lBMotorTherm2
float32 lBMotorTherm3
float32 lBMotorTherm4
float32 lBMotorTherm5
float32 rAMotorTherm0
float32 rAMotorTherm1
float32 rAMotorTherm2
float32 rAMotorTherm3
float32 rAMotorTherm4
float32 rAMotorTherm5
float32 rBMotorTherm0
float32 rBMotorTherm1
float32 rBMotorTherm2
float32 rBMotorTherm3
float32 rBMotorTherm4
float32 rBMotorTherm5

float32 lAMotorVoltage
float32 lBMotorVoltage
float32 lHipMotorVoltage
float32 rAMotorVoltage
float32 rBMotorVoltage
float32 rHipMotorVoltage

float32 lAMotorCurrent
float32 lBMotorCurrent
float32 rAMotorCurrent
float32 rBMotorCurrent
