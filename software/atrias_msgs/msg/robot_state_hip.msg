uint8   medullaState
uint8   errorFlags
# The raw limit switch values. These are latched on until a reset is commanded.
uint8   limitSwitches

float64 legBodyAngle
float64 legBodyVelocity
# motorVelocity is not used -- here so the GUI compiles.
float64 motorVelocity
# This is the position according to the absolute encoder.
float64 absoluteBodyAngle
float64 motorCurrent

# Limit switches
bool    InsideLimitSwitch
bool    OutsideLimitSwitch

# Hip thermistors
float64 motorThermA
float64 motorThermB
float64 motorThermC

# Begin IMU data.
# Accelerations
float64 accelX
float64 accelY
float64 accelZ

# Angular rates
float64 angRateX
float64 angRateY
float64 angRateZ

# A matrix... not sure exactly what's in here (DCM?)
float64 m11
float64 m12
float64 m13
float64 m21
float64 m22
float64 m23
float64 m31
float64 m32
float64 m33

# Some sort of timer... not sure what's here yet.
int32   IMUTimer

# End IMU data.

# Voltages
float64 logicVoltage
float64 motorVoltage
