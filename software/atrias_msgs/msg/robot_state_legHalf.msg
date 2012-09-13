Header header

uint8   medullaState
uint8   errorFlags
# The raw limit switch values. Clamped on until a reset is commanded.
uint8   limitSwitches

float64 legAngle
float64 legVelocity
float64 motorAngle
float64 motorVelocity

float64 rotorAngle
float64 rotorVelocity

# The total current for both amps combined.
float64 motorCurrent

# These sum to the value in motorCurrent
float64 amp1Current
float64 amp2Current

# Limit switches
bool    negDeflectSwitch
bool    posDeflectSwitch
bool    motorNegLimitSwitch
bool    motorPosLimitSwitch

# Thermistors
float64[6] motorTherms

# Voltages
float64 logicVoltage
float64 motorVoltage
