float64 xPosition
float64 yPosition
float64 zPosition

float64 xVelocity
float64 yVelocity
float64 zVelocity

# The angle of the boom. A horizontal boom has an angle of pi
float64 boomAngle
float64 boomAngleVelocity

# Angle of the boom (relative to startup) horizontally
# This is related to xPosition -- it derives from the same encoder
float64 xAngle
float64 xAngleVelocity

# The robot's body's pitch. 3 * pi/2 is vertical.
float64 bodyPitch

# How fast we are pitching
float64 bodyPitchVelocity

# Raw encoder counts
int32   zEncoderRaw
