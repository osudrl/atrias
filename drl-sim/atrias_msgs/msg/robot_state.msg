uint8 command
uint16 loopTime

# Body
float32 body_angle
float32 body_angle_vel
float32 body_pitch
float32 body_pitch_vel

# Leg A
uint8 limit_switch_A
float32 leg_angleA
float32 leg_velocityA
float32 motor_angleA
float32 motor_angleA_inc
float32 motor_torqueA
float32 motor_velocityA
float32 motorVoltageA
uint8 motor_currentA
float32 desired_motor_angleA
float32 desired_spring_defA
float32 logicVoltageA

# Leg B
uint8 limit_switch_B
float32 leg_angleB
float32 leg_velocityB
float32 motor_angleB
float32 motor_angleB_inc
float32 motor_torqueB
float32 motor_velocityB
float32 motorVoltageB
uint8 motor_currentB
float32 desired_motor_angleB
float32 desired_spring_defB
float32 logicVoltageB

# Hip
float32 motor_angle_hip
float32 motor_torque_hip
float32 motor_velocity_hip

# Positional
float32 xPosition
float32 yPosition
float32 zPosition
float32 xVelocity
float32 yVelocity
float32 zVelocity
float32 horizontal_velocity

# Thermistors
float32[3] thermistorA
float32[3] thermistorB

uint8 medullaStatusA
uint8 medullaStatusB
uint8 medullaStatusHip
uint8 medullaStatusBoom
uint8 limitSwitchA
uint8 limitSwitchB
uint8 limitSwitchHip

uint8 toe_switch


### DEBUG DATA ###

# Test controller
float32 phase
bool in_flight

float32 des_leg_angle
float32 des_leg_length
float32 des_hip_angle

float32 flightHipP
float32 flightHipD
float32 flightKneeP
float32 flightKneeD

float32 stanceHipP
float32 stanceHipD
float32 stanceKneeP
float32 stanceKneeD

float32 stance_time

