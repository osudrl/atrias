# Main Controller Options
uint8 main_controller
uint8 switch_method

# Gait Options
float64 swing_leg_retraction
float64 stance_leg_extension
float64 leg_length
float64 torso_angle
float64 q1
float64 q2
float64 q3
float64 q4

# Leg Gains
float64 leg_pos_kp
float64 leg_for_kp
float64 leg_for_ki
float64 leg_pos_kd
float64 leg_for_kd

# Hip Gains
float64 hip_pos_kp
float64 left_toe_pos
float64 hip_pos_kd
float64 right_toe_pos

# Torso options
float64 qvpp
float64 rvpp

# Debug
float64 current_limit
float64 velocity_limit
float64 deflection_limit
uint8 flight_to
uint8 flight_td
float64 walking_state
