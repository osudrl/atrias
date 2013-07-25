# Main Controller Options
uint8 main_controller
uint8 switch_method

# Gait Options
float64 atrias_spring
float64 slip_leg
float64 stance_leg_target
float64 flight_leg_target
float64 swing_leg_retraction
float64 pushoff_force

# Event Trigger Options
float64 force_threshold_td
float64 force_threshold_to
float64 position_threshold_td

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

# Debug
uint8 flight_to
uint8 flight_td
float64 walking_state
float64 left_position
float64 right_position
float64 left_force
float64 right_force
