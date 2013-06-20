# Main Controller Options
uint8 main_controller
uint8 gait_transitions
float64 pushoff_force
bool apply_hip_torque
float64 hip_torque

# Gait Options
float64 atrias_spring
float64 slip_leg
float64 stance_leg_target
float64 flight_leg_target

# Trajectory Options
float64 swing_leg_retraction

# Event Trigger Options
float64 force_threshold_td
float64 force_threshold_to
float64 position_threshold_td

# Leg Gains
float64 leg_pos_kp
float64 leg_for_kp
float64 leg_pos_kd
float64 leg_for_kd

# Hip Gains
float64 hip_pos_kp
float64 hip_pos_kd
float64 left_toe_pos
float64 right_toe_pos

# Debug
uint8 flight_to
uint8 flight_td
float64 walking_state
float64 td_force
float64 to_force
float64 td_position
