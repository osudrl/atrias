float64 maxPos
float64 minPos
float64 desVel
float64 Kp

# 0 for the motor encoder, 1 for the rotor encoder. Only affects velocity feedback, not position.
int8    sensor

# 0 for absolute encoder, 1 for incremental, 2 for no relay, 3 for reverse (no relay)
int8    relayMode
