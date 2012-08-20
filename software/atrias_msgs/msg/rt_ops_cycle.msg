# This message contains all data belonging to a single RT Ops logging cycle.
Header header

# The robot state from this cycle:
robot_state       robotState

# The time this cycle began (nanoseconds)
int64             startTime

# The time this cycle ended (nanoseconds)
int64             endTime

# The controller-demanded outputs
controller_output controllerOutput

# The clamped currents and actual output demanded state. (Does not include disabled-zeroing)
controller_output commandedOutput
