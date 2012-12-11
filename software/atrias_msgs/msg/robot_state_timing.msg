# This message contains timing information. Mostly connector-specific

# This is the time sent to the controllers for this cycle.
uint64 controllerTime

# The time of reception and transmission
uint64 receiveDCTime
# This is delayed by one cycle
uint64 lastTransmitDCTime

# Timing variables from the timing loop
int32  overshoot

# This block is delayed by one cycle
int32  dcCorrection
int32  sleepTime
uint64 targetTime
