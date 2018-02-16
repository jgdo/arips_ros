uint32 M_IDLE = 0 # motors are turned off
uint32 M_BREAK = 1 # motors are short-cut
uint32 M_HOLD = 2 # motors should hold position
uint32 M_RAW = 3 # raw motor voltage mode
uint32 M_TRAJECTORY = 4 # trajectory (setpoint sequence)

uint32 mode

time stamp

JointState[5] jointStates
TrajectoryState trajState

