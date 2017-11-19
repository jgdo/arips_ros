uint32 M_IDLE = 0 # motors are turned off
uint32 M_BREAK = 1 # motors are short-cut
uint32 M_HOLD = 2 # motors should hold position
uint32 M_SINGLE_GOAL = 3 # moving to a final goal
uint32 M_TRAJECTORY = 4 # trajectory (setpoint sequence)

uint32 mode

time stamp

JointState[1] jointStates
TrajectoryState trajState

