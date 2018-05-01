uint32 M_IDLE = 0 # motors are turned off
uint32 M_BREAK = 10 # motors are short-cut
uint32 M_HOLD = 15 # motors should hold position
uint32 M_RAW_MOTORS = 20 # raw motor voltage mode
uint32 M_DIRECT_JOINTS = 21 # direct joint effort control
uint32 M_DIRECT_CONTROLLER = 22 # apply controller directly on raw angle setpoints
uint32 M_TRAJECTORY = 30 # trajectory (setpoint sequence)

uint32 mode

time stamp

JointState[6] jointStates
TrajectoryState trajState

float32 controlLoopTime

