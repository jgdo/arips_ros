uint8 CMD_RELEASE = 5	# release motors
uint8 CMD_BREAK = 10	# break (shortcut) motors
uint8 CMD_HOLD = 15		# hold current position
uint8 CMD_RAW_MOTORS = 20		# raw motor voltage control mode, no joint limit or other checks!
uint8 CMD_DIRECT_JOINTS = 21   # direct joint power mode, joint limits are checked
uint8 CMD_DIRECT_CONTROLLER = 22  # apply controller on direct setpoints without trajectory, joint limits checked
uint8 CMD_START_TRAJECTORY = 30	# start a new trajectory

uint8 command

float32 st_position

