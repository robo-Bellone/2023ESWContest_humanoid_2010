
'''
communication_state
'''
state_comm = 0  #//0 com1 //1: com7 //2~: false comm

'''
action_state
'''
#action = 'marathon'
#action = 'curling'
#action = 'debug'
action = 'callib_cam'

'''
action_state
'''
action_opt = 'cam_on'
#action_opt = 'cam_off'


'''
camera_config
'''
cam_name = 'rapa'

cam_ratio = 2
cam_reso = 300

X_size = cam_reso*cam_ratio
Y_size = cam_reso

W_angle = 20
H_angle = 17


'''
robot_config
'''
Robot_height = 52
neck_UD = 24
neck_LR = 23
right_arm_UD = 13
left_arm_UD = 14
waist_LR = 12
right_ankle = 19

'''
motion_config
'''
CLAP         = "JK0001;"
WALK_FORWARD = "JK4000;"
WALK_SLIGHT  = "JK0002;"
THROW_STEADY = "JK0004;"
