# ur3e + robotiq140 robot config

ROBOT_CONFIG = {
    "gripper_dof_names": [
        "finger_joint", "right_outer_knuckle_joint",
        "left_inner_knuckle_joint", "right_inner_knuckle_joint",
        #"left_outer_finger_joint", "right_outer_finger_joint", 
        "left_inner_finger_joint", "right_inner_finger_joint",
             
    ],
    "gripper_open_position": 
        [-1/10, +1/10, +1/10, +1/10, -1/10, -1/10]
    ,
    "gripper_closed_position": 
        [1/2, -1/2, -1/2, -1/2, 1/2, 1/2]
    ,
}