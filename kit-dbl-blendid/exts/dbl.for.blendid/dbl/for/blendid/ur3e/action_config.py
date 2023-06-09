# task config for making coffee

action_config = {
    "go_home": {
        'base_prim': None,
        'steps':[
            {
                'action_type': 'move',
                'duration': 200,
                'position': [-0.2, 0.0, 0.6],
                'orientation': [-0.7071068, 0, 0, -0.7071068], # wxyz
            }
        ]
    },

    "pick_up_blender": {
        'base_prim': '/World/blender',
        'steps':[
            {
                'action_type': 'move',
                'duration': 200,
                'position': [0, -0.2, 0.3],
                'orientation': [-0.5, 0.5, 0.5, 0.5],
            },
            {
                'action_type': 'open',
                'duration': 60,
                'ratio': None,
            },
            {
                'action_type': 'move',
                'duration': 100,
                'position': [0, -0.2, 0.1],
                'orientation': [-0.5, 0.5, 0.5, 0.5],
            },
            {
                'action_type': 'close',
                'duration': 60,
                'ratio': 0.4,
            },
            # {
            #     'action_type': 'move',
            #     'duration': 100,
            #     'position': [0, -0.3, 0.15],
            #     'orientation': [-0.5, 0.5, 0.5, 0.5],
            # },           
        ]
    },

    "place_blender_to_point": {
        'base_prim': '/World/WorkingArea/BlendingArea/BlendingPoint0',
        'steps':[
            {
                'action_type': 'move',
                'duration': 200,
                'position': [0, -0.3, 0.1],
                'orientation': [-0.5, 0.5, 0.5, 0.5],
            },   
            {
                'action_type': 'move',
                'duration': 200,
                'position': [0, -0.1, 0.1],
                'orientation': [-0.5, 0.5, 0.5, 0.5],
            },   
            {
                'action_type': 'open',
                'duration': 60,
                'ratio': None,
            },
            {
                'action_type': 'move',
                'duration': 200,
                'position': [0, -0.3, 0.1],
                'orientation': [-0.5, 0.5, 0.5, 0.5],
            },   
            {
                'action_type': 'wait',
                'duration': 100,
            },
        ]
    },

    "pick_up_cup": {
        'base_prim': '/World/Cup/cup_00',
        'steps':[
            {
                'action_type': 'move',
                'duration': 200,
                'position': [0, -0.3, 0.05],
                'orientation': [-0.5, 0.5, 0.5, 0.5],
            },   
            {
                'action_type': 'move',
                'duration': 100,
                'position': [0, -0.2, 0.05],
                'orientation': [-0.5, 0.5, 0.5, 0.5],
            },   
            {
                'action_type': 'close',
                'duration': 60,
                'ratio': 0.8,
            },
            {
                'action_type': 'move',
                'duration': 100,
                'position': [0, -0.2, 0.0],
                'orientation': [-0.5, 0.5, 0.5, 0.5],
            },  
            {
                'action_type': 'move',
                'duration': 50,
                'position': [0, -0.4, 0.0],
                'orientation': [-0.5, 0.5, 0.5, 0.5],
            },  
            {
                'action_type': 'move',
                'duration': 100,
                'position': [0, -0.4, -0.2],
                'orientation': [-0.5, 0.5, 0.5, 0.5],
            },  
        ]
    },

    "place_cup": {
        'base_prim': '/World/WorkingArea/FrontAreaTop/CupPoint0',
        'steps':[
            {
                'action_type': 'move',
                'duration': 200,
                'position': [0, -0.4, 0.2],
                'orientation': [-0.5, 0.5, -0.5, -0.5],
            },   
            {
                'action_type': 'move',
                'duration': 200,
                'position': [0, -0.4, 0.2],
                'orientation': [-0.5, 0.5, 0.5, 0.5],
            },   
            {
                'action_type': 'move',
                'duration': 100,
                'position': [0, -0.2, 0.08],
                'orientation': [-0.5, 0.5, 0.5, 0.5],
            },   
            {
                'action_type': 'open',
                'duration': 60,
                'ratio': None,
            },   
            
        ]
    },

    "pour": {
        'base_prim': '/World/Cup/cup_01',
        'steps':[
            {
                'action_type': 'move',
                'duration': 100,
                'position': [0.1, 0, 0],
                'orientation': [-0.5, 0.5, 0.5, 0.5],
            },
            {
                'action_type': 'slerp',
                'duration': 200,
                'sub_steps': 10,
                'position': [0.1, 0, 0],
                'orientation': [-0.5, 0.5, 0.5, 0.5],
                'relative_rotation': [0.5, 0, -0.8660254, 0],
                'slerp_last': False,
                'slerp_offset': [0, 0, 0]
            },
        ]
    },

############################################################################################
    "press_coffee_machine_button": {
        'base_prim': '/World/Keurig_1_5_add_hold/XformHandle',
        'steps':[
            {
                'action_type': 'close', #open
                'duration': 30,
                'ratio': 1.1,
            },
            {
                'action_type': 'move',
                'duration': 100,
                'position': [0, -0.09, 0.2],
                'orientation': [0, 0.7071, 0.7071, 0],
            },
            {
                'action_type': 'move',
                'duration': 30,
                'position': [-0, -0.09, 0.18],
                'orientation': [0, 0.7071, 0.7071, 0],
            },
            {
                'action_type': 'move',
                'duration': 50,
                'position': [-0, -0.09, 0.2],
                'orientation': [0, 0.7071, 0.7071, 0],
            },
            {
                'action_type': 'move',
                'duration': 30,
                'position': [-0, -0.3, 0.2],
                'orientation': [0, 0.7071, 0.7071, 0],
            },
            
        ]
    },

    "pick_up_capsule": {
        'base_prim': '/World/k_cup',
        'steps':[
            {
                'action_type': 'move',
                'duration': 300,
                'position': [-0.12, 0.0, 0.3],
                'orientation': [0.5, 0.5, 0.5, 0.5],
            },
            {
                'action_type': 'move',
                'duration': 60,
                'position': [-0.12, 0.0, 0.1],
                'orientation': [0.5, 0.5, 0.5, 0.5],
            },
            {
                'action_type': 'move',
                'duration': 60,
                'position': [-0.12, 0.0, 0.03],
                'orientation': [0.5, 0.5, 0.5, 0.5],
            },
            {
                'action_type': 'close',
                'duration': 30,
                'ratio': 0.6,
            },
            {
                'action_type': 'move',
                'duration': 60,
                'position': [-0.12, 0.0, 0.3],
                'orientation': [0.5, 0.5, 0.5, 0.5],
            },
        ]
    },

    "pick_up_papercup": {
        'base_prim': '/World/papercup',
        'steps':[
            {
                'action_type': 'move',
                'duration': 300,
                'position': [-0.15, 0.0, 0.3],
                'orientation': [0.5, 0.5, 0.5, 0.5],
            },
            {
                'action_type': 'move',
                'duration': 100,
                'position': [-0.15, 0.0, 0.1],
                'orientation': [0.5, 0.5, 0.5, 0.5],
            },
            {
                'action_type': 'move',
                'duration': 100,
                'position': [-0.15, 0.0, 0.00],
                'orientation': [0.5, 0.5, 0.5, 0.5],
            },
            {
                'action_type': 'close',
                'duration': 60,
                'ratio': 0.4,
            },
            {
                'action_type': 'move',
                'duration': 100,
                'position': [-0.15, 0.0, 0.3],
                'orientation': [0.5, 0.5, 0.5, 0.5],
            },
        ]
    },
    
    "move_capsule_to_coffee_machine": {
        'base_prim': '/World/Keurig_1_5_add_hold/XformHandle',
        'steps':[
            {
                'action_type': 'move',
                'duration': 100,
                'position': [0, -0.3, 0.02],
                'orientation': [0, 0, 0.7071, 0.7071],
            },
            {
                'action_type': 'move',
                'duration': 60,
                'position': [0, -0.218, 0.02],
                'orientation':  [0, 0, 0.7071, 0.7071],
            },     
            {
                'action_type': 'close', #open
                'duration': 60,
                'ratio': 0.05,
            },      
            {
                'action_type': 'move',
                'duration': 100,
                'position': [0, -0.3, 0.025],
                'orientation': [0, 0, 0.7071, 0.7071],
            },
            {
                'action_type': 'move',
                'duration': 100,
                'position': [0, -0.2, 0.4],
                'orientation': [0, 0, 0.7071, 0.7071],
            },
        ]
    },


    "move_papercup_to_coffee_machine": {
        'base_prim': '/World/Keurig_1_5_add_hold/XformHandle',
        'steps':[
            {
                'action_type': 'move',
                'duration': 200,
                'position': [0, -0.4, -0.2],
                'orientation': [0, 0, 0.7071, 0.7071],
            },
            {
                'action_type': 'move',
                'duration': 100,
                'position': [0, -0.25, -0.2],
                'orientation':  [0, 0, 0.7071, 0.7071],
            },     
            {
                'action_type': 'close', #open
                'duration': 100,
                'ratio': 0.05,
            },      
            {
                'action_type': 'move',
                'duration': 100,
                'position': [0, -0.4,  -0.2],
                'orientation': [0, 0, 0.7071, 0.7071],
            },
            {
                'action_type': 'move',
                'duration': 100,
                'position': [0, -0.4, 0.4],
                'orientation': [0, 0, 0.7071, 0.7071],
            },
        ]
    },
    
    ################### low level test #############################
    "low_level_test": {
        "base_prim": None,
        "steps": [
            {
                "action_type": "low_level",
                "duration": 200,
                "joint_positions": [0.2] * 7,
            }
        ]
    }
}