from math import radians

servo_param = {
    1: {'name': 'head_pan_joint', 
        'home_encoder': 512,
        'max_speed': radians(180),
        'max_ang': radians(145.),
        'min_ang': radians(-145.)
       }, 
    2: {'name': 'head_tilt_joint',
        'home_encoder': 512,
        'max_speed': radians(180),
        'max_ang': radians(90.),
        'min_ang': radians(-90.)
       }, 
    3: {'name': 'right_shoulder_pan_joint',
        'home_encoder': 512,
        'max_speed': radians(180),
        'max_ang': radians(145.),
        'min_ang': radians(-145.)
       }, 
    4: {'name': 'right_shoulder_lift_joint',
        'home_encoder': 512,
        'max_speed': radians(180),
        'max_ang': radians(145.),
        'min_ang': radians(-145.)
       }, 
    5: {'name': 'right_arm_roll_joint',
        'home_encoder': 512,
        'max_speed': radians(180),
        'max_ang': radians(145.),
        'min_ang': radians(-145.)
       }, 
    6: {'name': 'right_wrist_joint',
        'home_encoder': 512,
        'max_speed': radians(180),
        'max_ang': radians(145.),
        'min_ang': radians(-145.)
       }, 
    7: {'name': 'left_shoulder_pan_joint',
        'home_encoder': 512,
        'max_speed': radians(180),
        'max_ang': radians(145.),
        'min_ang': radians(-145.)
       }, 
    8: {'name': 'left_shoulder_lift_joint',
        'home_encoder': 512,
        'max_speed': radians(180),
        'max_ang': radians(145.),
        'min_ang': radians(-145.)
       }, 
    9: {'name': 'left_arm_roll_joint',
        'home_encoder': 512,
        'max_speed': radians(180),
        'max_ang': radians(145.),
        'min_ang': radians(-145.)
       }, 
    10: {'name': 'left_wrist_joint',
        'home_encoder': 512,
        'max_speed': radians(180),
        'max_ang': radians(145.),
        'min_ang': radians(-145.)
       }, 
    11: {'name': 'torso_joint',
        'home_encoder': 512,
        'max_speed': radians(180),
        'max_ang': radians(145.),
        'min_ang': radians(-145.)
       }, 
    12: {'name': 'left_elbow_joint',
        'home_encoder': 512,
        'max_speed': radians(180),
        'max_ang': radians(145.),
        'min_ang': radians(-145.)
       }, 
    13: {'name': 'right_elbow_joint',
        'home_encoder': 512,
        'max_speed': radians(180),
        'max_ang': radians(145.),
        'min_ang': radians(-145.)
       }
}

