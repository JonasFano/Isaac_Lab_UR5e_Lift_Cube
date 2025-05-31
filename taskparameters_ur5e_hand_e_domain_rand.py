import math

class TaskParams:
    #################################
    ### General Simulation Params ###
    #################################
    decimation = 2
    episode_length_s = 15.0
    dt = 1/100
    render_interval = 2


    #######################
    ### Differential IK ###
    #######################
    command_type = "pose"
    use_relative_mode = True
    ik_method = "dls"
    action_scale = 0.05


    ##############
    ### Reward ###
    ##############
    reaching_object_weight = 1.0
    reaching_object_std = 0.1

    lift_min_height = 0.05 
    lift_weight = 35.0

    object_goal_tracking_coarse_std = 0.3
    object_goal_tracking_coarse_weight = 16.0

    object_goal_tracking_fine_grained_std = 0.05
    object_goal_tracking_fine_grained_weight = 5.0

    end_effector_orientation_tracking_weight = -6.0

    action_rate_weight = -1e-4
    action_rate_curriculum_weight = -0.1
    curriculum_num_steps = 10000


    #############
    ### Robot ###
    #############
    # Robot parameters/gains
    joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
    ee_body_name = "wrist_3_link"
    robot_base_init_position = (0.3, -0.1, 0.0)

    robot_vel_limit = 180.0
    robot_effort_limit = 87.0
    robot_stiffness = 1000.0

    shoulder_pan_mass = 3.761
    shoulder_lift_mass = 8.058
    elbow_mass = 2.846
    wrist_1_mass = 1.37
    wrist_2_mass = 1.3
    wrist_3_mass = 0.365

    # Critically damped damping 
    shoulder_pan_damping = 2 * math.sqrt(robot_stiffness * shoulder_pan_mass) 
    shoulder_lift_damping = 2 * math.sqrt(robot_stiffness * shoulder_lift_mass)
    elbow_damping = 2 * math.sqrt(robot_stiffness * elbow_mass)
    wrist_1_damping = 2 * math.sqrt(robot_stiffness * wrist_1_mass)
    wrist_2_damping = 2 * math.sqrt(robot_stiffness * wrist_2_mass)
    wrist_3_damping = 2 * math.sqrt(robot_stiffness * wrist_3_mass)
    
    # Domain randomize robot stiffness and damping
    robot_randomize_stiffness = (0.7, 1.3)
    robot_randomize_damping = (0.7, 1.3)
    robot_randomize_stiffness_operation = "scale"
    robot_randomize_damping_operation = "scale"
    robot_randomize_stiffness_distribution = "uniform"
    robot_randomize_damping_distribution = "uniform"

    robot_initial_joint_pos = [1.3, -2.0, 2.0, -1.5, -1.5, 0.0, 0.0, 0.0] # With gripper joint pos set to 0.0
    robot_reset_joints_pos_range = (0.9, 1.1)
    robot_reset_joints_vel_range = (0.0, 0.0)



    ###############
    ### Gripper ###
    ###############
    gripper_joint_names = ["joint_left", "joint_right"]
    gripper_body_names = ["finger_left", "finger_right"]
    gripper_open = [0.0, 0.0]
    gripper_close = [-0.025, -0.025]
    gripper_offset = [0.0, 0.0, 0.135]

    gripper_vel_limit = 1000000.0
    gripper_effort_limit = 200.0
    gripper_stiffness = 10000000.0
    gripper_damping = 50000.0 # slightly overdamped assuming gripper mass of 1.07 g

    # Domain randomize gripper stiffness and damping
    gripper_randomize_stiffness = (0.75, 1.05)
    gripper_randomize_damping = (0.75, 1.05)
    gripper_randomize_stiffness_operation = "scale"
    gripper_randomize_damping_operation = "scale"
    gripper_randomize_stiffness_distribution = "uniform"
    gripper_randomize_damping_distribution = "uniform"

    # Randomize gripper finger friction
    gripper_static_friction_distribution_params = (0.8, 1.2)
    gripper_dynamic_friction_distribution_params = (0.6, 1.2)
    gripper_restitution_distribution_params = (0.0, 0.3)
    gripper_randomize_friction_operation = "abs"
    gripper_randomize_friction_distribution = "uniform"
    gripper_randomize_friction_make_consistent = True # Ensure dynamic friction <= static friction


    ###############
    ### Command ###
    ###############
    resampling_time_range = (5.0, 5.0)
    visualize_frame = True

    sample_range_pos_x = (0.25, 0.35)
    sample_range_pos_y = (0.3, 0.4)
    sample_range_pos_z = (0.25, 0.35)
    sample_range_roll = (0.0, 0.0)
    sample_range_pitch = (math.pi, math.pi) # depends on end-effector axis
    sample_range_yaw = (-math.pi, math.pi)


    ##############
    ### Object ###
    ##############
    object_init_position = [0.04, 0.35, 0.055]
    object_init_rotation = [1, 0, 0, 0]
    object_scale = (0.4, 0.4, 0.4)

    object_randomize_pose_range_x = (-0.1, 0.1)
    object_randomize_pose_range_y = (-0.25, 0.25)
    object_randomize_pose_range_z = (0.0, 0.0)

    # Domain randomize object mass
    object_randomize_mass_range = (0.5, 0.5)
    object_randomize_mass_operation = "abs"
    object_randomize_mass_distribution = "uniform"
    object_randomize_recompute_inertia = True

    # Domain randomize object friction
    object_static_friction_distribution_params = (0.6, 1.0)
    object_dynamic_friction_distribution_params = (0.4, 1.0)
    object_restitution_distribution_params = (0.0, 0.3)
    object_randomize_friction_operation = "abs"
    object_randomize_friction_distribution = "uniform"
    object_randomize_friction_make_consistent = True # Ensure dynamic friction <= static friction

    min_height_object_dropping = -0.05