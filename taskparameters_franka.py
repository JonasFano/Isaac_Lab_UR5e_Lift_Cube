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
    joint_names = ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"]
    ee_body_name = "panda_hand"
    robot_base_init_position = (0.3, -0.1, 0.0)

    robot_reset_joints_pos_range = (0.9, 1.1)
    robot_reset_joints_vel_range = (0.0, 0.0)



    ###############
    ### Gripper ###
    ###############
    gripper_joint_names = ["panda_finger_joint1", "panda_finger_joint2"]
    gripper_body_names = ["panda_leftfinger", "panda_rightfinger"]
    gripper_open = 0.04
    gripper_close = 0.0
    gripper_offset = [0.0, 0.0, 0.107]

    # Randomize gripper finger friction
    gripper_static_friction_distribution_params = (1.4, 1.4)
    gripper_dynamic_friction_distribution_params = (1.4, 1.4)
    gripper_restitution_distribution_params = (0.1, 0.1)
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
    sample_range_pitch = (0.0, 0.0) # depends on end-effector axis
    sample_range_yaw = (-math.pi, math.pi)


    ##############
    ### Object ###
    ##############
    object_init_position = [0.04, 0.35, 0.055]
    object_init_rotation = [1, 0, 0, 0]
    object_scale = (0.8, 0.8, 0.8)

    object_randomize_pose_range_x = (-0.1, 0.1)
    object_randomize_pose_range_y = (-0.25, 0.25)
    object_randomize_pose_range_z = (0.0, 0.0)

    # Domain randomize object mass
    object_randomize_mass_range = (1.0, 1.0)
    object_randomize_mass_operation = "abs"
    object_randomize_mass_distribution = "uniform"
    object_randomize_recompute_inertia = True

    # Domain randomize object friction
    object_static_friction_distribution_params = (0.2, 0.2)
    object_dynamic_friction_distribution_params = (1.4, 1.4)
    object_restitution_distribution_params = (0.1, 0.1)
    object_randomize_friction_operation = "abs"
    object_randomize_friction_distribution = "uniform"
    object_randomize_friction_make_consistent = True # Ensure dynamic friction <= static friction

    min_height_object_dropping = -0.05