import os
import torch

class Settings:
    mode = "Joint_Control" # "Joint_Control" "Diff_IK_Control" "DMP"  # How to move the robot
    save_data = False # True False
    data_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")
    data_file_path = os.path.join(data_path, "demonstration_1") # File to which the joint positions are saved to
    dmp_params_file = os.path.join(data_path, "dmp_parameters_2")  # Path for saving DMP parameters
    model_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "scene_models")


    # For joint position or joint velocity control
    # X, Y, Z in world coordinates
    xyz_home = torch.tensor([0.05, 0.2, 0.3], device="cuda")
    xyz_above_pick_up = torch.tensor([0.04, 0.35, 0.16], device="cuda")
    xyz_pick_up = torch.tensor([0.04, 0.35, 0.095], device="cuda")
    xyz_pick_up_close = torch.tensor([0.04, 0.35, 0.095], device="cuda")
    xyz_above_pick_up_close = torch.tensor([0.04, 0.35, 0.16], device="cuda")
    xyz_above_place_close = torch.tensor([0.14619, 0.2452, 0.2], device="cuda")
    xyz_place_close = torch.tensor([0.14619, 0.2452, 0.105], device="cuda")
    xyz_place = torch.tensor([0.14619, 0.2452, 0.105], device="cuda")
    xyz_above_place = torch.tensor([0.14619, 0.2452, 0.2], device="cuda")

    # Quaternion (w, x, y, z)
    ee_quat_des = torch.tensor([0.0, 0.0, -1.0, 0.0], device="cuda")

    # Offset from end-effector to TCP
    gripper_offset = torch.tensor([0, 0, 0.135], device="cuda")

    # Corrects for non-linear cartesian movement of the controller
    ctrl_inacc_correction = torch.tensor([0.1456, 0.252], device="cuda")
    
    # Define gripper states
    gripper_open = torch.tensor([0.0, 0.0])
    # SDU gripper
    gripper_close = torch.tensor([0.02, 0.02])

    # Hand E
    # gripper_close = torch.tensor([-0.015, -0.015]) 

    # Joint positions
    joint_pos_above_pick_up = torch.cat((torch.tensor([1.57400844, -1.38136219,  1.87365737, -2.06385432, -1.57065705,  3.14481264]), gripper_open))
    joint_pos_pick_up = torch.cat((torch.tensor([1.57400297, -1.29256433,  1.95634084, -2.23534695, -1.57062928,  3.14480187]), gripper_open))
    joint_pos_pick_up_close = torch.cat((torch.tensor([1.57400297, -1.29256433,  1.95634084, -2.23534695, -1.57062928,  3.14480187]), gripper_close))
    joint_pos_above_pick_up_close = torch.cat((torch.tensor([1.57400844, -1.38136219,  1.87365737, -2.06385432, -1.57065705,  3.14481264]), gripper_close))
    joint_pos_above_place_close = torch.cat((torch.tensor([1.31718041, -1.67172325,  2.17931126, -2.07737736, -1.57061328,  2.88865622]), gripper_close))
    joint_pos_place_close = torch.cat((torch.tensor([1.317328,   -1.58043441,  2.26478794, -2.25573415, -1.57119171,  2.88805712]), gripper_close))
    joint_pos_place = torch.cat((torch.tensor([1.317328,   -1.58043441,  2.26478794, -2.25573415, -1.57119171,  2.88805712]), gripper_open))
    joint_pos_above_place = torch.cat((torch.tensor([1.31718041, -1.67172325,  2.17931126, -2.07737736, -1.57061328,  2.88865622]), gripper_open))
    

    # DMP
    # Time in seconds on when to start which Movement primitive after starting the movement at 0.0 seconds
    time_pick_up = 10.0
    time_place = 24.0

    
    