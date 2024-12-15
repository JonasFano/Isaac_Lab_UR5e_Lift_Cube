from settings import Settings
import torch

class PreProcessing:
    @staticmethod
    def preprocess_xyz_poses(scene, robot):
        xyz_home = Settings.xyz_home.unsqueeze(0).repeat(scene.num_envs, 1) # Shape: [num_envs, 3]
        xyz_above_pick_up = Settings.xyz_above_pick_up.unsqueeze(0).repeat(scene.num_envs, 1) # Shape: [num_envs, 3]
        xyz_pick_up = Settings.xyz_pick_up.unsqueeze(0).repeat(scene.num_envs, 1) # Shape: [num_envs, 3]
        xyz_pick_up_close = Settings.xyz_pick_up_close.unsqueeze(0).repeat(scene.num_envs, 1) # Shape: [num_envs, 3]
        xyz_above_pick_up_close = Settings.xyz_above_pick_up_close.unsqueeze(0).repeat(scene.num_envs, 1) # Shape: [num_envs, 3]
        xyz_above_place_close = Settings.xyz_above_place_close.unsqueeze(0).repeat(scene.num_envs, 1) # Shape: [num_envs, 3]
        xyz_place_close = Settings.xyz_place_close.unsqueeze(0).repeat(scene.num_envs, 1) # Shape: [num_envs, 3]
        xyz_place = Settings.xyz_place.unsqueeze(0).repeat(scene.num_envs, 1) # Shape: [num_envs, 3]
        xyz_above_place = Settings.xyz_above_place.unsqueeze(0).repeat(scene.num_envs, 1) # Shape: [num_envs, 3]

        gripper_offset = torch.tensor([0, 0, 0.135], device="cuda")
        root_pose_w = robot.data.root_state_w

        xyz_home += scene.env_origins + gripper_offset - root_pose_w[:, :3]
        xyz_above_pick_up += scene.env_origins + gripper_offset - root_pose_w[:, :3]
        xyz_pick_up += scene.env_origins + gripper_offset - root_pose_w[:, :3]
        xyz_pick_up_close += scene.env_origins + gripper_offset - root_pose_w[:, :3]
        xyz_above_pick_up_close += scene.env_origins + gripper_offset - root_pose_w[:, :3]
        xyz_above_place_close += scene.env_origins + gripper_offset - root_pose_w[:, :3]
        xyz_place_close += scene.env_origins + gripper_offset - root_pose_w[:, :3]
        xyz_place += scene.env_origins + gripper_offset - root_pose_w[:, :3]
        xyz_above_place += scene.env_origins + gripper_offset - root_pose_w[:, :3]

        ee_quat_des = Settings.ee_quat_des.unsqueeze(0).repeat(scene.num_envs, 1) # Shape: [num_envs, 4] # for wrist_3_link


        xyz_home_pose = torch.cat([xyz_home, ee_quat_des], dim=1) # Shape: [num_envs, 7]
        xyz_above_pick_up_pose = torch.cat([xyz_above_pick_up, ee_quat_des], dim=1) # Shape: [num_envs, 7]
        xyz_pick_up_pose = torch.cat([xyz_pick_up, ee_quat_des], dim=1)  # Shape: [num_envs, 7]
        xyz_pick_up_close_pose = torch.cat([xyz_pick_up_close, ee_quat_des], dim=1)  # Shape: [num_envs, 7]
        xyz_above_pick_up_close_pose = torch.cat([xyz_above_pick_up_close, ee_quat_des], dim=1)  # Shape: [num_envs, 7]
        xyz_above_place_close_pose = torch.cat([xyz_above_place_close, ee_quat_des], dim=1)  # Shape: [num_envs, 7]
        xyz_place_close_pose = torch.cat([xyz_place_close, ee_quat_des], dim=1)  # Shape: [num_envs, 7]
        xyz_place_pose = torch.cat([xyz_place, ee_quat_des], dim=1)  # Shape: [num_envs, 7]
        xyz_above_place_pose = torch.cat([xyz_above_place, ee_quat_des], dim=1)  # Shape: [num_envs, 7]


        ctrl_inacc_correction = Settings.ctrl_inacc_correction.unsqueeze(0).repeat(scene.num_envs, 1) # [0.14619, 0.2452, 0.94]

        xyz_above_place_close_pose[:, :2] = ctrl_inacc_correction + scene.env_origins[:, :2] - root_pose_w[:, :2]
        xyz_place_close_pose[:, :2] = ctrl_inacc_correction + scene.env_origins[:, :2] - root_pose_w[:, :2]
        xyz_place_pose[:, :2] = ctrl_inacc_correction + scene.env_origins[:, :2] - root_pose_w[:, :2]
        xyz_above_place_pose[:, :2] = ctrl_inacc_correction + scene.env_origins[:, :2] - root_pose_w[:, :2]

        return xyz_home_pose, xyz_above_pick_up_pose, xyz_pick_up_pose, xyz_pick_up_close_pose, xyz_above_place_close_pose, xyz_above_pick_up_close_pose, xyz_place_close_pose, xyz_place_pose, xyz_above_place_pose