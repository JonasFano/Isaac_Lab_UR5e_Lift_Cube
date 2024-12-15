"""
Controls various aspects of the robot controller, including joint positions, gripper movements, 
and trajectory execution. Manages the robot state and coordinates actions using utility functions.
"""
import numpy as np
import time
import torch
from settings import Settings
from utils.preprocessing import PreProcessing
from controller.diff_IK_ctrl import Diff_IK_Ctrl

class Controller:
    def __init__(self, sim, scene, robot):
        """
        Initializes the controller with the robot's state, joint positions, and necessary components 
        for controlling movement and the gripper.
        """
        self.default_root_state = scene["object"].data.default_root_state
        self.default_root_state[:, :3] += scene.env_origins
        self.count = 0
        self.dmp_nr = 1
        self.start_time = time.time()
        self.joint_pos_des = robot.data.default_joint_pos.clone()

        self.diff_ik_controller = Diff_IK_Ctrl(sim, scene)
        self.xyz_home_pose, self.xyz_above_pick_up_pose, self.xyz_pick_up_pose, self.xyz_pick_up_close_pose, self.xyz_above_place_close_pose, self.xyz_above_pick_up_close_pose, self.xyz_place_close_pose, self.xyz_place_pose, self.xyz_above_place_pose = PreProcessing.preprocess_xyz_poses(scene, robot)
        self.ee_pose_b_des = self.xyz_home_pose
        self.gripper_open = True


    def run_joint_position_control(self, robot, env):
        """
        Runs position control for joint movements based on the current state of the controller.
        
        Returns:
        - state (int): Updated state after movement.
        """
        terminate = False
        if self.count == 0:
            self.dmp_nr = 1
            self.gripper_open = True
            self.joint_pos_des, joint_vel_des = robot.data.default_joint_pos.clone(), robot.data.default_joint_vel.clone()
            robot.write_joint_state_to_sim(self.joint_pos_des, joint_vel_des)
        elif self.count == 1:
            self.joint_pos_des = Settings.joint_pos_above_pick_up.unsqueeze(0).repeat(env.unwrapped.scene.num_envs, 1)
            self.dmp_nr = 1
            self.gripper_open = True
        elif self.count == 100:
            self.joint_pos_des = Settings.joint_pos_pick_up.unsqueeze(0).repeat(env.unwrapped.scene.num_envs, 1)
            self.dmp_nr = 1
            self.gripper_open = True
        elif self.count == 150:
            self.joint_pos_des = Settings.joint_pos_pick_up_close.unsqueeze(0).repeat(env.unwrapped.scene.num_envs, 1)
            self.dmp_nr = 1
            self.gripper_open = False
        elif self.count == 200:
            self.joint_pos_des = Settings.joint_pos_above_pick_up_close.unsqueeze(0).repeat(env.unwrapped.scene.num_envs, 1)
            self.dmp_nr = 2
            self.gripper_open = False
        elif self.count == 250:
            self.joint_pos_des = Settings.joint_pos_above_place_close.unsqueeze(0).repeat(env.unwrapped.scene.num_envs, 1)
            self.dmp_nr = 2
            self.gripper_open = False
        elif self.count == 350:
            self.joint_pos_des = Settings.joint_pos_place_close.unsqueeze(0).repeat(env.unwrapped.scene.num_envs, 1)
            self.dmp_nr = 2
            self.gripper_open = False
        elif self.count == 400:
            self.joint_pos_des = Settings.joint_pos_place.unsqueeze(0).repeat(env.unwrapped.scene.num_envs, 1)
            self.dmp_nr = 3
            self.gripper_open = True
        elif self.count == 450:
            self.joint_pos_des = Settings.joint_pos_above_place.unsqueeze(0).repeat(env.unwrapped.scene.num_envs, 1)
            self.dmp_nr = 3
            self.gripper_open = True
        
        # Prepare the action for the environment
        arm_action = self.joint_pos_des[:, :6]


        gripper_action = torch.zeros((env.unwrapped.scene.num_envs, 1), device="cuda")  # 1 column for the gripper action
        if self.gripper_open:
            gripper_action.fill_(1)  # If gripper is open, set to 1 for "open"

        # Combine arm and gripper actions
        action = torch.cat((arm_action, gripper_action), dim=1)  # Concatenate along the appropriate dimension

        # Check termination condition
        if self.count % 500 == 0 and self.count is not 0:
            self.count = 0
            terminate = True
            env.unwrapped.scene["object"].write_root_pose_to_sim(self.default_root_state[:, :7])
            env.reset()

        self.count += 1
        return self.dmp_nr, robot, env, terminate, action
    

    def run_diff_IK_ctrl(self, robot, scene):
        # Get current end-effector pose in world frame and robot base pose in world frame
        ee_pose_w = robot.data.body_state_w[:, self.diff_ik_controller.robot_entity_cfg.body_ids[0], 0:7]

        terminate = False
        if self.count == 0:
            self.dmp_nr = 1
            self.ee_pose_b_des = self.xyz_home_pose
            self.gripper_open = True
        elif self.count == 1:
            self.dmp_nr = 1
            self.ee_pose_b_des = self.xyz_above_pick_up_pose
            self.gripper_open = True
        elif self.count == 100:
            self.dmp_nr = 1
            self.ee_pose_b_des = self.xyz_pick_up_pose
            self.gripper_open = True
        elif self.count == 150:
            self.dmp_nr = 1
            self.ee_pose_b_des = self.xyz_pick_up_close_pose
            self.gripper_open = False
        elif self.count == 225:
            self.dmp_nr = 2
            self.ee_pose_b_des = self.xyz_above_pick_up_close_pose
            self.gripper_open = False
        elif self.count == 300:
            self.dmp_nr = 2
            self.ee_pose_b_des = self.xyz_above_place_close_pose
            self.gripper_open = False
        elif self.count == 450:
            self.dmp_nr = 2
            self.ee_pose_b_des = self.xyz_place_close_pose
            self.gripper_open = False
        elif self.count == 525:
            self.dmp_nr = 3
            self.ee_pose_b_des = self.xyz_place_pose
            self.gripper_open = True
        elif self.count == 600:
            self.dmp_nr = 3
            self.ee_pose_b_des = self.xyz_above_place_pose
            self.gripper_open = True
        elif self.count % 650 == 0:
            self.count = 0
            self.ee_pose_b_des = self.xyz_home_pose
            self.gripper_open = True
            self.dmp_nr = 1
            terminate = True

            scene["peg"].write_root_pose_to_sim(self.default_root_state[: , :7])
            scene.reset()

        joint_pos_des = self.diff_ik_controller.run_diff_ik_ctrl(robot, robot.data.root_state_w, ee_pose_w, self.ee_pose_b_des, self.gripper_open)
        robot.set_joint_position_target(joint_pos_des)

        self.count += 1
        return self.dmp_nr, scene, robot, terminate
    

    def run_dmp(self, robot, scene, q_out_pick_up, q_out_place, q_out_home, ts_pick_up, ts_place, ts_home):
        """
        Executes motion using Dynamic Movement Primitives (DMPs) by evaluating joint positions at specific timestamps.

        Parameters:
        - q_out_pick_up: Array of joint positions for picking up an object.
        - q_out_place: Array of joint positions for placing an object.
        - q_out_home: Array of joint positions for returning to home position.
        - ts_pick_up: Time sequence corresponding to the pick-up trajectory.
        - ts_place: Time sequence corresponding to the place trajectory.
        - ts_home: Time sequence corresponding to the home trajectory.
        """
        # Get the elapsed time
        current_time = time.time() - self.start_time

        if current_time < Settings.time_pick_up:
            current_index = np.searchsorted(ts_pick_up, current_time)  # Find the index corresponding to the current time
            joint_position = q_out_pick_up[current_index]  # Get the joint positions at that time
        elif current_time < Settings.time_place:
            current_index = np.searchsorted(ts_place, current_time - Settings.time_pick_up)  # Find the index corresponding to the current time
            joint_position = q_out_place[current_index]  # Get the joint positions at that time
        elif current_time > Settings.time_place:
            current_index = np.searchsorted(ts_home, current_time - Settings.time_place)  # Find the index corresponding to the current time
            joint_position = q_out_home[current_index]  # Get the joint positions at that time

        joint_position_tensor = torch.tensor(joint_position).unsqueeze(0).repeat(scene.num_envs, 1)

        # Send the evaluated joint positions to the robot
        robot.set_joint_position_target(joint_position_tensor)
        return robot

