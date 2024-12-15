import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.scene import InteractiveScene
from omni.isaac.lab.controllers import DifferentialIKControllerCfg, DifferentialIKController
from omni.isaac.lab.managers import SceneEntityCfg
from omni.isaac.lab.utils.math import subtract_frame_transforms
import torch


class Diff_IK_Ctrl:
    def __init__(self, sim: sim_utils.SimulationContext, scene: InteractiveScene):
        self.sim = sim
        self.scene = scene
        robot = scene["robot"]

        # Create controller
        self.diff_ik_cfg = DifferentialIKControllerCfg(command_type="pose", use_relative_mode=False, ik_method="trans")
        self.diff_ik_controller = DifferentialIKController(self.diff_ik_cfg, num_envs=self.scene.num_envs, device=self.sim.device)

        # Create robot entity to keep track of the joint and body ids
        self.robot_entity_cfg = SceneEntityCfg("robot", joint_names=["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"], body_names=["wrist_3_link"])
        self.robot_entity_cfg.resolve(self.scene)

        # For a fixed base robot, the frame index is one less than the body index. This is because
        # the root body is not included in the returned Jacobians.
        if robot.is_fixed_base:
            self.ee_jacobi_idx = self.robot_entity_cfg.body_ids[0] - 1
        else:
            self.ee_jacobi_idx = self.robot_entity_cfg.body_ids[0]


    def run_diff_ik_ctrl(self, robot, root_pose_w, ee_pose_w, ee_pose_b_des, gripper_open = True):
        # obtain jacobian from simulation
        jacobian = robot.root_physx_view.get_jacobians()[:, self.ee_jacobi_idx, :, self.robot_entity_cfg.joint_ids]

        # Get current joint position
        joint_pos = robot.data.joint_pos[:, self.robot_entity_cfg.joint_ids]

        # compute end-effector pose in robot base (root) frame
        ee_pos_b, ee_quat_b = subtract_frame_transforms(
            root_pose_w[:, 0:3], root_pose_w[:, 3:7], ee_pose_w[:, 0:3], ee_pose_w[:, 3:7]
        )

        # Set target end-effector position (e.g., move above the pick-up point)
        self.diff_ik_controller.set_command(ee_pose_b_des)

        # Compute joint positions using the IK controller
        joint_pos_des = self.diff_ik_controller.compute(ee_pos_b, ee_quat_b, jacobian, joint_pos)

        if gripper_open:
            joint_pos_des = torch.cat([joint_pos_des, torch.tensor([0.0, 0.0], device="cuda").unsqueeze(0).repeat(self.scene.num_envs, 1)], dim=1)  # Shape: [num_envs, 8]
        else:
            joint_pos_des = torch.cat([joint_pos_des, torch.tensor([0.02, 0.02], device="cuda").unsqueeze(0).repeat(self.scene.num_envs, 1)], dim=1)  # Shape: [num_envs, 8]

        return joint_pos_des