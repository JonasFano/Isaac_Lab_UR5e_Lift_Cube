from isaaclab.utils import configclass
from isaaclab.sensors import FrameTransformerCfg
from isaaclab.markers.config import FRAME_MARKER_CFG  # isort: skip
from isaaclab.sensors.frame_transformer.frame_transformer_cfg import OffsetCfg
from isaaclab.controllers.differential_ik_cfg import DifferentialIKControllerCfg
from isaaclab.envs.mdp.actions.actions_cfg import DifferentialInverseKinematicsActionCfg

from isaaclab_assets.robots.franka import FRANKA_PANDA_HIGH_PD_CFG  # isort: skip

from . import lift_cube_env_cfg_franka
from taskparameters_franka import TaskParams

@configclass
class RelIK_Franka_LiftCubeEnvCfg(lift_cube_env_cfg_franka.Franka_LiftCubeEnvCfg):
    def __post_init__(self):
        super().__post_init__()

        # Set Franka as robot
        # We switch here to a stiffer PD controller for IK tracking to be better.
        self.scene.robot = FRANKA_PANDA_HIGH_PD_CFG.replace(prim_path="{ENV_REGEX_NS}/robot")

        # Listens to the required transforms
        marker_cfg = FRAME_MARKER_CFG.copy()
        marker_cfg.markers["frame"].scale = (0.1, 0.1, 0.1)
        marker_cfg.prim_path = "/Visuals/FrameTransformer"
        self.scene.ee_frame = FrameTransformerCfg(
            prim_path="{ENV_REGEX_NS}/robot/panda_link0",
            debug_vis=False,
            visualizer_cfg=marker_cfg,
            target_frames=[
                FrameTransformerCfg.FrameCfg(
                    prim_path="{ENV_REGEX_NS}/robot/panda_hand",
                    name="end_effector",
                    offset=OffsetCfg(
                        pos=TaskParams.gripper_offset,
                    ),
                ),
            ],
        )

        # print(self.commands.object_pose) # Do not show current end-effector frame
        self.commands.object_pose.current_pose_visualizer_cfg.markers['frame'].visible = False

        self.actions.arm_action = DifferentialInverseKinematicsActionCfg(
            asset_name="robot",
            joint_names=["panda_joint.*"],
            body_name=TaskParams.ee_body_name,
            controller=DifferentialIKControllerCfg(command_type=TaskParams.command_type, use_relative_mode=TaskParams.use_relative_mode, ik_method=TaskParams.ik_method),
            body_offset=DifferentialInverseKinematicsActionCfg.OffsetCfg(pos=TaskParams.gripper_offset),
            scale=0.6,
        )


@configclass
class RelIK_Franka_LiftCubeEnvCfg_PLAY(RelIK_Franka_LiftCubeEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()
        # make a smaller scene for play
        self.scene.num_envs = 4
        self.scene.env_spacing = 2.5
        # disable randomization for play
        self.observations.policy.enable_corruption = False

