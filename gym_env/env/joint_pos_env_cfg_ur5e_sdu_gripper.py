from isaaclab.utils import configclass
from isaaclab.sensors import FrameTransformerCfg
from isaaclab.markers.config import FRAME_MARKER_CFG  # isort: skip
from isaaclab.sensors.frame_transformer.frame_transformer_cfg import OffsetCfg

from . import lift_cube_env_cfg_ur5e_sdu_gripper, mdp
from taskparameters_ur5e_sdu_gripper import TaskParams

@configclass
class JointPos_UR5e_SDU_Gripper_LiftCubeEnvCfg(lift_cube_env_cfg_ur5e_sdu_gripper.UR5e_SDU_Gripper_LiftCubeEnvCfg):
    def __post_init__(self):
        super().__post_init__()

        # Listens to the required transforms
        marker_cfg = FRAME_MARKER_CFG.copy()
        marker_cfg.markers["frame"].scale = (0.1, 0.1, 0.1)
        marker_cfg.prim_path = "/Visuals/FrameTransformer"
        self.scene.ee_frame = FrameTransformerCfg(
            prim_path="{ENV_REGEX_NS}/robot/base_link",
            debug_vis=False,
            visualizer_cfg=marker_cfg,
            target_frames=[
                FrameTransformerCfg.FrameCfg(
                    prim_path="{ENV_REGEX_NS}/robot/wrist_3_link",
                    name="end_effector",
                    offset=OffsetCfg(
                        pos=TaskParams.gripper_offset,
                    ),
                ),
            ],
        )

        # print(self.commands.ee_pose) # Do not show current end-effector frame
        self.commands.object_pose.current_pose_visualizer_cfg.markers['frame'].visible = False

        self.actions.arm_action = mdp.JointPositionActionCfg(
            asset_name="robot", 
            joint_names=TaskParams.joint_names, 
            scale=0.5, 
            use_default_offset=True,
        )

@configclass
class JointPos_UR5e_SDU_Gripper_LiftCubeEnvCfg_PLAY(JointPos_UR5e_SDU_Gripper_LiftCubeEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()
        # make a smaller scene for play
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        # disable randomization for play
        self.observations.policy.enable_corruption = False

