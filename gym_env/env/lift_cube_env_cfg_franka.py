from dataclasses import MISSING

import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg, RigidObjectCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import CurriculumTermCfg as CurrTerm
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils import configclass
from . import mdp
import os
from isaaclab.sim.schemas.schemas_cfg import RigidBodyPropertiesCfg, MassPropertiesCfg
from isaaclab.sim.spawners.from_files.from_files_cfg import UsdFileCfg
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR

from isaaclab_assets.robots.franka import FRANKA_PANDA_CFG  # isort: skip

from taskparameters_franka import TaskParams

##
# Scene definition
##

MODEL_PATH = os.path.join(os.path.abspath(os.path.join(os.path.dirname(__file__), "../../")), "scene_models")

@configclass
class Franka_LiftCubeSceneCfg(InteractiveSceneCfg):
    """Configuration for the lift scene with a robot and a object."""
    # articulation
    robot = FRANKA_PANDA_CFG.replace(prim_path="{ENV_REGEX_NS}/robot")

    # Set Cube as object
    object = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Object",
        init_state=RigidObjectCfg.InitialStateCfg(pos=TaskParams.object_init_position, rot=TaskParams.object_init_rotation),
        spawn=UsdFileCfg(
            usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Blocks/DexCube/dex_cube_instanceable.usd",
            scale=TaskParams.object_scale,
            rigid_props=RigidBodyPropertiesCfg(
                solver_position_iteration_count=16,
                solver_velocity_iteration_count=1,
                max_angular_velocity=1000.0,
                max_linear_velocity=1000.0,
                max_depenetration_velocity=5.0,
                disable_gravity=False,
            ),
            mass_props=MassPropertiesCfg(
                mass=0.5,
            ),
            collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.005, rest_offset=0.0),
        ),
    )

    # ground plane
    ground = AssetBaseCfg(
        prim_path="/World/defaultGroundPlane",
        init_state=AssetBaseCfg.InitialStateCfg(pos=[0, 0, -0.74]), 
        spawn=sim_utils.GroundPlaneCfg())

    # lights
    dome_light = AssetBaseCfg(
        prim_path="/World/Lights/Dome", 
        spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
    )    
    
    # Add the siegmund table to the scene
    table = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/table", 
        spawn=sim_utils.UsdFileCfg(usd_path=os.path.join(MODEL_PATH, "Single_Siegmund_table.usd")), 
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, 0.0), rot=(0.7071, 0.0, 0.0, 0.7071)),
    )



##
# MDP settings
##

@configclass
class CommandsCfg:
    """Command terms for the MDP."""
    object_pose = mdp.UniformPoseCommandCfg(
        asset_name="robot",
        body_name=TaskParams.ee_body_name,
        resampling_time_range=TaskParams.resampling_time_range,
        debug_vis=TaskParams.visualize_frame,
        ranges=mdp.UniformPoseCommandCfg.Ranges(
            pos_x=TaskParams.sample_range_pos_x,
            pos_y=TaskParams.sample_range_pos_y,
            pos_z=TaskParams.sample_range_pos_z,
            roll=TaskParams.sample_range_roll,
            pitch=TaskParams.sample_range_pitch,  
            yaw=TaskParams.sample_range_yaw,
        ),
    )


@configclass
class ActionsCfg:
    """Action specifications for the MDP."""
    # Set actions
    arm_action: mdp.JointPositionActionCfg | mdp.DifferentialInverseKinematicsActionCfg = MISSING

    gripper_action = mdp.BinaryJointPositionActionCfg(
        asset_name="robot",
        joint_names=["panda_finger.*"],
        open_command_expr={"panda_finger_.*": TaskParams.gripper_open},
        close_command_expr={"panda_finger_.*": TaskParams.gripper_close},
    )


@configclass
class ObservationsCfg:
    """Observation specifications for the MDP."""

    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for policy group."""
        tcp_pose = ObsTerm(func=mdp.get_current_tcp_pose)
        # tcp_lin_vel = ObsTerm(func=mdp.base_lin_vel, params={"asset_cfg": SceneEntityCfg("robot", body_names="panda_hand")})

        object_position = ObsTerm(func=mdp.object_position_in_robot_root_frame)
        target_object_position = ObsTerm(func=mdp.generated_commands, params={"command_name": "object_pose"})
        actions = ObsTerm(func=mdp.last_action)

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True

    # observation groups
    policy: PolicyCfg = PolicyCfg()


@configclass
class EventCfg:
    """Configuration for events."""
    reset_all = EventTerm(func=mdp.reset_scene_to_default, mode="reset")

    reset_robot_joints = EventTerm(
        func=mdp.reset_joints_by_scale,
        mode="reset",
        params={
            "position_range": TaskParams.robot_reset_joints_pos_range,
            "velocity_range": TaskParams.robot_reset_joints_vel_range,
        },
    )

    reset_object_position = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "pose_range": {"x": TaskParams.object_randomize_pose_range_x, "y": TaskParams.object_randomize_pose_range_y, "z": TaskParams.object_randomize_pose_range_z},
            "velocity_range": {},
            "asset_cfg": SceneEntityCfg("object", body_names="Object"),
        },
    )

    randomize_object_mass = EventTerm(
        func=mdp.randomize_rigid_body_mass, 
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("object", body_names="Object"),
            "mass_distribution_params": TaskParams.object_randomize_mass_range,
            "operation": TaskParams.object_randomize_mass_operation,
            "distribution": TaskParams.object_randomize_mass_distribution,
            "recompute_inertia": TaskParams.object_randomize_recompute_inertia,
        }
    )

    randomize_gripper_fingers_friction_coefficients = EventTerm(
        func=mdp.randomize_friction_coefficients,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=TaskParams.gripper_body_names),
            "static_friction_distribution_params": TaskParams.gripper_static_friction_distribution_params,
            "dynamic_friction_distribution_params": TaskParams.gripper_dynamic_friction_distribution_params,
            "restitution_distribution_params": TaskParams.gripper_restitution_distribution_params,
            "operation": TaskParams.gripper_randomize_friction_operation,
            "distribution": TaskParams.gripper_randomize_friction_distribution,
            "make_consistent": TaskParams.gripper_randomize_friction_make_consistent,  
        }
    )

    randomize_object_friction_coefficients = EventTerm(
        func=mdp.randomize_friction_coefficients,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("object", body_names="Object"),
            "static_friction_distribution_params": TaskParams.object_static_friction_distribution_params,
            "dynamic_friction_distribution_params": TaskParams.object_dynamic_friction_distribution_params,
            "restitution_distribution_params": TaskParams.object_restitution_distribution_params,
            "operation": TaskParams.object_randomize_friction_operation,
            "distribution": TaskParams.object_randomize_friction_distribution,
            "make_consistent": TaskParams.object_randomize_friction_make_consistent,  
        }
    )


@configclass
class RewardsCfg:
    """Reward terms for the MDP."""
    reaching_object = RewTerm(func=mdp.object_ee_distance, params={"std": TaskParams.reaching_object_std}, weight=TaskParams.reaching_object_weight)

    lifting_object = RewTerm(func=mdp.object_is_lifted, params={"minimal_height": TaskParams.lift_min_height}, weight=TaskParams.lift_weight) # 15.0

    object_goal_tracking = RewTerm(
        func=mdp.object_goal_distance,
        params={"std": TaskParams.object_goal_tracking_coarse_std, "minimal_height": TaskParams.lift_min_height, "command_name": "object_pose"},
        weight=TaskParams.object_goal_tracking_coarse_weight,
    )

    object_goal_tracking_fine_grained = RewTerm(
        func=mdp.object_goal_distance,
        params={"std": TaskParams.object_goal_tracking_fine_grained_std, "minimal_height": TaskParams.lift_min_height, "command_name": "object_pose"},
        weight=TaskParams.object_goal_tracking_fine_grained_weight,
    )

    end_effector_orientation_tracking = RewTerm(
        func=mdp.orientation_command_error,
        weight=TaskParams.end_effector_orientation_tracking_weight,
        params={"minimal_height": TaskParams.lift_min_height, "command_name": "object_pose", "asset_cfg": SceneEntityCfg("robot", body_names=TaskParams.ee_body_name),},
    )

    # action penalty
    action_rate = RewTerm(func=mdp.action_rate_l2, weight=TaskParams.action_rate_weight)



@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""
    time_out = DoneTerm(func=mdp.time_out, time_out=True)

    object_dropping = DoneTerm(
        func=mdp.root_height_below_minimum, params={"minimum_height": TaskParams.min_height_object_dropping, "asset_cfg": SceneEntityCfg("object")}
    )


@configclass
class CurriculumCfg:
    """Curriculum terms for the MDP."""
    action_rate = CurrTerm(
        func=mdp.modify_reward_weight, params={"term_name": "action_rate", "weight": TaskParams.action_rate_curriculum_weight, "num_steps": TaskParams.curriculum_num_steps} 
    )


##
# Environment configuration
##

@configclass
class Franka_LiftCubeEnvCfg(ManagerBasedRLEnvCfg):
    """Configuration for the lifting environment."""
    # Scene settings
    scene: Franka_LiftCubeSceneCfg = Franka_LiftCubeSceneCfg(num_envs=4, env_spacing=2.5)

    # Basic settings
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    commands: CommandsCfg = CommandsCfg()

    # MDP settings
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    events: EventCfg = EventCfg()
    curriculum: CurriculumCfg = CurriculumCfg()

    def __post_init__(self):
        """Post initialization."""
        # general settings
        self.decimation = TaskParams.decimation
        self.episode_length_s = TaskParams.episode_length_s
        # simulation settings
        self.sim.dt = TaskParams.dt
        self.sim.render_interval = self.decimation

        self.sim.physx.bounce_threshold_velocity = 0.2
        self.sim.physx.bounce_threshold_velocity = 0.01
        self.sim.physx.gpu_found_lost_aggregate_pairs_capacity = 1024 * 1024 * 4
        self.sim.physx.gpu_total_aggregate_pairs_capacity = 16 * 1024
        self.sim.physx.friction_correlation_distance = 0.00625
        self.sim.physx.gpu_collision_stack_size = 4096 * 4096 * 100 # Was added due to an PhysX error: collisionStackSize buffer overflow detected