import gymnasium as gym
from . import agents, ik_rel_env_cfg_franka, ik_rel_env_cfg_ur5e_sdu_gripper, ik_rel_env_cfg_ur5e_hand_e, ik_rel_env_cfg_ur5e_hand_e_domain_rand, joint_pos_env_cfg_franka, joint_pos_env_cfg_ur5e_sdu_gripper, ik_abs_env_cfg_ur5e_hand_e_domain_rand

# Register Gym environments.


# Relative Differential Inverse Kinematics Action Space with SDU Gripper attached to UR5e

gym.register(
    id="UR5e-Lift-Cube-IK",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": ik_rel_env_cfg_ur5e_sdu_gripper.RelIK_UR5e_SDU_Gripper_LiftCubeEnvCfg,
        "rl_games_cfg_entry_point": f"{agents.__name__}:rl_games_ppo_cfg.yaml",
        "sb3_ppo_cfg_entry_point": f"{agents.__name__}:sb3_ppo_cfg.yaml",
        "sb3_ddpg_cfg_entry_point": f"{agents.__name__}:sb3_ddpg_cfg.yaml",
        "sb3_td3_cfg_entry_point": f"{agents.__name__}:sb3_td3_cfg.yaml",
        "skrl_ppo_cfg_entry_point": f"{agents.__name__}:skrl_ppo_cfg.yaml",
        "skrl_td3_cfg_entry_point": f"{agents.__name__}:skrl_td3_cfg.yaml",
    },
)

# Relative Differential Inverse Kinematics Action Space with Robotiq Hand E attached to UR5e

gym.register(
    id="UR5e-Hand-E-Lift-Cube-IK",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": ik_rel_env_cfg_ur5e_hand_e.RelIK_UR5e_Hand_E_LiftCubeEnvCfg,
        "rl_games_cfg_entry_point": f"{agents.__name__}:rl_games_ppo_cfg.yaml",
        "sb3_ppo_cfg_entry_point": f"{agents.__name__}:sb3_ppo_cfg.yaml",
        "sb3_ddpg_cfg_entry_point": f"{agents.__name__}:sb3_ddpg_cfg.yaml",
        "sb3_td3_cfg_entry_point": f"{agents.__name__}:sb3_td3_cfg.yaml",
        "skrl_ppo_cfg_entry_point": f"{agents.__name__}:skrl_ppo_cfg.yaml",
        "skrl_td3_cfg_entry_point": f"{agents.__name__}:skrl_td3_cfg.yaml",
    },
)


# Relative Differential Inverse Kinematics Action Space with Robotiq Hand E attached to UR5e AND domain randomization

gym.register(
    id="UR5e-Hand-E-Domain-Rand-Lift-Cube-IK",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": ik_rel_env_cfg_ur5e_hand_e_domain_rand.RelIK_UR5e_Hand_E_Domain_Rand_LiftCubeEnvCfg,
        "rl_games_cfg_entry_point": f"{agents.__name__}:rl_games_ppo_cfg.yaml",
        "sb3_ppo_cfg_entry_point": f"{agents.__name__}:sb3_ppo_cfg.yaml",
        "sb3_ddpg_cfg_entry_point": f"{agents.__name__}:sb3_ddpg_cfg.yaml",
        "sb3_td3_cfg_entry_point": f"{agents.__name__}:sb3_td3_cfg.yaml",
        "skrl_ppo_cfg_entry_point": f"{agents.__name__}:skrl_ppo_cfg.yaml",
        "skrl_td3_cfg_entry_point": f"{agents.__name__}:skrl_td3_cfg.yaml",
    },
)


# Absolute Differential Inverse Kinematics Action Space with Robotiq Hand E attached to UR5e AND domain randomization

gym.register(
    id="UR5e-Hand-E-Domain-Rand-Lift-Cube-Abs-IK",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": ik_abs_env_cfg_ur5e_hand_e_domain_rand.AbsIK_UR5e_Hand_E_Domain_Rand_LiftCubeEnvCfg,
        "rl_games_cfg_entry_point": f"{agents.__name__}:rl_games_ppo_cfg.yaml",
        "sb3_ppo_cfg_entry_point": f"{agents.__name__}:sb3_ppo_cfg.yaml",
        "sb3_ddpg_cfg_entry_point": f"{agents.__name__}:sb3_ddpg_cfg.yaml",
        "sb3_td3_cfg_entry_point": f"{agents.__name__}:sb3_td3_cfg.yaml",
        "skrl_ppo_cfg_entry_point": f"{agents.__name__}:skrl_ppo_cfg.yaml",
        "skrl_td3_cfg_entry_point": f"{agents.__name__}:skrl_td3_cfg.yaml",
    },
)


# Joint Position Action Space with SDU Gripper attached to UR5e

gym.register(
    id="UR5e-Lift-Cube",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": joint_pos_env_cfg_ur5e_sdu_gripper.JointPos_UR5e_SDU_Gripper_LiftCubeEnvCfg,
        "rl_games_cfg_entry_point": f"{agents.__name__}:rl_games_ppo_cfg.yaml",
        "sb3_ppo_cfg_entry_point": f"{agents.__name__}:sb3_ppo_cfg.yaml",
        "sb3_ddpg_cfg_entry_point": f"{agents.__name__}:sb3_ddpg_cfg.yaml",
        "sb3_td3_cfg_entry_point": f"{agents.__name__}:sb3_td3_cfg.yaml",
        "skrl_ppo_cfg_entry_point": f"{agents.__name__}:skrl_ppo_cfg.yaml",
    },
)


# Relative Differential Inverse Kinematics Action Space with Franka

gym.register(
    id="Franka-Lift-Cube-IK",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": ik_rel_env_cfg_franka.RelIK_Franka_LiftCubeEnvCfg,
        "sb3_ppo_cfg_entry_point": f"{agents.__name__}:sb3_ppo_cfg.yaml",
        "sb3_ddpg_cfg_entry_point": f"{agents.__name__}:sb3_ddpg_cfg.yaml",
        "sb3_td3_cfg_entry_point": f"{agents.__name__}:sb3_td3_cfg.yaml",
    },
)

# Joint Position Action Space with Franka

gym.register(
    id="Franka-Lift-Cube",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": joint_pos_env_cfg_franka.JointPos_Franka_LiftCubeEnvCfg,
        "sb3_ppo_cfg_entry_point": f"{agents.__name__}:sb3_ppo_cfg.yaml",
        "sb3_ddpg_cfg_entry_point": f"{agents.__name__}:sb3_ddpg_cfg.yaml",
        "sb3_td3_cfg_entry_point": f"{agents.__name__}:sb3_td3_cfg.yaml",
    },
)
