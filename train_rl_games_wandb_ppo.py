import argparse
from omni.isaac.lab.app import AppLauncher

# argparse for non-agent parameters
parser = argparse.ArgumentParser(description="Train an RL agent with Stable-Baselines3.")
parser.add_argument("--num_envs", type=int, default=None, help="Number of environments to simulate.")
parser.add_argument("--task", type=str, default=None, help="Name of the task.")

# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli, hydra_args = parser.parse_known_args()
args_cli.headless = True  # Set this based on your requirement

# launch the omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import gymnasium as gym
import numpy as np
import math
import gym_env.env  # Ensure custom environment is recognized

from rl_games.common import env_configurations, vecenv
from rl_games.common.algo_observer import IsaacAlgoObserver
from rl_games.torch_runner import Runner

from omni.isaac.lab_tasks.utils.parse_cfg import parse_env_cfg
from omni.isaac.lab_tasks.utils.wrappers.rl_games import RlGamesGpuEnv, RlGamesVecEnvWrapper
import wandb
import yaml
from wandb.integration.sb3 import WandbCallback


def main():
    """Train with stable-baselines agent."""
    # WandB initialization (config.yaml values come from WandB during sweep)
    with open("./config_rl_games_ppo.yaml") as file:
        config = yaml.load(file, Loader=yaml.FullLoader)

    run = wandb.init(
        project="rel_ik_rl_games_ppo_ur5e_lift_cube_0_05_hand_e",
        config=config,
        sync_tensorboard=True,  # auto-upload sb3's tensorboard metrics
        monitor_gym=False,  # auto-upload the videos of agents playing the game
        save_code=False,  # Save code for reproducibility
    )

    # Extract key parameters from wandb.config
    device = wandb.config["config.device"]
    clip_obs = wandb.config["env.clip_observations"]
    clip_actions = wandb.config["env.clip_actions"]
    task = wandb.config["config.name"]
    num_envs = wandb.config["config.num_actors"]

    # Parse environment configuration
    env_cfg = parse_env_cfg(task, device=device, num_envs=num_envs)
    env_cfg.seed = wandb.config["seed"]

    print("Wandb Config:", wandb.config)

    # Create Isaac environment
    env = gym.make(task, cfg=env_cfg, render_mode=None)

    # Wrap the environment for RL-Games
    env = RlGamesVecEnvWrapper(env, device, clip_obs, clip_actions)

    # Register the environment to RL-Games registry
    vecenv.register(
        "IsaacRlgWrapper", lambda config_name, num_actors, **kwargs: RlGamesGpuEnv(config_name, num_actors, **kwargs)
    )
    env_configurations.register("rlgpu", {"vecenv_type": "IsaacRlgWrapper", "env_creator": lambda **kwargs: env})

    # Create runner from RL-Games
    runner = Runner(IsaacAlgoObserver())
    runner.load(wandb.config)

    # Reset the agent and environment
    runner.reset()

    # Train the agent
    runner.run({"train": True, "play": False, "sigma": None})

    # Close the environment
    env.close()
    run.finish()


if __name__ == "__main__":
    # Run the main function
    main()
    # Close the simulation app
    simulation_app.close()
