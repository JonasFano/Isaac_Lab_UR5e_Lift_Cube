import argparse
from omni.isaac.lab.app import AppLauncher
import gymnasium as gym
import numpy as np
import torch.nn as nn  # Import nn to access activation functions
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import VecNormalize

# argparse for non-agent parameters
parser = argparse.ArgumentParser(description="Train an RL agent with Stable-Baselines3.")
parser.add_argument("--num_envs", type=int, default=None, help="Number of environments to simulate.")
parser.add_argument("--task", type=str, default=None, help="Name of the task.")
parser.add_argument("--seed", type=int, default=42, help="Seed used for the environment")

# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli, hydra_args = parser.parse_known_args()
# args_cli.headless = True  # Set this based on your requirement

# launch the omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import gym_env.env  # Ensure custom environment is recognized
from omni.isaac.lab_tasks.utils.parse_cfg import parse_env_cfg
from settings import Settings
from controller.controller import Controller


def save_demonstrations(states, actions, file_path=Settings.data_path + "expert_demonstrations.npz"):
    """Function to save expert demonstrations."""
    np.savez(file_path, states=states, actions=actions)
    print(f"[INFO] Expert demonstrations saved at {file_path}")


def run_simulator(env):
    """Runs the simulation loop."""
    states = []
    actions = []

    # Extract scene entities
    robot = env.unwrapped.scene["robot"]

    # Define simulation stepping
    sim_dt = env.unwrapped.sim.get_physics_dt()

    dmp_nr = 0
    terminate = False

    robot.set_joint_position_target(robot.data.default_joint_pos)
    robot.write_joint_state_to_sim(robot.data.default_joint_pos.clone(), robot.data.default_joint_vel.clone())
    env.reset()

    controller = Controller(env.unwrapped.sim, env.unwrapped.scene, robot)

    # Simulation loop
    while simulation_app.is_running():
        if Settings.mode == "Joint_Control":
            dmp_nr, robot, env, terminate, action = controller.run_joint_position_control(robot, env)

        # Step the environment with the generated actions
        obs, rew, terminated, truncated, info = env.step(action)

        print(action)
        print(obs)

        states.append(obs["policy"].cpu())
        actions.append(action.cpu())  # Store the action

        # env.unwrapped.scene.write_data_to_sim()
        # env.unwrapped.scene.update(sim_dt)

        if terminate:
            return controller.states, controller.actions




def main():
    """Collect expert demonstrations using a controller."""
    # Load environment configuration
    device = "cuda"
    env_cfg = parse_env_cfg(args_cli.task, device=device, num_envs=args_cli.num_envs)
    env_cfg.seed = args_cli.seed

    env = gym.make(args_cli.task, cfg=env_cfg)

    # Reset the simulator
    env.reset()
    print("[INFO]: Setup complete...")

    # Collect demonstrations
    states, actions = run_simulator(env)

    # Save the expert demonstrations
    file_path = Settings.data_path + "expert_demonstrations.npz"
    save_demonstrations(states, actions, file_path=file_path)

    # Close the environment
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
