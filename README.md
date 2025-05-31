# Short project description
This project was part of my Master's Thesis with the title "Reinforcement Learning for Robot Control in Isaac Lab: A Feasibility Study" for the Master's program "Robot Systems - Advanced Robotics Technology" at the University of Southern Denmark (SDU). The task was to assess the feasibility of using RL-based robot control for a peg-in-hole task using the advanced physics simulator NVIDIA Isaac Lab. A stepwise development process was used in which task complexity is gradually increased to enable systematic optimization and validation of key framework components and algorithm hyperparameters. Each task builds directly on the previous one, reusing components and introducing new challenges in isolation.

This Repository includes the implementation to train PPO, DDPG or TD3 agents (from Stable-Baselines3) in Isaac Lab. It also includes multiple non-optimized agents for the RL-Games and SKRL library. The considered task includes a UR5e (Hand E gripper or custom SDU gripper) or Franka robot and requires it to lift a cube to a desired pose with differential inverse kinematics (IK) control or joint position control.

This task builds on the simple Reach Target Pose Task (Repository: Isaac_Lab_Reach) that was used to validate key framework components and hyperparameters. The Lift Cube Task was the intermediate step in this thesis' project before progressing to the more complex peg-in-hole task. It was mainly used to validate domain randomization strategies in simulation.

The Weights&Biases tool was utilized to automate the hyperparameter search since it allows to extensively visualize the episode reward mean of a range of training runs each with different hyperparameter configurations.


# Steps to execute the code
Follow these steps to create a virtual python environment and to install Isaac Sim and Isaac Lab:

https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/index.html

Install requirements:
    
    pip install -r /path/to/requirements.txt 


# Hyperparameter optimization with Weights&Biases and Stable-Baselines3
## PPO
### UR5e with Hand E gripper - Rel IK

    source /path/to/virtual/environment/bin/activate
    cd /path/to/repository/sb3
    wandb sweep --project rel_ik_sb3_ppo_ur5e_lift_cube_hand_e config_sb3_ppo.yaml


### UR5e with Hand E gripper - Rel IK - Domain Randomization

    source /path/to/virtual/environment/bin/activate
    cd /path/to/repository/sb3
    wandb sweep --project rel_ik_sb3_ppo_ur5e_lift_cube_hand_e_domain_rand config_sb3_ppo.yaml


## DDPG - UR5e with Hand E gripper - Rel IK

    source /path/to/virtual/environment/bin/activate
    cd /path/to/repository/sb3
    wandb sweep --project rel_ik_sb3_ddpg_ur5e_lift_cube config_sb3_ddpg.yaml


## TD3 - UR5e with Hand E gripper - Rel IK

    source /path/to/virtual/environment/bin/activate
    cd /path/to/repository/sb3
    wandb sweep --project rel_ik_sb3_td3_ur5e_lift_cube_bayes_optimization config_sb3_td3.yaml


Notably, optimization project names and the specific environment that is used for training have to be changed inside train_sb3_wandb_ppo.py, train_sb3_wandb_ddpg.py, or train_sb3_wandb_td3.py, respectively. The task options are listed below. Hyperparameters and parameter sweep names have to be set inside config_sb3_ddpg.yaml, config_sb3_ppo.yaml, or config_sb3_td3.yaml, respectively.




# Train PPO agent using Stable-Baselines3 without Weights&Biases
Option 1:

    source /path/to/virtual/environment/bin/activate
    cd /path/to/repository/sb3
    python3 train_sb3_ppo.py --num_envs 2048 --task UR5e-Hand-E-Lift-Cube-IK--headless

Option 2:

    source /path/to/virtual/environment/bin/activate
    cd /path/to/isaac/lab/installation/directory
    ./isaaclab.sh -p /path/to/repository/sb3/train_sb3_ppo.py --num_envs 2048 --task UR5e-Hand-E-Lift-Cube-IK--headless

Tensorboard can be used to visualize training results

    tensorboard --logdir='directory'

Note: For this option, the hyperparameters are defined in /gym_env/env/agents/



# Play agent trained with PPO and Stable-Baselines3 
Option 1:

    source /path/to/virtual/environment/bin/activate
    cd /path/to/repository/sb3
    python3 play_sb3_ppo.py --num_envs 4 --task UR5e-Hand-E-Lift-Cube-IK--checkpoint /path/to/trained/model

Option 2:

    source /path/to/virtual/environment/bin/activate
    cd /path/to/isaac/lab/installation/directory
    ./isaaclab.sh -p /path/to/repository/sb3/play_sb3_ppo.py --num_envs 4 --task UR5e-Hand-E-Lift-Cube-IK--checkpoint /path/to/trained/model

Note: This repository includes several pre-trained models in sb3/models/. These models were used to obtain the result described in the Master's Thesis.


Example:
    source isaaclab/bin/activate
    cd isaaclab/IsaacLab/
    ./isaaclab.sh -p /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube/sb3/play_sb3_ppo.py --num_envs 2 --task UR5e-Hand-E-Domain-Rand-Lift-Cube-IK --checkpoint /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube/sb3/models/config_d/xlbrr2gw/model.zip






# Train PPO agent using RL-Games without Weights&Biases
Option 1:

    source /path/to/virtual/environment/bin/activate
    cd /path/to/repository/rl_games
    python3 train_rl_games.py --num_envs 2048 --task UR5e-Hand-E-Lift-Cube-IK--headless --max_iterations 2500

Option 2:

    source /path/to/virtual/environment/bin/activate
    cd /path/to/isaac/lab/installation/directory
    ./isaaclab.sh -p /path/to/repository/rl_games/train_rl_games.py --num_envs 2048 --task UR5e-Hand-E-Lift-Cube-IK--headless --max_iterations 2500



# Play agent trained with PPO and RL-Games
Option 1:

    source /path/to/virtual/environment/bin/activate
    cd /path/to/repository/rl_games
    python3 play_rl_games.py --num_envs 2048 --task UR5e-Hand-E-Lift-Cube-IK--checkpoint path/to/checkpoint

Option 2:

    source /path/to/virtual/environment/bin/activate
    cd /path/to/isaac/lab/installation/directory
    ./isaaclab.sh -p /path/to/repository/rl_games/play_rl_games.py --num_envs 4 --task UR5e-Hand-E-Lift-Cube-IK--use_last_checkpoint



# Train PPO agent using SKRL without Weights&Biases
Option 1:

    source /path/to/virtual/environment/bin/activate
    cd /path/to/repository/skrl
    python3 train_skrl_ppo.py --num_envs 2048 --task UR5e-Hand-E-Lift-Cube-IK--headless

Option 2:

    source /path/to/virtual/environment/bin/activate
    cd /path/to/isaac/lab/installation/directory
    ./isaaclab.sh -p /path/to/repository/skrl/train_skrl_ppo.py --num_envs 2048 --task UR5e-Hand-E-Lift-Cube-IK--headless


# Play agent trained with PPO and SKRL
Option 1:

    source /path/to/virtual/environment/bin/activate
    cd /path/to/repository/skrl
    python3 play_skrl_ppo.py --num_envs 4 --task UR5e-Hand-E-Lift-Cube-IK--checkpoint path/to/checkpoint

Option 2:

    source /path/to/virtual/environment/bin/activate
    cd /path/to/isaac/lab/installation/directory
    ./isaaclab.sh -p /path/to/repository/skrl/play_skrl_ppo.py --num_envs 4 --task UR5e-Hand-E-Lift-Cube-IK--checkpoint path/to/checkpoint




# Task options (defined in /gym_env/env/__init__.py)
UR5e with Hand E Gripper and Relative Differential Inverse Kinematics Action Space

    --task UR5e-Hand-E-Lift-Cube-IK


UR5e with Hand E Gripper, Relative Differential Inverse Kinematics Action Space and Domain Randomization Strategies

    --task UR5e-Hand-E-Domain-Rand-Lift-Cube-IK


UR5e with Hand E Gripper, Absolute Differential Inverse Kinematics Action Space and Domain Randomization Strategies

    --task UR5e-Hand-E-Domain-Rand-Lift-Cube-Abs-IK


UR5e with Custom SDU Gripper and Relative Differential Inverse Kinematics Action Space

    --task UR5e-Lift-Cube-IK


UR5e with Custom SDU Gripper and Joint Position Action Space

    --task UR5e-Lift-Cube


Franka and Relative Differential Inverse Kinematics Action Space

    --task Franka-Lift-Cube-IK


Franka and Joint Position Action Space

    --task Franka-Lift-Cube
