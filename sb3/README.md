Contains all necessary information on how to start training and how to run a trained agent with the rl_games library.


#######################
# IK Relative Control #
#######################

## UR5e
### Train PPO agent
    source isaaclab/bin/activate
    cd isaaclab/IsaacLab
    ./isaaclab.sh -p /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube/sb3/train_sb3_ppo.py --num_envs 4096 --task UR5e-Lift-Cube-IK --headless

For testing:

    ./isaaclab.sh -p /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube/sb3/train_sb3_ppo.py --num_envs 1 --task UR5e-Lift-Cube-IK --no_logging

Train pre-trained model:

    ./isaaclab.sh -p /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube/sb3/train_sb3_ppo.py --num_envs 256 --task UR5e-Lift-Cube-IK --checkpoint path/to/checkpoint


### Play the trained PPO agent


#### Examples:
Bad training run

    ./isaaclab.sh -p /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube/sb3/play_sb3_ppo.py --task UR5e-Lift-Cube-IK --num_envs 4 --checkpoint /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube/logs/sb3/ppo/UR5e-Lift-Cube-IK/2024-12-24_12-25-03/model_212992000_steps.zip

Training runs for optimizing reward function (penalizing wrong TCP orientation) - Requires UR5e SDU gripper and object of scale=(0.3, 0.3, 1.0)

Reward weight too high

    ./isaaclab.sh -p /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube/sb3/play_sb3_ppo.py --task UR5e-Lift-Cube-IK --num_envs 4 --checkpoint /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube/logs/sb3/ppo/UR5e-Lift-Cube-IK/adjusted_reward_tcp_10/model.zip


    ./isaaclab.sh -p /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube/sb3/play_sb3_ppo.py --task UR5e-Lift-Cube-IK --num_envs 4 --checkpoint /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube/logs/sb3/ppo/UR5e-Lift-Cube-IK/adjusted_reward_tcp_7/model.zip

Optimal reward weight

    ./isaaclab.sh -p /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube/sb3/play_sb3_ppo.py --task UR5e-Lift-Cube-IK --num_envs 4 --checkpoint /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube/logs/sb3/ppo/UR5e-Lift-Cube-IK/adjusted_reward_tcp_6/model.zip
    
Reward weight too low

    ./isaaclab.sh -p /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube/sb3/play_sb3_ppo.py --task UR5e-Lift-Cube-IK --num_envs 4 --checkpoint /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube/logs/sb3/ppo/UR5e-Lift-Cube-IK/adjusted_reward_tcp_3/model.zip


## Franka
### Train PPO agent
source isaaclab/bin/activate
cd isaaclab/IsaacLab
./isaaclab.sh -p /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube/sb3/train_sb3_ppo.py --num_envs 4096 --task Franka-Lift-Cube-IK --headless


### Play the trained PPO agent
./isaaclab.sh -p /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube/sb3/play_sb3_ppo.py --task Franka-Lift-Cube-IK --num_envs 4 --checkpoint path/to/checkpoint



##########################
# Joint Position Control #
##########################

## UR5e
### Train PPO agent
source isaaclab/bin/activate
cd isaaclab/IsaacLab
./isaaclab.sh -p /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube/sb3/train_sb3_ppo.py --num_envs 512 --task UR5e-Lift-Cube --headless


### Play the trained PPO agent
./isaaclab.sh -p /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube/play_sb3_ppo.py --task UR5e-Lift-Cube --num_envs 4 --checkpoint path/to/checkpoint




###################################
# Wandb (Weights and Biases) UR5e #
###################################

## PPO
    source isaaclab/bin/activate
    cd /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube
    wandb sweep --project rel_ik_sb3_ppo_ur5e_lift_cube_0_05_final config_0_05.yaml

    wandb sweep --project rel_ik_sb3_ppo_ur5e_lift_cube_0_05_hand_e config_0_05.yaml

    wandb sweep --project rel_ik_sb3_ppo_ur5e_lift_cube_0_05_hand_e_num_env_test config_sb3_ppo.yaml

    wandb sweep --project rel_ik_sb3_ppo_ur5e_lift_cube_0_05_hand_e_final_v2 config_sb3_ppo.yaml

    wandb sweep --project rel_ik_sb3_ppo_ur5e_lift_cube_0_05_hand_e_final_v3 config_sb3_ppo.yaml



## DDPG
    source isaaclab/bin/activate
    cd /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube
    wandb sweep --project rel_ik_sb3_ddpg_ur5e_lift_cube_0_05 config_sb3_ddpg.yaml


## TD3
    source isaaclab/bin/activate
    cd /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube
    wandb sweep --project rel_ik_sb3_td3_ur5e_lift_cube_0_05 config_sb3_td3.yaml

    wandb sweep --project rel_ik_sb3_td3_ur5e_lift_cube_0_05_noise_1_0 config_sb3_td3.yaml
    
    wandb sweep --project rel_ik_sb3_td3_ur5e_lift_cube_0_05_noise_100 config_sb3_td3.yaml

    wandb sweep --project rel_ik_sb3_td3_ur5e_lift_cube_0_05_bayes_64 config_sb3_td3.yaml