Contains all necessary information on how to start training and how to run a trained agent with the rl_games library.


#######################
# IK Relative Control #
#######################

## UR5e
### Train PPO agent
    source isaaclab/bin/activate
    cd isaaclab/IsaacLab
    ./isaaclab.sh -p /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube/rl_games/train_rl_games.py --num_envs 2048 --task UR5e-Lift-Cube-IK --headless --max_iterations 1500

or

    source isaaclab/bin/activate
    cd Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube/rl_games/
    python3 train_rl_games.py --num_envs 2048 --task UR5e-Lift-Cube-IK --headless --max_iterations 1000


### Play the trained PPO agent
    ./isaaclab.sh -p /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube/rl_games/play_rl_games.py --task UR5e-Lift-Cube-IK --num_envs 4 --checkpoint path/to/checkpoint

For example:

    ./isaaclab.sh -p /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube/rl_games/play_rl_games.py --task UR5e-Lift-Cube-IK --num_envs 4 --checkpoint /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube/logs/rl_games/ppo/2025-01-24_18-24-53/nn/UR5e-Lift-Cube-IK.pth



## Franka
### Train PPO agent
    source isaaclab/bin/activate
    cd isaaclab/IsaacLab
    ./isaaclab.sh -p /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube/rl_games/train_rl_games.py --num_envs 4096 --task Franka-Lift-Cube-IK --headless --max_iterations 2500


### Play the trained PPO agent
    ./isaaclab.sh -p /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube/rl_games/play_rl_games.py --num_envs 4 --task Franka-Lift-Cube-IK --use_last_checkpoint



##########################
# Joint Position Control #
##########################

## UR5e

### Train PPO agent
    source isaaclab/bin/activate
    cd isaaclab/IsaacLab
    ./isaaclab.sh -p /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube/rl_games/train_rl_games.py --num_envs 4096 --task UR5e-Lift-Cube --headless --max_iterations 1500


### Play the trained PPO agent
    source isaaclab/bin/activate
    cd isaaclab/IsaacLab
    ./isaaclab.sh -p /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube/rl_games/play_rl_games.py --task UR5e-Lift-Cube --num_envs 4 --checkpoint path/to/checkpoint

or

    ./isaaclab.sh -p /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube/rl_games/play_rl_games.py --task UR5e-Lift-Cube --num_envs 4 --use_last_checkpoint



##############################
# Wandb (Weights and Biases) #
##############################
    source isaaclab/bin/activate
    cd /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube/rl_games/
    wandb sweep --project rel_ik_rl_games_ppo_ur5e_lift_cube_0_05_hand_e config_rl_games_ppo.yaml




# Tensorboard
    tensorboard --logdir='directory'