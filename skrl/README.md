Contains all necessary information on how to start training and how to run a trained agent with the SKRL library.


#######################
# IK Relative Control #
#######################

## UR5e
### Train PPO agent
    source isaaclab/bin/activate
    cd isaaclab/IsaacLab
    ./isaaclab.sh -p /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube/skrl/train_skrl_ppo.py --num_envs 2048 --task UR5e-Lift-Cube-IK

or

    source isaaclab/bin/activate
    cd Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube/skrl/
    python3 train_skrl_ppo.py --num_envs 2048 --task UR5e-Lift-Cube-IK --headless


### Play the trained PPO agent
    python3 play_skrl_ppo.py --num_envs 4 --task UR5e-Lift-Cube-IK --checkpoint path/to/checkpoint
    python3 play_skrl_ppo.py --num_envs 4 --task UR5e-Lift-Cube-IK --checkpoint /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube/logs/skrl/ppo/UR5e-Lift-Cube-IK/2025-01-27_21-55-52_ppo_torch/checkpoints/best_agent.pt


##############################
# Wandb (Weights and Biases) #
##############################
    source isaaclab/bin/activate
    cd /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube/skrl/
    wandb sweep --project rel_ik_skrl_ppo_ur5e_lift_cube config_skrl_ppo.yaml