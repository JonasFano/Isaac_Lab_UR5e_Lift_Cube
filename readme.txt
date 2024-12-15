#######################
# IK Relative Control #
#######################

# Stable-baselines3 - UR5e

source isaaclab/bin/activate
cd isaaclab/IsaacLab
./isaaclab.sh -p /home/jofa/Downloads/Repositories/isaac_lab_lift_cube/train_sb3.py --num_envs 512 --task UR5e-Lift-Cube-IK
./isaaclab.sh -p /home/jofa/Downloads/Repositories/isaac_lab_lift_cube/train_sb3.py --num_envs 4096 --task UR5e-Lift-Cube-IK --headless
./isaaclab.sh -p /home/jofa/Downloads/Repositories/isaac_lab_lift_cube/train_sb3.py --num_envs 256 --task UR5e-Lift-Cube-IK --checkpoint /home/jofa/Downloads/Repositories/isaac_lab_lift_cube/logs/sb3/UR5e-Lift-Cube-IK/2024-10-18_14-01-49/model_2816000_steps.zip
./isaaclab.sh -p /home/jofa/Downloads/Repositories/isaac_lab_lift_cube/play_sb3.py --task UR5e-Lift-Cube --num_envs 4 --checkpoint /home/jofa/Downloads/Repositories/isaac_lab_lift_cube/logs/sb3/UR5e-Lift-Cube/2024-10-25_14-27-06/model_3584000_steps.zip
./isaaclab.sh -p /home/jofa/Downloads/Repositories/isaac_lab_lift_cube/play_sb3.py --task UR5e-Lift-Cube-IK --num_envs 4 --checkpoint /home/jofa/Downloads/Repositories/isaac_lab_lift_cube/logs/sb3/ppo/UR5e-Lift-Cube-IK/2024-11-18_21-51-22/model_524288000_steps.zip
./isaaclab.sh -p /home/jofa/Downloads/Repositories/isaac_lab_lift_cube/play_sb3.py --task UR5e-Lift-Cube-IK --num_envs 4 --checkpoint /home/jofa/Downloads/Repositories/isaac_lab_lift_cube/logs/sb3/ppo/UR5e-Lift-Cube-IK/2024-11-18_21-51-22/model_413696000_steps.zip
./isaaclab.sh -p /home/jofa/Downloads/Repositories/isaac_lab_lift_cube/play_sb3.py --task UR5e-Lift-Cube-IK --num_envs 4 --checkpoint /home/jofa/Downloads/Repositories/isaac_lab_lift_cube/logs/sb3/ppo/UR5e-Lift-Cube-IK/2024-11-18_21-51-22/model_217088000_steps.zip




# Stable-baselines3 - Franka
source isaaclab/bin/activate
cd isaaclab/IsaacLab
./isaaclab.sh -p /home/jofa/Downloads/Repositories/isaac_lab_lift_cube/train_sb3.py --num_envs 4096 --task Franka-Lift-Cube-IK --headless
./isaaclab.sh -p /home/jofa/Downloads/Repositories/isaac_lab_lift_cube/play_sb3.py --task Franka-Lift-Cube-IK --num_envs 4 --checkpoint /home/jofa/Downloads/Repositories/isaac_lab_lift_cube/logs/sb3/ppo/Franka-Lift-Cube-IK/2024-11-05_11-49-27/model_49152000_steps.zip
./isaaclab.sh -p /home/jofa/Downloads/Repositories/isaac_lab_lift_cube/play_sb3.py --task Franka-Lift-Cube-IK --num_envs 4 --checkpoint /home/jofa/Downloads/Repositories/isaac_lab_lift_cube/logs/sb3/ppo/Franka-Lift-Cube-IK/2024-11-05_16-31-32/model.zip



# Tensorboard
tensorboard --logdir='directory'



# RL games - UR5e

source isaaclab/bin/activate
cd isaaclab/IsaacLab
./isaaclab.sh -p /home/jofa/Downloads/Repositories/isaac_lab_lift_cube/train_rl_games.py --num_envs 1024 --task UR5e-Lift-Cube-IK --headless --max_iterations 1500
./isaaclab.sh -p /home/jofa/Downloads/Repositories/isaac_lab_lift_cube/play_rl_games.py --task UR5e-Lift-Cube-IK --num_envs 4 --checkpoint /home/jofa/Downloads/Repositories/isaac_lab_lift_cube/logs/rl_games/UR5e-Lift-Cube-IK/2024-11-04_15-25-12/nn/last_Ur5e-Lift-Cube_ep_1500_rew_1.097335.pth



# RL games - Franka

source isaaclab/bin/activate
cd isaaclab/IsaacLab
./isaaclab.sh -p /home/jofa/Downloads/Repositories/isaac_lab_lift_cube/train_rl_games.py --num_envs 4096 --task Franka-Lift-Cube-IK --headless --max_iterations 2500
./isaaclab.sh -p /home/jofa/Downloads/Repositories/isaac_lab_lift_cube/play_rl_games.py --num_envs 4 --task Franka-Lift-Cube-IK --use_last_checkpoint






##########################
# Joint Position Control #
##########################

# RL games

# Train RL agent in the manager-based RL environment sb3-ppo, inspired by the Isaac-Lift-Cube-Franka-v0 environment
source isaaclab/bin/activate
cd isaaclab/IsaacLab
./isaaclab.sh -p /home/jofa/Downloads/Repositories/isaac_lab_lift_cube/train_rl_games.py --num_envs 4096 --task UR5e-Lift-Cube --headless --max_iterations 1500



# Play the trained agent
source isaaclab/bin/activate
cd isaaclab/IsaacLab
./isaaclab.sh -p /home/jofa/Downloads/Repositories/isaac_lab_lift_cube/play_rl_games.py --task UR5e-Lift-Cube --num_envs 4 --checkpoint /home/jofa/Downloads/Repositories/isaac_lab_lift_cube/logs/rl_games/UR5e-Lift-Cube/2024-10-25_12-22-41/nn/last_Peg-in-hole_ep_750_rew_91.63596.pth
./isaaclab.sh -p /home/jofa/Downloads/Repositories/isaac_lab_lift_cube/play_rl_games.py --task UR5e-Lift-Cube --num_envs 4 --use_last_checkpoint




# Stable-baselines3 - UR5e

source isaaclab/bin/activate
cd isaaclab/IsaacLab
./isaaclab.sh -p /home/jofa/Downloads/Repositories/isaac_lab_lift_cube/train_sb3.py --num_envs 512 --task UR5e-Lift-Cube --headless
./isaaclab.sh -p /home/jofa/Downloads/Repositories/isaac_lab_lift_cube/play_sb3.py --task UR5e-Lift-Cube --num_envs 4 --checkpoint /home/jofa/Downloads/Repositories/isaac_lab_lift_cube/logs/sb3/UR5e-Lift-Cube/2024-10-25_14-27-06/model_3584000_steps.zip





##############################
# Save expert demonstrations #
##############################
source isaaclab/bin/activate
cd isaaclab/IsaacLab
./isaaclab.sh -p /home/jofa/Downloads/Repositories/isaac_lab_lift_cube/save_expert_demonstrations.py --num_envs 1 --task UR5e-Lift-Cube
