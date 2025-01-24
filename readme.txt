#######################
# IK Relative Control #
#######################

# Stable-baselines3 - UR5e

source isaaclab/bin/activate
cd isaaclab/IsaacLab
./isaaclab.sh -p /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube/train_sb3.py --num_envs 4096 --task UR5e-Lift-Cube-IK --headless
./isaaclab.sh -p /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube/train_sb3.py --num_envs 1 --task UR5e-Lift-Cube-IK --no_logging
./isaaclab.sh -p /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube/train_sb3.py --num_envs 256 --task UR5e-Lift-Cube-IK --checkpoint /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube/logs/sb3/UR5e-Lift-Cube-IK/2024-10-18_14-01-49/model_2816000_steps.zip

# Grasping it sideways - wrist downwards - sweep 11 - 'pi': [256, 128, 64], 'vf': [128, 64] - vf_coef=0.1
./isaaclab.sh -p /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube/play_sb3.py --task UR5e-Lift-Cube-IK --num_envs 4 --checkpoint /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube/models/trcytobk/model.zip

# Grasping it sideways - wrist upwards - sweep 9 - 'pi': [256, 128, 64], 'vf': [256, 128, 64] - vf_coef=0.1
./isaaclab.sh -p /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube/play_sb3.py --task UR5e-Lift-Cube-IK --num_envs 4 --checkpoint /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube/models/f3u3nism/model.zip

# Grasping it sideways - wrist upwards but gripper almost touches elbow - sweep 16 - 'pi': [128, 64], 'vf': [128, 64] - vf_coef=0.01
./isaaclab.sh -p /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube/play_sb3.py --task UR5e-Lift-Cube-IK --num_envs 4 --checkpoint /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube/models/kayuaa4n/model.zip

# Grasping it sideways - wrist downwards but gripper almost touches robot base - sweep 10 - 'pi': [256, 128, 64], 'vf': [256, 128, 64] - vf_coef=0.01
./isaaclab.sh -p /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube/play_sb3.py --task UR5e-Lift-Cube-IK --num_envs 4 --checkpoint /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube/models/0r2dmgdl/model.zip

# Adjusted reward training run - negative
./isaaclab.sh -p /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube/play_sb3.py --task UR5e-Lift-Cube-IK --num_envs 4 --checkpoint /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube/logs/sb3/ppo/UR5e-Lift-Cube-IK/adjusted_reward_negative_4/model.zip

# Adjusted reward training run - tanh
./isaaclab.sh -p /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube/play_sb3.py --task UR5e-Lift-Cube-IK --num_envs 4 --checkpoint /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube/logs/sb3/ppo/UR5e-Lift-Cube-IK/adjusted_reward_tanh_4/model.zip

# Adjusted reward training run - tcp
./isaaclab.sh -p /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube/play_sb3.py --task UR5e-Lift-Cube-IK --num_envs 4 --checkpoint /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube/logs/sb3/ppo/UR5e-Lift-Cube-IK/adjusted_reward_tcp_6_v2/model.zip




# Stable-baselines3 - Franka
source isaaclab/bin/activate
cd isaaclab/IsaacLab
./isaaclab.sh -p /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube/train_sb3.py --num_envs 4096 --task Franka-Lift-Cube-IK --headless
./isaaclab.sh -p /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube/play_sb3.py --task Franka-Lift-Cube-IK --num_envs 4 --checkpoint /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube/logs/sb3/ppo/Franka-Lift-Cube-IK/2024-11-05_11-49-27/model_49152000_steps.zip
./isaaclab.sh -p /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube/play_sb3.py --task Franka-Lift-Cube-IK --num_envs 4 --checkpoint /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube/logs/sb3/ppo/Franka-Lift-Cube-IK/2024-11-05_16-31-32/model.zip



# Tensorboard
tensorboard --logdir='directory'



# RL games - UR5e

source isaaclab/bin/activate
cd isaaclab/IsaacLab
./isaaclab.sh -p /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube/train_rl_games.py --num_envs 2048 --task UR5e-Lift-Cube-IK --headless --max_iterations 1500
./isaaclab.sh -p /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube/play_rl_games.py --task UR5e-Lift-Cube-IK --num_envs 4 --checkpoint /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube/logs/rl_games/UR5e-Lift-Cube-IK/2024-11-04_15-25-12/nn/last_Ur5e-Lift-Cube_ep_1500_rew_1.097335.pth

python3 train_rl_games.py --num_envs 2048 --task UR5e-Lift-Cube-IK --headless --max_iterations 1000




# RL games - Franka

source isaaclab/bin/activate
cd isaaclab/IsaacLab
./isaaclab.sh -p /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube/train_rl_games.py --num_envs 4096 --task Franka-Lift-Cube-IK --headless --max_iterations 2500
./isaaclab.sh -p /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube/play_rl_games.py --num_envs 4 --task Franka-Lift-Cube-IK --use_last_checkpoint






##########################
# Joint Position Control #
##########################

# RL games

# Train RL agent in the manager-based RL environment sb3-ppo, inspired by the Isaac-Lift-Cube-Franka-v0 environment
source isaaclab/bin/activate
cd isaaclab/IsaacLab
./isaaclab.sh -p /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube/train_rl_games.py --num_envs 4096 --task UR5e-Lift-Cube --headless --max_iterations 1500



# Play the trained agent
source isaaclab/bin/activate
cd isaaclab/IsaacLab
./isaaclab.sh -p /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube/play_rl_games.py --task UR5e-Lift-Cube --num_envs 4 --checkpoint /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube/logs/rl_games/UR5e-Lift-Cube/2024-10-25_12-22-41/nn/last_Peg-in-hole_ep_750_rew_91.63596.pth
./isaaclab.sh -p /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube/play_rl_games.py --task UR5e-Lift-Cube --num_envs 4 --use_last_checkpoint




# Stable-baselines3 - UR5e

source isaaclab/bin/activate
cd isaaclab/IsaacLab
./isaaclab.sh -p /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube/train_sb3.py --num_envs 512 --task UR5e-Lift-Cube --headless
./isaaclab.sh -p /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube/play_sb3.py --task UR5e-Lift-Cube --num_envs 4 --checkpoint /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube/logs/sb3/UR5e-Lift-Cube/2024-10-25_14-27-06/model_3584000_steps.zip





##############################
# Save expert demonstrations #
##############################
source isaaclab/bin/activate
cd isaaclab/IsaacLab
./isaaclab.sh -p /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube/save_expert_demonstrations.py --num_envs 1 --task UR5e-Lift-Cube
