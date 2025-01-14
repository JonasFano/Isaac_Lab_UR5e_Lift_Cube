#######################
# IK Relative Control #
#######################


# UR5e Scale of 0.05

# PPO
source isaaclab/bin/activate
cd /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube
wandb sweep --project rel_ik_sb3_ppo_ur5e_lift_cube_0_05_final config_0_05.yaml
wandb agent jofan23-university-of-southern-denmark/rel_ik_sb3_ppo_ur5e_lift_cube_0_05_final/u64j6irx

wandb sweep --project rel_ik_sb3_ppo_ur5e_lift_cube_0_05_hand_e config_0_05.yaml


# DDPG
source isaaclab/bin/activate
cd /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube
wandb sweep --project rel_ik_sb3_ddpg_ur5e_lift_cube_0_05 config_sb3_ddpg.yaml
wandb agent jofan23-university-of-southern-denmark/rel_ik_sb3_ddpg_ur5e_lift_cube_0_05/umvvcp9f


# TD3
source isaaclab/bin/activate
cd /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube
wandb sweep --project rel_ik_sb3_td3_ur5e_lift_cube_0_05 config_sb3_td3.yaml
wandb agent jofan23-university-of-southern-denmark/rel_ik_sb3_td3_ur5e_lift_cube_0_05/na8cl8j7

wandb sweep --project rel_ik_sb3_td3_ur5e_lift_cube_0_05_noise_1_0 config_sb3_td3.yaml
wandb sweep --project rel_ik_sb3_td3_ur5e_lift_cube_0_05_noise_100 config_sb3_td3.yaml



./isaaclab.sh -p /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube/play_sb3.py --task UR5e-Lift-Cube-IK --num_envs 4 --checkpoint /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube/logs/sb3/ppo/UR5e-Lift-Cube-IK/adjusted_reward_tcp_6_v2/model.zip

# PPO
python3 train_sb3.py --task UR5e-Lift-Cube-IK --num_envs 4 --no_logging --headless
python3 play_sb3.py --task UR5e-Lift-Cube-IK --num_envs 4 --checkpoint /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube/models/dond3q8q/model.zip

python3 play_sb3.py --task UR5e-Lift-Cube-IK --num_envs 4 --checkpoint /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube/models/urjhlqn1/model.zip
python3 play_sb3.py --task UR5e-Lift-Cube-IK --num_envs 4 --checkpoint /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube/models/vsccu703/model.zip


# TD3
python3 play_sb3_td3.py --task UR5e-Lift-Cube-IK --num_envs 4 --checkpoint /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube/models/yh1yv5bu/model.zip


# Remote control
ssh -I 10.178.107.200 jofa@ia-li-2wqd414.unirobotts.local