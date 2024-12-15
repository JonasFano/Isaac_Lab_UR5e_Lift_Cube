#######################
# IK Relative Control #
#######################

# Wandb for PPO IK: # Scale of 0.4
source env_isaacsim/bin/activate
cd /home/jofa/Downloads/Omniverse/Peg_in_hole/Isaac_Lab_Lift_Cube
wandb sweep --project rel_ik_sb3_ppo_franka_cube_lift config.yaml
wandb agent jofan23-university-of-southern-denmark/rel_ik_sb3_ppo_franka_cube_lift/6ag02hxq

source env_isaacsim/bin/activate
cd env_isaacsim/IsaacLab
./isaaclab.sh -p /home/jofa/Downloads/Omniverse/Peg_in_hole/Isaac_Lab_Lift_Cube/play_sb3.py --task Franka-Lift-Cube-IK --num_envs 4 --checkpoint /home/jofa/Downloads/Omniverse/Peg_in_hole/Isaac_Lab_Lift_Cube/models/zto2onj2/model.zip


# Scale of 0.1
source env_isaacsim/bin/activate
cd /home/jofa/Downloads/Omniverse/Peg_in_hole/Isaac_Lab_Lift_Cube
wandb sweep --project rel_ik_sb3_ppo_franka_cube_lift_0_1 config_0_1.yaml
 wandb agent jofan23-university-of-southern-denmark/rel_ik_sb3_ppo_franka_cube_lift_0_1/yv930ihp

# Scale of 0.6
source env_isaacsim/bin/activate
cd /home/jofa/Downloads/Omniverse/Peg_in_hole/Isaac_Lab_Lift_Cube
wandb sweep --project rel_ik_sb3_ppo_franka_cube_lift_0_6 config_0_6.yaml
wandb agent jofan23-university-of-southern-denmark/rel_ik_sb3_ppo_franka_cube_lift_0_6/m1okf79k

# Scale of 1.0
source env_isaacsim/bin/activate
cd /home/jofa/Downloads/Omniverse/Peg_in_hole/Isaac_Lab_Lift_Cube
wandb sweep --project rel_ik_sb3_ppo_franka_cube_lift_1_0 config_1_0.yaml
wandb agent jofan23-university-of-southern-denmark/rel_ik_sb3_ppo_franka_cube_lift_1_0/f4zmxtfo


# UR5e Scale of 0.5
source env_isaacsim/bin/activate
cd /home/jofa/Downloads/Omniverse/Peg_in_hole/Isaac_Lab_Lift_Cube
wandb sweep --project rel_ik_sb3_ppo_ur5e_cube_lift_0_5 config_0_5.yaml
wandb agent jofan23-university-of-southern-denmark/rel_ik_sb3_ppo_ur5e_cube_lift_0_5/f4qa2m1s

# UR5e Scale of 0.1
source env_isaacsim/bin/activate
cd /home/jofa/Downloads/Omniverse/Peg_in_hole/Isaac_Lab_Lift_Cube
wandb sweep --project rel_ik_sb3_ppo_ur5e_cube_lift_0_1 config_0_1.yaml
wandb agent jofan23-university-of-southern-denmark/rel_ik_sb3_ppo_ur5e_cube_lift_0_1/tdqr4rkv

source env_isaacsim/bin/activate
cd env_isaacsim/IsaacLab
./isaaclab.sh -p /home/jofa/Downloads/Omniverse/Peg_in_hole/Isaac_Lab_Lift_Cube/play_sb3.py --task UR5e-Lift-Cube-IK --num_envs 4 --checkpoint /home/jofa/Downloads/Omniverse/Peg_in_hole/Isaac_Lab_Lift_Cube/models/cdfw6vbl/model.zip


# UR5e Scale of 1.0
source env_isaacsim/bin/activate
cd /home/jofa/Downloads/Omniverse/Peg_in_hole/Isaac_Lab_Lift_Cube
wandb sweep --project rel_ik_sb3_ppo_ur5e_cube_lift_1_0 config_1_0.yaml
wandb agent jofan23-university-of-southern-denmark/rel_ik_sb3_ppo_ur5e_cube_lift_1_0/s35jz0ur


# UR5e Scale of 0.1 v2
source env_isaacsim/bin/activate
cd /home/jofa/Downloads/Omniverse/Peg_in_hole/Isaac_Lab_Lift_Cube
wandb sweep --project rel_ik_sb3_ppo_ur5e_cube_lift_0_1_v2 config_0_1.yaml
wandb agent jofan23-university-of-southern-denmark/rel_ik_sb3_ppo_ur5e_cube_lift_0_1_v2/r5jnlkvc

# UR5e Scale of 0.1 v3
source env_isaacsim/bin/activate
cd /home/jofa/Downloads/Omniverse/Peg_in_hole/Isaac_Lab_Lift_Cube
wandb sweep --project rel_ik_sb3_ppo_ur5e_cube_lift_0_1_v3 config_0_1.yaml
wandb agent jofan23-university-of-southern-denmark/rel_ik_sb3_ppo_ur5e_cube_lift_0_1_v3/7h0k9bqm

source env_isaacsim/bin/activate
cd env_isaacsim/IsaacLab
./isaaclab.sh -p /home/jofa/Downloads/Omniverse/Peg_in_hole/Isaac_Lab_Lift_Cube/play_sb3.py --task UR5e-Lift-Cube-IK --num_envs 4 --checkpoint /home/jofa/Downloads/Omniverse/Peg_in_hole/Isaac_Lab_Lift_Cube/models/wkhfbabm/model.zip




##########################
# Joint Position Control #
##########################

# Wandb for SAC Joint Position:
source env_isaacsim/bin/activate
cd /home/jofa/Downloads/Omniverse/Peg_in_hole/Isaac_Lab_Lift_Cube
wandb sweep --project joint_pos_sb3_sac_ur5e_cube_lift config_sb3_sac.yaml
wandb agent jofan23-university-of-southern-denmark/joint_pos_sb3_sac_ur5e_cube_lift/2mgx7y2r


# Wandb for TD3 Joint Position:
source env_isaacsim/bin/activate
cd /home/jofa/Downloads/Omniverse/Peg_in_hole/Isaac_Lab_Lift_Cube
wandb sweep --project joint_pos_sb3_td3_ur5e_cube_lift config_sb3_td3.yaml
wandb agent jofan23-university-of-southern-denmark/joint_pos_sb3_td3_ur5e_cube_lift/x8ds52ce