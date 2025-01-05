#######################
# IK Relative Control #
#######################


# UR5e Scale of 0.05
source isaaclab/bin/activate
cd /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube
wandb sweep --project rel_ik_sb3_ppo_ur5e_lift_cube_0_05_final config_0_05.yaml
wandb agent jofan23-university-of-southern-denmark/rel_ik_sb3_ppo_ur5e_lift_cube_0_05_final/u64j6irx
