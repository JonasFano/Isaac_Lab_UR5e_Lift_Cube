#######################
# IK Relative Control #
#######################


# UR5e Scale of 1.0
source isaaclab/bin/activate
cd /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube
wandb sweep --project rel_ik_sb3_ppo_ur5e_cube_lift_0_05 config_0_05.yaml

