# Remote control
ssh -I 10.178.107.200 jofa@ia-li-2wqd414.unirobotts.local


##############################
# Save expert demonstrations #
##############################
source isaaclab/bin/activate
cd isaaclab/IsaacLab
./isaaclab.sh -p /home/jofa/Downloads/Repositories/Isaac_Lab_UR5e_Lift_Cube/save_expert_demonstrations.py --num_envs 1 --task UR5e-Lift-Cube
