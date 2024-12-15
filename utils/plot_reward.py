import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Replace with the path to your log file
log_file_path = '/home/jofa/Downloads/Omniverse/Peg_in_hole/Isaac_Lab_Lift_Cube/logs/sb3/ppo/Franka-Lift-Cube-IK/2024-11-05_10-57-00/progress.csv'


log_file_path = "/home/jofa/Downloads/Omniverse/Peg_in_hole/Isaac_Lab_Lift_Cube/logs/sb3/ppo/Franka-Lift-Cube-IK/2024-11-05_16-31-32/progress.csv"

log_file_path = "/home/jofa/Downloads/Omniverse/Peg_in_hole/Isaac_Lab_Lift_Cube/logs/sb3/ppo/UR5e-Lift-Cube-IK/2024-11-11_09-16-12/progress.csv"

log_file_path = "/home/jofa/Downloads/Omniverse/Peg_in_hole/Isaac_Lab_Lift_Cube/logs/sb3/ppo/UR5e-Lift-Cube-IK/2024-11-18_21-51-22/progress.csv"

# Read the log file
data = pd.read_csv(log_file_path)

# Extract the relevant columns
timesteps = data['time/total_timesteps']
mean_rewards = data['rollout/ep_rew_mean']

# Remove rows where ep_rew_mean is NaN
valid_indices = ~mean_rewards.isna()
timesteps = timesteps[valid_indices].reset_index(drop=True)
mean_rewards = mean_rewards[valid_indices].reset_index(drop=True)

# Check if we have valid data
if timesteps.empty or mean_rewards.empty:
    print("No valid data available for plotting.")
else:
    # Create the plot
    plt.figure(figsize=(12, 6))
    plt.plot(timesteps, mean_rewards, color='blue', linewidth=2)
    
    # Add titles and labels
    plt.title('Mean Episode Reward Over Timesteps')
    plt.xlabel('Total Timesteps')
    plt.ylabel('Mean Episode Reward')
    plt.grid(True)
    
    # Show the plot
    plt.show()
