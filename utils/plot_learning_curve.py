import pandas as pd
import matplotlib.pyplot as plt

# Load CSV
df = pd.read_csv("lift_cube_domain_rand_training.csv")

# Create plot
plt.figure(figsize=(7, 4))  # inches
plt.plot(df["Step"], df["floral-sweep-1 - rollout/ep_rew_mean"])
plt.xlabel("Training Steps")
plt.ylabel("Mean Episode Reward")
plt.grid(True)
plt.tight_layout()

# Save as PDF
plt.savefig("mean_reward_plot.pdf", format="pdf", bbox_inches="tight")
