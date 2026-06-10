import pandas as pd
import matplotlib.pyplot as plt
import os

# Check if results exist
if not os.path.exists('experiment_results.csv'):
    print("Error: experiment_results.csv not found!")
    print("Make sure you run trajectory_logger.py during the simulation to generate the data.")
    exit(1)

# Load data
df = pd.read_csv('experiment_results.csv')

# Plot swarm trajectories
plt.figure(figsize=(10, 8))
for robot_id in df['robot_id'].unique():
    robot_data = df[df['robot_id'] == robot_id]
    plt.plot(robot_data['x'], robot_data['y'], label=f'Robot {robot_id}', alpha=0.6, linewidth=1.5)

plt.title('Neuromorphic Swarm Trajectories (50 Robots)')
plt.xlabel('X Position (meters)')
plt.ylabel('Y Position (meters)')
plt.grid(True)
plt.axis('equal')

# Save plot to image
plt.savefig('swarm_trajectories.png', dpi=300, bbox_inches='tight')
print("Successfully generated plot: swarm_trajectories.png")
