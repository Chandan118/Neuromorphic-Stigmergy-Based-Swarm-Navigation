import os
import subprocess
import random

num_robots = 50
print("Commanding swarm to move...")

# Generate a continuous publisher for each robot
processes = []
for i in range(num_robots):
    name = f"robot_{i}"
    # Random forward speed and turn rate to make them fan out organically
    linear_x = random.uniform(0.1, 0.5)
    angular_z = random.uniform(-0.2, 0.2)
    
    twist_msg = f'linear: {{x: {linear_x}}}, angular: {{z: {angular_z}}}'
    topic = f'/model/{name}/cmd_vel'
    
    # We will use gz topic -t <topic> -m gz.msgs.Twist -p <msg>
    cmd = [
        "gz", "topic", "-t", topic,
        "-m", "gz.msgs.Twist",
        "-p", twist_msg
    ]
    # Fire and forget (publish a single message, velocity control plugin will hold it)
    subprocess.run(cmd)

print("Swarm commands dispatched! Check the Gazebo GUI.")
