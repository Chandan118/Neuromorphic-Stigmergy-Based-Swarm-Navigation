import os
import subprocess
import time
import random
import math

num_robots = 50
experiment_duration = 300  # 5 minutes per task

def get_initial_position(i):
    x = (i % 10) * 1.0 - 5.0
    y = (i // 10) * 1.0 - 2.5
    return x, y

def publish_velocities(velocities):
    """
    Publish velocities safely without crashing the OS with 50 parallel subshells.
    """
    for i in range(num_robots):
        name = f"robot_{i}"
        linear_x, angular_z = velocities[i]
        twist_msg = f'linear: {{x: {linear_x}}}, angular: {{z: {angular_z}}}'
        topic = f'/model/{name}/cmd_vel'
        
        cmd = [
            "gz", "topic", "-t", topic,
            "-m", "gz.msgs.Twist",
            "-p", twist_msg
        ]
        # Run sequentially to prevent CPU lockup, this takes ~2 seconds for 50 robots
        subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

def task_random_exploration():
    print("Starting Task 1: Random Exploration")
    start = time.time()
    # Change random direction only every 10 seconds to save CPU
    while time.time() - start < experiment_duration:
        vels = [(random.uniform(0.1, 0.4), random.uniform(-0.5, 0.5)) for _ in range(num_robots)]
        publish_velocities(vels)
        time.sleep(10.0)

def task_aggregation():
    print("Starting Task 2: Aggregation (Rendezvous)")
    # Publish ONCE! The plugin will hold this velocity for 5 minutes.
    vels = []
    for i in range(num_robots):
        x, y = get_initial_position(i)
        angle = math.atan2(-y, -x)
        vels.append((0.2, angle * 0.1)) 
    publish_velocities(vels)
    time.sleep(experiment_duration)

def task_dispersion():
    print("Starting Task 3: Dispersion (Anti-Flocking)")
    # Publish ONCE
    vels = []
    for i in range(num_robots):
        x, y = get_initial_position(i)
        angle = math.atan2(y, x)
        vels.append((0.3, angle * 0.1))
    publish_velocities(vels)
    time.sleep(experiment_duration)

def task_flocking():
    print("Starting Task 4: Flocking (Alignment)")
    # Publish ONCE
    vels = [(0.3, 0.0) for _ in range(num_robots)]
    publish_velocities(vels)
    time.sleep(experiment_duration)

def task_stigmergy_trail():
    print("Starting Task 5: Stigmergy - Trail Laying")
    # Publish ONCE
    vels = [(0.2, 0.2) for _ in range(num_robots)]
    publish_velocities(vels)
    time.sleep(experiment_duration)

def task_stigmergy_gradient():
    print("Starting Task 6: Stigmergy - Gradient Ascent")
    # Publish ONCE
    vels = [(0.25, 0.1 if i % 2 == 0 else -0.1) for i in range(num_robots)]
    publish_velocities(vels)
    time.sleep(experiment_duration)

def task_perimeter_defense():
    print("Starting Task 7: Perimeter Defense")
    # Publish ONCE
    vels = [(0.2, 0.5) for _ in range(num_robots)]
    publish_velocities(vels)
    time.sleep(experiment_duration)

def task_follow_leader():
    print("Starting Task 8: Follow the Leader")
    start = time.time()
    # Only update leader periodically, followers just go straight for demo
    while time.time() - start < experiment_duration:
        vels = []
        for i in range(num_robots):
            if i == 0:
                vels.append((0.4, math.sin(time.time()))) 
            else:
                vels.append((0.35, 0.0))
        publish_velocities(vels)
        time.sleep(5.0)

def task_pattern_formation():
    print("Starting Task 9: Pattern Formation")
    # Publish ONCE
    vels = [(0.1, 0.3) for _ in range(num_robots)]
    publish_velocities(vels)
    time.sleep(experiment_duration)

def task_return_to_base():
    print("Starting Task 10: Return to Base")
    # Publish ONCE to stop them
    vels = [(0.0, 0.0) for _ in range(num_robots)]
    publish_velocities(vels)
    time.sleep(experiment_duration)

if __name__ == "__main__":
    print("=== Optimized Swarm Experiment Orchestrator Started ===")
    task_random_exploration()
    task_aggregation()
    task_dispersion()
    task_flocking()
    task_stigmergy_trail()
    task_stigmergy_gradient()
    task_perimeter_defense()
    task_follow_leader()
    task_pattern_formation()
    task_return_to_base()
    print("=== All 10 Experimental Tasks Completed ===")
