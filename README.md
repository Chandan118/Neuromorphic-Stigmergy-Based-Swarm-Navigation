# Neuromorphic Stigmergy-Based Swarm Navigation

![ROS 2](https://img.shields.io/badge/ROS_2-Iron-blue.svg)
![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-orange.svg)
![Build Status](https://github.com/Chandan118/Neuromorphic-Stigmergy-Based-Swarm-Navigation/actions/workflows/ros2_build.yml/badge.svg)

This repository contains a highly scalable ROS 2 workspace for simulating massive multi-robot swarms (50+ differential drive robots) executing neuromorphic, stigmergy-based navigation policies.

The simulation environment is built on **Gazebo Harmonic** and demonstrates 10 unique macroscopic swarm behaviors, including:
1. Random Exploration
2. Aggregation (Rendezvous)
3. Dispersion (Anti-Flocking)
4. Flocking (Alignment)
5. Stigmergy - Trail Laying
6. Stigmergy - Gradient Ascent
7. Perimeter Defense
8. Follow the Leader
9. Pattern Formation
10. Return to Base

## Requirements

To run the full physics engine and ROS 2 nodes, you will need a dedicated Linux environment:
- **OS**: Ubuntu 22.04 LTS
- **Middleware**: ROS 2 Iron Irwini
- **Simulation**: Gazebo Harmonic

## Building the Workspace

Clone the repository and build it using `colcon`:

```bash
# Clone the repository
git clone https://github.com/Chandan118/Neuromorphic-Stigmergy-Based-Swarm-Navigation.git
cd Neuromorphic-Stigmergy-Based-Swarm-Navigation

# Source your ROS 2 installation
source /opt/ros/iron/setup.bash

# Build the packages natively
colcon build --symlink-install
```

## Running the Simulation

After building, you can launch the Gazebo environment and spawn the 50 robots:

```bash
# Source the local install
source install/setup.bash

# Launch the Gazebo UI and ROS 2 bridges
ros2 launch swarm_bringup swarm_simulation.launch.py
```

### Automated Experiments

To run the automated 60-minute experimental suite (executing all 10 tasks sequentially), run the Python orchestrator in a separate terminal:

```bash
python3 experiment_orchestrator.py
```

### Data Logging and Results

To capture quantitative results (e.g., the precise X/Y/Z coordinate paths of all 50 robots) for analysis, start the data logger while the experiment is running:

```bash
source /opt/ros/iron/setup.bash
python3 trajectory_logger.py
```

This will generate an `experiment_results.csv` file. Once the experiment completes, you can visualize the swarm's trajectories by running:

```bash
python3 plot_results.py
```

This will parse the dataset and generate a high-resolution graph (`swarm_trajectories.png`) of the swarm's aggregated movements.
