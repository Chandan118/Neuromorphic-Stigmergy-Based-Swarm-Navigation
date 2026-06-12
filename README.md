# Neuromorphic Stigmergy-Based Swarm Navigation

![ROS 2](https://img.shields.io/badge/ROS_2-Iron-blue.svg)
![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-orange.svg)
![Build Status](https://github.com/Chandan118/Neuromorphic-Stigmergy-Based-Swarm-Navigation/actions/workflows/ros2_build.yml/badge.svg)

This repository contains a highly scalable framework for simulating massive multi-robot swarms (up to 500+ differential drive robots) executing neuromorphic, stigmergy-based navigation policies. It includes both a **Rigorous MATLAB Physics Simulator** for algorithm validation and benchmarking, and a **ROS 2 Workspace** for hardware-ready deployment and Gazebo simulation.

## Features & Upgrades (T-RO Rigor)

To ensure the highest standard of algorithm validation, this repository features:
- **Rigid Body Dynamics**: Robots are modeled with precise mass, inertia, wheelbases, and non-holonomic limits.
- **Physical Stigmergy (PDEs)**: Pheromone trails are simulated using Partial Differential Equations computed via Finite Difference Methods to handle real-world diffusion and evaporation.
- **Realistic Sensor Noise**: Simulated Lidar and IMU sensors actively inject Gaussian noise and drift to test the Spiking Neural Network (SNN) filter capabilities in GPS-denied, cluttered environments (e.g., Martian terrain mazes).

### Strong Algorithm Baselines
To prove the computational efficiency and success rate of our SNN, the repository includes two modern surrogates for direct benchmarking:
1. **Particle Swarm Optimization (PSO)**: A classic baseline utilizing global/local best gradients.
2. **Multi-Agent Reinforcement Learning (MARL)**: A decentralized Q-learning heuristic.

---

## 1. Running the MATLAB Rigor Experiments

The `matlab_sim` directory contains the core benchmarking suite. 

To run the experiments, open MATLAB, navigate to `matlab_sim/experiments/`, and execute:

```matlab
% 1. Direct Baseline Comparisons (SNN vs PSO vs MARL)
run_baseline_comparisons

% 2. Ablation Studies (SNN/Stigmergy toggle)
run_ablation_studies

% 3. Scalability Benchmarks (10 to 500 robots)
run_scalability_benchmarks
```

You can also run the core visual physics loop interactively with default settings:
```matlab
cd matlab_sim/
main_simulation
```

---

## 2. Running the ROS 2 C++ Workspace (Gazebo/Hardware)

The `src/` directory contains highly efficient, edge-ready C++ ROS 2 nodes (`neuromorphic_snn_controller` and `stigmergy_physics_env`). To run the full physics engine on a dedicated Linux environment (Ubuntu 22.04 LTS, ROS 2 Iron):

```bash
# Clone the repository
git clone https://github.com/Chandan118/Neuromorphic-Stigmergy-Based-Swarm-Navigation.git
cd Neuromorphic-Stigmergy-Based-Swarm-Navigation

# Source your ROS 2 installation
source /opt/ros/iron/setup.bash

# Build the packages natively
colcon build --symlink-install

# Source the local install
source install/setup.bash

# Launch the Gazebo UI and ROS 2 bridges
ros2 launch swarm_bringup swarm_simulation.launch.py
```

### Automated Experiments & Data Logging
To run the automated 60-minute experimental suite (executing all 10 macroscopic behaviors like Flocking, Perimeter Defense, etc.):

```bash
python3 experiment_orchestrator.py
```

To capture and plot quantitative trajectories (`swarm_trajectories.png`):
```bash
python3 trajectory_logger.py
# After completion:
python3 plot_results.py
```
