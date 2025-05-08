Arrange and format finally for my git  , avoid redundancy and add if anthing i missed 

# 🚗 CARLA_ROS2_MPC (Still under development)

This project integrates the CARLA autonomous driving simulator within the ROS 2 Humble framework to implement and test developed MPC algorithm for vehicle path tracking.

---

## 1. Introduction

This project integrates the CARLA autonomous driving simulator with ROS 2 Humble to implement and test a Model Predictive Control (MPC) algorithm for vehicle path tracking. The core objective is to simulate realistic vehicle control in a dynamic driving environment using MPC with CARLA's physics and visualization engine, allowing for development and future extension toward real-time, sensor-based, or hardware-in-the-loop implementations.

---

## 2. Project Objectives

- Develop an MPC-based steering and throttle control for an autonomous vehicle in a simulated urban scenario.
- Integrate the control system with the CARLA-ROS2 stack.
- Publish a static navigation goal and track the optimal path in real-time.
- Visualize the reference, optimized path, and actual vehicle movement using RViz2.
- Evaluate system behavior and performance through simulation.


## 3. 🧠 System Architecture Overview

The following diagram describes the communication flow between various ROS 2 nodes and topics used in this MPC-based autonomous driving system:

![ROS2 Node Graph](https://github.com/user-attachments/assets/de0edced-67b6-404b-a35c-743045b46bf7)


### Key Components:

- **`/static_goal_publisher`**  
  Publishes a fixed goal pose to `/carla/agent_0/goal`.

- **`/carla_waypoint_publisher`**  
  Subscribes to the goal and publishes a global reference path to `/carla/agent_0/waypoints`.

- **`/carla_ros_bridge`**  
  Bridges CARLA and ROS2 by publishing:
  - `/carla/agent_0/odometry`, `/vehicle_info`, `/vehicle_status`
  - `/lidar`, `/rgb_view/image`, `/enable_autopilot`, etc.

- **`/mpc_town04_node`**  
  Core node subscribing to:
  - `/carla/agent_0/odometry`, `/waypoints`  
  Publishes:
  - `/vehicle_control_cmd`, `/dubin_path`, `/opt_path`, `/car_marker`

- **`/carla_manual_control`**  
  Manual override tool subscribing to vehicle control/feedback topics.

- **Visualization & TF Nodes**  
  - `/carla_map_visualization`, `/goal_pose`, `/transform_listener`


## 4. Repository Structure
  ```bash
carla-ros-bridge/
├── catkin_ws/
│   ├── build/
│   ├── install/
│   ├── log/
│   └── src/
│       ├── carla-ros-bridge/
│       │   ├── carla_ros_bridge/
│       │   ├── carla_spawn_objects/
│       │   └── carla_waypoint_publisher/
│       └── mpc_ros/
│           ├── configure/
│           │   └── config.yaml
│           ├── launch/
│           │   └── mpc_Town04.launch.py
│           └── src/mpc_ros/
│               ├── static_goal_publisher.py
│               ├── mpc_town04_node.py
│               └── mpc_path_tracking.py
└── README.md
  ```

## 5. Software Stack and Dependencies

- **Ubuntu**: 22.04
- **CUDA**: 12.4 (Nvidia Driver 550.xx)
- **Python**: 3.10
- **ROS 2**: Humble
- **CARLA**: 0.9.15
- **Carla-ROS-Bridge**: [GitHub Repo](https://github.com/carla-simulator/carla-ros-bridge)
- **Python Libraries**:
  ```bash
  pip install numpy cvxpy casadi osqp`
  ```
- **Build Tool**: Colcon

---

### 5.1 Install CARLA

Refer to the official CARLA releases:  
[CARLA Releases](https://github.com/carla-simulator/carla/releases)

---

### 5.2 Install ROS 2 Humble

Follow the official ROS 2 Humble installation guide:  
[ROS 2 Humble Installation Guide](https://docs.ros.org/en/humble/Installation.html)

```bash
cd ~/carla-ros-bridge/catkin_ws/src
git clone https://github.com/your-org/carla_ros2_mpc.git

cd ~/carla-ros-bridge/catkin_ws
colcon build --symlink-install

source /opt/ros/humble/setup.bash
source install/setup.bash
```


## 6. Run Simulation

### Terminal 1 - Launch CARLA

```bash
./CarlaUE4.sh ---prefernvidia
```

### Terminal 2 - Launch ROS 2 + town04 Map 

```bash
ros2 launch carla_ros_bridge run_car_sim_Town04.launch.py
```

### Terminal 3 - MPC stack
```bash
ros2 launch mpc_ros mpc_Town04.launch.py
```

### RViz2 Displays:

- `/dubin_path` – Yellow (reference path)
- `/opt_path` – Green (MPC optimized path)
- `/car_marker` – Actual vehicle position
- `/carla/agent_0/odometry`


## 7. Node Descriptions

### 7.1 static_goal_publisher.py

- **Function**: Publishes a fixed `PoseStamped` goal.
- **Topic**: `/carla/agent_0/goal`
- **Edit**: Modify the coordinates in the code to change the goal position.

---

### 7.2 carla_waypoint_publisher

- **Function**: Uses CARLA’s GlobalRoutePlanner to compute waypoints.
- **Process**: Converts the goal into a global reference trajectory.
- **Topic**: `/carla/agent_0/waypoints`

---

### 7.3 mpc_town04_node.py

- **Subscribes to**:
  - `/carla/agent_0/odometry`
  - `/carla/agent_0/waypoints`

- **Publishes**:
  - `/carla/agent_0/vehicle_control_cmd`
  - `/dubin_path`, `/opt_path`, `/car_marker`

- **Logs**: Data is logged to CSV files.

---

### 7.4 mpc_path_tracking.py

- **Function**: Implements MPC using linearized bicycle dynamics.
- **Solver**: Solves QP using CasADi + IPOPT (fallback: OSQP).
- **Fallback**: Handles infeasibility with a safety fallback strategy.

---

### Trails
![Trails](https://github.com/user-attachments/assets/beeca252-3d9c-4dfd-9112-4deb7c6d1bde)


## 8. Data Logging & Plotting

### CSV Logging

- `mpc_town04_node.py` logs data to `~/mpc_logs/` (timestamped files):
  - **vehicle_data_*.csv**: Contains columns for `Time`, `X`, `Y`, `Yaw`, and `Closest_WP_Index`.
  - **control_data_*.csv**: Contains columns for `Time`, `Throttle`, `Steer`, and `Brake`.

---

### Plotting Script

- `plot_mpc_data.py` reads these CSVs to generate:
  - Vehicle trajectory plot.
    ![Trail_v1](https://github.com/user-attachments/assets/0dabdebb-77b9-4bc2-83cf-0f16d5347f4b)
  - Control inputs over time plot.
    ![Trail_v1](https://github.com/user-attachments/assets/7b7c94c7-cbbb-43dc-af61-6e70647f54d4)


## 9. Troubleshooting Tips

Below are common issues encountered during development and their solutions:

### Solver Failure (e.g., `SolverError: Solver 'OSQP' failed`)

**Cause**: Numerical instability or infeasibility in the QP problem.

**Solution**: Ensure `casadi` is installed (`pip install casadi`). Increase solver tolerances or iteration limits in `mpc_path_tracking.py` (e.g., `ipopt.tol=1e-2`, `ipopt.max_iter=1000`). Check if reference path is too far from current position, causing infeasible constraints.

### No Waypoints Received

**Cause**: `static_goal_publisher` didn’t publish or `carla_waypoint_publisher` failed.

**Solution**: Verify goal publication with `ros2 topic echo /carla/agent_0/goal`. Ensure CARLA server is running on `localhost:2000`.

### Vehicle Won’t Move

**Cause**: No control commands or solver always fails.

**Solution**: Check `ros2 topic echo /carla/agent_0/vehicle_control_cmd` for throttle/steer values. If zero, debug solver in `mpc_path_tracking.py`.

### RViz2 Shows Nothing

**Cause**: Missing display configurations.

**Solution**: In RViz2, add displays for `/dubin_path` (yellow path), `/opt_path` (green path), and `/car_marker` (vehicle marker). Set fixed frame to `map`.

### CARLA Connection Errors

**Cause**: CARLA server not running or wrong port.

**Solution**: Start CARLA server first (`./CarlaUE4.sh`). Confirm port in launch file matches (2000 by default).

---

## 10. Summary

Throughout the development process, the following key contributions were made:

- **Pipeline Design**: Created a full CARLA-ROS2-MPC pipeline with modular nodes for goal setting, path planning, and control.
- **Visualization**: Integrated RViz2 markers for reference path (`/dubin_path`), optimized horizon (`/opt_path`), and vehicle pose (`/car_marker`) for real-time monitoring.

---

## 11. 🙏 Acknowledgments

Special thanks to the following resources, whose contributions made this work possible:

- [carla-ros-bridge(Main fork)](https://github.com/carla-simulator/ros-bridge)
- [bearswang/mpc-ros](https://github.com/bearswang/mpc-ros)
- [LearnOpenCV ROS2 PID blog](https://learnopencv.com/pid-controller-ros-2-carla/)
- [MPC-Berkeley/carla-ros-bridge](https://github.com/MPC-Berkeley/carla-ros-bridge)
