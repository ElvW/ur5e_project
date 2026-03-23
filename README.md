
# UR5e Gazebo MoveIt2 Setup

This repository provides a containerized infrastructure for simulating the Universal Robots UR5e arm. It satisfies the core requirements for the robotics thesis starter kit, including physics simulation, motion planning, and data logging.

## 🛠 Setup & Installation

This project is fully containerized using Docker and WSL2 to ensure environment reproducibility.

1. **Clone the Repository:**
   ```bash
   git clone <your-repo-link>
   cd ur5e_starter_kit

2. **Build and Run the Docker Container:**
The following script builds the Dockerfile and launches the container with GUI (Gazebo/RViz) support enabled:
    ```bash
    chmod +x run_docker.sh
    ./run_docker.sh
3. **Workspace Compilation (Inside Container):**
Once inside the container, clone the simulation source code and compile the workspace:
    ```bash
    git clone -b humble [https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation.git](https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation.git) src/ur_simulation
    colcon build --symlink-install
    source install/setup.bash

## 🚀 Execution Guide
1. **Launching Gazebo Simulation:**
To initialize the physics world and spawn the UR5e robot:
    ```bash
    ros2 launch ur_simulation_gazebo ur_sim_control.launch.py ur_type:=ur5e launch_rviz:=false

2. **MoveIt2 Motion Planning**
Open a second terminal, connect to the running container (docker exec), and launch the MoveIt2 "brain":
    ```bash
    source install/setup.bash
    ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true use_sim_time:=true

* Planning: Drag the interactive marker at the robot tip in RViz and click the "Plan" button.

* Execution: Click "Execute" to send the trajectory to Gazebo and watch the physical arm move.

## 📊 Data Logging & Reproducibility (rosbag2)

To record robot state data for offline analysis:
    
    ros2 bag record /joint_states -o my_first_move

Stop the recording using Ctrl+C. To verify the data and replay the movement:

    ros2 bag info my_first_move
    ros2 bag play my_first_move
