
# **LIDAR-based SLAM for omnidirectional robot**

The following repo contains the implementation of the core functionalities of a Simultaneous Localization and Mapping (SLAM) system for an omnidirectional 4-wheel mobile robot. The project was developed using ROS1 Melodic and C++ and tested in Ubuntu 18.04. To enable others to reproduce the workflow and test the custom nodes on their own hardware, the code can be run using Docker, both on Linux and Windows (using WSL).

## **Workflow Description**

The general workflow of the implemented functionalities can be divided into three parts:

1. **Custom Wheel Odometry**:
   - Wheel encoder velocities are transformed into linear and angular body frame velocities.
   - Odometry is computed from velocities to estimate the robot pose from wheel encoders.
   - Different integration methods can be used as well as custom robot parameters.

2. **SLAM with GMapping**:
   - Two LIDARs are merged using the tools provided by the [ira_laser_tools](https://github.com/iralabdisco/ira_laser_tools) repository.
   - Odometry and LIDAR data are used to incrementally build the map using SLAM from [GMapping](http://wiki.ros.org/gmapping).
   - The generated map is saved using `map_server` once mapping is complete.

3. **Localization with AMCL**:
   - The previously saved map is used to localize more precisely the robot in real-time using [AMCL](http://wiki.ros.org/amcl).
   - AMCL can be used for precise localization in additional navigation and control pipelines with a known map.
   - The robot's trajectory can be visualized and optionally saved as an image.

## **Project Structure**

This work is organized into custom packages and utilizes known ROS1 libraries to achieve SLAM and localization functionalities. Below is a breakdown of the custom-developed features and their integration with existing ROS tools.

### **1. Custom Packages**

#### **`omni_robot_odom`: Wheel Odometry Calculation**

- This custom ROS package was developed to compute odometry based on wheel encoder data only.
- A node subscribes to wheel encoder topic `/wheel_states` and calculates the robot’s body frame velocities using a kinematic model for omnidirectional robots.
- Another node reads the velocity data and computes odometry using various integration methods.
- Odometry is published to the `/odom` topic and provides the essential data for SLAM and localization.

#### **`slam_gmapping_amcl`: SLAM with GMapping and AMCL Localization**

- This custom ROS package uses Gmapping to build a 2D occupancy grid map of the environment in real-time.
- It integrates LIDAR data from the `/full_scan` topic (a merged scan from two LIDAR sensors) and odometry data from the `/odom` topic provided by the custom odometry package.
- GMapping also dynamically publishes the transform between the `map` frame and the `odom` frame.
- The AMCL package (Adaptive Monte Carlo Localization) refines the robot’s estimated position by using the map created by GMapping and the same LIDAR scan and odometry data topics.
- AMCL publishes the robot’s pose on the `/amcl_pose` topic and the dynamic transform between the `map` and the `odom` frame.
- A custom node subscribes to the `/amcl_pose` and `/map_metadata` topics to track the robot’s trajectory in real-time.
- Using OpenCV, the node overlays the robot’s path onto the generated map and offers functionality to save it as a `.pgm` image.
- A ROS service is provided to allow users to save the current trajectory at any point during localization.

### **2. TF Tree Structure**

- **Static Transforms**:
  - Static tf between the robot’s base (`base_link`) and its lidar sensors (`laser_front` and `laser_rear`) are defined and the relationship between frames `base_link` and `base_footprint`.
- **Dynamic Transforms**:
  - The custom odometry package publishes the dynamic transform between `odom` and `base_link`, representing the robot’s pose.
  - GMapping or AMCL dynamically publishes the transform between `map` and `odom`, correcting for odometry drift and needed for SLAM.

## **Instructions for Running**

### **1. WSL Setup (Windows Only)**

Initial setup for running project using WSL inside Windows:

- **WSL2 Backend**: Required for running Docker containers in Linux-based environment on Windows.
  - [Install WSL2](https://docs.microsoft.com/en-us/windows/wsl/install)
  - Hardware virtualization must be enabled in the BIOS/UEFI settings of your Windows machine

Setup WSL correctly and [WSLg](https://github.com/microsoft/wslg) support for GUI applications:
```
wsl --set-default-version 2
```
```
wsl --set-default <chosen-distro-name>
```
From here on follow Linux instructions. All commands have to be executed inside a WSL shell which can be accessed using:
```
wsl
```

### **2. Initialization**

Inside the cloned repo `/omni_robot_SLAM_project`, build and run the system with Docker compose:
```
docker compose up --build
```

To interact with the container, run the following command in a new terminal instance:
```
docker exec -it omni_robot_slam bash
```

Use terminator for better multi window control:
```
terminator
```

### **3. Run SLAM with Custom Odometry**

Launch the odometry and SLAM processes in two separate terminals:

- **Odometry**:
  ```
  roslaunch omni_robot_odom odometry.launch
  ```
  
- **SLAM**:
  ```
  roslaunch slam_gmapping_amcl mapping.launch
  ```

Once all nodes are ready, run the SLAM test bag inside the `/omni_robot_slam_ws/src/data/bags` folder:

```
rosbag play --clock slam.bag
```

or run only `odometry.launch` and the following bag, to test and tune wheel odometry separately:

```
rosbag play --clock odom.bag
```

### **4. Run AMCL Localization**

First save the map at the end of mapping running (inside `/omni_robot_slam_ws/src/slam_gmapping_amcl/maps` folder):
```
rosrun map_server map_saver -f final_map
```

Together with the odometry, launch the localization nodes by running:
```
roslaunch slam_gmapping_amcl localization.launch
```

and run the same SLAM bag:
```
rosbag play --clock slam.bag
```

At any point of the trajectory, save the map + trajectory using:
```
rosservice call /save_trajectory_service [traj_name]
```

At the end of the steps the user should have 2 images inside the `/omni_robot_slam_ws/src/slam_gmapping_amcl/maps` folder: 

- `final_map.pgm` image representing the full map produced by Gmapping.
- `[traj_name].pgm` image representing the saved trajectory on top of the map, up to the service call.
