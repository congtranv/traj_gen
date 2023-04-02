# Trajectory generation and Geometric controller

## 1. Dependencies 
***
### Trajectory generation, optimization and sampling from [ethz-asl/mav_trajectory_generation](https://github.com/ethz-asl/mav_trajectory_generation) 

>This repository contains tools for polynomial trajectory generation and optimization based on methods described in [1]. These techniques are especially suitable for rotary-wing micro aerial vehicles (MAVs).

>**Authors**: Markus Achtelik, Michael Burri, Helen Oleynikova, Rik Bähnemann, Marija Popović
>
>**Maintainer**: Rik Bähnemann, brik@ethz.ch
>
>**Affiliation**: Autonomous Systems Lab, ETH Zurich

### Geometric controller and Controller message from [Jaeyoung-Lim/mavros_controllers](https://github.com/Jaeyoung-Lim/mavros_controllers)

>The repository contains controllers for controlling MAVs using the mavros package. The following packages are included in the repo
>
>- geometric_controller: Trajectory tracking controller based on geometric control
>- controller_msgs: custom message definitions
>- trajectory_publisher: Node publishing setpoints as states from motion primitives / trajectories for the controller to follow.

## 2. Installation
***
### 2.0. Environments
- **ROS**: tested on [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) (Ubuntu 18.04)
- **PX4 Firmware**: tested on v10.0.1 - setup [here](https://github.com/congtranv/px4_install)
- **Catkin workspace**: `catkin_ws`
  ```
  ## create a workspace if you've not had one
  mkdir -p [path/to/ws]/catkin_ws/src
  cd [path/to/ws]/catkin_ws
  ```
  ```
  catkin init
  catkin config --extend /opt/ros/melodic
  catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
  catkin config --merge-devel
  catkin build
  ```
- **MAVROS**: binary installation - setup [here](https://docs.px4.io/master/en/ros/mavros_installation.html#binary-installation-debian-ubuntu)

### 2.1. [ethz-asl/mav_trajectory_generation](https://github.com/ethz-asl/mav_trajectory_generation) 

```
cd [path/to/ws]/catkin_ws/src
```
```
git clone https://github.com/catkin/catkin_simple.git
git clone https://github.com/ethz-asl/eigen_catkin.git
git clone https://github.com/ethz-asl/eigen_checks.git
git clone https://github.com/ethz-asl/nlopt.git
git clone https://github.com/ethz-asl/glog_catkin.git
git clone https://github.com/ethz-asl/mav_comm.git
git clone https://github.com/ethz-asl/yaml_cpp_catkin.git
git clone https://github.com/ethz-asl/mav_trajectory_generation.git
```
```
cd [path/to/ws]/catkin_ws
```
```
catkin build
```
### 2.2. [Jaeyoung-Lim/mavros_controllers](https://github.com/Jaeyoung-Lim/mavros_controllers)

```
cd [path/to/ws]/catkin_ws/src
```
```
git clone https://github.com/Jaeyoung-Lim/mavros_controllers.git
```
```
cd [path/to/ws]/catkin_ws
```
```
catkin build
```

### 2.3. [congtranv/traj_gen](https://github.com/congtranv/traj_gen)
```
cd [path/to/ws]/catkin_ws/src
```
```
git clone https://github.com/congtranv/traj_gen.git
```
```
cd [path/to/ws]/catkin_ws
```
```
catkin build traj_gen
```

## 3. Usage
***
### 3.1. Simulation
- Terminal 1: Launch px4 sitl simulation, rviz and geometric controller
```
roslaunch traj_gen simulation.launch gui:=true
```
`gui:=true`: enable gazebo simulation window

Wait to drone takeoff and run trajectory generation

- Terminal 2: Launch trajectory generation
```
roslaunch traj_gen traj_gen.launch [dimension] [input_middle] [nonlinear_opt]
```
Where:

`dimension:=3` or `4`: 3 for position and 4 for position and yaw at vertices (start or end)

`input_middle:=true` or `false`: true to use middle point from param input in launch file and false to use middle point from average of start and end points

`nonlinear_opt:=true` or `false`: true to use nonlinear optimization and false to use linear optimization

After drone finish trajectory, can change target point (end point) and middle point in launch file and relaunch (Start point is current position from odometry)

### 3.2. Field test (ongoing test !!!)
- Terminal 1: Launch mavros (connect onboard to drone) and geometric controller
```
roslaunch traj_gen mav_run.launch [fcu_url] [gcs_url]
```
`fcu_url`: link to pixhawk autopilot (`"/dev/ttyTHS1:921600"`)

`gcs_url`: link to ground computer (`"udp://:14555@192.168.1.2:14555"`)

Wait nodes is run, use RC to ARM and switch OFFBOARD mode for takeoff. Then run trajectory generation

- Terminal 2: Launch trajectory generation
```
roslaunch traj_gen traj_gen.launch [dimension] [input_middle] [nonlinear_opt]
```
Where:

`dimension:=3` or `4`: 3 for position and 4 for position and yaw at vertices (start or end)

`input_middle:=true` or `false`: true to use middle point from param input in launch file and false to use middle point from average of start and end points

`nonlinear_opt:=true` or `false`: true to use nonlinear optimization and false to use linear optimization

After drone finish trajectory, can change target point (end point) and middle point in launch file and relaunch (Start point is current position from odometry)