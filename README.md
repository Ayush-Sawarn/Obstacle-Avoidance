# Obstacle Avoidance

ROS Noetic + Gazebo workspace for TurtleBot3 navigation in bookstore-style simulation environments, with TEB local planning, moving cardboard-box obstacles, and metric collection utilities.

## What Is Included

- `src/turtlebot3`
  TurtleBot3 navigation, SLAM, teleop, and bringup packages.
- `src/turtlebot3_simulations`
  Gazebo worlds, launch files, robot simulation assets, and custom moving-obstacle scripts.
- `src/turtlebot3_metrics`
  Scripts and launch files for recording and plotting navigation metrics.

## Main Features

- Bookstore world launch in Gazebo
- TurtleBot3 autonomous navigation with `move_base`
- TEB local planner integration
- RViz configuration with TEB local-plan, pose, and marker topics enabled
- Moving cardboard boxes in `bookstore.world`
- Randomized and oscillating cardboard boxes in `TD3.world`
- Metrics recording and plotting utilities

## Requirements

- Ubuntu 20.04
- ROS Noetic
- Gazebo
- TurtleBot3 dependencies

Recommended packages:

```bash
sudo apt-get update
sudo apt-get install -y \
  ros-noetic-gazebo-ros-pkgs \
  ros-noetic-urdf \
  ros-noetic-xacro \
  ros-noetic-slam-gmapping \
  ros-noetic-map-server \
  ros-noetic-rviz \
  ros-noetic-amcl \
  ros-noetic-move-base \
  ros-noetic-navigation \
  ros-noetic-teb-local-planner
```

## Workspace Setup

Clone this repository into the `src` folder of a catkin workspace:

```bash
cd ~/catkin_ws/src
git clone https://github.com/Ayush-Sawarn/Obstacle-Avoidance.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

Set the TurtleBot3 model before launching:

```bash
export TURTLEBOT3_MODEL=waffle_pi
```

## Running The Bookstore Simulation

Launch the bookstore world with TurtleBot3:

```bash
roslaunch turtlebot3_gazebo bookstore.launch
```

This world includes moving cardboard boxes controlled by:

- [bookstore.launch](/home/ayush/DL_Hackathon/Deimos_task1/src/turtlebot3_simulations/turtlebot3_gazebo/launch/bookstore.launch)
- [move_bookstore_boxes.py](/home/ayush/DL_Hackathon/Deimos_task1/src/turtlebot3_simulations/turtlebot3_gazebo/scripts/move_bookstore_boxes.py)

You can tune obstacle motion from launch arguments:

```bash
roslaunch turtlebot3_gazebo bookstore.launch \
  box_oscillation_amplitude:=1.4 \
  box_oscillation_speed:=0.25
```

## Teleoperation

```bash
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

## Mapping

Start SLAM:

```bash
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
```

Save the generated map:

```bash
rosrun map_server map_saver -f ~/bookstore_map
```

## Navigation

Launch navigation with the default map:

```bash
roslaunch turtlebot3_navigation turtlebot3_navigation.launch
```

Or provide a specific map:

```bash
roslaunch turtlebot3_navigation turtlebot3_navigation.launch \
  map_file:=/absolute/path/to/bookstore_map.yaml
```

This repo is configured to use TEB in the navigation stack. Relevant files:

- [move_base.launch](/home/ayush/DL_Hackathon/Deimos_task1/src/turtlebot3/turtlebot3_navigation/launch/move_base.launch)
- [move_base_teb.launch](/home/ayush/DL_Hackathon/Deimos_task1/src/turtlebot3/turtlebot3_navigation/launch/move_base_teb.launch)
- [teb_local_planner_params.yaml](/home/ayush/DL_Hackathon/Deimos_task1/src/turtlebot3/turtlebot3_navigation/param/teb_local_planner_params.yaml)
- [turtlebot3_navigation.rviz](/home/ayush/DL_Hackathon/Deimos_task1/src/turtlebot3/turtlebot3_navigation/rviz/turtlebot3_navigation.rviz)

The RViz config already enables:

- `/move_base/TebLocalPlannerROS/local_plan`
- `/move_base/TebLocalPlannerROS/teb_poses`
- `/move_base/TebLocalPlannerROS/teb_markers`

## TD3 World

The repo also contains a custom `TD3.world` setup with cardboard boxes that can be randomized and moved through:

- [TD3.launch](/home/ayush/DL_Hackathon/Deimos_task1/src/turtlebot3_simulations/turtlebot3_gazebo/launch/TD3.launch)
- [randomize_td3_boxes.py](/home/ayush/DL_Hackathon/Deimos_task1/src/turtlebot3_simulations/turtlebot3_gazebo/scripts/randomize_td3_boxes.py)

Launch it with:

```bash
roslaunch turtlebot3_gazebo TD3.launch
```

## Metrics

Metric utilities are available under `src/turtlebot3_metrics`.

Examples:

```bash
roslaunch turtlebot3_metrics record_metrics.launch
```

and

```bash
cd src/turtlebot3_metrics
./run_metrics.sh
```

Useful files:

- [record_metrics.py](/home/ayush/DL_Hackathon/Deimos_task1/src/turtlebot3_metrics/scripts/record_metrics.py)
- [compute_metrics.py](/home/ayush/DL_Hackathon/Deimos_task1/src/turtlebot3_metrics/scripts/compute_metrics.py)
- [plot_metrics.py](/home/ayush/DL_Hackathon/Deimos_task1/src/turtlebot3_metrics/scripts/plot_metrics.py)

## Repo Notes

- This repository tracks source code only.
- Build artifacts like `build/`, `devel/`, and Python `__pycache__/` should stay out of version control.
- Some world/model assets in this workspace include `:Zone.Identifier` files from Windows extraction; they are not required for ROS/Gazebo execution.

## Typical Workflow

1. Build the workspace with `catkin_make`
2. Launch a Gazebo world such as `bookstore.launch`
3. Optionally teleoperate or map the world
4. Start navigation with `turtlebot3_navigation.launch`
5. Visualize TEB output in RViz
6. Record and evaluate metrics if needed

## Acknowledgments

This repo is built on top of TurtleBot3 and TurtleBot3 simulation packages from ROBOTIS, with additional custom world, navigation, obstacle-motion, and evaluation changes for obstacle-avoidance experiments.
