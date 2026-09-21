# px4_ros_com (customised)

Main package of this workspace. It is derived from [PX4/px4_ros_com](https://github.com/PX4/px4_ros_com) (BSD 3-Clause), but the upstream C++ examples and frame-transform library were removed. It now contains:

- the offboard controller, keyboard teleop, PX4 → ROS odometry bridge and SLAM toggle service (`src/`)
- the launch files for simulation, the real drone and the ground station (`launch/`)
- the slam_toolbox and Nav2 parameters (`config/`)

See the [workspace README](../../README.md) for installation, usage and node documentation.
