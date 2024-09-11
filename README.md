# Summary
This repository contains the needed setup to simulate ANYmal C walking in Gazebo and acquiring data from a 2D Lidar.
- `anymal_c_simple_description` is a fork of the original [repostory](https://github.com/ANYbotics/anymal_c_simple_description) from ANYbotics, only written for ros2 (humble) and modified to use `xacro`, and with additional tags to work with Gazebo and simulate multiple 2D Lidars with different specs, as well as to send via topic the groundtruth odometry
- `rf2o_laser_odometry` is a nearly identical fork of the [RF2O](https://github.com/MAPIRlab/rf2o_laser_odometry) estimator by MAPIRlab
- `anymal_c_navigation`, `anymal_c_config`, and `anymal_c_bringup` are packages [champ](https://github.com/chvmp/champ) packages translated in ros2 and adapted to be used with ANYmal C (namely PID re-tuning).
- `simulation_bringup` contains the launch file to start the simulation, saving data, as well as plotting it, and some utility nodes used in the simulation

# Usage
To launch the Gazebo simulation with multiple Lidar at once, just run ``ros2 launch simulation_bringup multiple_sim.launch.py`` after compiling and sourcing the environment. To control the robot, `teleop_twist_keyboard` is needed.
