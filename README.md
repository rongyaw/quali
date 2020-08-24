# Qualifying code for F1tenth 2020 Competition

This project is designed for qualiying competition of F1tenth 2020 Competition.

# Installation
**ROS package requirements:**
- ros-melodic-move-base
- ros-melodic-teb-local-planner

**Python package requirements:**
- numpy
- cvs
- os

**Operation steps**
1. Clone this repo into the /f1tenth_gym_ros directory and rebuild the package.
2. Use planner_fast.launch file to start running.
```bash
$ roslaunch f1tenth_gym_ros planner_fast.launch
```
3. Use joystick control, launch joystick control file with joystick connected to bluetooth.
```bash
$ roslaunch f1tenth_gym_ros joystick_control.launch
```
4. ctrl-c to launch file to stop the navigation.
