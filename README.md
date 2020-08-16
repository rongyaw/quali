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
2. When in phase 1a of qualifying mode(2 lap rush), use planner_fast.launch file.
```bash
$ roslaunch f1tenth_gym_ros planner_fast.launch
```
3. ctrl-c to planner_fast.launch to stop the navigation.
