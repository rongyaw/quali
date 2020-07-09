# Qualifying code for F1tenth 2020 Competition

This project is designed for qualiying competition of F1tenth 2020 Competition.

# Installation
**ROS package requirements:**
- ros-melodic-move-base
- ros-melodic-amcl
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
3. When in pahse 2a of qualifying mode(obstacle avoidance), use planner.launch file. 
```bash
$ roslaunch f1tenth_gym_ros planner.launch
```
4. ctrl-c to planner.launch to stop the navigation.
