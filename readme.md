
# SLAM for Autonomous Robot

<!-- [![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT) 
[![codecov](https://codecov.io/gh/shantanuparabumd/project_legion/graph/badge.svg?token=WS0I96DIDU)](https://codecov.io/gh/shantanuparabumd/project_legion)
[![project_legion_build](https://github.com/shantanuparabumd/project_legion/actions/workflows/project_legion_git_ci.yml/badge.svg)](https://github.com/shantanuparabumd/project_legion/actions/workflows/project_legion_git_ci.yml) -->

[video3.webm](https://github.com/shantanuparabumd/slam_robot/assets/112659509/1bde4389-47e0-40a3-b85d-59b8568c6092)



## Authors

|Name|ID|Email|
|:---:|:---:|:---:|
|Shantanu Parab|119208625|sparab@umd.edu|


## Introduction

 Welcome to the Autonomous Turtle Robot SLAM Project! In this project, we delve into the exciting realm of Simultaneous Localization and Mapping (SLAM) within the Gazebo environment. Our primary goal is to develop an autonomous turtle robot capable of localizing itself in its environment while simultaneously mapping its surroundings.


## Project Overview

Naive SLAM Implementation: We kickstart our journey by implementing a naive SLAM approach, where we assume perfect odometry and LiDAR readings to establish a baseline understanding of SLAM techniques.

Enhanced SLAM with Kalman Filter: Building upon our initial results, we delve deeper into SLAM refinement techniques, integrating an Estimated Kalman Filter to improve localization accuracy and mapping robustness.

Exploring Particle Filter for Localization: As part of our exploration, we experiment with Particle Filter localization methods, particularly when the environment map is known, aiming to optimize our robot's localization capabilities.

## Results

![SLAM Map](/media/robot_scan_plot.png)



[video4.webm](https://github.com/shantanuparabumd/slam_robot/assets/112659509/bd74f3c4-9372-42b1-b22e-2ff7911210e6)





## Dependencies

- OS: Ubuntu Linux 22.04
- ROS Version: ROS2 Galactic
- C++




# Dependeny Installation and Setup

Installing ROS Controller (Run this in home directory)

`sudo apt install ros-galactic-ros2-control ros-galactic-ros2-controllers ros-galactic-gazebo-ros2-control`

Install xacro module to read xacro files
`pip install xacro`



