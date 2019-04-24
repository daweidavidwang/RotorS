# RotorS

RotorS simulator from https://github.com/ethz-asl/rotors_simulator

## Build
sudo apt-get install ros-kinetic-desktop-full ros-kinetic-joy ros-kinetic-octomap-ros ros-kinetic-mavlink python-wstool python-catkin-tools protobuf-compiler libgoogle-glog-dev ros-kinetic-control-toolbox

CLONE this repo to your ros workspace src folder

catkin build


## Run

roslaunch rotors_gazebo multidrones.launch 

## Modify the Scenes type
modify the beginner_tutorials/src/talker.cpp
const int scenario_type =2; at line 32

1: circle 2: random 3:ball

## Problem
1.the execution speed is very slow.
2.using rviz to visualization waypoints 
