# RotorS

RotorS simulator from https://github.com/ethz-asl/rotors_simulator

##Build
sudo apt-get install ros-kinetic-desktop-full ros-kinetic-joy ros-kinetic-octomap-ros ros-kinetic-mavlink python-wstool python-catkin-tools protobuf-compiler libgoogle-glog-dev ros-kinetic-control-toolbox
sudo rosdep init
rosdep update
wstool init
wstool merge rotors_hil.rosinstall
catkin build


##Run

roslaunch rotors_gazebo multidrones.launch 

