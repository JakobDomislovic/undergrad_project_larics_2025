# Agilex Piper robot arm
## Overview
This project uses an Agilex Piper robot arm mounted on an Agilex Scout UGV.
They are both connected to a NUC computer onboard the scout.
The control for both devices is implemented in ROS2 Humble on a Docker container.
In order to communicate with the robot arm, an external computer running ROS2 Humble with Agilex Piper software should connect to the ROS2 environment on the NUC.
## Setting up the environment
The robot arm control software requires a specific ROS workspace, for this reason you should set up a ROS2 Humble docker image:
```
docker pull osrf/ros:humble-desktop
```
Enable GUI applications in docker if you haven't done so already (run this on your host machine):
```
xhost +local:docker

# To make it permanent
echo "xhost +local:docker > /dev/null" >> ~/.profile
```
Next, run the docker container:
```
docker run -it \
--name piper_container \
--net=host \
--privileged \
--cap-add=NET_ADMIN \
--cap-add=SYS_RAWIO \
--env="DISPLAY=$DISPLAY" \
--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" osrf/ros:humble-desktop

# To open another terminal in a already running container
docker exec -i piper_container
```
## Setting up the arm
Official ROS2 Humble support for the Agilex Piper arm can be found here:
https://github.com/agilexrobotics/piper_ros/tree/humble

An additional resource that you might find useful is the quick start user manual for the robot arm, available here: https://cdn.robotshop.com/rbm/815d1a40-62cd-43a6-8dad-e05323e8953a/8/8c19397c-f1ab-4891-97e7-50fc64a8ef24/ac127c74_piper-quick-start-user-manual-en(1).pdf

There are english instructions available in the ROS repository, which we will be following closely.
In a terminal inside the container, clone the piper workspace:
```
cd ~
git clone -b humble https://github.com/agilexrobotics/piper_ros.git
```
Next, install the following dependencies:
```
pip3 install piper_sdk
sudo apt update
sudo apt install ros-humble-ros2-control -y
sudo apt install ros-humble-ros2-controllers -y
sudo apt install ros-humble-controller-manager -y
sudo apt install ros-humble-moveit* -y
sudo apt install ros-humble-control* -y
sudo apt install ros-humble-joint-trajectory-controller -y
sudo apt install ros-humble-joint-state-* -y
sudo apt install ros-humble-gripper-controllers -y
sudo apt install ros-humble-trajectory-msgs -y
sudo apt install gazebo -y
sudo apt install ros-humble-gazebo-ros-pkgs -y
sudo apt install ros-humble-gazebo-ros2-control -y
```
Finally, source and build your workspace:
```
cd piper_ros
source /opt/ros/humble/setup.bash
colcon build --symlink-install --merge-install
source install/setup.bash

# Optional: create aliases
echo alias source_ros2=\"source /opt/ros/humble/setup.bash\" >> ~/.bashrc
echo alias source_ros2ws=\"source ~/piper_ros/install/setup.bash\" >> ~/.bashrc
echo alias colcon_build=\"colcon build --symlink-install --merge-install\" >> ~/.bashrc
```
Now, you are able to run robot simulation and MoveIt scripts:
```
ros2 launch piper_gazebo piper_no_gripper_gazebo.launch.py
ros2 launch piper_no_gripper_moveit piper_moveit.launch.py
```
Check that you are able to control the simulated robot arm by dragging the end effector to a new pose and clicking "Plan & Execute". The robot arm should move both in RViz and Gazebo.
