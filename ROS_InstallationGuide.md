# Instructions for installing ROS and the Helene package

Installing ROS with all the associated packages and setting up a catkin-compatible workspace is actually not difficult, but can seem overwhelming at first glance. Therefore, the most important steps are summarized here. 

It is assumed that you have a computer with Debian Buster or Ubuntu 20.04. 

## Installing ROS

An installation guide for the basic installation of ROS noetic [can be found here](http://wiki.ros.org/noetic/Installation). It is recommended to do the full desktop installation. Please do not forget the "Environment setup" step!

However, after the successful installation, there are still some packages missing that are required to run Helene. Install all of them via this command:
```
sudo apt install ros-noetic-desktop ros-noetic-catkin python3-catkin-tools python3-osrf-pycommon ros-noetic-rosserial-arduino ros-noetic-rosserial ros-noetic-controller-manager ros-noetic-moveit ros-noetic-moveit-visual-tools ros-noetic-ros-control  ros-noetic-ros-controllers python3-rosdep
```
Also, it is recommended to install [Visual Studio Code](https://code.visualstudio.com/docs/setup/linux) to be able to edit Helene's ROS package. 

## Installing and Configuring Your ROS Environment

First let's create and build a [catkin workspace](http://wiki.ros.org/catkin/workspaces):
```
$ mkdir -p ~/ros_ws/src
$ cd ~/ros_ws/
$ catkin build
```
Then source this directory: 
```
echo "source ~/ros_ws/devel/setup.bash" >> ~/.bashrc
```
Afterwards clone this repo:
```
$ cd src
$ git clone https://github.com/felixherbst/helene_hardware
```
Then compile everything:
```
$ cd ~/ros_ws
$ catkin build
```

Congratulations! You should be ready to rock now
