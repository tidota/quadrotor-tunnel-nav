# What is this?

Code running on ROS to simulate UAV navigation in a tunnel modeled by Blender.
The program provides a reactive control for a UAV which takes an action as it receives sensory data from the environment so it can follow the wall of the tunnel while avoiding collision.

![sim view](./img/result3.png)

## Required Environment
- OS: Ubuntu 16.04
- ROS version: Kinetic

## Setup
This package requires hector-quadrotor. Check [the details](SETUP.md).

## Installation of this package
```
cd ~/catkin/src
git clone git@github.com:tidota/quadrotor-tunnel-nav.git
cd ..
catkin_make
```

## How to run
You need to launch two parts in separated terminal windows: gazebo and controller.

gazebo:
```
roslaunch quadrotor_tunnel_nav uav_Y-tunnel.launch
```
controller:<br>
The hector-quadrotor (built from the source) apparently [disables the motors in default](https://answers.ros.org/question/256590/hector-quadcopter-not-responding-to-cmd_vel-messages/) and it is required to enable them.
```
rosservice call /enable_motors true
roslaunch quadrotor_tunnel_nav control.launch
```
Then, you can see a quadrotor flying inside a tunnel in the simulator window.

## Memorandum
[Misc notes of ROS commands](MEMORANDUM.md)
