# SETUP
This page describes how to setup the environment. Basically, you need to install ROS Kinetic (of course), and the hector-quadrotor package.

## ROS Kinetic
Follow the instructions at [Ubuntu install of ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)

## hector-quadrotor
hecotr-quadrotor requires the following packages.
### Installation of hector packages
```
sudo apt-get install ros-kinetic-hector-localization ros-kinetic-hector-gazebo ros-kinetic-hector-models
```

### Installation of teleop
These are not really necessary for the package, but it will be useful to manually control the quadrotor in Gazebo.
```
sudo apt-get install ros-kinetic-teleop-twist-keyboard
sudo apt-get install ros-kinetic-teleop-twist-joy
```

### Installation of hector-quadrotor
As of Dec 5, 2017, the hector-quadrotor package is not released through apt yet. It is necessary to install it from the source code by catkin_make.

At the beginning, catkin_make failed to build the package and asked to install the followin
```
sudo apt-get install ros-kinetic-ros-control
sudo apt-get install ros-kinetic-gazebo-ros-control
```
After that, run the following commands
```
cd ~/catkin/src
git clone -b kinetic-devel git@github.com:tu-darmstadt-ros-pkg/hector_quadrotor.git
cd ..
catkin_make
```
It should be good to add this line in .bashrc so the compiled package is always available once a terminal window is opened.
```
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Enable motors
For some reason, motors are disabled so a UAV does not fly even if a message is sent to the topic /cmd_vel. To enable them, call the ROS service /enable_motors.
```
rosservice call /enable_motors true
```

