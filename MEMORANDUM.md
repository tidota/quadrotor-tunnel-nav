# Some Notes
Just in case that I forgot everything.

Anyway, the official tutorial is [here](http://wiki.ros.org/ROS/Tutorials).

## ROS commands

### rospack, roscd, rosls
```
rospack find roscpp
roscd roscpp
roscd log
rosls roscpp_tutorials
```

### To start
roscore will run the fundamental part of of ROS: rosout, Master, etc.
```
roscore
```
### rosrun
It starts a node: a unit of program in ROS.
```
rosrun turtlesim turtlesim_node
rosrun turtlesim turtlesim_node __name:=my_turtle
rosrun turtlesim turtle_teleop_key
```
### rosnode
It can inspect the currently running nodes.
```
rosnode list
rosnode info /rosout
rosnode ping my_turtle
```
### rostopic/rosmsg
- `Topic`: a kind of variable name
- `Message`: a kind of data type

In the following example, `geometry_msgs/Twist` is equivalent to a structure in C, and `/turtle_1/cmd_vel` is an object, i.e., it would be like `Twist cmd_vel;` in C.
```
rostopic echo /turtle1/cmd_vel
rostopic type /turtle1/cmd_vel
rosmsg show geometry_msgs/Twist
rostopic hz /turtle1/pose
rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]'
rostopic pub /turtle1/cmd_vel geometry_msgs/Twist -r 1 -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, -1.8]'
```
### rqt
They graphically display information about the running nodes.
```
rosrun rqt_graph rqt_graph
rosrun rqt_plot rqt_plot
rosrun rqt_console rqt_console
rosrun rqt_logger_level rqt_logger_level
```
### rosservice
A ROS servise works like a function. Some of them take arguments.
```
rosservice list
rosservice type /clear
rosservice call /clear
rosservice type /spawn | rossrv show
rosservice call /spawn 2 2 0.2 ""
```
### rosparam
A ROS parameter is a kind of global variable shared by nodes.
```
rosparam list
rosparam set /background_r 150
rosservice call /clear
rosparam get /background_g
rosparam dump params.yaml
rosparam load params.yaml copy
```
## launch file
```
roslaunch beginner_tutorials turtlemimic.launch
```
```
<launch>

  <group ns="turtlesim1">
    <node pkg="turtlesim" name="sim" type="turtlesim_node"/>
  </group>

  <group ns="turtlesim2">
    <node pkg="turtlesim" name="sim" type="turtlesim_node"/>
  </group>

  <node pkg="turtlesim" name="mimic" type="mimic">
    <remap from="input" to="turtlesim1/turtle1"/>
    <remap from="output" to="turtlesim2/turtle1"/>
  </node>

</launch>
```

## Creating a package
Each package directory must have **package.xml** and **CMakeLists.txt**, which are created by `catkin_create_pkg` command.
```
cd ~/catkin_ws/src
catkin_create_pkg beginner_tutorials std_msgs rospy roscpp
cd ~/catkin_ws
catkin_make
. ~/catkin_ws/devel/setup.bash
rospack depends1 beginner_tutorials
```
### package.xml

at line 5
```
<description>The beginner_tutorials package</description>
```
at lines 7 through 10
```
<!-- One maintainer tag required, multiple allowed, one person per tag -->
<!-- Example:  -->
<!-- <maintainer email="jane.doe@example.com">Jane Doe</maintainer> -->
<maintainer email="user@todo.todo">user</maintainer>
```
<!-- One license tag required, multiple allowed, one license per tag -->
at lines 12 through 15
```
<!-- Commonly used license strings: -->
<!--   BSD, MIT, Boost Software License, GPLv2, GPLv3, LGPLv2.1, LGPLv3 -->
<license>TODO</license>
```


## creating a msg and srv
msg and srv are data types for messages and services, respectively. From these definitions, corresponding source code is generated for a specific programming language, e.g., a header file for C/C++.

The [primitive data types](http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv#Introduction_to_msg_and_srv):
- int8, int16, int32, int64 (plus uint*)
- float32, float64
- string
- time, duration
- other msg files
- variable-length array[] and fixed-length array[C]

### msg
msg just contains a set of variables. Header is a special datatype containing time stamp and coordinates.
```
Header header
string child_frame_id
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist
```
#### to create a msg file
```
roscd beginner_tutorials
mkdir msg
echo "int64 num" > msg/Num.msg
```
#### in package.xml
Apparently, `message_generation` is charged to create necessary source code for the message data type for a specific programming langugae from a .msg file. `message_runtime` is charged to maintain the actual usage of messages in run time.
```
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>
```
#### in CMakeLists.txt
Apparently, `find_package` function picks all necessary projects for both built time and run time. `catkin_package` is focused on run time.
- add `message_generation` into `find_package`.
- make sure `catkin_package` has `message_runtime` after `CATKIN_DEPENDS`
- uncomment and edit `add_message_files`
```
add_message_files(
   FILES
   Num.msg
)
```
- uncomment and edit `generate_messages`
```
generate_messages(
   DEPENDENCIES
   std_msgs
)
```

### srv
srv has two sets: request and response. The request is a set of arguments which a service takes; the response is a value returnd from the service.
```
int64 A
int64 B
---
int64 Sum
```

#### to create a srv file
```
roscd beginner_tutorials
mkdir srv
roscp rospy_tutorials AddTwoInts.srv srv/AddTwoInts.srv
```

#### in package.xml
same as that for msg

#### in CMakeLists.txt
- add `message_generation` into `find_package`.
- uncomment and edit `add_service_files`
```
add_service_files(
   FILES
   AddTwoInts.srv
)
```
- uncomment and edit `generate_messages`
```
generate_messages(
   DEPENDENCIES
   std_msgs
)
```

### finally,
```
catkin_make Install
```

## [Recording + Replaying data](http://wiki.ros.org/ROS/Tutorials/Recording%20and%20playing%20back%20data)

```
mkdir ~/bagfiles
cd ~/bagfiles
rosbag record -a

rosbag info <your bagfile>
rosbag play <options> <your bagfile>

rosbag record -O subset /turtle1/cmd_vel /turtle1/pose
```

## Trouble shooting: [roswtf](http://wiki.ros.org/ROS/Tutorials/Getting%20started%20with%20roswtf)
```
roscd
roswtf
```

## Really Miscellaneous Notes

packages are usually located in
- /opt/ros/kinetic/share
- ~/catkin_ws/src

bash scripts for environment settings
- /opt/ros/kinetic/setup.bash: to use ROS itself
- ~/catkin/src/setup.bash: for development

~/catkin
- src: personal packages are here
- devel: setup.bash is here (it must be sourced to include ~/catkin/src in ROS_PACKAGE_PATH)
- build: built programs will be here
