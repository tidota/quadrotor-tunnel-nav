# Some Notes
Just in case that I forgot everything.

## ROS commands

rospack, roscd, rosls
```
rospack find roscpp
roscd roscpp
roscd log
rosls roscpp_tutorials
```

To start
```
roscore
```
rosnode
```
rosnode list
rosnode info /rosout
rosnode ping my_turtle
```
rosrun
```
rosrun turtlesim turtlesim_node
rosrun turtlesim turtlesim_node __name:=my_turtle
rosrun turtlesim turtle_teleop_key
```
rostopic
```
rostopic echo /turtle1/cmd_vel
rostopic type /turtle1/cmd_vel
rosmsg show geometry_msgs/Twist
rostopic hz /turtle1/pose
rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]'
rostopic pub /turtle1/cmd_vel geometry_msgs/Twist -r 1 -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, -1.8]'
```
rqt
```
rosrun rqt_graph rqt_graph
rosrun rqt_plot rqt_plot
rosrun rqt_console rqt_console
rosrun rqt_logger_level rqt_logger_level
```
rosservice
```
rosservice list
rosservice type /clear
rosservice call /clear
rosservice type /spawn| rossrv show
rosservice call /spawn 2 2 0.2 ""
```
rosparam
```
rosparam list
rosparam set /background_r 150
rosservice call /clear
rosparam get /background_g
rosparam dump params.yaml
rosparam load params.yaml copy
```
Launch file
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
```
cd ~/catkin_ws/src
catkin_create_pkg beginner_tutorials std_msgs rospy roscpp
cd ~/catkin_ws
catkin_make
. ~/catkin_ws/devel/setup.bash
rospack depends1 beginner_tutorials
```
package.xml

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
at lines 12 through 15
```
<!-- One license tag required, multiple allowed, one license per tag -->
<!-- Commonly used license strings: -->
<!--   BSD, MIT, Boost Software License, GPLv2, GPLv3, LGPLv2.1, LGPLv3 -->
<license>TODO</license>
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
