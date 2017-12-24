#!/bin/bash
# Dec 23, 2017
# This is setups for a non-master machine.
# Change the line of ROS_MASTER_URI if a different master is used.
export ROS_MASTER_URI=http://seilon-2:11311/
export ROS_IP=`hostname -I | awk '{print $1}'`
export ROS_HOSTNAME=$ROS_IP

