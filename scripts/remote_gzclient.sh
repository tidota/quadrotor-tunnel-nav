#!/bin/bash
# Sept 5, 2018
# This is setups for gzclient running on a local machine, which accesses
# gzserver running on a remote machine.
# Change the line of GAZEBO_MASTER_URI if a different master is used.
export GAZEBO_MASTER_URI=seilon-3
export GAZEBO_IP=`hostname -I | awk '{print $1}'`
