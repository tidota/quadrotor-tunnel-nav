#!/usr/bin/env python

import argparse
import math
import os
import random
import subprocess
import sys

import rospkg
import yaml

from time import sleep

if __name__ == '__main__':
	# Filter out any special ROS remapping arguments.
	# This is necessary if the script is being run from a ROS launch file.
	import rospy
	args = rospy.myargv(sys.argv)

	rospy.init_node('robot_preparer')

	rospack = rospkg.RosPack()
	try:
		f = open(rospack.get_path('quadrotor_tunnel_nav') + '/config/adhoc/robots.yaml', 'r')
		dict_robot = yaml.load(f.read())
	except Exception as e:
		print(e)

	# September 5, 2018
	# If there are too many robots, the following warning appears and the
	# controller cannot work:
	# "Controller Spawner couldn't find the expected controller_manager ROS
	# interface."
	# Apparently, it takes too long time to prepare some robots.
	# The following line forcefully delayes spawing controllers for a while so
	# it is completed to spawn a robot.
	sleep(1)

	try:
		cmd = [
			'roslaunch',
			'hector_quadrotor_controllers',
			'controller.launch',
			'controllers:=' + ' '.join(args[1:]),
		]
		print('Running command: ' + ' '.join(cmd))
		p = subprocess.Popen(cmd)
		p.wait()
	except KeyboardInterrupt:
		pass
	finally:
		p.wait()
