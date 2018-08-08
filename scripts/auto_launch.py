#!/usr/bin/env python

import argparse
import math
import os
import random
import subprocess
import sys

import rospkg
import yaml

if __name__ == '__main__':
	# Filter out any special ROS remapping arguments.
	# This is necessary if the script is being run from a ROS launch file.
	import rospy
	args = rospy.myargv(sys.argv)

	print ('start')
	if "--nogui" in args:
		gui='false'
	else:
		gui='true'

	rospy.init_node('robot_team_spawner')

	rospack = rospkg.RosPack()
	try:
		f = open(rospack.get_path('quadrotor_tunnel_nav') + '/config/adhoc/robots.yaml', 'r')
		dict_robot = yaml.load(f.read())
	except Exception as e:
		print(e)

	cmd_list = {}
	for robot in dict_robot['robots']:
		cmd = [
			'roslaunch',
			'quadrotor_tunnel_nav',
			'spawn_robot.launch',
			'ns:=' + robot,
			'x:=' + str(eval(str(dict_robot[robot]['x']))),
			'y:=' + str(eval(str(dict_robot[robot]['y']))),
			'Y:=' + str(eval(str(dict_robot[robot]['Y']))),
			'gui:=' + gui,
		]
		cmd_list[robot] = cmd

	proc_list = {}
	try:
		for robot in dict_robot['robots']:
			cmd = cmd_list[robot]
			print('Running command: ' + ' '.join(cmd))
			p = subprocess.Popen(cmd)
			proc_list[robot] = p
		for p in proc_list:
			proc_list[p].wait()
	except KeyboardInterrupt:
		pass
	finally:
		for p in proc_list:
			proc_list[p].wait()
