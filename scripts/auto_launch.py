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
		print ("nogui!")
	else:
		print ("yesgui!")

	# cmd = [
	# 	'roslaunch',
	# 	os.path.join(args.output, 'gear.launch'),
	# ]
	#
	# print('Running command: ' + ' '.join(cmd))
	# try:
	# 	p = subprocess.Popen(cmd)
	# 	p.wait()
	# except KeyboardInterrupt:
	# 	pass
	# finally:
	# 	p.wait()
