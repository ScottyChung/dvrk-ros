#!/usr/bin/env python

import dvrk
import math
import sys
import rospy
import numpy
import PyKDL
import numpy as np
import time
import copy
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point, Quaternion
from sympy.utilities.iterables import multiset_permutations
from std_msgs.msg import Header
from nav_msgs.msg import Path
from sensor_msgs.msg import Joy

# example of application using arm.py
class example_application:

    # configuration
    def configure(self, robot_name):
        print rospy.get_caller_id(), ' -> configuring dvrk_psm_test for ', robot_name
        self.arm = dvrk.psm(robot_name)

    # homing example
    def home(self):
        print rospy.get_caller_id(), ' -> starting home'
        self.arm.home()
        # get current joints just to set size
        goal = numpy.copy(self.arm.get_current_joint_position())
        # go to zero position, except for insert joint so we can't break tool in cannula
        goal.fill(0)
        goal[2] = 0.12
        self.arm.move_joint(goal, interpolate = True)

    def vmove(self, goal, speed, frame=PyKDL.Frame(), iterations=500):
        current_position = self.arm.get_current_position().p
        robot_goal = frame * goal
        distance =  robot_goal - current_position
        sleep_time = (distance/(speed*iterations)).Norm()

        #intermediate points
        interp_points = [np.linspace(s,e,iterations) for s,e in zip (current_position, robot_goal)]
        for x,y,z in zip(*interp_points):
            self.arm.move(PyKDL.Vector(x,y,z), blocking=False)
            rospy.sleep(sleep_time)

    def my_function(self):
        """
        Example:
        self.arm.move(PyKDL.Vector(x,y,z))
        where x,y,z are the cartesian coordinates of the desired points to move to
        """
	#Start at Home
	self.home() 
	#move in x direction 
	self.arm.move(PyKDL.Vector(.12,0,0))
	self.home()
	self.arm.move(PyKDL.Vector(-.12,0,0))
	self.home()
	#move in y direction 
	self.arm.move(PyKDL.Vector(0,.12,0))
	self.home()
	self.arm.move(PyKDL.Vector(0,-.12,0))
	self.home()
	#move in z direction
	self.arm.move(PyKDL.Vector(0,0,.12))
	self.home()
	self.arm.move(PyKDL.Vector(0,0,-.2))
	self.home()
	#create square on new plane 
        self.arm.move(PyKDL.Vector(0,0,-.2))
	self.arm.move(PyKDL.Vector(.12,0,-.2))
	self.arm.move(PyKDL.Vector(0,0,-.2))
	self.arm.move(PyKDL.Vector(-.12,0,-.2))
	self.arm.move(PyKDL.Vector(0,0,-.2))
	self.arm.move(PyKDL.Vector(0,.12,-.2))
	self.arm.move(PyKDL.Vector(0,0,-.2))
	self.arm.move(PyKDL.Vector(0,-.12,-.2))
	#touch same spots without going through middle
	self.home()
	self.arm.move(PyKDL.Vector(.12,0,0)) 
	self.arm.move(PyKDL.Vector(.12,0,-.2))
	self.arm.move(PyKDL.Vector(.12,0,0))
	self.home() 
	self.arm.move(PyKDL.Vector(-.12,0,0))
	self.arm.move(PyKDL.Vector(-.12,0,-.2))
	self.home()
	self.arm.move(PyKDL.Vector(0,.12,0))
	self.arm.move(PyKDL.Vector(0,.12,-.2))
	self.arm.move(PyKDL.Vector(0,.12,0))
	self.home()
	self.arm.move(PyKDL.Vector(0,-.12,0))
	self.arm.move(PyKDL.Vector(0,-.12,-.2))
	self.arm.move(PyKDL.Vector(0,-.12,0))
	self.home()
	#dmove 
	self.arm.dmove(PyKDL.Vector(0,.12,0))
	self.arm.dmove(PyKDL.Vector(2,0,0))
        print('hello lizo')

    # main method
    def run(self):
        self.home()
        self.my_function()

if __name__ == '__main__':
    try:
        if (len(sys.argv) != 2):
            print sys.argv[0], ' requires one argument, i.e. name of dVRK arm'
        else:
            application = example_application()
            application.configure(sys.argv[1])
            application.run()

    except rospy.ROSInterruptException:
        pass

