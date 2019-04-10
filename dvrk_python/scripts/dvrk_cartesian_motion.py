#!/usr/bin/env python

import dvrk
import math
import sys
import rospy
import numpy
import PyKDL
import numpy as np
import time
from geometry_msgs.msg import Twist, PoseStamped
from sympy.utilities.iterables import multiset_permutations
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

    def vmove(self, goal, speed, frame, iterations=50):
        current_position = self.arm.get_current_position().p
        robot_goal = frame * goal
        distance =  robot_goal - current_position
        sleep_time = (distance/(speed*iterations)).Norm()

        #intermediate points
        interp_points = [np.linspace(s,e,iterations) for s,e in zip (current_position, robot_goal)]
        for x,y,z in zip(*interp_points):
            self.arm.move(PyKDL.Vector(x,y,z), blocking=False)
            time.sleep(sleep_time)

    def speed_test(self):
        z_home = -0.1135
        for x in np.linspace(0,0.05):
            x_translation = PyKDL.Vector(x,0,z_home)
            self.arm.move(x_translation, blocking=False)
            time.sleep(0.02)
        self.home()
        for x in np.linspace(0,0.05):
            x_translation = PyKDL.Vector(x,0,z_home)
            self.arm.move(x_translation, blocking=False)
            time.sleep(0.05)
        self.home()
        for x in np.linspace(0,0.05):
            x_translation = PyKDL.Vector(x,0,z_home)
            self.arm.move(x_translation, blocking=False)
            time.sleep(0.1)
        self.home()

    def xyz_move(self):
        z_home = -0.1135
        distance = 0.1
        speed = 0.05
        for x in np.linspace(0,distance):
            x_translation = PyKDL.Vector(x,0,z_home)
            self.arm.move(x_translation, blocking=False)
            time.sleep(speed)
        for x in np.linspace(distance,0):
            x_translation = PyKDL.Vector(x,0,z_home)
            self.arm.move(x_translation, blocking=False)
            time.sleep(speed)
        for y in np.linspace(0,distance):
            y_translation = PyKDL.Vector(0,y,z_home)
            self.arm.move(y_translation, blocking=False)
            time.sleep(speed)
        for y in np.linspace(distance,0):
            y_translation = PyKDL.Vector(0,y,z_home)
            self.arm.move(y_translation, blocking=False)
            time.sleep(speed)
        for z in np.linspace(0,distance):
            z_translation = PyKDL.Vector(0,0,z_home+z)
            self.arm.move(z_translation, blocking=False)
            time.sleep(speed)
        for z in np.linspace(distance,0):
            z_translation = PyKDL.Vector(0,0,z_home+z)
            self.arm.move(z_translation, blocking=False)
            time.sleep(speed)
        self.home()

    # main method
    def run(self):
        self.home()
        raw_input('Ready to Move to Home')
        #self.speed_test()
        #self.xyz_move()
        #p = PyKDL.Vector(0,0,0)
        #sev = 0.7071
        #m = PyKDL.Rotation(sev, -sev, 0, sev, sev, 0, 0, 0, 1)
        #frame = PyKDL.Frame(m,p)
        r = PyKDL.Rotation( 
            -0.0608144,    0.944307,   -0.323398,
             0.997935,   0.0508103,  -0.0392961,
            0.0206757,     0.32512,    0.945447)
        p = PyKDL.Vector(   0.0877638,  -0.0284948,  -0.0948376)
        frame = PyKDL.Frame(r,p)
	speed = 0.05
	distance = 0.07
        self.vmove(PyKDL.Vector(0,0,0),speed,frame)
	raw_input('Ready to begin motions')
	motions = list(multiset_permutations([0,0,distance]))
	for motion in sorted(motions):
		motion = np.array(motion)
		print(motion)
		self.vmove(PyKDL.Vector(*motion),speed,frame)
		time.sleep(1)
		self.vmove(PyKDL.Vector(0,0,0),speed,frame)
		time.sleep(1)
		self.vmove(PyKDL.Vector(*(-motion)),speed,frame)
		time.sleep(1)
		self.vmove(PyKDL.Vector(0,0,0),speed,frame)
		time.sleep(1)
        rospy.spin()

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

