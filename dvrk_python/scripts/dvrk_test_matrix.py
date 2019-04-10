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

    def speed_test(self):
        z_home = -0.1235
        for x in np.linspace(0,0.05):
            x_translation = PyKDL.Vector(x,0,z_home)
            self.arm.move(x_translation, blocking=False)
            time.sleep(0.1)
        rospy.sleep(0.5)
        self.vmove(PyKDL.Vector(0,0,z_home),0.05)
        rospy.sleep(0.5)
        for x in np.linspace(0,0.05):
            x_translation = PyKDL.Vector(x,0,z_home)
            self.arm.move(x_translation, blocking=False)
            time.sleep(0.05)
        rospy.sleep(0.5)
        self.vmove(PyKDL.Vector(0,0,z_home),0.05)
        rospy.sleep(0.5)
        for x in np.linspace(0,0.05):
            x_translation = PyKDL.Vector(x,0,z_home)
            self.arm.move(x_translation, blocking=False)
            time.sleep(0.02)
        rospy.sleep(0.5)
        self.vmove(PyKDL.Vector(0,0,z_home),0.05)
        rospy.sleep(0.5)

    def xyz_move(self):
        distance = 0.05
        speed=0.04
        self.vmove(PyKDL.Vector(distance,0,-0.1235),speed)
        self.vmove(PyKDL.Vector(0,0,-0.1235),speed)
        self.vmove(PyKDL.Vector(0,distance,-0.1235),speed)
        self.vmove(PyKDL.Vector(0,0,-0.1235),speed)
        self.vmove(PyKDL.Vector(0,0,-0.1235+distance),speed)
        self.vmove(PyKDL.Vector(0,0,-0.1235),speed)

    # main method
    def run(self):
        self.home()
        raw_input('ready')
        #Test Matrix 
        r = 0.06
        theta = np.repeat(np.linspace(0,np.pi/2,4),4)
        phi = np.tile(np.linspace(0,np.pi/2,4),4)

        x = r*np.sin(theta)*np.cos(phi)
        y = r*np.sin(theta)*np.sin(phi)
        z = r*np.cos(theta)
        speed=0.05
        #uncomment to motion
        for a,b,c in zip(x[3:],y[3:],z[3:]):
            self.vmove(PyKDL.Vector(a,b,c-0.1235),speed)
            print(a,b,c)
            rospy.sleep(1)
            self.vmove(PyKDL.Vector(0,0,-0.1235),speed)
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

