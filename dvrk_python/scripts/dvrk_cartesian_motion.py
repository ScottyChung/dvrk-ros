#!/usr/bin/env python

import dvrk
import math
import sys
import rospy
import numpy
import PyKDL
import numpy as np
import time
import wfu_helpers
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

    def joint_motion_test(self):
        goal = np.zeros(6)
        self.arm.move_joint(goal)
        rospy.sleep(2)
        for i in np.linspace(0,math.pi):
            goal[0] = i
            self.arm.move_joint(goal) 
            rospy.sleep(2)

    def z_move_test(self):
        z_home = -0.1135
        raw_input('Will move in Z direction. Press any button to begin')
        for x in np.linspace(0,0.1,21):
            wfu_helpers.vmove(self.arm, PyKDL.Vector(0,x, z_home),0.05)
            rospy.sleep(1)
        print('Completed test')
        
    # main method
    def run(self):
        raw_input('Ready to Move to Home')
        self.home()
        self.z_move_test()
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

