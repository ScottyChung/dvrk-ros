#!/usr/bin/env python

# Author: Anton Deguet
# Date: 2015-02-22

# (C) Copyright 2015-2017 Johns Hopkins University (JHU), All Rights Reserved.

# --- begin cisst license - do not edit ---

# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.

# --- end cisst license ---

# Start a single arm using
# > rosrun dvrk_robot dvrk_console_json -j <console-file>

# To communicate with the arm using ROS topics, see the python based example dvrk_arm_test.py:
# > rosrun dvrk_python dvrk_arm_test.py <arm-name>

from __future__ import print_function
import dvrk
import math
import sys
import rospy
import numpy
import PyKDL


# example of application using arm.py
class example_application:

    # configuration
    def configure(self, robot_name):
        print(rospy.get_caller_id(), ' -> configuring dvrk_arm_test for ', robot_name)
        self.arm = dvrk.arm(robot_name)

    # homing example
    def home(self):
        print(rospy.get_caller_id(), ' -> starting home')
        self.arm.home()
        # get current joints just to set size
        goal = numpy.copy(self.arm.get_current_joint_position())
        # go to zero position, for PSM and ECM make sure 3rd joint is past cannula
        goal.fill(0)
        if ((self.arm.name() == 'PSM1') or (self.arm.name() == 'PSM2') or (self.arm.name() == 'PSM3') or (self.arm.name() == 'ECM')):
            goal[2] = 0.12
        self.arm.move_joint(goal, interpolate = True)


    # main method
    def run(self):
        self.home()
        delta=0.005
        goal = numpy.zeros(7)
        while True:
            user_input = raw_input('u to move up, d to move down')
            if user_input == 'u':
                increment = delta
                print('up')
            elif user_input == 'd':
                increment = -delta
                print('down')
            else:
                increment = 0
                print('wrong key')
            # get current joints just to set size
            goal = numpy.copy(self.arm.get_current_joint_position())
            # go to zero position, for PSM and ECM make sure 3rd joint is past cannula
            goal.fill(0)
            goal[2] = increment
            self.arm.dmove_joint(goal)
 


if __name__ == '__main__':
    try:
        if (len(sys.argv) != 2):
            print(sys.argv[0], ' requires one argument, i.e. name of dVRK arm')
        else:
            application = example_application()
            application.configure(sys.argv[1])
            application.run()

    except rospy.ROSInterruptException:
        pass
