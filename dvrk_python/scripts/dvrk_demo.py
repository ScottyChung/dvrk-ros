#!/usr/bin/env python

from __future__ import print_function
import dvrk
import math
import sys
import rospy
import numpy
import PyKDL
import wfu_helpers

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

    def joint_speed(self):
        joints = range(0,3)
        speeds = ['slow', 'fast']
        joint_angle = numpy.deg2rad([40,40,2.5])
        rate = 200
        for joint in joints:
            for speed in speeds:
                print('Moving joint {0} at {1} speed'.format(joint,speed))
                goal = numpy.copy(self.arm.get_current_joint_position())
                goal.fill(0)
                goal[2]=0.12
                if speed=='slow':
                    duration = 10
                else:
                    duration = 5
                samples = duration*rate
                #move joint in sinusoidal motion
                for i in range(samples):
                    goal[joint] = joint_angle[joint]*numpy.sin(i*math.pi*2/samples)
                    if joint==2:
                        goal[joint] += 0.12
                    self.arm.move_joint(goal, interpolate=False)
                    rospy.sleep(1.0/rate)
                rospy.sleep(1.0)

    def move_speed(self):
        z_home = -0.1135    
        amp = 0.1
        directions = {'x':PyKDL.Vector(amp,0,z_home),'y':PyKDL.Vector(0,amp,z_home)}
        for d in directions:
            print('Moving slow in  direction')
            wfu_helpers.vmove(self.arm,directions[d], 0.03)
            rospy.sleep(1)
            self.home()
            rospy.sleep(1)
            print('Moving fast in x direction')
            wfu_helpers.vmove(self.arm,directions[d], 0.1)
            rospy.sleep(1)
            self.home()
            rospy.sleep(1)
    def test_matrix(self,speed):
        wfu_helpers.test_matrix(self.arm,speed)

    # main method
    def run(self):
        self.home()
        raw_input('Ready to move joints at two speeds')
        self.joint_speed()
        raw_input('Ready to move at two speeds')
        self.move_speed()
        raw_input('Ready to perform slow test matrix')
        self.test_matrix(0.05)
        raw_input('Ready to perform fast test matrix')
        self.test_matrix(0.1)

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
