#!/usr/bin/env python

import dvrk
import math
import sys
import rospy
import numpy
import PyKDL
from geometry_msgs.msg import Twist, PoseStamped
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

    # move based on space mouse
    def space_move(self, data):
        if self.arm.name() in ['PSM1','PSM2','PSM3','ECM']:
            # set in position joint mode
            scale_factor = 0.01
            x_y_scale_factor = 10 * scale_factor

            ang_sf = self.ang_sf
            rospy.loginfo(self.arm.get_current_position().p)

            if False:
                goal = numpy.zeros(6)
                goal[0] = data.linear.y * x_y_scale_factor
                goal[1] = data.linear.x * x_y_scale_factor
                goal[2] = data.linear.z * -scale_factor
                goal[3] = data.angular.z * -ang_sf
                goal[4] = data.angular.y * ang_sf
                goal[5] = data.angular.x * ang_sf
                self.arm.dmove_joint(goal, interpolate = True, blocking=False)
            if True:
                goal = numpy.zeros(3)
                goal[0] = data.linear.y * scale_factor
                goal[1] = data.linear.x * scale_factor
                goal[2] = data.linear.z * scale_factor
                #self.arm.dmove(PyKDL.Vector(goal[0],goal[1],goal[2]), interpolate=True, blocking=False)
                rot = numpy.zeros(3)
                rot[0] = data.angular.y*0.1
                rot[1] = data.angular.x*0.1
                rot[2] = data.angular.z*0.1
                #self.arm.dmove(PyKDL.Rotation.EulerZYX(rot[0], rot[1], rot[2]), interpolate=True, blocking=False) 
                frame = PyKDL.Frame(PyKDL.Rotation.EulerZYX(rot[0], rot[1], rot[2]), 
                                    PyKDL.Vector(goal[0], goal[1], goal[2]))
                self.arm.dmove(frame, interpolate=True, blocking=False)

            rospy.sleep(1.0/200)

    def toggle_angular(self, data):
        if data.buttons[0] == 1:
            self.toggle = True

        if self.toggle:
            self.ang_sf = 0.1
            self.toggle = False

    def arm_position(self, data):
        pub = rospy.Publisher('tool_path', Path)
        path = Path()
        path.header.frame_id='world'
        path.poses = data.pose
        #pub.publish(path)


    # main method
    def run(self):
        self.home()
        self.toggle = False
        self.ang_sf = 0
        print("Listening to Spacemouse")
        rospy.Subscriber('spacenav/joy', Joy, self.toggle_angular)
        rospy.Subscriber('spacenav/twist', Twist, self.space_move)
        rospy.Subscriber('dvrk/PSM1/position_cartesian_current', PoseStamped, self.arm_position)
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


