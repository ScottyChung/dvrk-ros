#!/usr/bin/env python

import rospy
import rosbag
import sys
import dvrk
import numpy
import PyKDL
import timeit
import time

from geometry_msgs.msg import PoseStamped


class example_application:

    def __init__(self):
        self.arm = None
        self.t_dvrk_auro = None
        self.data = None
        self.dvrk_home = None
        self.previous_vector = PyKDL.Vector(0, 0, 0)
        # NDI Sensor topic
        self.topic = '/ndi/Tool/position_cartesian_current'

        # Create nodes to listen to ros data
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber(self.topic, PoseStamped, self.get_pose)

    def configure(self, robot_name):
        # configuration
        print(rospy.get_caller_id(), ' -> configuring dvrk_arm_test for ', robot_name)
        self.arm = dvrk.psm(robot_name)
        self.arm.close_jaw()
        # listern

    def home(self):
        print(rospy.get_caller_id(), ' -> starting home')
        self.arm.home()
        # get current joints just to set size
        goal = numpy.copy(self.arm.get_current_joint_position())
        # go to zero position, for PSM and ECM make sure 3rd joint is past cannula
        goal.fill(0)
        if ((self.arm.name() == 'PSM1') or (self.arm.name() == 'PSM2') or (self.arm.name() == 'PSM3') or (
                self.arm.name() == 'ECM')):
            goal[2] = 0.12
        self.arm.move_joint(goal, interpolate=True)
        rospy.sleep(1)
        self.dvrk_home = self.arm.get_current_position()

    def get_pose(self, data):
        # Gets a single message
        #return rospy.wait_for_message(self.topic, PoseStamped)
        self.data = data

    def pose_to_vector(self, pose):
        return numpy.array([pose.position.x, pose.position.y, pose.position.z])

    def get_transformation(self):
        home = self.data.pose.position
        set_distance = 0.04
        self.arm.dmove(PyKDL.Vector(0, 0, set_distance))
        rospy.sleep(2)
        z_axis = self.data.pose.position
        z_axis = PyKDL.Vector(z_axis.x - home.x, z_axis.y - home.y, z_axis.z - home.z)
        z_axis.Normalize()
        print('Z_axis: ' + str(z_axis))


        self.home()
        self.arm.dmove(PyKDL.Vector(0, set_distance, 0))
        rospy.sleep(2)
        y_axis = self.data.pose.position
        y_axis = PyKDL.Vector(y_axis.x - home.x, y_axis.y - home.y, y_axis.z - home.z)
        y_axis.Normalize()
        print('Y_axis: ' + str(y_axis))


        self.home()
        self.arm.dmove(PyKDL.Vector(set_distance, 0, 0))
        rospy.sleep(2)
        x_axis = self.data.pose.position
        x_axis = PyKDL.Vector(x_axis.x - home.x, x_axis.y - home.y, x_axis.z - home.z)
        x_axis.Normalize()
        print('X_axis: ' + str(x_axis))

        r_dvrk_auro = PyKDL.Rotation(x_axis, y_axis, z_axis)

        print(r_dvrk_auro)
        self.home()
        self.t_dvrk_auro = PyKDL.Frame(r_dvrk_auro, PyKDL.Vector(home.x, home.y, home.z))
        return self.t_dvrk_auro

    def run(self):
        self.home = self.data
        while True:
            self.follow()

    def follow(self):
        current_pose = self.pose_to_vector(self.data.pose)
        #print('Get Pose: {0}'.format(rospy.get_rostime()-start_time))
        delta = self.pose_to_vector(self.home.pose) - current_pose
        dvrk_vector = self.t_dvrk_auro.M.Inverse() * PyKDL.Vector(*delta)
        #print('dvrk_home_position: {0}'.format(self.dvrk_home.p))
        #print('Transform_vector: {0}'.format(rospy.get_rostime()-start_time))
        self.arm.dmove(0.2*dvrk_vector, interpolate=True, blocking=False)
        #print('Move Arm: {0}'.format(rospy.get_rostime()-start_time))
        self.previous_vector = self.previous_vector + 0.001*dvrk_vector
        rospy.sleep(0.02)


if __name__ == '__main__':
    try:
        if (len(sys.argv) != 2):
            print(sys.argv[0], ' requires one argument, i.e. name of dVRK arm')
        else:
            application = example_application()
            application.configure(sys.argv[1])
            application.home()
            application.get_transformation()
            application.run()

    except rospy.ROSInterruptException:
        pass
