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
from visualization_msgs.msg import Marker,MarkerArray
from nav_msgs.msg import Path
from sensor_msgs.msg import Joy

# example of application using arm.py
class example_application:


    # configuration
    def configure(self, robot_name):
        print rospy.get_caller_id(), ' -> configuring dvrk_psm_test for ', robot_name
        self.arm = dvrk.psm(robot_name)
        self.markerArray = MarkerArray()
        self.publisher = rospy.Publisher('/visualization_marker_array',MarkerArray,queue_size=100)
        rospy.sleep(2)
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

    def addMarker(self,position,id):
        
        marker = Marker()
        marker.header.frame_id = "PSM1_psm_base_link"
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.SPHERE
        marker.id = id
        marker.action = Marker.ADD
        marker.scale.x = 0.005
        marker.scale.y = 0.005 
        marker.scale.z = 0.005
        marker.color.a = 1.0 
        marker.color.r = 0.1 
        marker.color.g = 0.9
        marker.color.b = 0.6
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        self.markerArray.markers.append(marker)
        self.publisher.publish(self.markerArray)

        return()

    # convert units
    def convert_units(self, point):
        #from inches to meters
        #point = point * 0.0254
        return point * 0.0254 # same as above

    # perform point test
    def point_test(self):
        # Read in csv file points
        ## read_csv (filename that user provided)
        filename = sys.argv[2]
        points = np.loadtxt(filename, delimiter=',')

        # Command robot go to each point 
        for idx,point in enumerate(points):
            point = self.convert_units(point)
            # Add slight z offset
            point[2] -= 0.2 
            self.addMarker(point,idx)
            print('Moving to: ' + str(point))
            self.arm.move(PyKDL.Vector(point[0],point[1],point[2]))

            # Pause at point
            #rospy.sleep(1.5)

            # Data collection
            print(self.arm.get_current_position().p)
            # Repeat for remaining point

    # main method
    def run(self):
        rospy.sleep(2)
        #self.home()
        self.point_test()
        #rospy.spin()

if __name__ == '__main__':
    try:
        if (len(sys.argv) < 2):
            print sys.argv[0], ' requires one argument, i.e. name of dVRK arm'
        else:
            application = example_application()
            application.configure(sys.argv[1])
            application.run()

    except rospy.ROSInterruptException:
        pass

