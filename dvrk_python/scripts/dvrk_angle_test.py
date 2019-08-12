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
from sensor_msgs.msg import Joy, JointState
import rosbag 

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
        self.__position_jaw_desired = 0.1
        #self.arm.close_jaw()
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
        marker.color.g = 0.5
        marker.color.b = 0.7
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
        marker.color.g = 0.5
        marker.color.b = 0.7
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

 # perform angle test
    def angle_test(self):
        # Read in csv file points
        ## read_csv (filename that user provided)
        filename = sys.argv[2]
        angles = np.loadtxt(filename, delimiter=',')

        #NDI Sensor Topic
        topic = '/ndi/0B_3B803800_610059___T6F0_L00080/position_cartesian_current'
        # Bag file holds data
        #read bag file (filename that user provides)
        bagfile_name = sys.argv[3]
        bagfile_dir = '/home/robotuser/Data/point_test/'
        bagfile_name = bagfile_dir + bagfile_name

        bag = rosbag.Bag(bagfile_name,'w')
        print('Name of Bag File:' + bagfile_name)
        # Command robot go to each point 
        for idx,angle in enumerate(angles):
            #point = self.convert_units(point)
            self.addMarker(self.arm.get_current_position().p,idx)
            print('Moving to: ' + str(angle))
            self.arm.move_joint_one(float(angle),5)
            rospy.sleep(1)
 #####
            for letter in 'a':

                # Pause at point
                rospy.sleep(1)

                #Get Message from Aurora 
                print('Getting Aurora message')
                aurora_message = rospy.wait_for_message(topic, PoseStamped)
                #Bag file
                bag.write(topic, aurora_message)


                #Get /dvrk/PSM1/joint_states topic message
                print('Getting joint message from DVRK')
                aurora_message = rospy.wait_for_message('/dvrk/'+sys.argv[1]+'/state_joint_current',JointState)
                print(aurora_message)
                #Save to bag file
                bag.write('/dvrk/'+sys.argv[1]+'/state_joint_current',aurora_message)

                # print(self.dmove.get_joint_one().p)
                # Repeat for remaining point
    
        #Close bag
        bag.close()


    # main method
    def run(self):
        rospy.sleep(2)
        self.home()
        rospy.sleep(1)
        self.angle_test()
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

