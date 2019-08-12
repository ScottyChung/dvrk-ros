import dvrk
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from time import sleep
import numpy
import PyKDL
import rospy
import sys
from visualization_msgs.msg import Marker,MarkerArray
from geometry_msgs.msg import Pose,Vector3,Quaternion,Point
from std_msgs.msg import Bool
import pickle

rospy.init_node("test")
publisher = rospy.Publisher('/dvrk/console/teleop/enable',Bool,queue_size=100)
rospy.sleep(2)
test2 = True
test3 = False
publisher.publish(test3)
sleep(5)
publisher.publish(test2)
