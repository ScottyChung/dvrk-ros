import dvrk
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy
import PyKDL
import rospy
import sys
from visualization_msgs.msg import Marker,MarkerArray
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose,Vector3,Quaternion,Point
import pickle
import wfu_helpers

markerArray = MarkerArray()

def addArrow(position,clr,id=0):
	origin = [xyz1.p[0],xyz1.p[1],xyz1.p[2]]
	end = [position[0],position[1],position[2]]
	marker = Marker()
	marker.header.frame_id = "PSM1_psm_base_link"
	marker.header.stamp = rospy.Time.now()
	marker.type = Marker.ARROW
	marker.id = id
	marker.action = Marker.ADD
	marker.points = [Point(*origin),Point(*end)]
	marker.scale.x = 0.001
	marker.scale.y = 0.002 
	marker.scale.z = 0
	marker.color.a = clr[0]
	marker.color.r = clr[1]
	marker.color.g = clr[2]
	marker.color.b = clr[3]
	#marker.pose.position.x = position[0]
	#marker.pose.position.y = position[1]
	#marker.pose.position.z = position[2]
	#marker.pose.orientation.x = 0.0
	#marker.pose.orientation.y = 0.0
	#marker.pose.orientation.z = 0.0
	#marker.pose.orientation.w = 1.0
	markerArray.markers.append(marker)
	publisherM.publish(markerArray)
	print("Point plotted")
	return()

def addMarker(position,id):
	
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
	marker.color.r = 0 
	marker.color.g = 0
	marker.color.b = 0 
	marker.pose.position.x = position[0]
	marker.pose.position.y = position[1]
	marker.pose.position.z = position[2]
	marker.pose.orientation.x = 0.0
	marker.pose.orientation.y = 0.0
	marker.pose.orientation.z = 0.0
	marker.pose.orientation.w = 1.0
	markerArray.markers.append(marker)
	publisherM.publish(markerArray)
	print("Point plotted")
	return()

rospy.init_node("test")
publisherM = rospy.Publisher('/visualization_marker_array',MarkerArray,queue_size=100)
publisherC = rospy.Publisher('/dvrk/console/teleop/enable',Bool,queue_size=100)
rospy.sleep(2)
			 
marker = Marker()
marker.header.frame_id = "PSM1_psm_base_link"
marker.header.stamp = rospy.Time.now()
marker.type = Marker.CUBE
marker.action = Marker.ADD
marker.id = 5
marker.scale.x = 0.2
marker.scale.y = 0.1 
marker.scale.z = 0.1
marker.color.a = 1.0 
marker.color.r = 1.0 
marker.color.g = 1.0
marker.color.b = 1.0 
marker.pose.position.x = 0
marker.pose.position.y = 0
marker.pose.position.z = -0.1

marker.pose.orientation.x = 0.0
marker.pose.orientation.y = 0.0
marker.pose.orientation.z = 0.0
marker.pose.orientation.w = 1.0
markerArray.markers.append(marker)
publisherM.publish(markerArray)

p = dvrk.psm(sys.argv[1])
x = raw_input("press 1 when ready for first point: ")
xyz1 = p.get_current_position()
print("XYZ:" + str(xyz1.p))
print("Rotation" + str(xyz1.M))
addMarker(xyz1.p,1)


y = raw_input("press 2 when ready for second point: ")
xyz2 = p.get_current_position()
print("XYZ:" + str(xyz2.p))
print("Rotation" + str(xyz2.M))
addMarker(xyz2.p,2)


z = raw_input("press 3 when ready for third point: ")
xyz3 = p.get_current_position()
print("XYZ:" + str(xyz3.p))
print("Rotation" + str(xyz3.M))
addMarker(xyz3.p,3)


#Define the axes
x = xyz2.p - xyz1.p      #X axis defined by frist two points
y_temp = xyz3.p - xyz1.p #Create temporary z from first and third point
z = x*y_temp             #Crossing x and ytemp to form z
y = z*x                  #Cross z and x to get final y

#Normalize vectors
z.Normalize()
y.Normalize()
x.Normalize()
print("X normalized: " + str(x))
print("Y normalized: " + str(y))
print("Z normalized: " + str(z))

origin_point = PyKDL.Vector(xyz1.p[0],xyz1.p[1],xyz1.p[2])
dcs_rotation = PyKDL.Rotation(x[0],y[0],z[0],
                       x[1],y[1],z[1],
                       x[2],y[2],z[2])
dcs = PyKDL.Frame(dcs_rotation, origin_point)
fh = open('coordinate', 'w')
pickle.dump(dcs, fh)

print("Control switched to MTM")
publisherC.publish(True)
raw_input('Press any button to switch back to  dVRK')
publisherC.publish(False)
p.close_jaw()

#Moving in defined coordinate system
raw_input('Moving 5cm in positive z')
goal = PyKDL.Frame()
goal.p = PyKDL.Vector(0,0,0.05)
#p.move(dcs*goal.p)
wfu_helpers.vmove(p, goal.p, 0.05, dcs)

raw_input('Press any button to switch back to  MTM')
publisherC.publish(True)
