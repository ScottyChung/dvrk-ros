#!/usr/bin/env python

import dvrk
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA

class marker_application:

    def sphere_marker(self, position, unique_id=0,rgba=(1.0,1.0,1.0,1.0)):
        marker = Marker(
                    type=Marker.SPHERE,
                    id=unique_id,
                    pose=Pose(Point(*position), Quaternion(0, 0, 0, 1)),
                    scale=Vector3(0.06, 0.06, 0.06),
                    header=Header(frame_id='PSM1_psm_base_link'),
                    color=ColorRGBA(*rgba))
        self.publisher.publish(marker)

    def main(self):
        print('published')
        rospy.spin()

if __name__=='__main__':
    rospy.init_node("testing")
    ma = marker_application()
    ma.publisher = rospy.Publisher('vmarker', Marker, queue_size=5)
    #Must sleep to allow subscribers to find the publisher 
    rospy.sleep(1)
    ma.main()
