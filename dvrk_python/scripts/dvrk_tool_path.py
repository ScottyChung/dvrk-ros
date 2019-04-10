#!/usr/bin/env python

import dvrk
import rospy
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
from nav_msgs.msg import Path
import PyKDL

class path_tracer:
    def init(self):
        rospy.init_node('tool_path', anonymous=True)
    def trace_path(self,data):
            #pose = PoseStamped(Header(), Pose(Point(*data.p), Quaternion(*data.M.GetQuaternion())))
            pose = data
            self.poses.append(pose)
            pub = rospy.Publisher('tool_path2',Path,queue_size=10)
            pub.publish(Header(frame_id='PSM1_psm_base_link'), self.poses)
            rospy.sleep(0.1)
        

if __name__ == '__main__':
    try:
        app = path_tracer()
        app.init()
        app.poses = []
        rospy.Subscriber('/dvrk/PSM1/position_cartesian_current', PoseStamped, app.trace_path,queue_size=1)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


