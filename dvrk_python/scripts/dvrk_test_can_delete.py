import rospy # module to interact with ROS
from geometry_msgs.msg import PoseStamped # message type

# Make node
rospy.init_node('listener')

# Name of our topic
topic = '/ndi/tool/position_cartesian_current'

# Obtaining a fresh message 
aurora_message = rospy.wait_for_message(topic, PoseStamped)

# Do whatever with message. Here we just print
print(aurora_message)
