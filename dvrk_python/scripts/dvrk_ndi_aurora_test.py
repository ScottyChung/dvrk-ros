#!/user/bin/env python\
import rospy
import rosbag
from geometry_msgs.msg import PoseStamped

def listener():
    # Create nodes to listen to ros data
    rospy.init_node('listener', anonymous=True)

    # NDI Sensor topic
    topic = '/ndi/01_3B3B6C00_610066___T6D0_S01398/position_cartesian_current'

    # Gets a single message
    my_message = rospy.wait_for_message(topic, PoseStamped)

    # Data getting recorded
    bag.write(topic, my_message)

    # Couple messages
    print('Printing message from ndi node:')
    print(my_message)
    print('Printing only the position information')
    print('The position is: ' + str(my_message.pose.position))
    print('Printing only the rotation information')
    print('The rotation is: ' + str(my_message.pose.orientation))
    bag.close()

def print_bag():
    print('Test reading from the created bag and printing information')
    read_bag = rosbag.Bag('ndi_test.bag')
    for topic, msg, t in read_bag.read_messages():
        print(msg)

if __name__ == '__main__':
    # Bag file holds data
    bag = rosbag.Bag('ndi_test.bag','w')

    listener()
    print_bag()
