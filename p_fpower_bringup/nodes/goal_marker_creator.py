#!/usr/bin/python

import rospy
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker

def create_marker(pose):
    my_marker = Marker()
    my_marker.header.frame_id = "world"
    my_marker.header.stamp = rospy.Time.now()
    my_marker.ns = "p_fpower"
    my_marker.id = 0
    my_marker.type = my_marker.ARROW
    my_marker.action = my_marker.ADD
    my_marker.pose = pose
    my_marker.scale.x = 1
    my_marker.scale.y = 0.1
    my_marker.scale.z = 0.1
    my_marker.color.a = 1.0
    my_marker.color.r = 1.0
    my_marker.color.g = 0.0
    my_marker.color.b = 1.0  
    
    return my_marker

def callback(pose_stamped):
    goal = pose_stamped.pose
    
    marker_pub = rospy.Publisher('goal_marker', Marker, queue_size=10)
    marker_pub.publish(create_marker(goal))
    
def listener():
    rospy.init_node('goal_marker_creator', anonymous=True)

    rospy.Subscriber("p_fpower/move_base_simple/goal", PoseStamped, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()