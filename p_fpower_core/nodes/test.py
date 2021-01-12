#!/usr/bin/python

# Start up ROS pieces.
import roslib
import rospy
import tf

# ROS messages.
from nav_msgs.msg import Odometry

class QuatToEuler():
    def __init__(self):

        # Create subscribers and publishers.
        sub_odom  = rospy.Subscriber("/green1/odom", Odometry, self.odom_callback)

    # Odometry callback function.
    def odom_callback(self, msg):
        # Convert quaternions to Euler angles.
        (r, p, y) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        print(y)

# Main function.    
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('quat_to_euler')
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        quat_to_euler = QuatToEuler()
        rospy.spin()
    except rospy.ROSInterruptException: pass