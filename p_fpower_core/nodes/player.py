#!/usr/bin/python

import rospy
from geometry_msgs.msg import Twist, Pose, PoseStamped
from nav_msgs.msg import Odometry
import tf

from math import pow, atan2, sqrt

goal = None
current_pose = None

def eucledian_distance(pose1, pose2):
    d = sqrt((pose1.position.x - pose2.position.x)**2 + \
        (pose1.position.y - pose2.position.y)**2)
    return d

def steering_angle(pose1, pose2):
    [_, _, rz] = tf.transformations.euler_from_quaternion([pose2.orientation.x, pose2.orientation.y, pose2.orientation.z, pose2.orientation.w])
    a = (atan2(pose1.position.y - pose2.position.y, pose1.position.x - pose2.position.x) - rz)
    return a


def driver():
    pub = rospy.Publisher('/p_fpower/cmd_vel', Twist, queue_size=10)
    
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        
        listener()
        
        twist = Twist()
        
        if goal!=None and current_pose!=None:
            if eucledian_distance(goal, current_pose) >= 0.1:
                twist.linear.x = 1 * eucledian_distance(goal, current_pose)
                twist.angular.z = 1 * steering_angle(goal, current_pose)

        rospy.loginfo(twist)

        pub.publish(twist)
        
        rate.sleep()
        
        
def goal_callback(pose_stamped):
    global goal
    goal = pose_stamped.pose
    # rospy.loginfo(goal)
    
def pos_callback(odometry):
    global current_pose
    current_pose = odometry.pose.pose
    
def listener():
    rospy.Subscriber("/p_fpower/move_base_simple/goal", PoseStamped, goal_callback)
    rospy.Subscriber("/p_fpower/odom", Odometry, pos_callback)


if __name__ == '__main__':
    
    rospy.init_node('player_core', anonymous=True)
    
    try:
        driver()
    except rospy.ROSInterruptException:
        pass