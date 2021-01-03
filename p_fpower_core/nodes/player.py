#!/usr/bin/python

import rospy
from geometry_msgs.msg import Twist, Pose, PoseStamped
from nav_msgs.msg import Odometry

from math import pow, atan2, sqrt

goal = None
current_pose = None

def eucledian_distance(pose1, pose2):
    if goal == None or current_pose == None:
        d = 0
    else:
        d = sqrt((pose1.position.x - pose2.position.x)**2 + \
            (pose1.position.y - pose2.position.y)**2)
    return d

def steering_angle(pose1, pose2):
    if goal == None or current_pose == None:
        a = 0
    else:
        a = (atan2(pose1.position.y - pose2.position.y, pose1.position.x - pose2.position.x) - \
            pose2.orientation.z)
    return a

def driver():
    pub = rospy.Publisher('/p_fpower/differential_drive_controller/cmd_vel', Twist, queue_size=10)
    
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        
        listener()
        
        twist = Twist()
    
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        
        if eucledian_distance(goal, current_pose) >= 0.1:
            twist.linear.x = 1 * eucledian_distance(goal, current_pose)
            twist.angular.z = 2.0 * steering_angle(goal, current_pose)
        else:
            twist.linear.x = 0
            twist.angular.z = 0
        
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
    rospy.Subscriber("p_fpower/move_base_simple/goal", PoseStamped, goal_callback)
    rospy.Subscriber("/p_fpower/differential_drive_controller/odom", Odometry, pos_callback)

if __name__ == '__main__':
    
    rospy.init_node('player_core', anonymous=True)
    
    try:
        driver()
    except rospy.ROSInterruptException:
        pass