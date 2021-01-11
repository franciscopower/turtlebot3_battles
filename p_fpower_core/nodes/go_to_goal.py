#!/usr/bin/python

import rospy
from geometry_msgs.msg import Twist, Pose, PoseStamped
from nav_msgs.msg import Odometry
import tf
import tf2_ros
import tf2_geometry_msgs
from copy import  deepcopy

from math import pow, atan2, sqrt

class Driver():
    def __init__(self, name):
        self.name = name
        
        self.goal = None
        
        self.cmd_vel_pub = rospy.Publisher(self.name + '/cmd_vel', Twist, queue_size=10)    
        self.goal_sub = rospy.Subscriber(self.name + "/move_base_simple/goal", PoseStamped, self.goal_callback)
                
        self.tfBuffer = tf2_ros.Buffer(rospy.Duration(1200)) #? what is this time
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        
        rospy.loginfo('Waiting for goal...\n')
    
    def goal_callback(self, pose_stamped):
        self.goal = pose_stamped
    
    def drive(self):
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            twist = Twist()
            
            if self.goal==None:
                continue

            goal_copy = deepcopy(self.goal)
            goal_copy.header.stamp = rospy.Time.now()
            goal_base_link = self.tfBuffer.transform(goal_copy, self.name + '/base_footprint',
                                                    rospy.Duration(0.05)) #? what is this time

            if sqrt(goal_base_link.pose.position.x**2 +  goal_base_link.pose.position.y**2) >= 0.2:
                a = atan2(goal_base_link.pose.position.y, goal_base_link.pose.position.x)
                d = sqrt(goal_base_link.pose.position.x**2 +  goal_base_link.pose.position.y**2)
                twist.linear.x = 1 * min(d,1.0)
                twist.angular.z = 1 * a
      
            
            print('\nPublished velocidy:\n\tLinear: ' + str(twist.linear.x) + '\n\tAngular: ' + str(twist.angular.z))
       
            self.cmd_vel_pub.publish(twist)
            rate.sleep()
            
if __name__ == '__main__':
    rospy.init_node('p_fpower', anonymous=False)
    name = rospy.get_name().strip('/')
            
    try:
        driver = Driver(name)
        driver.drive()
    except rospy.ROSException:
        pass