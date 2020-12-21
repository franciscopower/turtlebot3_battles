#!/usr/bin/python

import rospy
from geometry_msgs.msg import Twist

def talker():
    pub = rospy.Publisher('/p_fpower/differential_drive_controller/cmd_vel', Twist, queue_size=10)
    rospy.init_node('player_core', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    
    while not rospy.is_shutdown():
        twist = Twist()
        
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 1.0
        
        
        rospy.loginfo(twist)
        pub.publish(twist)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass