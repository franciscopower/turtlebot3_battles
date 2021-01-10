#!/usr/bin/python

import rospy
from geometry_msgs.msg import Twist, Pose, PoseStamped
from nav_msgs.msg import Odometry
import tf
import tf2_ros
import tf2_geometry_msgs
from copy import  deepcopy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import cv2 as cv
from math import pow, atan2, sqrt
import numpy as np

class Player():
    def __init__(self, name):
        self.name = name
        
        # get player color, hunter, prey ---------------
        try:
            names_red = rospy.get_param('/red_players')
            names_green = rospy.get_param('/green_players')
            names_blue = rospy.get_param('/blue_players')
        except:
            rospy.logfatal('No team parameters loaded')
            exit(0)
        else:
            if self.name in names_red:
                self.my_team = 'red'
                self.my_team_players = names_red
                self.prey_team_players = names_green
                self.hunter_team_players = names_blue
            elif self.name in names_green:
                self.my_team = 'green'
                self.my_team_players = names_green
                self.prey_team_players = names_blue
                self.hunter_team_players = names_red
            elif self.name in names_blue:
                self.my_team =  'blue'
                self.my_team_players = names_blue
                self.prey_team_players = names_red
                self.hunter_team_players = names_green
            else:
                rospy.logfatal('Name is not in any team')
                exit(0)
        
            print('My name is '+ self.name + '. I am team ' + self.my_team +
                '. I am hunting ' + str(self.prey_team_players) + ' and being hunted by ' + str(self.hunter_team_players))
        # --------------------------------------------
        
        # initialize cvbridge
        self.cv_bridge = CvBridge()  
        
        # initialize publishers and subscribers         
        self.cmd_vel_pub = rospy.Publisher(self.name + '/cmd_vel', Twist, queue_size=10)    
        self.goal_sub = rospy.Subscriber(self.name + "/move_base_simple/goal", PoseStamped, self.goalCallback)
        self.camera_sub = rospy.Subscriber(self.name + '/camera/rgb/image_raw', Image, self.imageCallback)
              
        # initialize transform buffer  
        self.tfBuffer = tf2_ros.Buffer(rospy.Duration(1200)) #? what is this time
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        
        
    
    def goalCallback(self, pose_stamped):
        self.goal = pose_stamped
        
    def imageCallback(self, image):
        self.image = self.cv_bridge.imgmsg_to_cv2(image, 'bgr8')
        
    def imageProcessor(self):
        rate = rospy.Rate(10)
        k = ''
        while not rospy.is_shutdown() and k!=ord('q'):
            
            try:
                cv.imshow('Camera raw', self.image)
                
                
                
                
                
                
                k = cv.waitKey(1)
            except:
                pass
            
            rate.sleep
    
    def driveToGoal(self):
        """
        Creates twist message to drive robot to goal
        """
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            twist = Twist()
            try:
                goal_copy = deepcopy(self.goal)
                goal_copy.header.stamp = rospy.Time.now()
                goal_base_link = self.tfBuffer.transform(goal_copy, self.name + '/base_footprint',
                                                        rospy.Duration(0.05)) #? what is this time

                if sqrt(goal_base_link.pose.position.x**2 +  goal_base_link.pose.position.y**2) >= 0.2:
                    a = atan2(goal_base_link.pose.position.y, goal_base_link.pose.position.x)
                    d = sqrt(goal_base_link.pose.position.x**2 +  goal_base_link.pose.position.y**2)
                    twist.linear.x = 1 * min(d,1.0)
                    twist.angular.z = 1 * a
            
            except:
                pass         
            
            print('\nPublished velocidy:\n\tLinear: ' + str(twist.linear.x) + '\n\tAngular: ' + str(twist.angular.z))
       
            self.cmd_vel_pub.publish(twist)
            rate.sleep()
            
if __name__ == '__main__':
    rospy.init_node('green1', anonymous=False)
    name = rospy.get_name().strip('/')
                    
    try:
        player = Player(name)
        player.imageProcessor()
    except KeyboardInterrupt:
        print("Shutting down vision node.")
        cv.DestroyAllWindows()
    except rospy.ROSException:
        pass
    