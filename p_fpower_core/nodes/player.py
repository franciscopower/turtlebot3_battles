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
                
                point, frame = self.findCentroid(self.image, 'red')
                
                
                print(point)
                
                cv.imshow('Camera raw', self.image)
                cv.imshow('red centroid', frame)
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
      
     
    def findCentroid(self, frame, color):
        """Find the centroid of the largest blob given in an image, given a certain dictionary with binarization limits
            If shake prevention (SP) mode is active, return centroid as (0,0) 
            
        Args:
            frame (np.ndarray): Original Image
            limits_dict (dictionary): dictionary with binarization limits and color space

        Returns:
            tuple: (x,y) coordinates of centroid of largest blob
            np.ndarray: frame_one_area camera image with drawing tool marked
        """
        
        #create copy of frame
        frame_one_area = np.copy(frame)
        
        # segmentate the image
        if color == 'blue':
            I_bin = cv.inRange(frame, (70,0,0), (255,17,17))
        elif color == 'green':
            I_bin = cv.inRange(frame, (0,70,0), (17,255,17))
        elif color == 'red':
            I_bin = cv.inRange(frame, (0,0,70), (17,17,255))
        else:
            print('Color not recognized')
            return (0,0), frame

        #create labels
        _, labels, stats, centroids = cv.connectedComponentsWithStats(I_bin, connectivity=4)

        #identify largest area
        stats[np.where(stats[:, 4] == stats[:, 4].max())[0][0], :] = 0
        big_area_idx = np.where(stats[:, 4] == stats[:, 4].max())[0][0]

        #find centroid
        x, y = centroids[big_area_idx]
        x = int(x)
        y = int(y)
        
        # select mask of largest area
        M_SA = np.zeros(labels.shape, dtype=np.uint8)
        M_SA[labels == big_area_idx] = 255

        if len(stats) != 1:
            #show selected area in green
            frame_one_area[M_SA == 255] = (0,255,0)
            #show centroid of selected area
            frame_one_area = cv.circle(frame_one_area, (x,y), 5, (0,0,255), -1)

        # #show binarized image
        # cv.imshow('bin img', I_bin)

        #use shake prevention
        # discard if area is too small
        # if stats[big_area_idx, 4] < frame.shape[0]*frame.shape[1]*0.01:
        #     x = 0
        #     y = 0
            
        #     if len(stats) != 1:
        #         #show selected area in red
        #         frame_one_area[M_SA == 255] = (0,0,255)
        # discard if it cannot find any whitepoints
        if len(stats) == 1:
            x = 0
            y = 0
        
        return (x,y), frame_one_area 
            
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
    