#!/usr/bin/python

import rospy
from geometry_msgs.msg import Twist, Pose, PoseStamped, TransformStamped
from copy import  deepcopy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from gazebo_msgs.msg import ContactsState, ModelState
from std_msgs.msg import Int16
from nav_msgs.msg import Odometry
import tf

import cv2 as cv
from math import pow, atan2, sqrt
import numpy as np
import random

class Player():
    def __init__(self, name):
        self.name = name
        
        # get player color, hunter, prey ---------------
        try:
            names_red = rospy.get_param('/red_players')
            names_green = rospy.get_param('/green_players')
            names_blue = rospy.get_param('/blue_players')
            self.arena_size = rospy.get_param('/arena_size')
        except:
            rospy.logfatal('No team parameters loaded')
            exit(0)
        else:
            if self.name in names_red:
                self.my_team = 'red'
                self.my_team_players = names_red
                self.prey_team = 'green'
                self.prey_team_players = names_green
                self.hunter_team = 'blue'
                self.hunter_team_players = names_blue
            elif self.name in names_green:
                self.my_team = 'green'
                self.my_team_players = names_green
                self.prey_team = 'blue'
                self.prey_team_players = names_blue
                self.hunter_team = 'red'
                self.hunter_team_players = names_red
            elif self.name in names_blue:
                self.my_team =  'blue'
                self.my_team_players = names_blue
                self.prey_team = 'red'
                self.prey_team_players = names_red
                self.hunter_team = 'green'
                self.hunter_team_players = names_green
            else:
                rospy.logfatal('Name is not in any team')
                exit(0)
        
        # --------------------------------------------
        
        # initialize cvbridge
        self.cv_bridge = CvBridge()  
        
        self.goal = None
        self.score = 0
        
        # initialize publishers and subscribers         
        self.cmd_vel_pub = rospy.Publisher(self.name + '/cmd_vel', Twist, queue_size=10)    
        self.score_pub = rospy.Publisher(self.name + '/score', Int16, queue_size=10 )
        self.model_state_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
        
        self.camera_sub = rospy.Subscriber(self.name + '/camera/rgb/image_raw', Image, self.imageCallback)
        self.collision_sub = rospy.Subscriber(self.name + '/contact', ContactsState, self.updateScore)
        self.odom_sub = rospy.Subscriber(self.name + "/odom", Odometry, self.odomCallback)
       
    def odomCallback(self, odom):
        self.my_pose = odom.pose.pose            
       
    def imageCallback(self, image):
        self.image = self.cv_bridge.imgmsg_to_cv2(image, 'bgr8')
        
    def checkBoundaries(self):
        _,_,orientation = tf.transformations.euler_from_quaternion([self.my_pose.orientation.x, self.my_pose.orientation.y, self.my_pose.orientation.z, self.my_pose.orientation.w])
        
        a = 0
        in_bounds = True
        
        x_bound = self.arena_size[0]/2
        y_bound = self.arena_size[1]/2
        
        if self.my_pose.position.x >= x_bound:
            a = np.sign(orientation)*np.pi - orientation
            in_bounds = False
            # print('Out +X. Correcting: ' + str(a))
        elif self.my_pose.position.x <= -x_bound:
            a = - orientation
            in_bounds = False
            # print('Out -X. Correcting: ' + str(a))
            
        if self.my_pose.position.y >= y_bound:
            if orientation <= np.pi/2:
                a = -(np.pi/2 + orientation)
            else:
                a = 3*np.pi/2 - orientation
            in_bounds = False
            # print('Out +Y. Correcting: ' + str(a))
        elif self.my_pose.position.y <= -y_bound:
            if orientation >= -np.pi/2:
                a = np.pi/2 - orientation
            else:
                a = -(3*np.pi/2 + orientation)
            in_bounds = False
            # print('Out -Y. Correcting: ' + str(a))
        
        return in_bounds, a
        
        
    def playCatch(self):
        rate = rospy.Rate(10)
        pm = 1
        while not rospy.is_shutdown():
            
            try:  
            
                twist = Twist()
                
                point, frame = self.findCentroid(self.image, self.prey_team)
                in_bounds, bound_correction_angle = self.checkBoundaries()   
                if in_bounds:    
                    if point==(0,0):
                        angular_vel = pm * 0.8
                        linear_vel = 0 
                    else:
                        angular_vel = -0.005 * (point[0] - self.image.shape[1]/2)
                        linear_vel = 1
                        pm = np.sign(angular_vel)
                
                else:
                    linear_vel = 1
                    angular_vel = bound_correction_angle
                    
                twist.angular.z = angular_vel
                twist.linear.x = linear_vel
                                
                # cv.imshow('Camera raw', self.image)
                # cv.imshow('prey centroid', frame)
                # k = cv.waitKey(1)
                # if k == ord('q'):
                #     twist.angular.z = 0
                #     twist.linear.x = 0
                #     self.cmd_vel_pub.publish(twist)
                #     break

                self.cmd_vel_pub.publish(twist)
            except KeyboardInterrupt:
                twist.angular.z = 0
                twist.linear.x = 0
                self.cmd_vel_pub.publish(twist)
            except Exception as e:
                pass
                # print(e)
                
            rate.sleep
     
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

    def updateScore(self, contact_state):
        if contact_state.states!=[]:
            if any(prey in str(contact_state.states[0]) for prey in self.prey_team_players):
                self.score += 1
                self.score_pub.publish(self.score)
                # print('Captured prey! :D')
                
            elif any(hunter in str(contact_state.states[0]) for hunter in self.hunter_team_players):
                self.score -= 1
                self.score_pub.publish(self.score)
                # print('Got captured... :(')
                
                # return to spawn point
                base = ModelState()
                base.model_name = self.name
                base.pose.position.x = random.uniform(-self.arena_size[0]/2,self.arena_size[0]/2)
                base.pose.position.y = random.uniform(-self.arena_size[1]/2,self.arena_size[1]/2)
                base.pose.position.z = 0.5
                
                self.model_state_pub.publish(base)
                
            # rate = rospy.Rate(10)
            # rate.sleep()
                     
    def __str__(self):
        s = '----------------------------' + \
            '\nName: ' + self.name + \
            '\nPrey: ' + str(self.prey_team_players) + \
            '\nHunters: ' + str(self.hunter_team_players) + \
            '\n\nScore: ' + str(self.score) + \
            '\n----------------------------'
        return s
           
if __name__ == '__main__':
    rospy.init_node('green1', anonymous=False)
    name = rospy.get_name().strip('/')
                    
    try:
        player = Player(name)
        print(player)
        player.playCatch()
        print(player)
    except KeyboardInterrupt:
        print("Shutting down vision node.")
        cv.DestroyAllWindows()
    except rospy.ROSException:
        pass
    