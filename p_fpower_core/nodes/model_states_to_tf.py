#!/usr/bin/python

import rospy
from std_msgs.msg import String
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseArray

def talker(states):
    pub = rospy.Publisher('turtlebot_poses', PoseArray, queue_size=10)
    
    pose = PoseArray()
    pose.header = states.name
    pose.poses = states.pose
    
    rospy.loginfo(pose)
    pub.publish(pose)

def callback(states):
    # rospy.loginfo(str(states.name) + ": " + str(states.pose))
    try:
        talker(states)
    except rospy.ROSInterruptException:
        pass
    
def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/gazebo/model_states", ModelStates, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()