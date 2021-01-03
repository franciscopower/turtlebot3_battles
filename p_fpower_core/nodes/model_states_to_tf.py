#!/usr/bin/python

import rospy
from std_msgs.msg import String
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseArray
import tf

def talker(states):
    # pub = rospy.Publisher('turtlebot_poses', PoseArray, queue_size=10)
    br = tf.TransformBroadcaster()
    
    
    pose = PoseArray()
    names = states.name
    poses = states.pose
    
    for i in range(2,len(names)):
        name = names[i]
        pose = poses[i]
        
        br.sendTransform((pose.position.x, pose.position.y, pose.position.z),
                        tf.transformations.quaternion_from_euler(pose.orientation.x, pose.orientation.y, pose.orientation.z),
                        rospy.Time.now(),
                        name+"/base_footprint",
                        "world")
    
    rospy.loginfo(poses)
    # pub.publish(pose)

def callback(states):
    # rospy.loginfo(str(states.name) + ": " + str(states.pose))
    try:
        talker(states)
    except rospy.ROSInterruptException:
        pass
    
def listener():

    rospy.init_node('model_states_to_tf', anonymous=True)

    rospy.Subscriber("/gazebo/model_states", ModelStates, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()