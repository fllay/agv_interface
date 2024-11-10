#!/usr/bin/env python3
import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.

import mbf_msgs.msg as mbf_msgs
from mbf_msgs.msg import ExePathAction
from mbf_msgs.msg import ExePathResult
from mbf_msgs.msg import ExePathGoal

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Path


def callback(data):

    global p_init 
    p_init = data
    

def callbackPath(data):
    #print(data.poses)
    ppaths = data.poses
    ppaths.insert(0,p_init)

    ii = 0
    usr_path = Path()


    
    for pp in ppaths:
        #print(pp)
        pose = PoseStamped()
        pose.header.seq = ii
        pose.header.frame_id = 'map'
        pose.pose.position.x = pp.position.x
        pose.pose.position.y =  pp.position.y
        pose.pose.position.z =  pp.position.z

        pose.pose.orientation.x = pp.orientation.x
        pose.pose.orientation.y = pp.orientation.y
        pose.pose.orientation.z = pp.orientation.z
        pose.pose.orientation.w = pp.orientation.w
        usr_path.poses.append(pose)
        ii = ii + 1

    goal = ExePathGoal(path=usr_path)

    ret = client.send_goal_and_wait(goal)
    outcome = client.get_result().outcome


def path_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    global client
    client = actionlib.SimpleActionClient('/move_base_flex/exe_path', ExePathAction)
    rospy.Subscriber("robot_pose_p", Pose, callback)
    rospy.Subscriber("path_to_follow", PoseArray, callbackPath)
    

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()
    print("Server is up")
 
    outcome = 0

    rospy.spin()
   

    
    return outcome  

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('path_client')
        path_client()
       
    except rospy.ROSInterruptException:
        print("program interrupted before completion")
