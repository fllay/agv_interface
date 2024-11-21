#!/usr/bin/env python3

# to test go to waypoint
# mosquitto_pub -h localhost -t "PLC_AMR/GWP/" -m  '{"CMD": {"map_name": "home2nd", "waypoint_name": "w1"}}'
# 
import time
import json
import rospy
import signal
import sys

def signal_handler(sig, frame):
    print("Disconnecting from broker...")
    sys.exit(0)
#from <package_name>.srv import <ServiceName>  # Import the service type
from agv_interface.srv import awaypoint, awaypointResponse
from agv_interface.srv import getpaths, getpathsResponse
from agv_interface.srv import getrcvpoint, getrcvpointResponse


import paho.mqtt.client as mqtt
import actionlib

from mbf_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult, ExePathActionResult
from actionlib_msgs.msg import GoalStatusArray
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseWithCovariance

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Path

# Local broker settings
#BROKER_ADDRESS = "localhost"  # Since the broker is running locally
BROKER_ADDRESS = "127.0.0.1"
BROKER_PORT = 1883

# Define topic and message
TOPIC_GOAL = "PLC_AMR/GWP/"
TOPIC_PATH = "PLC_AMR/PATH/"
TOPIC_POSE = "PLC_AMR/POSE/"
TOPIC_CANCEL_GOAL = "PLC_AMR/CANCEL/"
TOPIC_A = "home/a"
TOPIC_AMR_STATUS = "AMR_PLC/Status/"
TOPIC_PLC_STATUS = "PLC_AMR/Status/"

# Create MQTT client instance
#client = mqtt.Client("LocalPythonClient")
client = mqtt.Client(protocol=mqtt.MQTTv311)

is_mbf_goal_server_available = False 

READY = '0'
CMDRCV = '0'
FINISH =  '0'

status_running = True  # Control variable to toggle loop
def publish_status_message(event):
    global READY
    global CMDRCV
    global FINISH
    global status_running
    if status_running == True:
        status_running = False
        client.publish(TOPIC_AMR_STATUS, READY + ',' + CMDRCV + ',' + FINISH + ',' + 'A' + ',' + 'A')
    else:
        status_running = True 
        client.publish(TOPIC_AMR_STATUS, READY + ',' + CMDRCV + ',' + FINISH + ',' + 'A' + ',' + 'A')



def check_mbf_goal_result(event):
    """Timer callback function to check if the action result is available."""
    global FINISH
    print("Navigation state")
    print(mbf_client.get_state())
    if mbf_client.get_state() in [actionlib.GoalStatus.SUCCEEDED, actionlib.GoalStatus.ABORTED,
                              actionlib.GoalStatus.REJECTED, actionlib.GoalStatus.PREEMPTED]:
        rospy.loginfo("Action completed with state: %d", mbf_client.get_state())
        result = mbf_client.get_result()
        #rospy.loginfo("Action result ==== : %s", result.message)
        
        if mbf_client.get_state() == 3:
          FINISH = '1'
          rospy.loginfo("Action result: %s", result)
        
# Callback function to process the GoalStatusArray message
def goal_status_callback(data):
    global FINISH
    print("mbf result-======")
    print(data.status.status)
    if data.status.status == 3:
        print("Goal reach!!!!")
        FINISH = '1'
    elif data.status.status == 2:
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.linear.y = 0.0
        stop_msg.linear.z = 0.0
        stop_msg.angular.x = 0.0
        stop_msg.angular.y = 0.0
        stop_msg.angular.z = 0.0

        # Publish the stop message
    
        stop_pub.publish(stop_msg)
        
def path_status_callback(data):
    global FINISH
    print("path result-======")
    print(data.status.status)
    if data.status.status == 3:
        print("Path reach!!!!")
        FINISH = '1'
    elif data.status.status == 2:
        pass
    else:
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.linear.y = 0.0
        stop_msg.linear.z = 0.0
        stop_msg.angular.x = 0.0
        stop_msg.angular.y = 0.0
        stop_msg.angular.z = 0.0

        # Publish the stop message
    
        stop_pub.publish(stop_msg)

def check_mbf_goal_server_availability(event):
    # Check if the action server is available and publish the status
    global is_mbf_goal_server_available
    server_available = mbf_client.wait_for_server(timeout=rospy.Duration(1.0))
    if server_available:
        rospy.loginfo("Action server is available.")
        is_mbf_goal_server_available = True
    else:
        rospy.logwarn("Action server not available.")
        is_mbf_goal_server_available = False


def mbf_goal(t_pose):
    # Create a goal message
    print("in Goal")
    print(t_pose)
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"  # Typically "map" or "odom"
    goal.target_pose.header.stamp = rospy.Time.now()
    
    # Set the goal position
    goal.target_pose.pose.position.x = t_pose.pose.position.x
    goal.target_pose.pose.position.y = t_pose.pose.position.y
    goal.target_pose.pose.position.z = t_pose.pose.position.z
    
    # Set orientation using yaw
 
    goal.target_pose.pose.orientation.x = t_pose.pose.orientation.x
    goal.target_pose.pose.orientation.y = t_pose.pose.orientation.y
    goal.target_pose.pose.orientation.z = t_pose.pose.orientation.z
    goal.target_pose.pose.orientation.w = t_pose.pose.orientation.w

    return goal 



# Define callbacks
def on_connect(client, userdata, flags, rc):

    global READY
    global CMDRCV
    global FINISH
    print("Connected to local broker with result code " + str(rc))
    if rc == 0:
        print("Connected successfully")
        # Subscribe to each topic separately
        client.subscribe(TOPIC_GOAL,qos=1 )
        client.subscribe(TOPIC_A, qos=1)
        client.subscribe(TOPIC_PLC_STATUS)
        client.subscribe(TOPIC_CANCEL_GOAL, qos=1)
        client.subscribe(TOPIC_PATH, qos=1)
        client.subscribe(TOPIC_POSE, qos=1)
        
        READY =  '1'
    else:
        print(f"Connection failed with code {rc}")


def on_message(client, userdata, msg):
    #print(f"Received message: {msg.payload.decode()} on topic {msg.topic}")
    global CMDRCV
    global FINISH
    if msg.topic == TOPIC_GOAL:
        CMDRCV = '1'
        try:
            # Create a service proxy with the service name and type
            service_proxy = rospy.ServiceProxy('/get_a_waypoint', awaypoint)

            # Prepare the request data (replace `request_param` with actual parameters)
            print(msg.payload.decode())
            payload = json.loads(msg.payload.decode())
            print(payload)
            #response = service_proxy('w1', 'office1')
         
            response = service_proxy(payload['CMD']["waypoint_name"], payload['CMD']["map_name"])

            # Process the response
            rospy.loginfo("Response: %s", response)
            if is_mbf_goal_server_available == True:
                tgoal = mbf_goal(response)
                print(tgoal)
                print("Sending goal")
                client.publish(TOPIC_AMR_STATUS, '1,1,0,GWP,' + payload['CMD']["waypoint_name"])
                mbf_client.send_goal(tgoal)
            else:
                print("Pass")

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)

    elif msg.topic == TOPIC_CANCEL_GOAL:
        print("Cancel goal -------->")
        # Create a GoalID message
        cancel_msg = GoalID()
        # Option 1: Cancel all goals (leave id as an empty string)
        cancel_msg.id = ""
        # Publish the cancel message
        cancel_pub.publish(cancel_msg)
    elif msg.topic == TOPIC_A:
        print("TOPIC A accepted")
    elif msg.topic == TOPIC_PATH:
        CMDRCV = '1'
        try:
            # Create a service proxy with the service name and type
            service_path_proxy = rospy.ServiceProxy('/get_path', getpaths)

            # Prepare the request data (replace `request_param` with actual parameters)
            print(msg.payload.decode())
            payload = json.loads(msg.payload.decode())
            print(payload)
            #response = service_proxy('w1', 'office1')
         
            response = service_path_proxy( payload['CMD']["map_name"], payload['CMD']["path_name"])

            # Process the response
            rospy.loginfo("Response: %s", response.path_array)
            if is_mbf_goal_server_available == True:
                #tgoal = mbf_goal(response)
                #print(tgoal)
                print("Sending path")
                client.publish(TOPIC_AMR_STATUS, '1,1,0,PATH,' + payload['CMD']["path_name"])
                path_pub.publish(response.path_array)
                
                #mbf_client.send_goal(tgoal)
            else:
                print("Pass")

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
    elif msg.topic == TOPIC_POSE:
        CMDRCV = '1'
        try:
            # Create a service proxy with the service name and type
           
        
            service_rcv_proxy = rospy.ServiceProxy('/get_rcv', getrcvpoint)

            # Prepare the request data (replace `request_param` with actual parameters)
            print(msg.payload.decode())
            payload = json.loads(msg.payload.decode())
            print(payload)
            #response = service_proxy('w1', 'office1')
         
            response = service_rcv_proxy( payload['CMD']["map_name"], payload['CMD']["rcv_name"])

            # Process the response
            rospy.loginfo("Response: %s", response.pose)
            pose = PoseWithCovariance(pose=response.pose)
            pose2 = PoseWithCovarianceStamped(pose=pose)
            pub_init_pose.publish(pose2)
            
        
            print("Sending pose")
            client.publish(TOPIC_AMR_STATUS, '1,1,0,POSE,' + payload['CMD']["rcv_name"])
        

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)



       
    elif msg.topic == TOPIC_PLC_STATUS:
        payload = json.loads(msg.payload.decode())
        print("Got PLC status")
        print(payload['Status'])
        
        if payload['Status']['CMD_REQ'] == 1:
          CMDRCV = '0'

        if payload['Status']['Finish_ACK'] == 1:
          FINISH = '0'

        
# Initialize the ROS node
rospy.init_node('service_client_node')

# Wait for the service to be available
print("b4 service")
rospy.wait_for_service('/get_a_waypoint')
print("service is up")

# Create a MoveBaseFlex client
mbf_client = actionlib.SimpleActionClient('/move_base_flex/move_base', MoveBaseAction)
rospy.Subscriber('/move_base_flex/move_base/result', MoveBaseActionResult, goal_status_callback)
rospy.Subscriber('/move_base_flex/exe_path/result', ExePathActionResult, path_status_callback)


# Publisher to the cancel topic
cancel_pub = rospy.Publisher('/move_base_flex/move_base/cancel', GoalID, queue_size=10)
stop_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
path_pub = rospy.Publisher('/path_to_follow', PoseArray, queue_size=10)
pub_init_pose = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=1)

rospy.Timer(rospy.Duration(5), check_mbf_goal_server_availability)
   
# Start a timer to check the result every 2 seconds (or desired interval)
#rospy.Timer(rospy.Duration(2), check_mbf_goal_result)

rospy.Timer(rospy.Duration(2), publish_status_message)

# Attach callbacks to client
client.on_connect = on_connect
client.on_message = on_message

# Connect to the broker
client.connect(BROKER_ADDRESS, BROKER_PORT, 60)



# Start the loop
client.loop_start()
client.publish(TOPIC_AMR_STATUS, '1,0,0,A,A')


def signal_handler(sig, frame):
    print("Disconnecting from broker...")
    client.loop_stop()
    client.disconnect()
    sys.exit(0)

# Set up signal handler for Ctrl-C
signal.signal(signal.SIGINT, signal_handler)

while True:
    time.sleep(1)


