#!/usr/bin/env python3
import json
import rospy
#from <package_name>.srv import <ServiceName>  # Import the service type
from agv_interface.srv import awaypoint, awaypointResponse


import paho.mqtt.client as mqtt
import actionlib

from mbf_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped

# Local broker settings
#BROKER_ADDRESS = "localhost"  # Since the broker is running locally
BROKER_ADDRESS = "203.150.243.242"
BROKER_PORT = 1883

# Define topic and message
TOPIC_GOAL = "PLC_AMR/GWP/"
TOPIC_A = "home/a"
TOPIC_AMR_STATUS = "AMR_PLC/Status/"
TOPIC_PLC_STATUS = "PLC_AMR/Status/"

# Create MQTT client instance
#client = mqtt.Client("LocalPythonClient")
client = mqtt.Client(protocol=mqtt.MQTTv311)

is_mbf_goal_server_available = False 
status_running = True  # Control variable to toggle loop
def publish_status_message(event):
    global status_running
    if status_running == True:
        status_running = False
        client.publish(TOPIC_AMR_STATUS, '1,0,0,A,A')
    else:
        status_running = True 
        client.publish(TOPIC_AMR_STATUS, '1,0,0,A,A')



def check_mbf_goal_result(event):
    """Timer callback function to check if the action result is available."""
    if mbf_client.get_state() in [actionlib.GoalStatus.SUCCEEDED, actionlib.GoalStatus.ABORTED,
                              actionlib.GoalStatus.REJECTED, actionlib.GoalStatus.PREEMPTED]:
        rospy.loginfo("Action completed with state: %d", mbf_client.get_state())
        result = mbf_client.get_result()
        client.publish(TOPIC_AMR_STATUS, '1,1,1,A,A')
        rospy.loginfo("Action result: %s", result)
        


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
    print("Connected to local broker with result code " + str(rc))
    if rc == 0:
        print("Connected successfully")
        # Subscribe to each topic separately
        client.subscribe(TOPIC_GOAL)
        client.subscribe(TOPIC_A, qos=1)
        client.subscribe(TOPIC_PLC_STATUS)
    else:
        print(f"Connection failed with code {rc}")


def on_message(client, userdata, msg):
    #print(f"Received message: {msg.payload.decode()} on topic {msg.topic}")
    if msg.topic == TOPIC_GOAL:
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
    elif msg.topic == TOPIC_A:
        print("TOPIC A accepted")
    elif msg.topic == TOPIC_PLC_STATUS:
        pass
        #print("Got PLC status")
        #print(msg.payload.decode())

# Initialize the ROS node
rospy.init_node('service_client_node')

# Wait for the service to be available
print("b4 service")
rospy.wait_for_service('/get_a_waypoint')
print("service is up")

# Create a MoveBaseFlex client
mbf_client = actionlib.SimpleActionClient('/move_base_flex/move_base', MoveBaseAction)
rospy.Timer(rospy.Duration(5), check_mbf_goal_server_availability)
   
# Start a timer to check the result every 2 seconds (or desired interval)
rospy.Timer(rospy.Duration(2), check_mbf_goal_result)

#rospy.Timer(rospy.Duration(2), publish_status_message)

# Attach callbacks to client
client.on_connect = on_connect
client.on_message = on_message

# Connect to the broker
client.connect(BROKER_ADDRESS, BROKER_PORT, 60)



# Start the loop
client.loop_start()
client.publish(TOPIC_AMR_STATUS, '1,0,0,A,A')

try:
    while True:
        pass
except KeyboardInterrupt:
    print("Disconnecting from broker...")
finally:
    client.loop_stop()
    client.disconnect()
