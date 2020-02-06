#!/usr/bin/env python

from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3, PoseStamped, PoseWithCovarianceStamped, PoseWithCovariance
from std_msgs.msg import Header, ColorRGBA
import rospy
from agv_interface.srv import waypoint, waypointResponse
from agv_interface.srv import poseestimate, poseestimateResponse
from agv_interface.srv import navigatesrv, navigatesrvResponse
from agv_interface.srv import slamsrv, slamsrvResponse
from agv_interface.srv import maps, mapsResponse
from agv_interface.srv import getpost, getpostResponse
from agv_interface.srv import savemaps, savemapsResponse


import json
from tinydb import TinyDB, Query

import os
import tf2_ros
import geometry_msgs.msg
import tf2_geometry_msgs
from rospy_message_converter import json_message_converter
import math
from visualization_msgs.msg import Marker
import roslaunch
import subprocess

import Queue

q_slam = Queue.Queue()
q_nav = Queue.Queue()
q_map_name = Queue.Queue()

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


class DestinationMarker(object):
    def __init__(self, server, x, y, name, goal_pub):
        self._server = server
        self._x = x
        self._y = y
        self._name = name
        self._goal_pub = goal_pub
        self.int_marker = InteractiveMarker()
        self.pose = Pose()

    def getPose(self):
        return self.pose
    
    def markerOn(self):
        self._server.insert(self.int_marker, self._callback)
        self._server.applyChanges()

    def markerOff(self):
        self._server.erase(self._name)
        self._server.applyChanges()

    def setPose(self, x, y):
        #print(x)
        #print(y)
        #self.int_marker.pose.position.x = x
        #self.int_marker.pose.position.y = y
        self.int_marker.pose = Pose(Point(x, y, 0.01), Quaternion(0, 0, 0, 1))
        #print(self.int_marker.pose)
        self._server.applyChanges()


    def start(self):
        # create an interactive marker for our server
        #int_marker = InteractiveMarker()
        self.int_marker.header.frame_id = "map"
        self.int_marker.name = self._name
        self.int_marker.pose.position.x = self._x
        self.int_marker.pose.position.y = self._y
        self.int_marker.pose.position.z = 0.01
        self.int_marker.pose.orientation.w = 1
        self.int_marker.description = self._name

        arrow_marker = Marker()
        arrow_marker.type = Marker.ARROW
        arrow_marker.pose.orientation.w = 1
        arrow_marker.pose.position.z = 0.05
        arrow_marker.scale.x = 0.2
        arrow_marker.scale.y = 0.2
        arrow_marker.scale.z = 0.2
        arrow_marker.color.r = 1.0
        arrow_marker.color.g = 0.2
        arrow_marker.color.b = 0.2
        arrow_marker.color.a = 1.0
   

        text_marker = Marker()
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.pose.orientation.w = 1
        text_marker.pose.position.z = 0.05
        text_marker.scale.x = 0.2
        text_marker.scale.y = 0.2
        text_marker.scale.z = 0.2
        text_marker.color.r = 0.1
        text_marker.color.g = 0.9
        text_marker.color.b = 0.0
        text_marker.color.a = 1
        text_marker.text = self._name

        arrow_control = InteractiveMarkerControl()
        arrow_control.orientation.w = 1
        arrow_control.orientation.y = 1
        arrow_control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        arrow_control.markers.append(arrow_marker)
        arrow_control.markers.append(text_marker)
        arrow_control.always_visible = True
        self.int_marker.controls.append(arrow_control)


        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.y = 1 
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        self.int_marker.controls.append(control)

        self._server.insert(self.int_marker, self._callback)
        self._server.applyChanges()


    def _callback(self, msg):
        interactive_marker = self._server.get(msg.marker_name)
        #print msg.marker_name
        
        #if msg.marker_name == "pose estimate":
        self.pose = interactive_marker.pose
        print interactive_marker.pose
        print msg.event_type
            #self._goal_pub.publish(position)

        if (msg.event_type == InteractiveMarkerFeedback.BUTTON_CLICK):
            print "Click Click"
            interactive_marker = self._server.get(msg.marker_name)
            position = interactive_marker.pose.position
            rospy.loginfo('User clicked {} at {}, {}, {}'.format(msg.marker_name, position.x, position.y, position.z))
            self._goal_pub.publish(position)

def turn_waypoint_on_off(mess):
 if mess.onezero==1:
    waypoint_marker.markerOn()
    return  waypointResponse('ON')
 else:
    waypoint_marker.markerOff()
    return  waypointResponse('OFF')
    

def turn_poseestimate_on_off(mess):
 if mess.onezero==1:
    pose_marker.markerOn()
    return  poseestimateResponse('ON')
 else:
    pose_marker.markerOff()
    return  poseestimateResponse('OFF')


def set_post(mess):
    print "set_pose"
    print  pose_marker.getPose()
    pose = PoseWithCovariance(pose=pose_marker.getPose())
    pose2 = PoseWithCovarianceStamped(pose=pose)
    pose2.header.frame_id = 'map'
    pub_init_pose.publish(pose2)
    if mess.onezero==1:
        return  poseestimateResponse('ON')
    else:
        return  poseestimateResponse('OFF')

def get_maps(mess):
    print "get map"
    file_names = []

    for r, d, f in os.walk('/home/pi/linorobot_ws/src/linorobot/maps'):
        for file in f:
            if ".yaml" in file:
                filename, file_extension = os.path.splitext(file)
                file_names.append(filename)
                print file

    if mess.onezero==1:
        return  mapsResponse(file_names)
    else:
        return  mapsResponse(file_names)

def save_maps(mess):
    print "save map"
    print mess
    q_map_name.put(mess.mapfile)

    # do stuff
    #process.stop()

    
    return  savemapsResponse("OK")
   


def get_pose(mess):
    json_str = json_message_converter.convert_ros_message_to_json(waypoint_marker.getPose())
    
    #results = db.search(Todo.Category == 'Home')
    waypoint_db.insert({'name': 'map', 'id': 1, 'waypoint': json_str})

        

    print json_str
    return getpostResponse(waypoint_marker.getPose())



def turn_slam_on_off(mess):
 if mess.onezero==1:
    q_slam.put(1)
    #launch_slam.start()
    print mess.map_file
    return  slamsrvResponse('ON')
 else:
    #launch_slam.shutdown()
    q_slam.put(0)
    return  slamsrvResponse('OFF')

def turn_nav_on_off(mess):
 if mess.onezero==1:
    item = {'sw': 1, 'mapname': mess.map_file}
    q_nav.put(item)
    #launch_slam.start()
    print mess.map_file
    return  navigatesrvResponse('ON')
 else:
    #launch_slam.shutdown()
    #q_nav.put(0)
    item = {'sw': 0, 'mapname': mess.map_file}
    q_nav.put(item)
    return  navigatesrvResponse('OFF')

def __pub_initial_position(self, x, y, theta):
    """
    Publishing new initial position (x, y, theta) --> for localization
    :param x x-position of the robot
    :param y y-position of the robot
    :param theta theta-position of the robot
    """
    initpose = PoseWithCovarianceStamped()
    initpose.header.stamp = rospy.get_rostime()
    initpose.header.frame_id = "map"
    initpose.pose.pose.position.x = x
    initpose.pose.pose.position.y = y
    quaternion = self.__yaw_to_quat(theta)

    initpose.pose.pose.orientation.w = quaternion[0]
    initpose.pose.pose.orientation.x = quaternion[1]
    initpose.pose.pose.orientation.y = quaternion[2]
    initpose.pose.pose.orientation.z = quaternion[3]
    self.__initialpose_pub.publish(initpose)
    return 

def robot_marker_callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def main():
    rospy.init_node('simple_marker')
    wait_for_time()
    global pub_init_pose
    pub_init_pose = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=1)

    pub = rospy.Publisher('odometry_goal', Point, queue_size=5)
    marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=5)
    rospy.sleep(0.5)
    server = InteractiveMarkerServer('simple_marker')
    marker1 = DestinationMarker(server, 0, 0, 'dest1', pub)
    global pose_marker
    pose_marker = DestinationMarker(server, 1, 0, 'pose estimate', pub)
    global waypoint_marker
    waypoint_marker = DestinationMarker(server, 2, -1, 'waypoint', pub)
    marker1.start()
    pose_marker.start()
    waypoint_marker.start()

    global waypoint_list
    #marker1.markerOff()
    waypoint_marker.markerOff()
    pose_marker.markerOff()

    rospy.Subscriber("robot_pose", PoseStamped, robot_marker_callback)

    service_waypoint=rospy.Service('waypoint_markers_service',waypoint,turn_waypoint_on_off)
    service_pose_estimate=rospy.Service('poseestimate_markers_service',poseestimate,turn_poseestimate_on_off)
    service_start_slam=rospy.Service('start_slam', slamsrv, turn_slam_on_off)
    service_start_nav=rospy.Service('start_navigation', navigatesrv, turn_nav_on_off)
    set_pose=rospy.Service('set_pose', poseestimate, set_post)
    get_map=rospy.Service('get_map', maps,  get_maps ) 
    getpose=rospy.Service('get_pose', getpost,  get_pose) 
    get_map=rospy.Service('save_map', savemaps,  save_maps ) 


    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    global waypoint_db
    waypoint_db = TinyDB('/home/pi/db.json')
    
   
    rate = rospy.Rate(10.0)
    is_slam_running = False
    is_nav_running = False
    while not rospy.is_shutdown():
        if not q_slam.empty():
            status =  q_slam.get()
            if status == 1:
                if is_slam_running == False:
                    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
                    roslaunch.configure_logging(uuid)
                    launch_slam = roslaunch.parent.ROSLaunchParent(uuid, ["/home/pi/linorobot_ws/src/linorobot/launch/slam.launch"])
                    launch_slam.start()
                    is_slam_running = True
                    print("start")
            else:
                if is_slam_running == True:
                    launch_slam.shutdown()
                    is_slam_running = False
                    print("stop")

        if not q_nav.empty():
            nav_status =  q_nav.get()
            if nav_status["sw"] == 1:
                if is_slam_running == False:
                    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
                    roslaunch.configure_logging(uuid)
                    cli_args = ["/home/pi/linorobot_ws/src/linorobot/launch/navigate.launch", "map_file:=/home/pi/linorobot_ws/src/linorobot/maps/" + nav_status["mapname"] +".yaml"]
                    roslaunch_args = cli_args[1:]
                    roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
                    launch_nav = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)  #map_file
                    launch_nav.start()
                    is_nav_running = True
                    print("start")
            else:
                if is_nav_running == True:
                    launch_nav.shutdown()
                    is_nav_running = False
                    print("stop")
        if not q_map_name.empty():
            bin_cmd = 'rosrun map_server map_saver -f /home/pi/linorobot_ws/src/linorobot/maps/' +  q_map_name.get()
            print bin_cmd

            os.system(bin_cmd)

            #subprocess.run(["rosrun", bin_cmd])

        
        try:
            point_odom = tfBuffer.lookup_transform('map', 'base_footprint', rospy.Time(0))
            marker = Marker(
                type=Marker.ARROW,
                id=0,
                lifetime=rospy.Duration(1.5),
                pose= Pose(point_odom.transform.translation, point_odom.transform.rotation),
                scale=Vector3(0.2, 0.2, 0.2),
                header=Header(frame_id='map'),
                color=ColorRGBA(0.0, 1.0, 0.0, 1.0),
                text="Hello world")
            marker_publisher.publish(marker)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            #rate.sleep()
            continue
        rate.sleep()
 

    #rospy.spin()


if __name__ == '__main__':
    main()