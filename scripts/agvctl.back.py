#!/usr/bin/env python3



from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
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
from agv_interface.srv import waypointsarray, waypointsarrayResponse
from agv_interface.srv import awaypoint, awaypointResponse
from agv_interface.srv import waypointname, waypointnameResponse
from agv_interface.srv import deletemap, deletemapResponse
from agv_interface.srv import deletewaypoint, deletewaypointResponse


import json
import socket

#from tinydb import TinyDB, Query

import os

from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Int8MultiArray
import numpy as np
import cv2 as cv

import tf2_ros
import geometry_msgs.msg
import tf2_geometry_msgs
from rospy_message_converter import json_message_converter
import math
from visualization_msgs.msg import Marker
import roslaunch
import subprocess

#import Queue
import queue as Queue


q_map_name = Queue.Queue()
q_wayponts = Queue.Queue()
q_wayponts_names = Queue.Queue()

MAP_PATH = '/home/pi/amr_ws/src/linorobot/maps'

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
        self.int_marker.pose.position.z = 0.1
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
        print(interactive_marker.pose)
        print(msg.event_type)
            #self._goal_pub.publish(position)

        if (msg.event_type == InteractiveMarkerFeedback.BUTTON_CLICK):
            print("Click Click")
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
    print("Pose estimate")
    if mess.onezero==1:
        pose_marker.markerOn()
        return  poseestimateResponse('ON')
    else:
        pose_marker.markerOff()
        return  poseestimateResponse('OFF')


def set_post(mess):
    print("set_pose")
    print(pose_marker.getPose())
    pose = PoseWithCovariance(pose=pose_marker.getPose())
    pose2 = PoseWithCovarianceStamped(pose=pose)
    pose2.header.frame_id = 'map'
    pub_init_pose.publish(pose2)
    if mess.onezero==1:
        return  poseestimateResponse('ON')
    else:
        return  poseestimateResponse('OFF')

def get_maps(mess):
    print("get map")
    file_names = []

    for r, d, f in os.walk(MAP_PATH):
        for file in f:
            if ".yaml" in file:
                filename, file_extension = os.path.splitext(file)
                file_names.append(filename)
                print(file)

    if mess.onezero==1:
        return  mapsResponse(file_names)
    else:
        return  mapsResponse(file_names)

def save_maps(mess):
    print("save map")
    print(mess)
    q_map_name.put(mess.mapfile)

    # do stuff
    #process.stop()

    
    return  savemapsResponse("OK")
   


def get_pose(mess):
    json_str = json_message_converter.convert_ros_message_to_json(waypoint_marker.getPose())

    print(json_str)
    print(mess)
    
    #results = db.search(Todo.Category == 'Home')
    #waypoint_db.insert({'mapname': mess.mapname, 'name' : mess.name, 'id': mess.seq, 'waypoint': json_str})
    with open(os.path.join(MAP_PATH, mess.mapname + '.json')) as json_file:
        data = json.load(json_file)

    data[mess.name] = json_message_converter.convert_ros_message_to_json(waypoint_marker.getPose())

    with open(os.path.join(MAP_PATH, mess.mapname + '.json'), 'w') as fp:
            json.dump(data, fp)

    print(json_str)
    return getpostResponse(waypoint_marker.getPose())

def getwaypoints(mess):
    print(mess)
    
    pose_a1 = []
    name_a1 = []
   
    with open(os.path.join(MAP_PATH, mess.name + '.json')) as json_file:
        data = json.load(json_file)

    for key, val in data.items():
        pose_w = json_message_converter.convert_json_to_ros_message('geometry_msgs/Pose', val)
        pose_a1.append(pose_w)
        name_a1.append(key)
    
    print(pose_a1)
    print(name_a1)

    q_wayponts.put(pose_a1)
    q_wayponts_names.put(name_a1)
    return waypointsarrayResponse(pose_a1)

def getwaypointName(mess):
    print(mess)
    pose_a1 = []
    with open(os.path.join(MAP_PATH, mess.name + '.json')) as json_file:
        data = json.load(json_file)

    for key, val in data.items():
        pose_a1.append(key)
   

        #print pose_w
    #print rel


    return waypointsarrayResponse(pose_a1)

def getWaypointname(mess):
    print(mess)
    pose_a1 = []
   
    with open(os.path.join(MAP_PATH, mess.mapname + '.json')) as json_file:
        data = json.load(json_file)

    for key, val in data.items():
        pose_a1.append(key)
     


    return waypointnameResponse(pose_a1)


def getwayAwaypoint(mess):
    print(mess)


    with open(os.path.join(MAP_PATH, mess.mapname + '.json')) as json_file:
        data = json.load(json_file)


    pose1 = json_message_converter.convert_json_to_ros_message('geometry_msgs/Pose', data[mess.name])

    return awaypointResponse(pose1)



def deleteMap(mess):
    print(mess)
    f_path_m = MAP_PATH +  '/' + mess.mapfile+'.pgm'
    f_path_y = MAP_PATH +  '/' + mess.mapfile+'.yaml'
    f_path_j = MAP_PATH +  '/' + mess.mapfile+'.json'
    print(f_path_m)
    print(f_path_y)

    if os.path.exists(f_path_m):
        os.remove(f_path_m)
    else:
        print("The file does not exist")

    if os.path.exists(f_path_y):
        os.remove(f_path_y)
    else:
        print("The file does not exist")
    
    if os.path.exists(f_path_j):
        os.remove(f_path_j)
    else:
        print("The file does not exist")

    return 'OK'

def deleteWaypoint(mess):
    print(mess)


    with open(os.path.join(MAP_PATH, mess.mapfile + '.json')) as json_file:
        data = json.load(json_file)
    del data[mess.waypoint]

    with open(os.path.join(MAP_PATH, mess.mapfile + '.json'), 'w') as fp:
        json.dump(data, fp)
    return 'OK'

def turn_slam_on_off(mess):
    if mess.onezero == 1:
        if turn_slam_on_off.is_slam_running == False:
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            turn_slam_on_off.launch_slam = roslaunch.parent.ROSLaunchParent(uuid, ["/home/pi/amr_ws/src/linorobot/launch/slam.launch"])
            turn_slam_on_off.launch_slam.start()
            turn_slam_on_off.is_slam_running = True
            print("start")
            return  slamsrvResponse('ON')
    else:
        if turn_slam_on_off.is_slam_running == True:
            turn_slam_on_off.launch_slam.shutdown()
            turn_slam_on_off.is_slam_running = False
            print("stop")
            return  slamsrvResponse('OFF')

def turn_nav_on_off(mess):
    turn_nav_on_off.img = DrawMapImg()
    turn_nav_on_off.blank_map_pub = rospy.Publisher('map', OccupancyGrid, queue_size = 1000)
    turn_nav_on_off.blankmap_msg = MakeMapMsg(turn_nav_on_off.img, 1, [400, 400, 0], [1, 0, 0, 0])
    if mess.onezero==1:
        item = {'sw': 1, 'mapname': mess.map_file}
        if turn_nav_on_off.is_nav_running == False:
            turn_nav_on_off.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(turn_nav_on_off.uuid)
            turn_nav_on_off.hostname = socket.gethostname()
            turn_nav_on_off.cli_args = ["/home/pi/amr_ws/src/linorobot/launch/navigate.launch", "map_file:=/home/pi/amr_ws/src/linorobot/maps/" + item["mapname"] +".yaml"]
            turn_nav_on_off.roslaunch_args = turn_nav_on_off.cli_args[1:]
            turn_nav_on_off.roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(turn_nav_on_off.cli_args)[0], turn_nav_on_off.roslaunch_args)]
            print("Start nav")
            print(turn_nav_on_off.roslaunch_file)
            turn_nav_on_off.launch_nav = roslaunch.parent.ROSLaunchParent(turn_nav_on_off.uuid, turn_nav_on_off.roslaunch_file)  #map_file
            turn_nav_on_off.launch_nav.start()
            print(turn_nav_on_off.hostname)                    
                    
  

            turn_nav_on_off.is_nav_running = True
            print("start")
            #launch_slam.start()
            print(mess.map_file)
            return  navigatesrvResponse('ON')
    else:
        item = {'sw': 0, 'mapname': mess.map_file}
        if turn_nav_on_off.is_nav_running == True:
            turn_nav_on_off.launch_nav.shutdown()
            turn_nav_on_off.is_nav_running = False
            turn_nav_on_off.blank_map_pub.publish(turn_nav_on_off.blankmap_msg)
                    
            print("stop")
        
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
    pass
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def MakeMapMsg(img, resolution, position, orientation):
    map_msg = OccupancyGrid()
    map_msg.header.stamp = rospy.Time.now()
    map_msg.info.map_load_time = rospy.Time.now()
    map_msg.info.width = img.shape[0]
    map_msg.info.height = img.shape[1]
    map_msg.info.resolution = resolution
    map_msg.info.origin.position.x = position[0]
    map_msg.info.origin.position.y = position[1]
    map_msg.info.origin.position.z = position[2]
    map_msg.info.origin.orientation.w = orientation[0]
    map_msg.info.origin.orientation.x = orientation[1]
    map_msg.info.origin.orientation.y = orientation[2]
    map_msg.info.origin.orientation.z = orientation[3]
    img = cv.flip(img, 0, dst=None)
    img_data_int8array = [i for row in img.tolist() for i in row]
    map_msg.data = Int8MultiArray(data=img_data_int8array).data
    return map_msg

def DrawMapImg():
    img = np.zeros((800, 800, 1), np.uint8)
    #cv.rectangle(img, (200,100), (400,150), 100, -1)
    #cv.rectangle(img, (350,100), (400,400), 100, -1)
    return img

def main():
    rospy.init_node('simple_marker')
    wait_for_time()
    global pub_init_pose
    pub_init_pose = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=1)

    pubpst = rospy.Publisher('robot_pose', PoseStamped, queue_size=10)
    pubp = rospy.Publisher('robot_pose_p', Pose, queue_size=10)

    pub = rospy.Publisher('odometry_goal', Point, queue_size=5)
    hostname = socket.gethostname()
    marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=5)
    marker_publisher_host = rospy.Publisher(hostname + '/visualization_marker', Marker, queue_size=5)
    waypoints_publisher = rospy.Publisher('visualization_marker_array', MarkerArray,  queue_size=5)
    waypoints_publisher_text = rospy.Publisher('visualization_marker_array_text', MarkerArray,  queue_size=5)
    
    rospy.sleep(0.5)
    server = InteractiveMarkerServer('simple_marker')
    marker1 = DestinationMarker(server, 0, 0, 'dest1', pub)
    global pose_marker
    pose_marker = DestinationMarker(server, 1, 0, 'pose estimate', pub)
    global waypoint_marker
    waypoint_marker = DestinationMarker(server, 2, -1, 'waypoint', pub)
    marker1.start()
    marker1.markerOff()
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
    get_waypoint=rospy.Service('get_waypoint', waypointsarray,  getwaypoints ) 
    get_a_waypoint=rospy.Service('get_a_waypoint', awaypoint,  getwayAwaypoint ) 
    get_waypoint_name=rospy.Service('get_waypoint_name', waypointname,  getWaypointname ) 

    delete_map=rospy.Service('delete_map', deletemap,  deleteMap) 
    delete_waypoint=rospy.Service('delete_waypoint', deletewaypoint,  deleteWaypoint) 


    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
   
    rate = rospy.Rate(10.0)
    turn_slam_on_off.is_slam_running = False
    turn_nav_on_off.is_nav_running =False
    is_nav_running = False
    wpts = []
    p_wpts = []
    markerArray2 = MarkerArray()
    markerArray3 = MarkerArray()
    p_waypoint_length = 0
    print("Start loop")  
    while not rospy.is_shutdown():
      
        if not q_map_name.empty():
            mapname = q_map_name.get()
            bin_cmd = 'rosrun map_server map_saver -f /home/pi/amr_ws/src/linorobot/maps/' +  mapname
            print(bin_cmd)

            os.system(bin_cmd)

            data = {}

            with open(os.path.join(MAP_PATH, mapname + '.json'), 'w') as fp:
                json.dump(data, fp)


            #subprocess.run(["rosrun", bin_cmd])
        
        if not q_wayponts.empty():
            wpts = q_wayponts.get()
            wp_names = q_wayponts_names.get()
            print("========>")
            print(len(wpts))
            #print wpts
            mk_length = len(markerArray2.markers)
            id = 0
            id_t = len(markerArray2.markers) + 1
            i = 0
            del markerArray2.markers[:]
            del markerArray3.markers[:]
            for x in range(0, mk_length):
                #marker = 
                markerArray2.markers.append(Marker(
                    type=Marker.ARROW,
                    id=id,
                    action=2,
                    scale=Vector3(0.2, 0.2, 0.2),
                    header=Header(frame_id='map'),
                    color=ColorRGBA(1.0, 1.0, 0.0, 1.0),
                    text="AGV"))


                markerArray3.markers.append(Marker(
                    type=Marker.TEXT_VIEW_FACING,
                    id=id_t,
                    action=2,
                    scale=Vector3(0.3, 0.3, 0.3),
                    header=Header(frame_id='map'),
                    color=ColorRGBA(0.0, 0.1, 1.0, 1.0),
                    text="null"))
                
                i = i + 1

                    
                id += 1
                id_t += 1


            waypoints_publisher.publish(markerArray2)
            waypoints_publisher_text.publish(markerArray3)
            print("clear========>")

            del markerArray2.markers[:]
            del markerArray3.markers[:]
            #markerArray2.markers = []
            #markerArray3.markers = []
            id = 0
            id_t = len(wpts) + 1
            i = 0
            for x in wpts:
                #marker = 
                markerArray2.markers.append(Marker(
                    type=Marker.ARROW,
                    id=id,
                    pose= x,
                    lifetime = rospy.Duration.from_sec(1),
                    scale=Vector3(0.2, 0.2, 0.2),
                    header=Header(frame_id='map'),
                    color=ColorRGBA(1.0, 1.0, 0.0, 1.0),
                    text="AGV"))
           
        
                
                ik_pose = Pose()
                ik_pose.position.x = x.position.x
                ik_pose.position.y = x.position.y
                ik_pose.position.z = x.position.z + 0.1
                ik_pose.orientation.x = x.orientation.x
                ik_pose.orientation.y = x.orientation.y
                ik_pose.orientation.z = x.orientation.z
                ik_pose.orientation.w = x.orientation.w
                markerArray3.markers.append(Marker(
                    type=Marker.TEXT_VIEW_FACING,
                    id=id_t,
                    pose=ik_pose,
                    scale=Vector3(0.3, 0.3, 0.3),
                    header=Header(frame_id='map'),
                    color=ColorRGBA(0.0, 0.1, 1.0, 1.0),
                    text=wp_names[i]))
                
                i = i + 1

                    
                id += 1
                id_t += 1

            p_waypoint_length = len(wpts)
            p_wpts = wpts
            
                
            
            #print markerArray3

            waypoints_publisher.publish(markerArray2)
            waypoints_publisher_text.publish(markerArray3)


         

        
        try:
            point_odom = tfBuffer.lookup_transform('map', 'base_footprint', rospy.Time(0))
            hhname = socket.gethostname()
            marker = Marker(
                type=Marker.ARROW,
                id=0,
                lifetime=rospy.Duration(1.5),
                pose= Pose(point_odom.transform.translation, point_odom.transform.rotation),
                scale=Vector3(0.2, 0.2, 0.2),
                header=Header(frame_id='map'),
                color=ColorRGBA(0.0, 1.0, 0.0, 1.0),
                text=hhname)
            marker_publisher.publish(marker)
            marker_publisher_host.publish(marker)


            rpose = PoseStamped()
            rpose.header.frame_id = "/map"
            rpose.header.stamp = rospy.Time.now()

            rpose.pose = Pose(point_odom.transform.translation, point_odom.transform.rotation)
            pubpst.publish(rpose)

            rppose = rpose.pose
            pubp.publish(rppose)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            #continue
        #print("loop")    
        rate.sleep()
    #rospy.sleep(0.1)

    #rospy.spin()


if __name__ == '__main__':
    main()
