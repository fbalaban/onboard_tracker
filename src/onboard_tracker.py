#!/usr/bin/env python
import rospy
import tracker.calc
import tracker.const

from mavros_msgs.msg import State
from mavros_msgs.msg import Waypoint
from mavros_msgs.msg import WaypointList
from mavros_msgs.srv import WaypointClear
from mavros_msgs.srv import WaypointPull
from mavros_msgs.srv import WaypointPush
from mavros_msgs.srv import WaypointPushRequest
from mavros_msgs.srv import WaypointSetCurrent

from sensor_msgs.msg import NavSatFix

global the_home
global waypoint_iter
global pr_closest_point
global mission_started
global once
global integration_started
global integrated_area

integration_started = False

the_home = Waypoint()
the_home.frame = 0
the_home.command = 22
the_home.is_current = 0
the_home.autocontinue = 1
the_home.param1 = 25
the_home.param2 = 0
the_home.param3 = 0
the_home.param4 = 0
the_home.x_lat = 0
the_home.y_long = 0
the_home.z_alt = 0

waypoint_iter = 0
pr_closest_point = tracker.calc.waypoint(0,0)
mission_started = False
integrated_area = 0
once = True

def position_callback(data):

    global state
    global mission_started
    global wp_plan
    global waypoint_iter
    global integrated_area
    global integration_started
    global vasi_mikri
    global pr_closest_point

    global the_home
    global has_wp_plan
    global closest_point

    if state == "AUTO" and has_wp_plan == True:
        mission_started = True
        integrated_area = 0
        actual_position = tracker.calc.waypoint(data.latitude, data.longitude)
        previous_wp = wp_plan[waypoint_iter]
        next_wp = wp_plan[waypoint_iter + 1]

        closest_point = tracker.calc.get_closest_point_on_segment(previous_wp.latitude, previous_wp.longitude, next_wp.latitude, next_wp.longitude, data.latitude, data.longitude)

        if waypoint_iter > 0:
            if integration_started == False:
                rospy.loginfo("========= INTEGRATION STARTED =========")
                integration_started = True
                vasi_mikri = tracker.calc.get_distance(data.latitude, data.longitude, closest_point.latitude, closest_point.longitude)
                pr_closest_point = tracker.calc.waypoint(latitude=closest_point.latitude, longitude=closest_point.longitude)
            else:
                vasi_megali  = tracker.calc.get_distance(data.latitude, data.longitude, closest_point.latitude, closest_point.longitude)
                integrated_area +=  ((vasi_mikri + vasi_megali) * tracker.calc.get_distance(pr_closest_point.latitude, pr_closest_point.longitude,closest_point.latitude, closest_point.longitude)) / 2
                vasi_mikri = vasi_megali
                pr_closest_point = closest_point

        bearing = tracker.calc.find_bearing(previous_wp, next_wp)

        # the next is the second way of calculating the next waypoint, by including the distance from the actual position of the UAV
        #remaining_distance = tracker.const.DISTANCE - get_distance(data.latitude, data.longitude, closest_point.latitude, closest_point.longitude)
        #c_wp = find_command_waypoint(remaining_distance, bearing, closest_point)

        c_wp = tracker.calc.find_command_waypoint(tracker.const.DISTANCE, bearing, closest_point)

        if tracker.calc.get_distance(data.latitude,data.longitude,next_wp.latitude,next_wp.longitude) < 0.03: # meters
            waypoint_iter += 1
            rospy.loginfo("[Tracker] Waypoint id: %s", str(waypoint_iter))
            if waypoint_iter == (len(wp_plan) - 1):
                # introduce angle if needed from previous file
                rospy.loginfo("[Tracker] For distance of: %s km the integrated area is: %s sq.kilometers ", str(tracker.const.DISTANCE), str(integrated_area))
                state = "RTL"
                has_wp_plan = False

        # CONSTRUCTING THE WP. TODO exchanging only the lat,long

        the_wp = Waypoint()
        the_wp.frame = 3
        the_wp.command = 16
        the_wp.is_current = 1
        the_wp.autocontinue = 1
        the_wp.param1 = 0
        the_wp.param2 = 10
        the_wp.param3 = 0
        the_wp.param4 = 0
        #the_wp.x_lat = 37.5200634
        #the_wp.y_long = -5.8591536
#the_req.waypoints.insert(1,the_wp)
#37.5200634
#-5.8591536
        the_wp.x_lat = c_wp.latitude
        the_wp.y_long = c_wp.longitude
        the_wp.z_alt = tracker.const.ALTITUDE
        #the_wp.z_alt = 41.6
        the_req = WaypointPushRequest()
        #the_req.waypoints.insert(0,the_home)
        #the_req.waypoints.insert(1,the_wp)
        the_req.waypoints.insert(0,the_wp)

        # SENDING THE LIST
        rospy.loginfo("[Tracker] Intermediate waypoint, latitude: %s, longitude: %s ", str(c_wp.latitude), str(c_wp.longitude))
        # TODO maybe add the "the_req.start_index = 1" instead of the full wp list (with home) which will replace only the 1st wp instead of the whole

        # list http://docs.ros.org/api/mavros_msgs/html/srv/WaypointPush.html. This would probably mean that the current service wouldn't be needed
        command_service(the_req)
        #current_service(0)

def update_mission_callback(data):

    global state
    global has_wp_plan
    global the_home
    global wp_plan    
    global mission_started
    
    if mission_started == False:
        mavros_wp_list = data.waypoints
        wp_plan = []
        count = 0
        
        if len(mavros_wp_list) > 2:
            for each_wp in mavros_wp_list:
                wp_plan.append(tracker.calc.waypoint(latitude=each_wp.x_lat, longitude=each_wp.y_long))
            rospy.loginfo("[Tracker] New WP Plan received :")
            for norm_wp in wp_plan:
                rospy.loginfo("[Tracker] %s: Latitude: %s, Longitude: %s", str(count), str(norm_wp.latitude), str(norm_wp.longitude))
                count = count + 1
            rospy.loginfo("[Tracker] Total waypoints: %s", str(count))
            if len(data.waypoints) == len(wp_plan):
                rospy.loginfo("[Tracker] Got the WP plan")
                has_wp_plan = True
                #The semi-constant we were talking about..
                the_home.x_lat = wp_plan[0].latitude
                the_home.y_long = wp_plan[0].longitude

def state_callback(data):

    global initial_run
    global state
    global has_wp_plan
    global mission_started
    global waypoint_iter
    global once

    if data.mode == "AUTO" or data.mode == "AUTO.MISSION":
        if initial_run == True:
            waypoint_pull_service()
            has_wp_plan = False
            initial_run = False
            state = "AUTO"
            rospy.loginfo("[Tracker - STATE] AUTO - MISSION")
        initial_run = False
    if data.mode == "STABILIZED" or data.mode == "STABILIZE":
        state = "STABILIZED"
        waypoint_iter = 0
        mission_started = False
        initial_run = True
        has_wp_plan = False
        #clearing_service()
        rospy.loginfo("[Tracker - STATE] STABILIZED")
    if data.mode == "AUTO.RTL" or data.mode == "RTL":
        state = "RTL"
        waypoint_iter = 0
        mission_started = False
        initial_run = True
        has_wp_plan = False
        clearing_service()
        rospy.loginfo("[Tracker - STATE] RTL - clearing mission")
    if data.mode == "MANUAL":
        state = "MANUAL"
        rospy.loginfo("[Tracker - STATE] MANUAL")
        rospy.signal_shutdown("[Tracker - INFO] Manual mode, gathering ROS bags and shutting down")
        
if __name__ == '__main__':

    # Sleeping waiting for mavros to start
    rospy.sleep(10)

    global initial_run
    global state
    global has_wp_plan

    initial_run = True
    state = "STABILIZED"
    has_wp_plan = False

    rospy.init_node('onboard_tracker', anonymous=False)

    rospy.loginfo("[Tracker] Just started")

    waypoint_pull_service = rospy.ServiceProxy('mavros/mission/pull', WaypointPull)
    current_service = rospy.ServiceProxy('mavros/mission/set_current', WaypointSetCurrent)
    command_service = rospy.ServiceProxy('mavros/mission/push', WaypointPush)
    clearing_service = rospy.ServiceProxy('mavros/mission/clear', WaypointClear)

    # Waiting for mode to change to AUTO
    rospy.Subscriber("mavros/state", State, state_callback)
    # Get the mission
    rospy.Subscriber("mavros/mission/waypoints", WaypointList, update_mission_callback)
    # Update vehicle position
    rospy.Subscriber("mavros/global_position/global", NavSatFix, position_callback)
    # clearing_service()

    rospy.spin()
