#!/usr/bin/env python

# External imports
import rospy
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import WaypointList, WaypointClear, WaypointPull, WaypointPush, WaypointPushRequest, State, Waypoint

# Internal imports
import calc
from calc import waypoint as calculation_waypoint
from src.tracker import ALTITUDE, DISTANCE


def construct_waypoint(c_wp):
    """
    Construct and return a proper mavros Waypoint structure
    """
    the_wp = Waypoint()
    the_wp.frame = 3
    the_wp.command = 16
    the_wp.is_current = 1
    the_wp.autocontinue = 1
    the_wp.param1 = 0
    the_wp.param2 = 10
    the_wp.param3 = 0
    the_wp.param4 = 0
    the_wp.x_lat = c_wp.latitude
    the_wp.y_long = c_wp.longitude
    the_wp.z_alt = ALTITUDE
    return the_wp


class UAV:
    def __init__(self):
        self.home_waypoint = Waypoint()
        self.home_waypoint.frame = 0
        self.home_waypoint.command = 22
        self.home_waypoint.is_current = 0
        self.home_waypoint.autocontinue = 1
        self.home_waypoint.param1 = 25
        self.home_waypoint.param2 = 0
        self.home_waypoint.param3 = 0
        self.home_waypoint.param4 = 0
        self.home_waypoint.x_lat = 0
        self.home_waypoint.y_long = 0
        self.home_waypoint.z_alt = 0

        self.home = calculation_waypoint()
        self.wp_plan = []

        self.initial_run = True
        self.state = "STABILIZED"
        self.has_wp_plan = False
        self.mission_started = False
        self.waypoint_iter = 0
        self.integration = False

        self.integrated_area = 0
        self.vasi_mikri = 0
        self.pr_closest_point = calculation_waypoint(0, 0)

    def state_callback(self, data):
        """
        ROS Service Callback which is called when UAV state changes. Resets class and mission variables
        """
        if data.mode == "AUTO" or data.mode == "AUTO.MISSION":
            if self.initial_run:
                self.state = "AUTO"
                self.has_wp_plan = False
                s = rospy.ServiceProxy('mavros/mission/pull', WaypointPull)
                s()
                rospy.loginfo("[Tracker - STATE] AUTO - MISSION")
            self.initial_run = False
        if data.mode == "STABILIZED" or data.mode == "STABILIZE":
            self.state = "STABILIZED"
            self.has_wp_plan = False
            self.waypoint_iter = 0
            self.mission_started = False
            self.initial_run = True
            rospy.loginfo("[Tracker - STATE] STABILIZED")
        if data.mode == "AUTO.RTL" or data.mode == "RTL":
            self.state = "RTL"
            self.has_wp_plan = False
            self.waypoint_iter = 0
            self.mission_started = False
            self.initial_run = True
            s = rospy.ServiceProxy('mavros/mission/clear', WaypointClear)
            s()
            rospy.loginfo("[Tracker - STATE] RTL - clearing mission")
        if data.mode == "MANUAL":
            self.state = "MANUAL"
            rospy.loginfo("[Tracker - STATE] MANUAL")
            rospy.signal_shutdown("[Tracker - INFO] Manual mode, gathering ROS bags and shutting down")

    def update_mission_callback(self, data):
        """
        ROS Service Callback which is called in every mission update (receiving new global waypoint plan)
        """
        if not self.mission_started:
            self.wp_plan.clear()
            count = 0
            if len(data.waypoints) > 2:
                for each_wp in data.waypoints:
                    self.wp_plan.append(calculation_waypoint(latitude=each_wp.x_lat, longitude=each_wp.y_long))
                rospy.loginfo("[Tracker] New WP Plan received :")
                for norm_wp in self.wp_plan:
                    rospy.loginfo("[Tracker] %s: Latitude: %s, Longitude: %s", str(count), str(norm_wp.latitude),
                                  str(norm_wp.longitude))
                    count = count + 1
                rospy.loginfo("[Tracker] Total waypoints: %s", str(count))
                if len(data.waypoints) == len(self.wp_plan):
                    rospy.loginfo("[Tracker] Got the WP plan")
                    self.has_wp_plan = True
                    self.home.latitude = self.wp_plan[0].latitude
                    self.home.longitude = self.wp_plan[0].longitude

    def position_callback(self, data):
        """
        ROS Service Callback which is called whenever the UAV reaches the next waypoint. Responsible for calculating
        the global space integration metric
        """
        if self.state == "AUTO" and self.has_wp_plan == True:
            self.mission_started = True
            integrated_area = 0
            previous_wp = self.wp_plan[self.waypoint_iter]
            next_wp = self.wp_plan[self.waypoint_iter + 1]

            closest_point = calc.get_closest_point_on_segment(previous_wp.latitude, previous_wp.longitude,
                                                              next_wp.latitude, next_wp.longitude,
                                                              data.latitude, data.longitude)
            if self.waypoint_iter > 0:
                if not self.integration:
                    rospy.loginfo("========= INTEGRATION STARTED ==========")
                    self.integration = True
                    self.vasi_mikri = calc.get_distance(data.latitude, data.longitude, closest_point.latitude,
                                                        closest_point.longitude)
                    self.pr_closest_point.latitude = closest_point.latitude
                    self.pr_closest_point.longitude = closest_point.longitude

                else:
                    vasi_megali = calc.get_distance(data.latitude, data.longitude, closest_point.latitude,
                                                    closest_point.longitude)
                    integrated_area = integrated_area + ((self.vasi_mikri + vasi_megali) * calc.get_distance(
                        self.pr_closest_point.latitude, self.pr_closest_point.longitude, closest_point.latitude,
                        closest_point.longitude)) / 2
                    self.vasi_mikri = vasi_megali
                    self.pr_closest_point = closest_point

            bearing = calc.find_bearing(previous_wp, next_wp)

            # This section is commented out indicating another tested way for this path planning approach:
            # For calculating the next waypoint by including the distance from the actual position of the UAV:
            #
            # remaining_distance = DISTANCE - get_distance(data.latitude, data.longitude, closest_point.latitude,
            #                                              closest_point.longitude)
            # c_wp = find_command_waypoint(remaining_distance, bearing, closest_point)

            c_wp = calc.find_command_waypoint(DISTANCE, bearing, closest_point)

            if calc.get_distance(data.latitude, data.longitude, next_wp.latitude, next_wp.longitude) < 0.03:  # meters
                self.waypoint_iter += 1
                rospy.loginfo("[Tracker] Waypoint id: %s", str(self.waypoint_iter))
                if self.waypoint_iter == (len(self.wp_plan) - 1):
                    rospy.loginfo("[Tracker] For distance of: %s km the integrated area is: %s sq.kilometers ",
                                  str(DISTANCE), str(integrated_area))
                    self.state = "RTL"
                    self.has_wp_plan = False

            the_req = WaypointPushRequest()
            the_req.waypoints.insert(0, construct_waypoint(c_wp))

            rospy.loginfo("[Tracker] Intermediate waypoint, latitude: %s, longitude: %s ", str(c_wp.latitude),
                          str(c_wp.longitude))

            # http://docs.ros.org/api/mavros_msgs/html/srv/WaypointPush.html.
            # An indication that the current service might not be needed
            # current_service(0)

            command_service = rospy.ServiceProxy('mavros/mission/push', WaypointPush)
            command_service(the_req)

    def subscribe(self):

        rospy.Subscriber("mavros/state", State, self.state_callback)
        rospy.Subscriber("mavros/mission/waypoints", WaypointList, self.update_mission_callback)
        rospy.Subscriber("mavros/global_position/global", NavSatFix, self.position_callback)
