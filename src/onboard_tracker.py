#!/usr/bin/env python
import rospy
import tracker.uav

if __name__ == '__main__':
    """
    Main function
    """
    rospy.sleep(10)

    rospy.init_node('onboard_tracker', anonymous=False)
    rospy.loginfo("[Tracker] Just started")

    fixed_wing = tracker.uav.UAV
    fixed_wing.subscribe()

    rospy.spin()
