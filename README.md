# ROSPY onboard tracker

This is a ROS python node for executing on-board each UAV, for research purposes. 

Given a specific waypoint plan, the node sends a single waypoint to emulate a "go-to" command (not an actual go-to command due to autopilot restrictions), at a 3Hz rate, in order to reduce the flight error of following the plan. 

This is the node used for the results of the paper presented in RED-UAS conference: https://ieeexplore.ieee.org/abstract/document/8101643/

----

ROS Python (rospy) package mavros is required for exchanging messages with the ROS publishers and subscribers. 
