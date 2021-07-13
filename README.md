# ROSPY onboard tracker

This is a ROS python node for executing on-board each UAV, for research purposes. 

Given a specific waypoint plan, the node sends a single waypoint to emulate a "go-to" command (not an actual go-to command due to autopilot restrictions), at a 3Hz rate, in order to reduce the flight error of following the plan. 

This is the node used for the results of the paper presented in RED-UAS conference: https://ieeexplore.ieee.org/abstract/document/8101643/

----
# Installation

ROS Python (rospy) package mavros and mavros_msgs are required for exchanging messages with the ROS publishers and subscribers. 
Rospy is not available through pip. You can either "sudo apt-get install -y python-rospy" or

```bash
virtualenv -p python3 venv
. ./venv/bin/activate
pip install --extra-index-url https://rospypi.github.io/simple/ rospy
```

from https://github.com/rospypi/simple.

This installs only the headers for rospy. You still need ROS and mavros to operate.

# Usage

This script needs to run on the onboard computer of a UAV with ardupilot. 
During a waypoint list flight in AUTO mode, the script sends new waypoint plans in 3Hz indicating a new waypoint to be followed.
Just execute onboard_tracker node in order to activate.

# License
GPLv3.

