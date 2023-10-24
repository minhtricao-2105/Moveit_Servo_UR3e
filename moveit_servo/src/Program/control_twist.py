#!/usr/bin/env python3

import rospy, sys, os

# Get the current script directory
script_dir = os.path.dirname(os.path.abspath(__file__))
# Append the parent directory of the script directory to the Python path
sys.path.append(os.path.join(script_dir, ".."))

from RobotBaseClass.twist_controller import TwistController
from utils.config import*

# Ros Node Initialization:
rospy.init_node('controller', anonymous=True)

controller = TwistController()

rospy.spin()