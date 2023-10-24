#!/usr/bin/env python3

import rospy, sys, os

# Get the current script directory
script_dir = os.path.dirname(os.path.abspath(__file__))
# Append the parent directory of the script directory to the Python path
sys.path.append(os.path.join(script_dir, ".."))

from Controller.controller import Controller
from utils.config import*

config = load_main_config()
control_method = config["control_method"]

# Ros Node Initialization:
rospy.init_node('controller', anonymous=True)

controller = Controller()

rospy.spin()