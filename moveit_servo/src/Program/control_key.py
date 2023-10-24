#!/usr/bin/env python3

import rospy
import curses
import sys
import os

# Get the current script directory
script_dir = os.path.dirname(os.path.abspath(__file__))
# Append the parent directory of the script directory to the Python path
sys.path.append(os.path.join(script_dir, ".."))

from Controller.keyboard import KeyBoardController

def main(stdscr):
    # Clear and refresh the curses window
    stdscr.clear()
    stdscr.refresh()

    # ROS node initialization
    rospy.init_node('turtlebot_controller', anonymous=True)

    # Create an instance of the Controller class
    controller = KeyBoardController(stdscr)

    # Start the keyboard control
    controller.keyboard_control()

    # Cleanup for curses (just in case, although curses.wrapper should handle this)
    curses.nocbreak()
    stdscr.keypad(0)
    curses.echo()
    curses.endwin()

if __name__ == "__main__":
    curses.wrapper(main)