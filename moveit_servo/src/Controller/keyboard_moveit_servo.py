#!/usr/bin/env python3

# Importing modules:
import rospy
import curses

# Importing messages:
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped

class KeyBoardController:

    # Constructor:
    def __init__(self, stdscr):

        # Publisher:
        self.pub = rospy.Publisher('/servo_server/delta_twist_cmds', TwistStamped, queue_size=10)

        # Initializing the Twist message:
        self.command = TwistStamped()

        self.stdscr = stdscr
        curses.cbreak()
        self.stdscr.keypad(1)
        self.stdscr.refresh()

        self.stdscr.addstr(0,10,"Hit 'q' to quit")
        self.stdscr.refresh()

    def keyboard_control(self):
        
        rate = rospy.Rate(10)  # 10Hz

        while True:
            key = self.stdscr.getch()
            if key == ord('w'):
                self.command.twist.linear.x = 0.05
            elif key == ord('s'):
                self.command.twist.linear.x = -0.05
            else:
                self.command.twist.linear.x = 0

            if key == ord('a'):
                self.command.twist.linear.y = 0.05
            elif key == ord('d'):
                self.command.twist.linear.y = -0.05
            else:
                self.command.twist.linear.y = 0

            if key == ord('r'):
                self.command.twist.linear.z = 0.05
            elif key == ord('f'):
                self.command.twist.linear.z = -0.05
            else:
                self.command.twist.linear.z = 0
            
            if key == ord('q'): # to quit
                break

            # Update the header timestamp to current time
            self.command.header.stamp = rospy.Time.now()
            self.pub.publish(self.command)


    