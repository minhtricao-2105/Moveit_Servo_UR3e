#Import Library:
import rospy, keyboard
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Header

class Controller:

    def __init__(self):
        print('Robot is controlles by Controller!')

        # publisher to moveit servo
        self.pub = rospy.Publisher('/servo_server/delta_twist_cmds', TwistStamped, queue_size=10)
        self.command = TwistStamped()
        self.sub = rospy.Subscriber('joy', Joy, self.joy_callback)
        self.command_loop()
            
    def joy_callback(self, msg):
        # Define the command and send to the robot:
        if msg.buttons[5] == 1:
            self.command.twist.linear.x = msg.axes[0]*0.05
            self.command.twist.linear.y = msg.axes[1]*0.05
            self.command.twist.linear.z = msg.axes[3]*0.05
            self.command.twist.angular.x = 0
            self.command.twist.angular.y = 0
            self.command.twist.angular.z = 0

            print(f'Sennding Linear x: {msg.axes[0]*0.05}, Linear y: {msg.axes[1]*0.05}, Linear z: {msg.axes[2]*0.05}')
        else:
            print('Press RB to begin sending velocity to the robot')
            self.command.twist.linear.x = 0
            self.command.twist.linear.y = 0
            self.command.twist.linear.z = 0
            self.command.twist.angular.x = 0
            self.command.twist.angular.y = 0
            self.command.twist.angular.z = 0


    def command_loop(self):
        rate = rospy.Rate(100)  # 100Hz
        while not rospy.is_shutdown():
            # Update the header timestamp to current time
            self.command.header.stamp = rospy.Time.now()
            self.pub.publish(self.command)
            rate.sleep()

        print('Sending command')
    

    
        
        